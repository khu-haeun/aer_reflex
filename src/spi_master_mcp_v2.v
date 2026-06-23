`timescale 1ns / 1ps
//============================================================================
// spi_master_mcp_v2 — TT 칩 안에서 MCP2515 를 부리는 SPI 마스터 드라이버
//  ★v1(04_rs_C) 대비 단 하나 바뀐 것: SCLK 분주를 합성 고정 파라미터(HALF)에서
//    ★런타임 입력(half_div)으로★ 바꿨다. → PS 가 PL GPIO 로 칩 SPI 레지스터(0x03)에
//    써넣으면, 재합성 없이 칩→MCP SPI 속도를 바꿀 수 있다. (sclk = clk/(2*half_div))
//  나머지(명령 FSM·바이트 시프트·Mode 0 상승샘플)는 04 검증본과 동일.
//----------------------------------------------------------------------------
//  지원 명령(op):
//    OP_RESET (0):[0xC0]              OP_WRITE(1):[0x02,addr,wdata]
//    OP_READ  (2):[0x03,addr,캡처]    OP_RTS  (3):[0x80|wmask[2:0]]
//    OP_STATUS(4):[0xA0,캡처]         OP_BITMOD(5):[0x05,addr,wmask,wdata]
//  사용: req 1클럭 펄스(op/addr/wdata/wmask 동반) → busy 대기 → done 펄스에 rdata 유효.
//============================================================================
module spi_master_mcp_v2 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  half_div,        // ★런타임 SCLK 반주기(clk 사이클). 0이면 1로 클램프.
    // 요청 인터페이스
    input  wire        req,             // 1클럭 펄스로 트랜잭션 시작
    input  wire [2:0]  op,              // OP_*
    input  wire [6:0]  addr,            // 레지스터 주소 (WRITE/READ/BITMOD)
    input  wire [7:0]  wdata,           // 쓰기 데이터
    input  wire [7:0]  wmask,           // BITMOD 마스크 / RTS 버퍼선택비트
    output reg  [7:0]  rdata,           // READ/STATUS 결과
    output reg         busy,
    output reg         done,            // 완료 1클럭 펄스
    // MCP2515 SPI 마스터 핀
    output reg         m_sclk,
    output wire        m_mosi,
    output reg         m_csn,
    input  wire        m_miso
);
    localparam [2:0] OP_RESET=3'd0, OP_WRITE=3'd1, OP_READ=3'd2,
                     OP_RTS=3'd3, OP_STATUS=3'd4, OP_BITMOD=3'd5;

    // ★0 분주 방지 (최소 1). 합성 시 상수전파로 비교기 간소화됨.
    wire [7:0] hd = (half_div == 8'd0) ? 8'd1 : half_div;

    // ── 래치된 요청 ──
    reg [2:0] op_r;
    reg [6:0] addr_r;
    reg [7:0] wdata_r, wmask_r;

    // ── op 별 바이트 수 (조합) ──
    reg [2:0] nbytes;
    always @* begin
        case (op_r)
            OP_RESET:  nbytes = 3'd1;
            OP_WRITE:  nbytes = 3'd3;
            OP_READ:   nbytes = 3'd3;
            OP_RTS:    nbytes = 3'd1;
            OP_STATUS: nbytes = 3'd2;
            OP_BITMOD: nbytes = 3'd4;
            default:   nbytes = 3'd1;
        endcase
    end

    // ── 현재 바이트 값 + 읽기여부 (조합, byteidx 로 인덱싱) ──
    reg [2:0] byteidx;
    reg [7:0] curbyte;
    reg       curread;
    always @* begin
        curbyte = 8'h00;
        curread = 1'b0;
        case (op_r)
            OP_RESET:  curbyte = 8'hC0;
            OP_RTS:    curbyte = (8'h80 | {5'b0, wmask_r[2:0]});
            OP_WRITE:
                case (byteidx)
                    3'd0:    curbyte = 8'h02;
                    3'd1:    curbyte = {1'b0, addr_r};
                    default: curbyte = wdata_r;
                endcase
            OP_READ:
                case (byteidx)
                    3'd0:    curbyte = 8'h03;
                    3'd1:    curbyte = {1'b0, addr_r};
                    default: begin curbyte = 8'h00; curread = 1'b1; end
                endcase
            OP_STATUS:
                case (byteidx)
                    3'd0:    curbyte = 8'hA0;
                    default: begin curbyte = 8'h00; curread = 1'b1; end
                endcase
            OP_BITMOD:
                case (byteidx)
                    3'd0:    curbyte = 8'h05;
                    3'd1:    curbyte = {1'b0, addr_r};
                    3'd2:    curbyte = wmask_r;
                    default: curbyte = wdata_r;
                endcase
            default: curbyte = 8'h00;
        endcase
    end

    // ── 바이트 시프트 엔진 (MSB first, Mode 0) ──
    reg [7:0]  tx_sh, rx_sh;
    reg [2:0]  bitcnt;
    reg [15:0] hcnt;
    reg        phase;          // 0 = 낮은 반주기, 1 = 높은 반주기
    reg        byte_busy, byte_done, byte_start;
    assign m_mosi = tx_sh[7];

    always @(posedge clk) begin
        if (!rst_n) begin
            m_sclk<=1'b0; bitcnt<=0; hcnt<=0; phase<=0;
            byte_busy<=0; byte_done<=0; tx_sh<=0; rx_sh<=0;
        end else begin
            byte_done <= 1'b0;
            if (byte_start) begin
                tx_sh <= curbyte; bitcnt<=0; hcnt<=0; phase<=0;
                m_sclk<=1'b0; byte_busy<=1'b1;
            end else if (byte_busy) begin
                if (hcnt == {8'd0, hd} - 16'd1) begin
                    hcnt <= 0;
                    if (phase==1'b0) begin
                        m_sclk <= 1'b1; phase <= 1'b1;     // 상승엣지: MISO 샘플
                        rx_sh  <= {rx_sh[6:0], m_miso};
                    end else begin
                        m_sclk <= 1'b0; phase <= 1'b0;     // 하강엣지: 다음 비트
                        if (bitcnt==3'd7) begin byte_busy<=1'b0; byte_done<=1'b1; end
                        else begin bitcnt <= bitcnt+1'b1; tx_sh <= {tx_sh[6:0],1'b0}; end
                    end
                end else hcnt <= hcnt + 1'b1;
            end
        end
    end

    // ── 명령 FSM: CS 로우 → 바이트들 순차 → CS 하이 ──
    localparam [2:0] C_IDLE=3'd0, C_CSLOW=3'd1, C_BSTART=3'd2,
                     C_BWAIT=3'd3, C_CSHIGH=3'd4, C_DONE=3'd5;
    reg [2:0] cs;
    reg [3:0] setup;

    always @(posedge clk) begin
        if (!rst_n) begin
            cs<=C_IDLE; m_csn<=1'b1; busy<=0; done<=0; byte_start<=0;
            byteidx<=0; rdata<=0; op_r<=0; addr_r<=0; wdata_r<=0; wmask_r<=0; setup<=0;
        end else begin
            done <= 1'b0; byte_start <= 1'b0;
            case (cs)
                C_IDLE:
                    if (req) begin
                        op_r<=op; addr_r<=addr; wdata_r<=wdata; wmask_r<=wmask;
                        byteidx<=0; busy<=1'b1; m_csn<=1'b0; setup<=0; cs<=C_CSLOW;
                    end
                C_CSLOW:                                   // CS 셋업 시간
                    if (setup==4'd3) cs<=C_BSTART; else setup<=setup+1'b1;
                C_BSTART: begin byte_start<=1'b1; cs<=C_BWAIT; end
                C_BWAIT:
                    if (byte_done) begin
                        if (curread) rdata<=rx_sh;
                        if (byteidx == nbytes-1'b1) cs<=C_CSHIGH;
                        else begin byteidx<=byteidx+1'b1; cs<=C_BSTART; end
                    end
                C_CSHIGH: begin m_csn<=1'b1; setup<=0; cs<=C_DONE; end
                C_DONE:                                    // CS 회복 시간
                    if (setup==4'd3) begin busy<=0; done<=1'b1; cs<=C_IDLE; end
                    else setup<=setup+1'b1;
                default: cs<=C_IDLE;
            endcase
        end
    end
endmodule

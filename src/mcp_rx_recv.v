`timescale 1ns / 1ps
//============================================================================
// mcp_rx_recv — MCP2515 수신버퍼0 에서 한 CAN 프레임을 읽어온다 (C 단계 4단계)
//----------------------------------------------------------------------------
//  MCP2515 의 인터럽트(mcp_int, active-low)가 서면(=수신버퍼0 에 프레임 있음),
//  spi_master_mcp 드라이버로 RXB0 레지스터들을 순차로 READ 한다:
//    RXB0SIDH(0x61)=식별자[10:3], RXB0SIDL(0x62)[7:5]=식별자[2:0],
//    RXB0DLC(0x65)[3:0]=길이, RXB0D0~D7(0x66~0x6D)=데이터 8바이트.
//  마지막에 CANINTF 의 RX0IF(비트0)를 BIT MODIFY 로 지워 인터럽트를 해제한다.
//  완료되면 rx_valid 1클럭 펄스와 함께 rx_id / rx_dlc / rx_data 유효.
//============================================================================
module mcp_rx_recv (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        mcp_int,          // active-low (0 = 수신 프레임 대기)
    input  wire        grant,            // ★중재기 grant
    output wire        seq_active,        // ★시퀀스 진행 중
    // 드라이버 핸드셰이크
    output reg         req,
    output reg  [2:0]  op,
    output reg  [6:0]  addr,
    output reg  [7:0]  wdata,
    output reg  [7:0]  wmask,
    input  wire [7:0]  rdata,
    input  wire        busy,
    input  wire        done,
    // 디코드된 프레임 출력
    output reg  [10:0] rx_id,
    output reg  [3:0]  rx_dlc,
    output reg  [63:0] rx_data,
    output reg         rx_valid          // 1클럭 펄스
);
    localparam [2:0] OP_READ=3'd2, OP_BITMOD=3'd5;
    localparam CANINTF=7'h2C;
    localparam integer NSTEPS = 12;

    reg [3:0] step;

    // 현재 스텝 주소/동작 (조합)
    reg       s_bitmod;
    reg [6:0] s_addr;
    always @* begin
        s_bitmod=1'b0; s_addr=7'h00;
        case (step)
            4'd0:  s_addr=7'h61;
            4'd1:  s_addr=7'h62;
            4'd2:  s_addr=7'h65;
            4'd3:  s_addr=7'h66;
            4'd4:  s_addr=7'h67;
            4'd5:  s_addr=7'h68;
            4'd6:  s_addr=7'h69;
            4'd7:  s_addr=7'h6A;
            4'd8:  s_addr=7'h6B;
            4'd9:  s_addr=7'h6C;
            4'd10: s_addr=7'h6D;
            4'd11: begin s_bitmod=1'b1; s_addr=CANINTF; end   // RX0IF 클리어
            default: ;
        endcase
    end

    localparam [2:0] R_IDLE=3'd0, R_ISSUE=3'd1, R_WAIT=3'd2, R_CAP=3'd3, R_NEXT=3'd4, R_DONE=3'd5, R_WAIT_HIGH=3'd6;
    reg [2:0] r;
    reg [8:0] wh_cnt;   // ★R_WAIT_HIGH 타임아웃(연속/버스트 수신서 INT 안 풀려도 재무장 → 현재포즈 안 얼게)
    // ★HW 버그 수정(2026-06-18, 04 parked 원인): mcp_int 비동기핀 2단 동기화 + 읽기후 INT high 대기.
    //   실 MCP 는 RX0IF 클리어 후 INT 가 몇 SPI클럭 뒤 풀림 → 동기화·대기 없이 레벨로 보면 클리어 직후
    //   R_IDLE 이 아직 low 인 INT 를 또 잡아 ★빈 RXB0 재독(쓰레기 포즈) + c2 점유로 송신 굶음★.
    //   sim 모델 int_n 은 조합(~CANINTF[0])이라 즉시 풀려 못 잡았음.
    reg [1:0] int_sync;
    always @(posedge clk) if (!rst_n) int_sync <= 2'b11; else int_sync <= {int_sync[0], mcp_int};
    wire int_asserted = ~int_sync[1];                          // 동기화된 INT 로우 = 수신 프레임 대기
    assign seq_active = (r != R_IDLE) && (r != R_WAIT_HIGH);   // ★대기 중엔 드라이버 점유 해제(송신/관측 안 굶김)
    always @(posedge clk) begin
        if (!rst_n) begin
            r<=R_IDLE; req<=0; op<=0; addr<=0; wdata<=0; wmask<=0;
            step<=0; rx_valid<=0; rx_id<=0; rx_dlc<=0; rx_data<=0; wh_cnt<=9'd0;
        end else begin
            req<=1'b0; rx_valid<=1'b0;
            case (r)
                R_IDLE: if (int_asserted) begin step<=0; r<=R_ISSUE; end  // ★동기화 INT 로우 → 수신 시작
                R_ISSUE: if (grant) begin       // ★점유 허가 받은 뒤에만 발행
                    if (s_bitmod) begin op<=OP_BITMOD; wmask<=8'h01; wdata<=8'h00; end
                    else          begin op<=OP_READ;   wmask<=8'h00; wdata<=8'h00; end
                    addr<=s_addr; req<=1'b1; r<=R_WAIT;
                end
                R_WAIT: if (done) r<=R_CAP;
                R_CAP: begin
                    case (step)
                        4'd0:  rx_id[10:3] <= rdata;
                        4'd1:  rx_id[2:0]  <= rdata[7:5];
                        4'd2:  rx_dlc      <= rdata[3:0];
                        4'd3:  rx_data[7:0]   <= rdata;
                        4'd4:  rx_data[15:8]  <= rdata;
                        4'd5:  rx_data[23:16] <= rdata;
                        4'd6:  rx_data[31:24] <= rdata;
                        4'd7:  rx_data[39:32] <= rdata;
                        4'd8:  rx_data[47:40] <= rdata;
                        4'd9:  rx_data[55:48] <= rdata;
                        4'd10: rx_data[63:56] <= rdata;
                        default: ;     // step 11 = BIT MODIFY, 캡처 없음
                    endcase
                    r<=R_NEXT;
                end
                R_NEXT: if (step==NSTEPS-1) r<=R_DONE;
                        else begin step<=step+1'b1; r<=R_ISSUE; end
                R_DONE: begin rx_valid<=1'b1; r<=R_WAIT_HIGH; wh_cnt<=9'd0; end   // ★읽기완료 → INT 풀릴 때까지(재독 방지)
                R_WAIT_HIGH: if (!int_asserted) r<=R_IDLE;                        // INT high(de-assert) → 재무장
                             else if (wh_cnt >= 9'd256) r<=R_IDLE;                // ★연속/버스트 수신: 타임아웃(~13µs, de-assert지연보다 충분히 김) 후 재무장 → 현재포즈 계속 갱신
                             else wh_cnt <= wh_cnt + 1'b1;
                default: r<=R_IDLE;
            endcase
        end
    end
endmodule

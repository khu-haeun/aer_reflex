`timescale 1ns / 1ps
//============================================================================
// mcp_probe — ★관측성(observability)의 핵심: 칩이 MCP2515 레지스터를 되읽어 노출
//----------------------------------------------------------------------------
//  지난 실패의 근본 원인은 "칩이 MCP 를 제대로 설정/송신했는지 실물에서 볼 길이 없었다"
//  였다(블랙박스). 이 모듈이 그걸 깬다: 초기화가 끝나면(init_done), 주기적으로
//  MCP2515 의 진단 레지스터들을 READ 해서 칩 안의 reg 로 보관한다. 이 값을
//  spi_slave 의 읽기 레지스터로 노출하면 → PL 이 SPI 로 되읽어 → PS GPIO 로 → 시리얼.
//
//  되읽는 레지스터(데이터시트):
//    CANSTAT(0x0E) : 상위3비트 OPMOD — 000=정상모드 도달했나
//    CANCTRL(0x0F) : 요청 모드/클럭출력
//    CNF1/2/3(0x2A/0x29/0x28) : 비트타이밍이 우리가 쓴 값(00/C0/80)으로 들어갔나
//    EFLG(0x2D)    : 에러 플래그(버스오류·수신오버플로)
//    TEC(0x1C)/REC(0x1D) : 송신/수신 에러 카운터 — CAN 이 ACK 못 받으면 TEC 폭증
//    CANINTF(0x2C) : 인터럽트 플래그(수신·에러)
//  → "설정에서 틀렸나 / 송신에서 틀렸나" 를 실물에서 직접 구분 가능.
//
//  드라이버는 mcp_arb 가 중재(이 모듈은 ★최저 우선순위★ — 안전 송신을 절대 안 늦춤).
//============================================================================
module mcp_probe #(
    parameter integer PROBE_DIV = 100000     // 되읽기 주기(클럭). 합성 기본 ~2ms@50MHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init_done,        // 초기화 끝난 뒤에만 되읽기 시작
    input  wire        grant,            // 중재기 grant
    output wire        seq_active,        // 시퀀스 진행 중(점유 요청)
    // 드라이버 핸드셰이크
    output reg         req,
    output reg  [2:0]  op,
    output reg  [6:0]  addr,
    output reg  [7:0]  wdata,
    output reg  [7:0]  wmask,
    input  wire [7:0]  rdata,
    input  wire        busy,
    input  wire        done,
    // 되읽은 진단값 (spi_slave 로)
    output reg  [7:0]  canstat, canctrl, cnf1, cnf2, cnf3, eflg, tec, rec, canintf,
    output reg         probe_valid       // 한 바퀴 끝날 때마다 1클럭 펄스
);
    localparam [2:0] OP_READ=3'd2;
    localparam integer NSTEPS = 9;        // 9개 레지스터

    // 되읽기 주소표 (조합)
    reg [3:0] step;
    reg [6:0] s_addr;
    always @* begin
        case (step)
            4'd0: s_addr = 7'h0E;   // CANSTAT
            4'd1: s_addr = 7'h0F;   // CANCTRL
            4'd2: s_addr = 7'h2A;   // CNF1
            4'd3: s_addr = 7'h29;   // CNF2
            4'd4: s_addr = 7'h28;   // CNF3
            4'd5: s_addr = 7'h2D;   // EFLG
            4'd6: s_addr = 7'h1C;   // TEC
            4'd7: s_addr = 7'h1D;   // REC
            default: s_addr = 7'h2C; // CANINTF
        endcase
    end

    reg [31:0] divcnt;
    localparam [2:0] P_IDLE=3'd0, P_WAITDIV=3'd1, P_ISSUE=3'd2, P_WAIT=3'd3, P_CAP=3'd4, P_NEXT=3'd5, P_DONE=3'd6;
    reg [2:0] p;
    assign seq_active = (p != P_IDLE) && (p != P_WAITDIV);   // 실제 트랜잭션 묶음 동안만 점유

    always @(posedge clk) begin
        if (!rst_n) begin
            p<=P_IDLE; req<=0; op<=0; addr<=0; wdata<=0; wmask<=0; step<=0; divcnt<=0;
            canstat<=0; canctrl<=0; cnf1<=0; cnf2<=0; cnf3<=0; eflg<=0; tec<=0; rec<=0; canintf<=0;
            probe_valid<=0;
        end else begin
            req <= 1'b0; probe_valid <= 1'b0;
            case (p)
                P_IDLE: if (init_done) begin divcnt<=0; p<=P_WAITDIV; end
                P_WAITDIV:
                    if (divcnt >= PROBE_DIV-1) begin step<=0; p<=P_ISSUE; end
                    else divcnt <= divcnt + 1'b1;
                P_ISSUE: if (grant) begin
                    op<=OP_READ; addr<=s_addr; wdata<=8'h00; wmask<=8'h00; req<=1'b1; p<=P_WAIT;
                end
                P_WAIT: if (done) p<=P_CAP;
                P_CAP: begin
                    case (step)
                        4'd0: canstat <= rdata;
                        4'd1: canctrl <= rdata;
                        4'd2: cnf1    <= rdata;
                        4'd3: cnf2    <= rdata;
                        4'd4: cnf3    <= rdata;
                        4'd5: eflg    <= rdata;
                        4'd6: tec     <= rdata;
                        4'd7: rec     <= rdata;
                        default: canintf <= rdata;
                    endcase
                    p <= P_NEXT;
                end
                P_NEXT: if (step==NSTEPS-1) p<=P_DONE;
                        else begin step<=step+1'b1; p<=P_ISSUE; end
                P_DONE: begin probe_valid<=1'b1; divcnt<=0; p<=P_WAITDIV; end
                default: p<=P_IDLE;
            endcase
        end
    end
endmodule

`timescale 1ns / 1ps
//============================================================================
// mcp_init — 부팅 시 MCP2515 초기화 시퀀스 (C 단계 2단계)
//----------------------------------------------------------------------------
//  spi_master_mcp 드라이버를 순서대로 부려 MCP2515 를 설정한다:
//    0) 소프트웨어 리셋(0xC0) → 설정 모드 진입
//    1~3) 비트타이밍 CNF1/CNF2/CNF3 (8MHz/1Mbps 검증값)
//    4) RXB0CTRL = 0x60 (지금은 모든 프레임 수신; 필터는 4단계에서 0x2A1·0x2A5~7 로 좁힘)
//    5) CANINTE = 0x01 (수신버퍼0 인터럽트 활성)
//    6) CANCTRL = 0x00 (정상 모드 요청)
//    7) CANSTAT 의 OPMOD[7:5] 가 000(정상) 될 때까지 폴링
//  완료되면 init_done=1. 그 뒤 드라이버는 런타임(송수신) 로직이 사용.
//
//  ★초기화 동안 드라이버를 단독 점유한다. 통합(5단계)에서 런타임과 중재.
//============================================================================
module mcp_init #(
    parameter [7:0] V_CNF1 = 8'h00,   // 8MHz, 1Mbps (검증값)
    parameter [7:0] V_CNF2 = 8'hC0,
    parameter [7:0] V_CNF3 = 8'h80,
    // ★리셋(0xC0) 후 발진기 안정화 지연. 이게 없으면 CNF write 가 씹힘(HIL서 확인,
    //   되읽기 CNF=0). 듀에 골든레퍼런스도 reset 후 delay(10ms). 50MHz×500000=10ms.
    parameter integer RESET_DELAY = 500000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,             // 1클럭 펄스로 초기화 시작
    input  wire        grant,             // ★중재기 grant (드라이버 점유 허가)
    output wire        seq_active,         // ★시퀀스 진행 중(중재기에 점유 요청)
    // 드라이버 핸드셰이크
    output reg         req,
    output reg  [2:0]  op,
    output reg  [6:0]  addr,
    output reg  [7:0]  wdata,
    output reg  [7:0]  wmask,
    input  wire [7:0]  rdata,
    input  wire        busy,
    input  wire        done,
    output reg         init_done,
    output reg  [3:0]  step               // 디버그용 현재 스텝
);
    localparam [2:0] OP_RESET=3'd0, OP_WRITE=3'd1, OP_READ=3'd2;
    localparam CANSTAT=7'h0E, CANCTRL=7'h0F, CNF3=7'h28, CNF2=7'h29, CNF1=7'h2A,
               CANINTE=7'h2B, RXB0CTRL=7'h60;
    localparam integer NSTEPS = 8;        // 스텝 0..7

    // 현재 스텝의 동작 (조합)
    reg       s_poll;
    reg [2:0] s_op;
    reg [6:0] s_addr;
    reg [7:0] s_data;
    always @* begin
        s_poll=1'b0; s_op=OP_WRITE; s_addr=7'h00; s_data=8'h00;
        case (step)
            4'd0: begin s_op=OP_RESET;                              end
            4'd1: begin s_addr=CNF1;     s_data=V_CNF1;             end
            4'd2: begin s_addr=CNF2;     s_data=V_CNF2;             end
            4'd3: begin s_addr=CNF3;     s_data=V_CNF3;             end
            4'd4: begin s_addr=RXB0CTRL; s_data=8'h60;             end
            4'd5: begin s_addr=CANINTE;  s_data=8'h01;             end
            4'd6: begin s_addr=CANCTRL;  s_data=8'h00;             end
            4'd7: begin s_poll=1'b1;     s_addr=CANSTAT;           end
            default: ;
        endcase
    end

    localparam [2:0] M_IDLE=3'd0, M_ISSUE=3'd1, M_WAIT=3'd2, M_CHK=3'd3, M_NEXT=3'd4, M_DONE=3'd5, M_DELAY=3'd6;
    reg [2:0]  m;
    reg [19:0] dly_cnt;
    assign seq_active = (m != M_IDLE) && (m != M_DELAY);   // 지연 동안엔 드라이버 양보(점유 안 함)
    always @(posedge clk) begin
        if (!rst_n) begin
            m<=M_IDLE; req<=0; op<=0; addr<=0; wdata<=0; wmask<=0; step<=0; init_done<=0; dly_cnt<=0;
        end else begin
            req <= 1'b0;
            case (m)
                M_IDLE: if (start) begin step<=0; init_done<=0; m<=M_ISSUE; end
                M_ISSUE: if (grant) begin       // ★점유 허가 받은 뒤에만 발행
                    op    <= s_poll ? OP_READ : s_op;
                    addr  <= s_addr;
                    wdata <= s_data;
                    wmask <= 8'h00;
                    req   <= 1'b1;
                    m     <= M_WAIT;
                end
                M_WAIT: if (done) m<=M_CHK;
                M_CHK:
                    if (s_poll) begin
                        if ((rdata & 8'hE0)==8'h00) m<=M_NEXT;   // OPMOD=000 정상모드 도달
                        else                        m<=M_ISSUE;  // 아직 → 다시 폴
                    end else m<=M_NEXT;
                M_NEXT:
                    if (step==NSTEPS-1) m<=M_DONE;
                    else begin
                        step<=step+1'b1;
                        if (step==4'd0) begin dly_cnt<=0; m<=M_DELAY; end  // ★리셋 직후 안정화 지연
                        else m<=M_ISSUE;
                    end
                M_DELAY:                                   // ★발진기 안정화 대기 후 CNF write
                    if (dly_cnt >= RESET_DELAY-1) m<=M_ISSUE;
                    else dly_cnt <= dly_cnt + 1'b1;
                M_DONE: begin init_done<=1'b1; m<=M_IDLE; end
                default: m<=M_IDLE;
            endcase
        end
    end
endmodule

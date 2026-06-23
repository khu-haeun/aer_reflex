`timescale 1ns / 1ps
//============================================================================
// mcp_tx_send — 한 CAN 프레임을 MCP2515 송신버퍼0 에 적재 후 송신요청 (C 단계 3단계)
//----------------------------------------------------------------------------
//  send 펄스가 들어오면 (id, dlc, data 동반) spi_master_mcp 드라이버를 순차로 부려:
//    TXB0SIDH(0x31)=식별자[10:3], TXB0SIDL(0x32)={식별자[2:0],5'b0},
//    TXB0DLC(0x35)={4'b0,dlc}, TXB0D0~D7(0x36~0x3D)=데이터 8바이트,
//    그리고 RTS(송신요청) 버퍼0.
//  완료되면 tx_done 1클럭 펄스. 표준 클래식 CAN, 11비트 식별자.
//
//  ★단일 레지스터 WRITE 들을 이어 붙여 적재(드라이버 단순 유지). 속도가 부족하면
//    나중에 드라이버에 LOAD TX BUFFER 다바이트 명령을 더해 줄일 수 있음.
//============================================================================
module mcp_tx_send (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        send,             // 1클럭 펄스
    input  wire [10:0] id,
    input  wire [3:0]  dlc,
    input  wire [63:0] data,             // D0=data[7:0] ... D7=data[63:56]
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
    output reg         tx_done           // 1클럭 펄스
);
    localparam [2:0] OP_WRITE=3'd1, OP_READ=3'd2, OP_RTS=3'd3;
    localparam integer NSTEPS = 12;
    localparam [9:0] POLL_TIMEOUT = 10'd500;   // ★TXREQ 폴 타임아웃(데드버스 안전망, ~2.5ms)

    reg [10:0] id_r;
    reg [3:0]  dlc_r;
    reg [63:0] data_r;
    reg [3:0]  step;

    // 현재 스텝 동작 (조합)
    reg [2:0] s_op; reg [6:0] s_addr; reg [7:0] s_data, s_mask;
    always @* begin
        s_op=OP_WRITE; s_addr=7'h00; s_data=8'h00; s_mask=8'h00;
        case (step)
            4'd0:  begin s_addr=7'h31; s_data=id_r[10:3];        end
            4'd1:  begin s_addr=7'h32; s_data={id_r[2:0],5'b0};  end
            4'd2:  begin s_addr=7'h35; s_data={4'b0,dlc_r};      end
            4'd3:  begin s_addr=7'h36; s_data=data_r[7:0];       end
            4'd4:  begin s_addr=7'h37; s_data=data_r[15:8];      end
            4'd5:  begin s_addr=7'h38; s_data=data_r[23:16];     end
            4'd6:  begin s_addr=7'h39; s_data=data_r[31:24];     end
            4'd7:  begin s_addr=7'h3A; s_data=data_r[39:32];     end
            4'd8:  begin s_addr=7'h3B; s_data=data_r[47:40];     end
            4'd9:  begin s_addr=7'h3C; s_data=data_r[55:48];     end
            4'd10: begin s_addr=7'h3D; s_data=data_r[63:56];     end
            4'd11: begin s_op=OP_RTS;  s_mask=8'h01;             end   // RTS TXB0
            default: ;
        endcase
    end

    localparam [2:0] T_IDLE=3'd0, T_ISSUE=3'd1, T_WAIT=3'd2, T_NEXT=3'd3,
                     T_POLL=3'd4, T_POLLW=3'd5, T_DONE=3'd6;
    reg [2:0] t;
    reg [9:0] poll_cnt;
    assign seq_active = (t != T_IDLE);
    always @(posedge clk) begin
        if (!rst_n) begin
            t<=T_IDLE; req<=0; op<=0; addr<=0; wdata<=0; wmask<=0;
            step<=0; tx_done<=0; id_r<=0; dlc_r<=0; data_r<=0; poll_cnt<=0;
        end else begin
            req<=1'b0; tx_done<=1'b0;
            case (t)
                T_IDLE: if (send) begin
                            id_r<=id; dlc_r<=dlc; data_r<=data; step<=0; t<=T_ISSUE;
                        end
                T_ISSUE: if (grant) begin       // ★점유 허가 받은 뒤에만 발행
                    op<=s_op; addr<=s_addr; wdata<=s_data; wmask<=s_mask;
                    req<=1'b1; t<=T_WAIT;
                end
                T_WAIT: if (done) t<=T_NEXT;
                T_NEXT: if (step==NSTEPS-1) begin poll_cnt<=10'd0; t<=T_POLL; end  // ★RTS 끝 → 송신완료 폴
                        else begin step<=step+1'b1; t<=T_ISSUE; end
                // ★TXB0CTRL(0x30).TXREQ(bit3) 가 0 될 때까지 폴 → MCP 가 TXB0 를 다 직렬화한 뒤에야
                //   다음 프레임 적재. 안 그러면 송신중 TXB0 덮어써 "앞=N 뒤=N+1" splice(HIL 실측 손상).
                //   타임아웃 = 데드버스(ACK 없음 → TXREQ 영영 안 내려감) 안전망.
                T_POLL: if (grant) begin
                            op<=OP_READ; addr<=7'h30; wdata<=8'h00; wmask<=8'h00; req<=1'b1; t<=T_POLLW;
                        end
                T_POLLW: if (done) begin
                            if (!rdata[3] || poll_cnt==POLL_TIMEOUT) t<=T_DONE;   // TXREQ clear 또는 타임아웃
                            else begin poll_cnt<=poll_cnt+1'b1; t<=T_POLL; end
                         end
                T_DONE: begin tx_done<=1'b1; t<=T_IDLE; end
                default: t<=T_IDLE;
            endcase
        end
    end
endmodule

`timescale 1ns / 1ps
//============================================================================
// reflex_tx_src — 반사 송신 프레임 선택: 비상정지(0x150) vs 포즈(0x155~7)
//----------------------------------------------------------------------------
//  반사 코어의 estop_active / pose_active 에 따라 mcp_tx_mux 의 반사 입력을 만든다:
//    - estop_active : 0x150 비상정지 프레임을 SEND_DIV 주기로 송신.
//    - 아니고 pose_active : reflex_pose_gen 의 포즈 프레임을 그대로 통과.
//  gate_active = estop_active | pose_active : 둘 중 아무 반사든 정상명령을 막아야 하므로.
//============================================================================
module reflex_tx_src #(
    parameter integer SEND_DIV   = 1000,
    parameter [10:0]  ESTOP_ID   = 11'h150,
    parameter [63:0]  ESTOP_DATA = 64'h0           // ★Piper 비상정지 페이로드로 채울 것
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        estop_active,
    input  wire        pose_active,
    // reflex_pose_gen 출력(포즈 프레임)
    input  wire [10:0] pose_id,
    input  wire [3:0]  pose_dlc,
    input  wire [63:0] pose_data,
    input  wire        pose_send,
    // mcp_tx_mux 반사 입력
    output wire [10:0] rid,
    output wire [3:0]  rdlc,
    output wire [63:0] rdata,
    output wire        rsend,
    output wire        gate_active
);
    // 비상정지 주기 송신 펄스
    reg [31:0] divc;            // ★32비트 (SEND_DIV=100000 > 16비트)
    reg        estop_send;
    always @(posedge clk) begin
        if (!rst_n) begin divc<=0; estop_send<=0; end
        else begin
            estop_send <= 1'b0;
            if (estop_active) begin
                if (divc==SEND_DIV-1) begin divc<=0; estop_send<=1'b1; end
                else divc<=divc+1'b1;
            end else divc<=0;
        end
    end

    assign rid   = estop_active ? ESTOP_ID   : pose_id;
    assign rdlc  = estop_active ? 4'd8       : pose_dlc;
    assign rdata = estop_active ? ESTOP_DATA : pose_data;
    assign rsend = estop_active ? estop_send : pose_send;
    assign gate_active = estop_active | pose_active;
endmodule

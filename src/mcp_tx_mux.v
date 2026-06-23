`timescale 1ns / 1ps
//============================================================================
// mcp_tx_mux — 정상/반사 송신 멀티플렉서 = ★게이트의 본체 (정상명령 차단 지점)
//----------------------------------------------------------------------------
//  칩이 CAN 유일 송신자라 정상명령(PS→PL→SPI→칩)과 반사(칩 생성)가 같은 송신경로
//  (mcp_tx_send→MCP)를 공유한다. 충돌·우선순위를 결정하는 ★단 한 곳★ 이 여기다:
//    - 평상시(reflex_active=0): 정상 프레임을 송신, 정상 send 트리거 통과.
//    - 반사중(reflex_active=1): 반사 프레임을 송신, ★정상 send 무시(차단)★.
//  = "위험하면 TT가 PC 정상명령을 끊고 자기 반사를 주입" 의 구현.
//============================================================================
module mcp_tx_mux (
    input  wire        reflex_active,
    // 정상 명령 프레임 (PL 이 SPI 로 칩에 적재)
    input  wire [10:0] normal_id,
    input  wire [3:0]  normal_dlc,
    input  wire [63:0] normal_data,
    input  wire        normal_send,
    // 반사 프레임 (칩 생성)
    input  wire [10:0] reflex_id,
    input  wire [3:0]  reflex_dlc,
    input  wire [63:0] reflex_data,
    input  wire        reflex_send,
    // 선택된 프레임 → mcp_tx_send
    output wire [10:0] sel_id,
    output wire [3:0]  sel_dlc,
    output wire [63:0] sel_data,
    output wire        sel_send
);
    assign sel_id   = reflex_active ? reflex_id   : normal_id;
    assign sel_dlc  = reflex_active ? reflex_dlc  : normal_dlc;
    assign sel_data = reflex_active ? reflex_data : normal_data;
    assign sel_send = reflex_active ? reflex_send : normal_send;   // ★게이트
endmodule

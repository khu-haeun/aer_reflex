`timescale 1ns / 1ps
//============================================================================
// pose_status_decode — 수신 프레임에서 도달 여부 + 현재 포즈 추출 (C 단계 4단계)
//----------------------------------------------------------------------------
//  mcp_rx_recv 가 내놓은 프레임(rx_id, rx_data)을 식별자로 분기해 해석한다:
//   - 0x2A1 (팔 상태): 다섯 번째 바이트(바이트 인덱스 4) = motion_status.
//                      그 값이 0x00 이면 "도달", 0x01 이면 "미도달". reached 래치.
//   - 0x2A5 (관절1·2), 0x2A6 (관절3·4), 0x2A7 (관절5·6):
//                      각 관절은 4바이트 부호있는 정수(0.001도 단위, big-endian).
//                      앞 4바이트 = 앞 관절, 뒤 4바이트 = 뒤 관절. 현재 포즈 래치.
//
//  ★현재 포즈는 "반사가 보낼 목표 포즈를 계산하는 재료" 로 쓴다(트리거 아님).
//    트리거는 FSR(XADC) 그대로. 반사 발동 시 reflex_pose_gen 이 이 현재 포즈에
//    움츠림 델타를 더해 목표를 만든다. 그래서 여기선 도달여부 + 현재 포즈만 내보낸다.
//============================================================================
module pose_status_decode (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx_valid,
    input  wire [10:0] rx_id,
    input  wire [63:0] rx_data,
    output reg         reached,
    output reg signed [31:0] j1, j2, j3, j4, j5, j6
);
    // big-endian 4바이트 → 부호있는 32비트 (앞바이트가 최상위)
    //   앞 관절 = D0 D1 D2 D3 = rx_data[7:0],[15:8],[23:16],[31:24]
    //   뒤 관절 = D4 D5 D6 D7 = rx_data[39:32],[47:40],[55:48],[63:56]
    wire signed [31:0] lo_joint = {rx_data[7:0],  rx_data[15:8], rx_data[23:16], rx_data[31:24]};
    wire signed [31:0] hi_joint = {rx_data[39:32], rx_data[47:40], rx_data[55:48], rx_data[63:56]};

    always @(posedge clk) begin
        if (!rst_n) begin
            reached<=1'b0; j1<=0; j2<=0; j3<=0; j4<=0; j5<=0; j6<=0;
        end else if (rx_valid) begin
            case (rx_id)
                11'h2A1: reached <= (rx_data[39:32] == 8'h00);   // 다섯 번째 바이트=0 → 도달
                11'h2A5: begin j1 <= lo_joint; j2 <= hi_joint; end
                11'h2A6: begin j3 <= lo_joint; j4 <= hi_joint; end
                11'h2A7: begin j5 <= lo_joint; j6 <= hi_joint; end
                default: ;
            endcase
        end
    end
endmodule

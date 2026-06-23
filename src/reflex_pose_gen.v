`timescale 1ns / 1ps
//============================================================================
// reflex_pose_gen — 현재 포즈 기반 반사 목표 생성기 (C 단계, 4단계 디코드 활용)
//----------------------------------------------------------------------------
//  트리거(FSR 등)로 반사가 발동(reflex_active=1)되면:
//   1) 발동 순간의 현재 포즈(cur_j1~6)를 ★래치(스냅샷)★ 한다. (안 잠그면 목표가
//      따라 움직여 못 도달.) 현재 포즈는 pose_status_decode 가 0x2A5~7 로 받은 것.
//   2) 목표[i] = 클램프(스냅샷[i] + 델타[i], URDF 관절한계). 델타는 "움츠림" 양으로,
//      ★부팅 프로그래밍 단계에서 SPI 레지스터로 설정★(여기선 입력 포트, 통합 시 연결).
//      델타는 16비트 부호(±32.767도). 32비트로 부호확장해 더함.
//   3) reflex_active 동안 0x155(J1·J2)/0x156(J3·J4)/0x157(J5·J6) 세 프레임을
//      SEND_DIV 주기로 돌아가며 송신(mcp_tx_mux 의 반사 입력으로).
//  관절 명령 프레임 = 2×부호32비트(0.001도, big-endian). 디코드와 역연산 일치.
//============================================================================
module reflex_pose_gen #(
    parameter signed [31:0] J1_MIN=-32'sd150000, J1_MAX= 32'sd150000,
    parameter signed [31:0] J2_MIN= 32'sd0,      J2_MAX= 32'sd180000,
    parameter signed [31:0] J3_MIN=-32'sd170000, J3_MAX= 32'sd0,
    parameter signed [31:0] J4_MIN=-32'sd100000, J4_MAX= 32'sd100000,
    parameter signed [31:0] J5_MIN=-32'sd70000,  J5_MAX= 32'sd70000,
    parameter signed [31:0] J6_MIN=-32'sd120000, J6_MAX= 32'sd120000,
    // 홈복귀(절대) 목표 — URDF 기본 홈 = 전부 0
    parameter signed [31:0] HOME_J1=32'sd0, HOME_J2=32'sd0, HOME_J3=32'sd0,
    parameter signed [31:0] HOME_J4=32'sd0, HOME_J5=32'sd0, HOME_J6=32'sd0,
    parameter integer SEND_DIV = 1000            // 프레임 간 주기(클럭). 합성 기본은 크게.
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        reflex_active,            // 반사 발동·유지 (반사 코어가 줌)
    input  wire        pose_mode,                // 0=홈복귀(절대), 1=움츠림(현재+델타)
    input  wire        freeze,                   // ★1=현재포즈 그대로 정지(estop/DIP, 델타·홈 무시)
    input  wire [7:0]  reflex_speed,             // ★0x151 속도율(1~100) — 실로봇이 이 속도로 이동
    // 현재 포즈 (pose_status_decode 출력)
    input  wire signed [31:0] cur_j1, cur_j2, cur_j3, cur_j4, cur_j5, cur_j6,
    // 움츠림 델타 (부팅 프로그래밍: SPI 레지스터에서; 16비트 부호)
    input  wire signed [15:0] d_j1, d_j2, d_j3, d_j4, d_j5, d_j6,
    // mcp_tx_mux 의 반사 프레임 입력
    output reg  [10:0] reflex_id,
    output reg  [3:0]  reflex_dlc,
    output reg  [63:0] reflex_data,
    output reg         reflex_send               // 1클럭 펄스
);
    // 부호있는 클램프
    function signed [31:0] clamp;
        input signed [31:0] x, lo, hi;
        begin clamp = (x < lo) ? lo : ((x > hi) ? hi : x); end
    endfunction

    // 2×부호32비트 → 64비트 프레임 데이터 (big-endian: 각 관절 최상위바이트가 먼저)
    //   lo=앞 관절(D0~D3), hi=뒤 관절(D4~D7). data[7:0]=D0.
    function [63:0] enc_frame;
        input signed [31:0] lo, hi;
        begin
            enc_frame = { hi[7:0], hi[15:8], hi[23:16], hi[31:24],
                          lo[7:0], lo[15:8], lo[23:16], lo[31:24] };
        end
    endfunction

    // 16비트 델타를 32비트로 부호확장
    wire signed [31:0] e_j1 = {{16{d_j1[15]}}, d_j1};
    wire signed [31:0] e_j2 = {{16{d_j2[15]}}, d_j2};
    wire signed [31:0] e_j3 = {{16{d_j3[15]}}, d_j3};
    wire signed [31:0] e_j4 = {{16{d_j4[15]}}, d_j4};
    wire signed [31:0] e_j5 = {{16{d_j5[15]}}, d_j5};
    wire signed [31:0] e_j6 = {{16{d_j6[15]}}, d_j6};

    reg signed [31:0] t1, t2, t3, t4, t5, t6;   // 래치된 목표
    reg [1:0]  frame_idx;
    reg [31:0] divcnt;          // ★32비트 (SEND_DIV=100000 > 16비트, HW 버그 예방)
    reg        active_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            t1<=0; t2<=0; t3<=0; t4<=0; t5<=0; t6<=0;
            frame_idx<=0; divcnt<=0; active_d<=0;
            reflex_id<=0; reflex_dlc<=4'd8; reflex_data<=0; reflex_send<=0;
        end else begin
            reflex_send <= 1'b0;
            active_d    <= reflex_active;

            if (reflex_active && !active_d) begin
                // 발동 순간 목표 래치: 움츠림이면 현재+델타, 홈복귀면 절대 홈. 둘 다 클램프.
                // ★freeze(estop): 현재포즈 그대로(델타0). 아니면 pose_mode: 1=현재+델타, 0=홈
                t1 <= freeze ? clamp(cur_j1, J1_MIN, J1_MAX) : (pose_mode ? clamp(cur_j1 + e_j1, J1_MIN, J1_MAX) : clamp(HOME_J1, J1_MIN, J1_MAX));
                t2 <= freeze ? clamp(cur_j2, J2_MIN, J2_MAX) : (pose_mode ? clamp(cur_j2 + e_j2, J2_MIN, J2_MAX) : clamp(HOME_J2, J2_MIN, J2_MAX));
                t3 <= freeze ? clamp(cur_j3, J3_MIN, J3_MAX) : (pose_mode ? clamp(cur_j3 + e_j3, J3_MIN, J3_MAX) : clamp(HOME_J3, J3_MIN, J3_MAX));
                t4 <= freeze ? clamp(cur_j4, J4_MIN, J4_MAX) : (pose_mode ? clamp(cur_j4 + e_j4, J4_MIN, J4_MAX) : clamp(HOME_J4, J4_MIN, J4_MAX));
                t5 <= freeze ? clamp(cur_j5, J5_MIN, J5_MAX) : (pose_mode ? clamp(cur_j5 + e_j5, J5_MIN, J5_MAX) : clamp(HOME_J5, J5_MIN, J5_MAX));
                t6 <= freeze ? clamp(cur_j6, J6_MIN, J6_MAX) : (pose_mode ? clamp(cur_j6 + e_j6, J6_MIN, J6_MAX) : clamp(HOME_J6, J6_MIN, J6_MAX));
                frame_idx <= 2'd0;
                divcnt    <= 16'd0;
            end else if (reflex_active) begin
                if (divcnt == SEND_DIV-1) begin
                    divcnt <= 16'd0;
                    case (frame_idx)
                        2'd0: begin reflex_id<=11'h151; reflex_data<={40'd0, reflex_speed, 16'h0101}; end  // ★속도(ctrl1·move1·속도) 먼저
                        2'd1: begin reflex_id<=11'h155; reflex_data<=enc_frame(t1,t2); end
                        2'd2: begin reflex_id<=11'h156; reflex_data<=enc_frame(t3,t4); end
                        default: begin reflex_id<=11'h157; reflex_data<=enc_frame(t5,t6); end
                    endcase
                    reflex_dlc  <= 4'd8;
                    reflex_send <= 1'b1;
                    frame_idx   <= frame_idx + 1'b1;   // 0(151)→1(155)→2(156)→3(157)→0 (2비트 wrap)
                end else divcnt <= divcnt + 1'b1;
            end else begin
                frame_idx <= 2'd0;
                divcnt    <= 16'd0;
            end
        end
    end
endmodule

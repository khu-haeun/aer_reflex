`timescale 1ns / 1ps
//============================================================================
// reflex_core_c — C단계 반사 결정 코어 (규칙표 + 우선순위 + e-stop/홈/움츠림)
//----------------------------------------------------------------------------
//  reflex_core_tt_v3 의 "규칙표 4개 + XADC 임계 + 디바운스 + 우선순위 중재" 를 그대로
//  쓰되(설정 레지스터는 외부 spi_slave_v4 에서 입력으로 받음), 선택된 action_id 에 따라
//  세 가지 반사 행동을 분기한다:
//    action 1 = 비상정지 : 즉시 발사, 센서 떼면 바로 풀림 (rule a). estop_active.
//    action 2 = 홈복귀   : 절대 목표(현재 무시). 유지 후 도달+센서뗌에 해제 (rule b).
//    action 3 = 움츠림   : 현재+델타 목표. 유지 후 도달+센서뗌에 해제 (rule b).
//  우선순위 중재(높은 prio 우선)가 백스톱: 여러 규칙이 동시에 켜져도 하나만 고른다.
//  (프로그래밍 관례: RULE0=DIP비상정지 prio3 항상, RULE1~3=FSR 중 하나만.)
//
//  출력:
//    estop_active : action 1 활성(즉시·유지 없음).
//    pose_active  : action 2/3 유지 래치(=포즈 게이트).
//    pose_mode    : 0=홈(절대), 1=움츠림(상대) — reflex_pose_gen 모드.
//============================================================================
module reflex_core_c #(
    parameter integer HB_BIT   = 20
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] debounce,       // ★FSR 디바운스 사이클 (PS 프로그래밍, spi_slave 0x49). 노이즈 자가발동 방지
    input  wire [3:0]  danger,         // 디지털 위험 입력(danger[0]=DIP)
    input  wire        arm_enable,
    input  wire        reached,         // 로봇 도달(pose_status_decode)
    // 설정 레지스터 (spi_slave_v4)
    input  wire [15:0] control, rule0, rule1, rule2, rule3,
    input  wire [15:0] thresh0, thresh1, thresh2, thresh3, xadc_val,
    input  wire [31:0] flinch_ticks,     // ★act3(움츠림) 1회성 지속 틱 (PS가 클럭상대로 설정)
    // 출력
    output wire        estop_active,
    output wire        pose_active,
    output wire        pose_mode,
    output wire        pose_freeze,        // ★act1(DIP)=현재포즈 정지: pose_gen 에 "델타0, 현재 그대로" 지시
    output reg  [2:0]  action_id,
    output reg         valid,
    output reg         fire,
    output wire        heartbeat
);
    // ── 규칙 풀기 ──
    wire [2:0] act  [0:3];
    wire       en   [0:3];
    wire [1:0] prio [0:3];
    wire       src  [0:3];
    wire [15:0] thr [0:3];
    assign act[0]=rule0[2:0]; assign en[0]=rule0[3]; assign prio[0]=rule0[5:4]; assign src[0]=rule0[6]; assign thr[0]=thresh0;
    assign act[1]=rule1[2:0]; assign en[1]=rule1[3]; assign prio[1]=rule1[5:4]; assign src[1]=rule1[6]; assign thr[1]=thresh1;
    assign act[2]=rule2[2:0]; assign en[2]=rule2[3]; assign prio[2]=rule2[5:4]; assign src[2]=rule2[6]; assign thr[2]=thresh2;
    assign act[3]=rule3[2:0]; assign en[3]=rule3[3]; assign prio[3]=rule3[5:4]; assign src[3]=rule3[6]; assign thr[3]=thresh3;

    // ── 위험 원천: 디지털 핀 또는 XADC 임계 비교 ──
    wire [3:0] danger_raw;
    assign danger_raw[0] = src[0] ? (xadc_val >= thr[0]) : danger[0];
    assign danger_raw[1] = src[1] ? (xadc_val >= thr[1]) : danger[1];
    assign danger_raw[2] = src[2] ? (xadc_val >= thr[2]) : danger[2];
    assign danger_raw[3] = src[3] ? (xadc_val >= thr[3]) : danger[3];

    // ── 2단 동기화 + 디바운스 ──
    wire [3:0] danger_stable;
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_db
            reg [1:0] sync;
            reg [15:0] cnt;
            reg        stable;
            always @(posedge clk) begin
                if (!rst_n) begin sync <= 0; cnt <= 0; stable <= 0; end
                else begin
                    sync <= {sync[0], danger_raw[gi]};
                    if (sync[1] == stable)        cnt <= 0;
                    else if (cnt >= debounce) begin stable <= sync[1]; cnt <= 0; end
                    else                          cnt <= cnt + 1'b1;
                end
            end
            assign danger_stable[gi] = stable;
        end
    endgenerate

    wire g_en = control[0] & arm_enable;
    wire [3:0] cand = { g_en & en[3] & danger_stable[3],
                        g_en & en[2] & danger_stable[2],
                        g_en & en[1] & danger_stable[1],
                        g_en & en[0] & danger_stable[0] };

    // ── 우선순위 중재 (높은 prio 우선, 동률은 낮은 index) ──
    reg [2:0] sel_action;
    reg [1:0] sel_prio;
    reg       sel_valid;
    always @* begin
        sel_action = 3'd0; sel_prio = 2'd0; sel_valid = 1'b0;
        if (cand[3] && (!sel_valid || prio[3] >= sel_prio)) begin sel_action=act[3]; sel_prio=prio[3]; sel_valid=1; end
        if (cand[2] && (!sel_valid || prio[2] >= sel_prio)) begin sel_action=act[2]; sel_prio=prio[2]; sel_valid=1; end
        if (cand[1] && (!sel_valid || prio[1] >= sel_prio)) begin sel_action=act[1]; sel_prio=prio[1]; sel_valid=1; end
        if (cand[0] && (!sel_valid || prio[0] >= sel_prio)) begin sel_action=act[0]; sel_prio=prio[0]; sel_valid=1; end
    end

    // ── 행동 분기 (★05_통합, 4종): estop(act1·즉시) / 덕포즈(act2·홈·레벨) /
    //     움츠림_덕포즈(act3·홈·1회성) / 움츠림_현재(act4·현재+델타·1회성) ──
    wire is_freeze_now = sel_valid & (sel_action == 3'd1);   // ★act1 = 현재포즈 정지(freeze). 구 estop(0x150)은 실Piper서 토크해제(처짐)라 폐기
    wire is_home_now   = sel_valid & (sel_action == 3'd2);   // ★덕포즈 (홈, 레벨/센서추종)
    wire is_hflinch    = sel_valid & (sel_action == 3'd3);   // ★움츠림_덕포즈 (홈으로 1회성)
    wire is_rflinch    = sel_valid & (sel_action == 3'd4);   // ★움츠림_현재 (현재+델타 1회성)
    wire oneshot_sel   = is_hflinch | is_rflinch;

    // ★act3/act4 1회성: FSR 상승엣지 → flinch_run(flinch_ticks 동안) → 자동해제 → 센서 내려야 재무장.
    //   flinch_recoil 로 모드 래치: act4(현재+델타) vs act3(홈). estop 우선.
    reg  oneshot_d, flinch_run, flinch_done, flinch_recoil;
    reg  [31:0] fcnt;
    always @(posedge clk) begin
        if (!rst_n) begin oneshot_d<=0; flinch_run<=0; flinch_done<=0; flinch_recoil<=0; fcnt<=0; end
        else begin
            oneshot_d <= oneshot_sel;
            if (is_freeze_now) flinch_run<=1'b0;                                                      // ★freeze(DIP, 최우선) → 진행중 flinch 취소
            else if (oneshot_sel & ~oneshot_d & ~flinch_done) begin flinch_run<=1'b1; fcnt<=0; flinch_recoil<=is_rflinch; end  // 엣지→시작+모드래치
            else if (flinch_run) begin
                if (fcnt >= flinch_ticks) begin flinch_run<=1'b0; flinch_done<=1'b1; end              // 만료 → 자동해제
                else fcnt <= fcnt + 1'b1;
            end
            if (~oneshot_sel) flinch_done<=1'b0;             // 센서 내리면 재무장
        end
    end

    assign estop_active = 1'b0;                              // ★0x150 비상정지 안 씀(실Piper=토크해제=처짐). act1 은 아래 freeze 로 정지
    assign pose_active  = is_home_now | flinch_run | is_freeze_now;     // ★freeze(현재포즈 정지) 도 포즈게이트 — 정상명령 막고 현재포즈 유지
    assign pose_mode    = (flinch_run & flinch_recoil) | is_freeze_now; // 1=현재기준(act4 움츠림_현재 / freeze), 0=홈(act2/act3)
    assign pose_freeze  = is_freeze_now;                     // ★freeze=현재포즈 그대로(델타0). pose_gen 이 cur_j 만 목표로

    // ── 텔레메트리 출력(등록) ──
    always @(posedge clk) begin
        if (!rst_n) begin valid<=0; fire<=0; action_id<=0; end
        else begin
            valid <= 1'b1;
            if (is_freeze_now)   begin fire<=1'b1; action_id<=3'd1; end   // ★action_id=1: DIP 현재포즈 정지(freeze)
            else if (flinch_run) begin fire<=1'b1; action_id<= flinch_recoil ? 3'd4 : 3'd3; end  // 움츠림_현재/움츠림_덕포즈
            else if (is_home_now)begin fire<=1'b1; action_id<=3'd2; end                          // 덕포즈(레벨)
            else                 begin fire<=sel_valid; action_id<= sel_valid ? sel_action : 3'd0; end
        end
    end

    // ── heartbeat ──
    reg [HB_BIT:0] hb;
    always @(posedge clk) if (!rst_n) hb <= 0; else hb <= hb + 1'b1;
    assign heartbeat = hb[HB_BIT];
endmodule

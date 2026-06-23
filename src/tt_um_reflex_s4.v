`timescale 1ns / 1ps
//============================================================================
// tt_um_reflex_s4 — 스텝4 칩 (★정상 패스스루 + estop + 홈 + 현재포즈 움츠림, 먹스)
//----------------------------------------------------------------------------
//  스텝1 패스스루+먹스 토대 + 스텝2/3(estop/홈) + ★현재포즈 움츠림(최종)★.
//   평상시: PC 정상명령 통과. 위험시: 먹스가 정상 끊고 반사 주입.
//   ★RX 경로(mcp_rx_recv, 중재기 c2): 로봇이 can0 에 브로드캐스트하는 현재포즈(0x2A5~7)·
//     도달(0x2A1 B4)을 칩이 읽음 → pose_status_decode → 현재포즈/reached.
//   움츠림(act3): reflex_pose_gen 이 발동순간 현재포즈 래치 + 델타(SPI recoil_d*) → 목표.
//   해제 = 센서뗌 + 도달(reached). DIP→estop, 소프트(cfg[9])→움츠림(rule1=0x1B).
//============================================================================
module tt_um_reflex_s4 #(
    parameter integer SEND_DIV    = 20000,    // ★1ms@20MHz (TT칩 20MHz, 05@50MHz는 50000). 클럭상대
    parameter integer PROBE_DIV   = 20000,    // ★되읽기 1ms@20MHz
    parameter integer RESET_DELAY = 200000    // ★MCP 발진안정 10ms@20MHz
)(
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    wire dip        = ui_in[0];     // estop 트리거 (DIP SW0)
    wire danger1    = ui_in[1];     // 움츠림 트리거 (PL서 소프트 cfg[9])
    wire mcp_int    = ui_in[2];
    wire m_miso     = ui_in[3];
    wire arm_enable = ui_in[7];

    wire        s_sclk = uio_in[0];
    wire        s_mosi = uio_in[1];
    wire        s_csn  = uio_in[2];
    wire        s_miso, s_miso_oe;
    wire [15:0] control, spi_div;
    wire [10:0] norm_id; wire [63:0] norm_data; wire norm_send;
    wire [15:0] rule0,rule1,rule2,rule3, thresh0,thresh1,thresh2,thresh3, xadc_val;
    wire [15:0] recoil_d1,recoil_d2,recoil_d3,recoil_d4,recoil_d5,recoil_d6;
    wire [15:0] flinch_lo, flinch_hi, reflex_speed, debounce;   // ★flinch=움츠림 1회성 틱. debounce=FSR 디바운스(0x49)

    wire [7:0]  p_canstat,p_canctrl,p_cnf1,p_cnf2,p_cnf3,p_eflg,p_tec,p_rec,p_canintf;

    wire        ic_req,ic_grant,ic_done,ic_active,init_done; wire [2:0] ic_op; wire [6:0] ic_addr; wire [7:0] ic_wdata,ic_wmask; wire [3:0] ic_step;
    wire        tx_req,tx_grant,tx_done_a,tx_active,tx_frame_done; wire [2:0] tx_op; wire [6:0] tx_addr; wire [7:0] tx_wdata,tx_wmask;
    wire        rxc_req,rxc_grant,rxc_done,rxc_active,rx_valid; wire [2:0] rxc_op; wire [6:0] rxc_addr; wire [7:0] rxc_wdata,rxc_wmask;
    wire [10:0] rx_id; wire [3:0] rx_dlc; wire [63:0] rx_data;
    wire        pb_req,pb_grant,pb_done,pb_active,probe_valid; wire [2:0] pb_op; wire [6:0] pb_addr; wire [7:0] pb_wdata,pb_wmask;
    wire        drv_req; wire [2:0] drv_op; wire [6:0] drv_addr; wire [7:0] drv_wdata,drv_wmask;
    wire [7:0]  arb_rdata,drv_rdata; wire drv_busy,drv_done;
    wire        m_sclk,m_mosi,m_csn;

    // ── 반사 코어/RX/디코드/포즈생성/송신원 ──
    wire        estop_active, pose_active, pose_mode, pose_freeze, valid, fire, heartbeat, reached;
    wire [2:0]  action_id;
    wire signed [31:0] cj1,cj2,cj3,cj4,cj5,cj6;
    wire [10:0] pg_id, rf_id; wire [3:0] pg_dlc, rf_dlc; wire [63:0] pg_data, rf_data;
    wire        pg_send, rf_send, gate_active;

    // ── 먹스 출력 ──
    wire [10:0] sel_id; wire [3:0] sel_dlc; wire [63:0] sel_data; wire sel_send;

    // status[3]=init_done [4]=gate_active [5]=pose_active [6]=pose_mode [7]=reached [10:8]=action_id
    wire [15:0] status = {5'd0, action_id, reached, pose_mode, pose_active, gate_active, init_done, 3'b000};

    reg init_started, init_start;
    always @(posedge clk) begin
        if (!rst_n) begin init_started<=0; init_start<=0; end
        else begin init_start<=1'b0; if (!init_started) begin init_start<=1'b1; init_started<=1'b1; end end
    end

    spi_slave_full #(.VERSION(16'h0542)) u_slv (    // 0x0542 = RX버그수정 + 움츠림 1회성타이머
        .clk(clk), .rst_n(rst_n), .sclk(s_sclk), .mosi(s_mosi), .csn(s_csn),
        .miso(s_miso), .miso_oe(s_miso_oe),
        .control(control), .spi_div(spi_div),
        .rule0(rule0), .rule1(rule1), .rule2(rule2), .rule3(rule3),
        .thresh0(thresh0), .thresh1(thresh1), .thresh2(thresh2), .thresh3(thresh3), .xadc_val(xadc_val),
        .recoil_d1(recoil_d1), .recoil_d2(recoil_d2), .recoil_d3(recoil_d3),
        .recoil_d4(recoil_d4), .recoil_d5(recoil_d5), .recoil_d6(recoil_d6),
        .flinch_lo(flinch_lo), .flinch_hi(flinch_hi), .reflex_speed(reflex_speed), .debounce(debounce),
        .norm_id(norm_id), .norm_data(norm_data), .norm_send(norm_send),
        .status(status),
        .canstat(p_canstat), .canctrl(p_canctrl), .cnf1(p_cnf1), .cnf2(p_cnf2), .cnf3(p_cnf3),
        .eflg(p_eflg), .tec(p_tec), .rec(p_rec), .canintf(p_canintf)
    );

    reflex_core_c u_core (
        .clk(clk), .rst_n(rst_n), .debounce(debounce),
        .danger({2'b00, danger1, dip}), .arm_enable(arm_enable), .reached(reached),
        .control(control), .rule0(rule0), .rule1(rule1), .rule2(rule2), .rule3(rule3),
        .thresh0(thresh0), .thresh1(thresh1), .thresh2(thresh2), .thresh3(thresh3), .xadc_val(xadc_val),
        .flinch_ticks({flinch_hi, flinch_lo}),    // ★움츠림 1회성 지속(틱)
        .estop_active(estop_active), .pose_active(pose_active), .pose_mode(pose_mode), .pose_freeze(pose_freeze),
        .action_id(action_id), .valid(valid), .fire(fire), .heartbeat(heartbeat)
    );

    pose_status_decode u_dec (
        .clk(clk), .rst_n(rst_n), .rx_valid(rx_valid), .rx_id(rx_id), .rx_data(rx_data),
        .reached(reached), .j1(cj1), .j2(cj2), .j3(cj3), .j4(cj4), .j5(cj5), .j6(cj6)
    );

    reflex_pose_gen #(.SEND_DIV(SEND_DIV)) u_pgen (
        .clk(clk), .rst_n(rst_n), .reflex_active(pose_active), .pose_mode(pose_mode), .freeze(pose_freeze),
        .reflex_speed(reflex_speed[7:0]),    // ★0x151 속도 주입
        .cur_j1(cj1), .cur_j2(cj2), .cur_j3(cj3), .cur_j4(cj4), .cur_j5(cj5), .cur_j6(cj6),
        .d_j1(recoil_d1), .d_j2(recoil_d2), .d_j3(recoil_d3),
        .d_j4(recoil_d4), .d_j5(recoil_d5), .d_j6(recoil_d6),
        .reflex_id(pg_id), .reflex_dlc(pg_dlc), .reflex_data(pg_data), .reflex_send(pg_send)
    );

    reflex_tx_src #(.SEND_DIV(SEND_DIV), .ESTOP_DATA(64'h0000_0000_0000_0001)) u_rsrc (
        .clk(clk), .rst_n(rst_n), .estop_active(estop_active), .pose_active(pose_active),
        .pose_id(pg_id), .pose_dlc(pg_dlc), .pose_data(pg_data), .pose_send(pg_send),
        .rid(rf_id), .rdlc(rf_dlc), .rdata(rf_data), .rsend(rf_send), .gate_active(gate_active)
    );

    // ★먹스: 평상시 정상명령, 위험시(gate_active) 반사(estop/홈/움츠림) 주입
    mcp_tx_mux u_mux (
        .reflex_active(gate_active),
        .normal_id(norm_id), .normal_dlc(4'd8), .normal_data(norm_data), .normal_send(norm_send),
        .reflex_id(rf_id), .reflex_dlc(rf_dlc), .reflex_data(rf_data), .reflex_send(rf_send),
        .sel_id(sel_id), .sel_dlc(sel_dlc), .sel_data(sel_data), .sel_send(sel_send)
    );

    mcp_init #(.RESET_DELAY(RESET_DELAY)) u_init (
        .clk(clk), .rst_n(rst_n), .start(init_start), .grant(ic_grant), .seq_active(ic_active),
        .req(ic_req), .op(ic_op), .addr(ic_addr), .wdata(ic_wdata), .wmask(ic_wmask),
        .rdata(arb_rdata), .busy(drv_busy), .done(ic_done), .init_done(init_done), .step(ic_step)
    );

    mcp_rx_recv u_rx (
        .clk(clk), .rst_n(rst_n), .mcp_int(mcp_int), .grant(rxc_grant), .seq_active(rxc_active),
        .req(rxc_req), .op(rxc_op), .addr(rxc_addr), .wdata(rxc_wdata), .wmask(rxc_wmask),
        .rdata(arb_rdata), .busy(drv_busy), .done(rxc_done),
        .rx_id(rx_id), .rx_dlc(rx_dlc), .rx_data(rx_data), .rx_valid(rx_valid)
    );

    mcp_tx_send u_tx (
        .clk(clk), .rst_n(rst_n),
        .send(sel_send & init_done), .id(sel_id), .dlc(sel_dlc), .data(sel_data),
        .grant(tx_grant), .seq_active(tx_active),
        .req(tx_req), .op(tx_op), .addr(tx_addr), .wdata(tx_wdata), .wmask(tx_wmask),
        .rdata(arb_rdata), .busy(drv_busy), .done(tx_done_a), .tx_done(tx_frame_done)
    );

    mcp_probe #(.PROBE_DIV(PROBE_DIV)) u_probe (
        .clk(clk), .rst_n(rst_n), .init_done(init_done), .grant(pb_grant), .seq_active(pb_active),
        .req(pb_req), .op(pb_op), .addr(pb_addr), .wdata(pb_wdata), .wmask(pb_wmask),
        .rdata(arb_rdata), .busy(drv_busy), .done(pb_done),
        .canstat(p_canstat), .canctrl(p_canctrl), .cnf1(p_cnf1), .cnf2(p_cnf2), .cnf3(p_cnf3),
        .eflg(p_eflg), .tec(p_tec), .rec(p_rec), .canintf(p_canintf), .probe_valid(probe_valid)
    );

    mcp_arb4 u_arb (
        .clk(clk), .rst_n(rst_n),
        .c0_active(ic_active), .c0_req(ic_req), .c0_op(ic_op), .c0_addr(ic_addr), .c0_wdata(ic_wdata), .c0_wmask(ic_wmask), .c0_grant(ic_grant), .c0_done(ic_done),
        .c1_active(tx_active), .c1_req(tx_req), .c1_op(tx_op), .c1_addr(tx_addr), .c1_wdata(tx_wdata), .c1_wmask(tx_wmask), .c1_grant(tx_grant), .c1_done(tx_done_a),
        .c2_active(rxc_active), .c2_req(rxc_req), .c2_op(rxc_op), .c2_addr(rxc_addr), .c2_wdata(rxc_wdata), .c2_wmask(rxc_wmask), .c2_grant(rxc_grant), .c2_done(rxc_done),
        .c3_active(pb_active), .c3_req(pb_req), .c3_op(pb_op), .c3_addr(pb_addr), .c3_wdata(pb_wdata), .c3_wmask(pb_wmask), .c3_grant(pb_grant), .c3_done(pb_done),
        .rdata(arb_rdata),
        .req(drv_req), .op(drv_op), .addr(drv_addr), .wdata(drv_wdata), .wmask(drv_wmask),
        .drv_rdata(drv_rdata), .drv_busy(drv_busy), .drv_done(drv_done)
    );

    spi_master_mcp_v2 u_drv (
        .clk(clk), .rst_n(rst_n), .half_div(spi_div[7:0]),
        .req(drv_req), .op(drv_op), .addr(drv_addr), .wdata(drv_wdata), .wmask(drv_wmask),
        .rdata(drv_rdata), .busy(drv_busy), .done(drv_done),
        .m_sclk(m_sclk), .m_mosi(m_mosi), .m_csn(m_csn), .m_miso(m_miso)
    );

    wire mcp_rst = 1'b1;
    assign uo_out  = {heartbeat, mcp_rst, gate_active, action_id, fire, valid};
    assign uio_out = {1'b0, m_csn, m_mosi, m_sclk, s_miso, 3'b000};
    assign uio_oe  = {1'b0, 1'b1, 1'b1, 1'b1, s_miso_oe, 3'b000};

    wire _unused = &{1'b0, ena, ui_in[6:4], uio_in[7:3], ic_step, tx_frame_done, probe_valid, drv_req, sel_dlc,
                     pg_dlc, rx_dlc, cj4, cj5, cj6};
endmodule

`timescale 1ns / 1ps
//============================================================================
// spi_slave_full — 모든 스텝 공용 칩 SPI 슬레이브 (설정 + 정상프레임 + 반사설정 + 되읽기)
//----------------------------------------------------------------------------
//  ★C단계 핵심: 칩이 CAN 유일 송신자라 ★정상명령도 칩을 통과★(패스스루)해야 한다.
//  그래서 슬레이브에 ★정상 프레임 레지스터(0x50~0x55)★ 를 둔다 — PL 이 PS(이더넷)에서 받은
//  CAN 프레임을 여기 써넣고 0x55 에 쓰면 norm_send 1클럭 펄스가 나가 칩이 MCP 로 중계한다.
//  반사(rule/thresh/recoil)와 되읽기(관측)는 그대로. 04 v4 검증 타이밍(2단동기+엣지검출).
//
//  레지스터 맵(16비트):
//   0x00 ID_MAGIC(R)=0xCAFD  0x01 VERSION(R)  0x02 CONTROL(R/W) bit0=enable
//   0x03 SPI_DIV(R/W) [7:0]=칩→MCP SCLK 반주기
//   0x10~0x13 RULE0~3(R/W)   0x18~0x1B THRESH0~3(R/W)   0x30 XADC_VAL(R/W)
//   0x40~0x45 RECOIL_DELTA_J1~6(R/W)
//   ★0x46 FLINCH_LO, 0x47 FLINCH_HI (R/W) = 움찔(act3) 1회성 지속시간(클럭 틱, 32비트). PS가 설정(클럭 상대)
//   ★0x48 REFLEX_SPEED (R/W) = 반사(덕포즈/움찔)가 주입하는 0x151 속도율(1~100). 실로봇 move_spd_rate.
//   ★0x50 NORM_ID(R/W)[10:0]  0x51 {D1,D0}  0x52 {D3,D2}  0x53 {D5,D4}  0x54 {D7,D6}
//   ★0x55 NORM_SEND(W) — 쓰면 norm_send 1클럭 펄스(정상 프레임 송신 트리거)
//   0x20 STATUS(R)  0x21~0x28 MCP 되읽기(R)
//============================================================================
module spi_slave_full #(
    parameter [15:0] ID_MAGIC = 16'hCAFD,
    parameter [15:0] VERSION  = 16'h0510
)(
    input  wire clk,
    input  wire rst_n,
    input  wire sclk,
    input  wire mosi,
    input  wire csn,
    output reg  miso,
    output wire miso_oe,
    // 설정 출력
    output reg  [15:0] control,
    output reg  [15:0] spi_div,
    output reg  [15:0] rule0, rule1, rule2, rule3,
    output reg  [15:0] thresh0, thresh1, thresh2, thresh3,
    output reg  [15:0] xadc_val,
    output wire [15:0] recoil_d1, recoil_d2, recoil_d3, recoil_d4, recoil_d5, recoil_d6,  // ★10: 상수 고정
    output reg  [15:0] flinch_lo, flinch_hi,   // ★움찔(act3) 1회성 지속 틱 (32비트={hi,lo}) — 프로그래밍 유지
    output wire [15:0] reflex_speed,           // ★10: 상수 고정 (100)
    output wire [15:0] debounce,               // ★10: 상수 고정 (DEBOUNCE_MS 2 → 40000)
    // ★정상 프레임 (PL 이 PS 명령을 여기 적재)
    output reg  [10:0] norm_id,
    output wire [63:0] norm_data,
    output reg         norm_send,            // 1클럭 펄스
    // 텔레메트리/관측 입력
    input  wire [15:0] status,
    input  wire [7:0]  canstat, canctrl, cnf1, cnf2, cnf3, eflg, tec, rec, canintf
);
    reg [15:0] norm_d10, norm_d32, norm_d54, norm_d76;
    assign norm_data = {norm_d76, norm_d54, norm_d32, norm_d10};   // D0=norm_d10[7:0]

    // ★10: options.config 의 정착된 튜닝값 3종을 상수 고정 (나머지 설정은 06 그대로 프로그래밍 유지).
    //   recoil(J5만 0.30rad), reflex_speed(100), debounce(2ms). PACE_US는 PS측이라 칩에 없음.
    assign recoil_d1 = 16'h0000;
    assign recoil_d2 = 16'h0000;
    assign recoil_d3 = 16'h0000;
    assign recoil_d4 = 16'h0000;
    assign recoil_d5 = 16'h4325;     // RECOIL_RAD 0.30 rad → 17189 (0.001도)
    assign recoil_d6 = 16'h0000;
    assign reflex_speed = 16'h0064;  // 100
    assign debounce     = 16'h9C40;  // DEBOUNCE_MS 2 × 20MHz = 40,000

    reg [1:0] sclk_s, mosi_s, csn_s;
    reg       sclk_d;
    always @(posedge clk) begin
        if (!rst_n) begin sclk_s<=0; mosi_s<=0; csn_s<=2'b11; sclk_d<=0; end
        else begin
            sclk_s <= {sclk_s[0], sclk}; mosi_s <= {mosi_s[0], mosi};
            csn_s  <= {csn_s[0],  csn};  sclk_d <= sclk_s[1];
        end
    end
    wire sclk_rise = sclk_s[1] & ~sclk_d;
    wire sclk_fall = ~sclk_s[1] & sclk_d;
    wire csn_active = ~csn_s[1];
    assign miso_oe = csn_active;

    reg [22:0] rx; reg [4:0] bitcnt; reg rw; reg [6:0] addr; reg [15:0] tx; reg sending;
    wire [7:0]  cmd_now  = {rx[6:0],  mosi_s[1]};
    wire [15:0] data_now = {rx[14:0], mosi_s[1]};

    always @(posedge clk) begin
        if (!rst_n) begin
            rx<=0; bitcnt<=0; rw<=0; addr<=0; tx<=0; sending<=0; miso<=0; norm_send<=0;
            control <= 16'h0001; spi_div <= 16'h0004;
            rule0 <= 16'h0039; rule1 <= 16'h001C; rule2 <= 16'h0000; rule3 <= 16'h0000;
            thresh0<=0; thresh1<=0; thresh2<=0; thresh3<=0; xadc_val<=0;
            flinch_lo<=16'h9680; flinch_hi<=16'h0098;   // ★기본 10,000,000틱 = 0.2s@50MHz (PS가 덮어씀) — 프로그래밍 유지
            // ★10: recoil_d*/reflex_speed/debounce 는 상수 고정이라 리셋·쓰기 없음
            norm_id<=0; norm_d10<=0; norm_d32<=0; norm_d54<=0; norm_d76<=0;
        end else begin
            norm_send <= 1'b0;                        // ★기본: 매 클럭 클리어(1클럭 펄스용)
            if (!csn_active) begin
                bitcnt<=0; sending<=0;
            end else begin
                if (sclk_rise) begin
                    rx <= {rx[21:0], mosi_s[1]}; bitcnt <= bitcnt + 1'b1;
                    if (bitcnt == 5'd7) begin
                        rw <= cmd_now[7]; addr <= cmd_now[6:0];
                        if (cmd_now[7]) begin
                            sending <= 1'b1;
                            case (cmd_now[6:0])
                                7'h00: tx <= ID_MAGIC;   7'h01: tx <= VERSION;  7'h02: tx <= control;  7'h03: tx <= spi_div;
                                7'h10: tx <= rule0;      7'h11: tx <= rule1;    7'h12: tx <= rule2;     7'h13: tx <= rule3;
                                7'h18: tx <= thresh0;    7'h19: tx <= thresh1;  7'h1A: tx <= thresh2;   7'h1B: tx <= thresh3;
                                7'h30: tx <= xadc_val;   7'h20: tx <= status;
                                7'h21: tx <= {8'h00,canstat}; 7'h22: tx <= {8'h00,canctrl};
                                7'h23: tx <= {8'h00,cnf1};    7'h24: tx <= {8'h00,cnf2}; 7'h25: tx <= {8'h00,cnf3};
                                7'h26: tx <= {8'h00,eflg};    7'h27: tx <= {tec,rec};    7'h28: tx <= {8'h00,canintf};
                                7'h40: tx <= recoil_d1;  7'h41: tx <= recoil_d2; 7'h42: tx <= recoil_d3;
                                7'h43: tx <= recoil_d4;  7'h44: tx <= recoil_d5; 7'h45: tx <= recoil_d6;
                                7'h46: tx <= flinch_lo;  7'h47: tx <= flinch_hi;  7'h48: tx <= reflex_speed;  7'h49: tx <= debounce;
                                7'h50: tx <= {5'b0,norm_id}; 7'h51: tx <= norm_d10; 7'h52: tx <= norm_d32;
                                7'h53: tx <= norm_d54;       7'h54: tx <= norm_d76;
                                default: tx <= 16'h0000;
                            endcase
                        end
                    end
                    if (bitcnt == 5'd23) begin
                        if (!rw) case (addr)
                            7'h02: control <= data_now;  7'h03: spi_div <= data_now;
                            7'h10: rule0 <= data_now;    7'h11: rule1 <= data_now;  7'h12: rule2 <= data_now;  7'h13: rule3 <= data_now;
                            7'h18: thresh0<=data_now;    7'h19: thresh1<=data_now;  7'h1A: thresh2<=data_now;  7'h1B: thresh3<=data_now;
                            7'h30: xadc_val<=data_now;
                            // ★10: recoil(0x40~45)·speed(0x48)·debounce(0x49) 쓰기 제거 (상수 고정)
                            7'h46: flinch_lo<=data_now;  7'h47: flinch_hi<=data_now;   // flinch 는 프로그래밍 유지
                            7'h50: norm_id <= data_now[10:0];
                            7'h51: norm_d10<= data_now;  7'h52: norm_d32<=data_now;  7'h53: norm_d54<=data_now;  7'h54: norm_d76<=data_now;
                            7'h55: norm_send <= 1'b1;    // ★정상 프레임 송신 트리거(1클럭 펄스)
                            default: ;
                        endcase
                    end
                end
                if (sclk_fall && sending) begin miso <= tx[15]; tx <= {tx[14:0], 1'b0}; end
            end
        end
    end
endmodule

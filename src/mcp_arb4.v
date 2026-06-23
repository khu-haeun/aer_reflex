`timescale 1ns / 1ps
//============================================================================
// mcp_arb4 — SPI 드라이버 1개를 4 클라이언트가 ★시퀀스 단위★로 나눠 쓰는 중재기
//----------------------------------------------------------------------------
//  04 의 3클라이언트 중재기를 4개로 확장(되읽기 probe 추가). 우선순위(높은→낮은):
//    c0 = 초기화(mcp_init)   — 부팅 1회 점유
//    c1 = 송신(tx)           — ★안전 출력(e-stop/포즈) 최우선
//    c2 = 수신(rx)           — 피드백 수신(스텝4; 그전엔 비활성)
//    c3 = 되읽기(probe)      — 관측, ★최저: 안전 송신을 절대 안 늦춤
//  핸드셰이크: 각 클라이언트가 seq_active(시퀀스 동안 1)로 점유 요청, grant 받은 뒤에만
//  드라이버에 트랜잭션 발행. 임자 없을 때 우선순위로 하나 골라 grant, seq_active 내려갈
//  때까지 유지(=한 트랜잭션 묶음 독점). rdata 는 공유, done 은 임자에게만.
//============================================================================
module mcp_arb4 (
    input  wire        clk,
    input  wire        rst_n,
    // c0 = 초기화 (최고 우선순위)
    input  wire c0_active, input wire c0_req, input wire [2:0] c0_op, input wire [6:0] c0_addr,
    input  wire [7:0] c0_wdata, input wire [7:0] c0_wmask, output wire c0_grant, output wire c0_done,
    // c1 = 송신
    input  wire c1_active, input wire c1_req, input wire [2:0] c1_op, input wire [6:0] c1_addr,
    input  wire [7:0] c1_wdata, input wire [7:0] c1_wmask, output wire c1_grant, output wire c1_done,
    // c2 = 수신
    input  wire c2_active, input wire c2_req, input wire [2:0] c2_op, input wire [6:0] c2_addr,
    input  wire [7:0] c2_wdata, input wire [7:0] c2_wmask, output wire c2_grant, output wire c2_done,
    // c3 = 되읽기(probe, 최저)
    input  wire c3_active, input wire c3_req, input wire [2:0] c3_op, input wire [6:0] c3_addr,
    input  wire [7:0] c3_wdata, input wire [7:0] c3_wmask, output wire c3_grant, output wire c3_done,
    // 공유 결과
    output wire [7:0]  rdata,
    // 드라이버 쪽
    output wire        req,
    output wire [2:0]  op,
    output wire [6:0]  addr,
    output wire [7:0]  wdata,
    output wire [7:0]  wmask,
    input  wire [7:0]  drv_rdata,
    input  wire        drv_busy,
    input  wire        drv_done
);
    reg [1:0] owner;
    reg       has;

    always @(posedge clk) begin
        if (!rst_n) begin owner<=2'd0; has<=1'b0; end
        else if (!has) begin
            if      (c0_active) begin owner<=2'd0; has<=1'b1; end   // 우선순위 init
            else if (c1_active) begin owner<=2'd1; has<=1'b1; end   //          > tx
            else if (c2_active) begin owner<=2'd2; has<=1'b1; end   //          > rx
            else if (c3_active) begin owner<=2'd3; has<=1'b1; end   //          > probe
        end else begin
            if      (owner==2'd0 && !c0_active) has<=1'b0;
            else if (owner==2'd1 && !c1_active) has<=1'b0;
            else if (owner==2'd2 && !c2_active) has<=1'b0;
            else if (owner==2'd3 && !c3_active) has<=1'b0;
        end
    end

    assign c0_grant = has && (owner==2'd0);
    assign c1_grant = has && (owner==2'd1);
    assign c2_grant = has && (owner==2'd2);
    assign c3_grant = has && (owner==2'd3);

    assign req   = (c0_grant & c0_req) | (c1_grant & c1_req) | (c2_grant & c2_req) | (c3_grant & c3_req);
    assign op    = c0_grant ? c0_op    : c1_grant ? c1_op    : c2_grant ? c2_op    : c3_grant ? c3_op    : 3'd0;
    assign addr  = c0_grant ? c0_addr  : c1_grant ? c1_addr  : c2_grant ? c2_addr  : c3_grant ? c3_addr  : 7'd0;
    assign wdata = c0_grant ? c0_wdata : c1_grant ? c1_wdata : c2_grant ? c2_wdata : c3_grant ? c3_wdata : 8'd0;
    assign wmask = c0_grant ? c0_wmask : c1_grant ? c1_wmask : c2_grant ? c2_wmask : c3_grant ? c3_wmask : 8'd0;

    assign c0_done = c0_grant & drv_done;
    assign c1_done = c1_grant & drv_done;
    assign c2_done = c2_grant & drv_done;
    assign c3_done = c3_grant & drv_done;
    assign rdata   = drv_rdata;
endmodule

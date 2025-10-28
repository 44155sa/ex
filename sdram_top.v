`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: anlgoic
// Author: 	xg
// description:  sdram top wrapper (merged / adapted to camera-standard engine usage)
// This version accepts both SYS_CLK and ext_mem_clk so it matches the top-level
// instantiation in your camera project (which passes both clk_50m and ext_mem_clk).
//
// Key points:
//  - Keeps the existing app_wrrd + sdr_as_ram + PHY instantiation structure,
//    but exposes/accepts ext_mem_clk so frame_read_write (mem domain) and
//    the SDRAM controller use the same mem clock from top.
//  - Preserves Sdr_init_done, wr_done, Sdr_rd_en, Sdr_rd_dout, and udp_wrusedw signals
//    to match the rest of your design and the camera standard engine wiring.
//  - Use SYS_CLK as reference clock for PLL that produces Clk_sft used by sdr_as_ram.
//    ext_mem_clk is passed into the app_wrrd and sdr_as_ram as Sdr_clk (mem domain).
//////////////////////////////////////////////////////////////////////////////////

`define DEBUG

`include "../include/global_def.v"

module sdram_top(
    input                                    SYS_CLK,          // board/system reference clock (e.g. 50MHz)
    input                                    ext_mem_clk,      // external memory clock (from top / pll)
    input                                    rst_n,
    input                                    sd_clk,           // preserved for compatibility (unused in many flows)
   // output                                   LED,
    output                                   Sdr_init_done,
    output                                   wr_done,
    input                                    sdr_data_valid,
    input              [  23: 0]             sdr_data,
    output                                   Sdr_rd_en,
    output             [`DATA_WIDTH-1: 0]    Sdr_rd_dout,
    input                                    full_flag,
    output                                   full_flag_sdr,
    input           [11:0]                   udp_wrusedw,
    output                                   SDRAM_CLK,
    output                                   SDR_CKE,
    output                                   SDR_RAS,
    output                                   SDR_CAS,
    output                                   SDR_WE,
    output             [`BA_WIDTH-1: 0]      SDR_BA,
    output             [`ROW_WIDTH-1:0]      SDR_ADDR,
    output             [`DM_WIDTH-1:0]       SDR_DM,
    output                                   video_clk,
    inout              [`DATA_WIDTH-1:0]     SDR_DQ,
    input                                    video_read_req,       // request from top/frame consumer
    output                                   video_read_req_ack,   // ack back to requester
    output                                   video_read_en,        // data valid out (to top)
    output     [31:0]                        video_read_data,      // read data out (32-bit)

    // camera / frame write interface (exposed to top)
    input                                    cam_pclk,             // camera pixel clock (from top)
    input                                    cam_write_req,        // frame write request (from camera pipeline)
    output                                   cam_write_req_ack,    // ack to camera pipeline
    input                                    cam_write_en,         // per-word write enable (from camera pipeline)
    input      [31:0]                        cam_write_data    
    );

    // Internal signals
    wire                                lock;
    wire                                local_clk;
    wire                                Clk_sft;
    wire                                Rst_n;
    wire                                Rst /*synthesis syn_keep=1 */;
    wire                                Sdr_init_ref_vld;
    wire                                Sdr_busy;

    wire        App_wr_en_app;
wire [ `ADDR_WIDTH-1:0 ] App_wr_addr_app;
wire [ `DM_WIDTH-1:0 ]   App_wr_dm_app;
wire [ `DATA_WIDTH-1:0 ] App_wr_din_app;

wire        App_rd_en_app;
wire [ `ADDR_WIDTH-1:0 ] App_rd_addr_app;

// frame_read_write (或 frame_fifo_write) 的本地信号 (来源 2)
wire        App_wr_en_frame;
wire [ `ADDR_WIDTH-1:0 ] App_wr_addr_frame;
wire [ `DM_WIDTH-1:0 ]   App_wr_dm_frame;
wire [ `DATA_WIDTH-1:0 ] App_wr_din_frame;

wire        App_rd_en_frame;
wire [ `ADDR_WIDTH-1:0 ] App_rd_addr_frame;

// 仲裁输出（单一路由到 sdr_as_ram）
reg         App_wr_en_mux;
reg [ `ADDR_WIDTH-1:0 ] App_wr_addr_mux;
reg [ `DM_WIDTH-1:0 ]   App_wr_dm_mux;
reg [ `DATA_WIDTH-1:0 ] App_wr_din_mux;

reg         App_rd_en_mux;
reg [ `ADDR_WIDTH-1:0 ] App_rd_addr_mux;

    // ------------------------------------------------------------------
    // PLL: derive any shifted clocks required by vendor sdr_as_ram instance
    // SYS_CLK is used as PLL reference to generate Clk_sft used by sdr_as_ram.
    // ext_mem_clk (provided by top) is used as Sdr_clk (memory domain).
    // ------------------------------------------------------------------
    clk_pll u0_clk(
        .refclk    (SYS_CLK),
        .reset     (1'b0),
        .extlock   (lock),
        .clk0_out  (local_clk),    // not used directly, kept for compatibility
        .clk2_out  (Clk_sft),      // shifted clock used by sdr_as_ram
        .clk3_out  (video_clk)              // video_clk (unused here)
    );

    // Combine reset with PLL lock to avoid releases before PLL stable
    assign Rst_n = rst_n & lock;
    assign Rst   = ~Rst_n;
// Capture frame module requests for arbitration
assign frame_write_req = cam_write_req;  // frame_read_write exposes write_req; use cam_write_req from top connection
assign frame_read_req  = video_read_req; // frame_read_write exposes read_req -> video_read_req

    // ------------------------------------------------------------------
    // app_wrrd: converts incoming sdr_data (from e.g. SD card or camera writer)
    // into the sdram application interface (App_wr_*/App_rd_*). It also
    // provides read data path Sdr_rd_dout and read enable Sdr_rd_en.
    // Keep clk domain for app_wrrd on ext_mem_clk (mem domain).
    // sd_clk is passed to app_wrrd for any SD side timing (preserved).
    // ------------------------------------------------------------------
    app_wrrd u1_app_wrrd(
        .clk                (ext_mem_clk),          // mem domain clock
    .rst_n              (Rst_n),
    .sd_clk             (sd_clk),
    .Sdr_init_done      (Sdr_init_done),
    .Sdr_init_ref_vld   (Sdr_init_ref_vld),
    .full_flag_net      (full_flag),
    .sdr_data_valid     (sdr_data_valid),
    .sdr_data           (sdr_data),
    // app interface (rename to *_app)
    .App_wr_en          (App_wr_en_app),
    .App_wr_addr        (App_wr_addr_app),
    .App_wr_dm          (App_wr_dm_app),
    .App_wr_din         (App_wr_din_app),
    .wr_done            (wr_done),
    .App_rd_en          (App_rd_en_app),
    .App_rd_addr        (App_rd_addr_app),
    .Sdr_rd_en          (Sdr_rd_en),        // Sdr_rd_en will be driven by sdr_as_ram and routed to both modules
    .Sdr_rd_dout        (Sdr_rd_dout),
    .Sdr_busy           (Sdr_busy),
    .full_flag          (full_flag_sdr),
    .udp_wrusedw        (udp_wrusedw)
    );
    frame_read_write frame_read_write_m0(
     .rst                    (~rst_n),
    .mem_clk                (ext_mem_clk),
    .Sdr_init_done          (Sdr_init_done),
    .Sdr_init_ref_vld       (Sdr_init_ref_vld),
    .Sdr_busy               (Sdr_busy),

    // frame_read_write 的 read-side (它会产生 App_rd_en/addr)
    .App_rd_en              (App_rd_en_frame),
    .App_rd_addr            (App_rd_addr_frame),

    // connect SDR read path into frame_read_write
    .Sdr_rd_en              (Sdr_rd_en),
    .Sdr_rd_dout            (Sdr_rd_dout),

    // read clk/domain to frame_read_write (video domain)
    .read_clk               (video_clk),
    .read_req               (video_read_req),
    .read_req_ack           (video_read_req_ack),
    .read_finish            (),
    .read_addr_0            (24'd0),
    .read_addr_1            (24'd0),
    .read_addr_2            (24'd0),
    .read_addr_3            (24'd0),
    .read_addr_index        (2'd0),
    .read_len               (24'd786432),
    .read_en                (video_read_en),
    .read_data              (video_read_data),

    // write side from frame pipeline (camera)
    .App_wr_en              (App_wr_en_frame),
    .App_wr_addr            (App_wr_addr_frame),
    .App_wr_din             (App_wr_din_frame),
    .App_wr_dm              (App_wr_dm_frame),

    .write_clk              (cam_pclk),
    .write_req              (cam_write_req),
    .write_req_ack          (cam_write_req_ack),
    .write_finish           (),
    .write_addr_0           (24'd0),
    .write_addr_1           (24'd0),
    .write_addr_2           (24'd0),
    .write_addr_3           (24'd0),
    .write_addr_index       (2'd0),
    .write_len              (24'd786432),
    .write_en               (cam_write_en),
    .write_data             (cam_write_data)
 );
 always @(*) begin
    // write path arbitration
    if (frame_write_req) begin
        App_wr_en_mux  = App_wr_en_frame;
        App_wr_addr_mux = App_wr_addr_frame;
        App_wr_dm_mux   = App_wr_dm_frame;
        App_wr_din_mux  = App_wr_din_frame;
    end else begin
        App_wr_en_mux  = App_wr_en_app;
        App_wr_addr_mux = App_wr_addr_app;
        App_wr_dm_mux   = App_wr_dm_app;
        App_wr_din_mux  = App_wr_din_app;
    end

    // read path arbitration
    if (frame_read_req) begin
        App_rd_en_mux  = App_rd_en_frame;
        App_rd_addr_mux = App_rd_addr_frame;
    end else begin
        App_rd_en_mux  = App_rd_en_app;
        App_rd_addr_mux = App_rd_addr_app;
    end
end
    // ------------------------------------------------------------------
    // sdr_as_ram instance: vendor / project provided SDRAM controller wrapper
    // Uses ext_mem_clk as Sdr_clk and Clk_sft from PLL for the "sft" clock path.
    // ------------------------------------------------------------------
    sdr_as_ram #(.self_refresh_open(1'b1))
    u2_ram(
        .Sdr_clk            (ext_mem_clk),
        .Sdr_clk_sft        (Clk_sft),
        .Rst                (!Rst_n),

        .Sdr_init_done      (Sdr_init_done),
        .Sdr_init_ref_vld   (Sdr_init_ref_vld),
        .Sdr_busy           (Sdr_busy),

        .App_ref_req        (1'b0),

        .App_wr_en          (App_wr_en_mux),
        .App_wr_addr        (App_wr_addr_mux),
        .App_wr_dm          (App_wr_dm_mux),
        .App_wr_din         (App_wr_din_mux),

        .App_rd_en          (App_rd_en_mux),
        .App_rd_addr        (App_rd_addr_mux),
        .Sdr_rd_en          (Sdr_rd_en),
        .Sdr_rd_dout        (Sdr_rd_dout),

        .SDRAM_CLK          (SDRAM_CLK),
        .SDR_RAS            (SDR_RAS),
        .SDR_CAS            (SDR_CAS),
        .SDR_WE             (SDR_WE),
        .SDR_BA             (SDR_BA),
        .SDR_ADDR           (SDR_ADDR),
        .SDR_DM             (SDR_DM),
        .SDR_DQ             (SDR_DQ)
    );

    // Keep SDRAM CKE enabled
    assign SDR_CKE = 1'b1;

`ifndef SIMULATION
    EG_PHY_SDRAM_2M_32 sdram_phy (
        .clk    (SDRAM_CLK),
        .ras_n  (SDR_RAS),
        .cas_n  (SDR_CAS),
        .we_n   (SDR_WE),
        .addr   (SDR_ADDR[10:0]),
        .ba     (SDR_BA),
        .dq     (SDR_DQ),
        .cs_n   (1'b0),
        .dm0    (SDR_DM[0]),
        .dm1    (SDR_DM[1]),
        .dm2    (SDR_DM[2]),
        .dm3    (SDR_DM[3]),
        .cke    (1'b1)
    );
`endif

endmodule
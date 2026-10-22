//------------------------------------------------------------------------------
// File       : yolo11n_accelerator_top.sv
// Module     : yolo11n_accelerator_top
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Full 7-Module Autonomous Hardware Accelerator Top-Level.
//              Modules 1-5 (Datapath) + Module 6 (Command Dispatcher) +
//              Module 7 (Sync Barrier Unit).
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module yolo11n_accelerator_top #(
    parameter int AXI_ADDR_WIDTH   = 32,
    parameter int AXI_DATA_WIDTH   = 64,
    parameter int NUM_COLS         = 8,
    parameter int WIN_TAPS         = 9
)(
    input  wire                        clk,
    input  wire                        rst,
    output wire                        accel_irq,

    //--------------------------------------------------------------------------
    // AXI4-Lite Slave Control Interface (From ARM CPU)
    //--------------------------------------------------------------------------
    input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire                        s_axi_awvalid,
    output wire                        s_axi_awready,
    input  wire [31:0]                 s_axi_wdata,
    input  wire [3:0]                  s_axi_wstrb,
    input  wire                        s_axi_wvalid,
    output wire                        s_axi_wready,
    output wire [1:0]                  s_axi_bresp,
    output wire                        s_axi_bvalid,
    input  wire                        s_axi_bready,
    input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire                        s_axi_arvalid,
    output wire                        s_axi_arready,
    output wire [31:0]                 s_axi_rdata,
    output wire [1:0]                  s_axi_rresp,
    output wire                        s_axi_rvalid,
    input  wire                        s_axi_rready,

    //--------------------------------------------------------------------------
    // AXI4-HP Master Bus Interface (To DDR3 Memory Controller)
    //--------------------------------------------------------------------------
    output wire [AXI_ADDR_WIDTH-1:0]   m_axi_araddr,
    output wire [7:0]                  m_axi_arlen,
    output wire [2:0]                  m_axi_arsize,
    output wire [1:0]                  m_axi_arburst,
    output wire                        m_axi_arvalid,
    input  wire                        m_axi_arready,
    input  wire [AXI_DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]                  m_axi_rresp,
    input  wire                        m_axi_rlast,
    input  wire                        m_axi_rvalid,
    output wire                        m_axi_rready,

    output wire [AXI_ADDR_WIDTH-1:0]   m_axi_awaddr,
    output wire [7:0]                  m_axi_awlen,
    output wire [2:0]                  m_axi_awsize,
    output wire [1:0]                  m_axi_awburst,
    output wire                        m_axi_awvalid,
    input  wire                        m_axi_awready,
    output wire [AXI_DATA_WIDTH-1:0]   m_axi_wdata,
    output wire [7:0]                  m_axi_wstrb,
    output wire                        m_axi_wlast,
    output wire                        m_axi_wvalid,
    input  wire                        m_axi_wready,
    input  wire [1:0]                  m_axi_bresp,
    input  wire                        m_axi_bvalid,
    output wire                        m_axi_bready,

    // Weight Preload Side-Channel (For Testbench Preloading)
    input  wire [NUM_COLS-1:0]         weight_wr_en,
    input  wire [4:0]                  weight_wr_addr,
    input  wire signed [17:0][7:0]     weight_wr_data
);

    // Configuration Wires from Module 6
    wire                               cfg_layer_start;
    wire [31:0]                        cfg_dma_src;
    wire [31:0]                        cfg_dma_dst;
    wire [15:0]                        cfg_dma_beats;
    wire [18:0]                        cfg_plane_stride;
    wire [9:0]                         cfg_h_in, cfg_w_in, cfg_h_out, cfg_w_out, cfg_c_in;
    wire [1:0]                         cfg_stride, cfg_pad;
    wire signed [NUM_COLS-1:0][31:0]   cfg_bias;
    wire signed [NUM_COLS-1:0][15:0]   cfg_mult;
    wire [NUM_COLS-1:0][4:0]           cfg_shift;
    wire signed [NUM_COLS-1:0][7:0]    cfg_zp;

    // Barrier Control Wires from Module 7
    wire                               compute_buf_sel;
    wire                               dma_buf_sel;
    wire                               comp_start;
    wire                               dma_rd_start;
    wire                               layer_done;

    // Status Tokens from Datapath
    wire                               dma_rd_done;
    wire                               comp_done;
    wire                               dma_wr_done;
    wire                               comp_busy;
    wire                               dma_rd_busy;
    wire                               dma_wr_busy;

    //--------------------------------------------------------------------------
    // Instantiate Module 6: Instruction Dispatcher
    //--------------------------------------------------------------------------
    instruction_dispatcher #(
        .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH (32),
        .NUM_COLS       (NUM_COLS)
    ) u_module6_dispatcher (
        .clk                   (clk),
        .rst                   (rst),
        .s_axi_awaddr          (s_axi_awaddr),
        .s_axi_awvalid         (s_axi_awvalid),
        .s_axi_awready         (s_axi_awready),
        .s_axi_wdata           (s_axi_wdata),
        .s_axi_wstrb           (s_axi_wstrb),
        .s_axi_wvalid          (s_axi_wvalid),
        .s_axi_wready          (s_axi_wready),
        .s_axi_bresp           (s_axi_bresp),
        .s_axi_bvalid          (s_axi_bvalid),
        .s_axi_bready          (s_axi_bready),
        .s_axi_araddr          (s_axi_araddr),
        .s_axi_arvalid         (s_axi_arvalid),
        .s_axi_arready         (s_axi_arready),
        .s_axi_rdata           (s_axi_rdata),
        .s_axi_rresp           (s_axi_rresp),
        .s_axi_rvalid          (s_axi_rvalid),
        .s_axi_rready          (s_axi_rready),
        .layer_complete_pulse  (layer_done),
        .accel_busy            (),
        .accel_irq             (accel_irq),
        .cfg_layer_start_pulse (cfg_layer_start),
        .cfg_dma_src_addr      (cfg_dma_src),
        .cfg_dma_dst_addr      (cfg_dma_dst),
        .cfg_dma_beats         (cfg_dma_beats),
        .cfg_plane_stride      (cfg_plane_stride),
        .cfg_h_in              (cfg_h_in),
        .cfg_w_in              (cfg_w_in),
        .cfg_h_out             (cfg_h_out),
        .cfg_w_out             (cfg_w_out),
        .cfg_c_in              (cfg_c_in),
        .cfg_stride            (cfg_stride),
        .cfg_pad               (cfg_pad),
        .cfg_bias              (cfg_bias),
        .cfg_mult              (cfg_mult),
        .cfg_shift             (cfg_shift),
        .cfg_zero_point        (cfg_zp)
    );

    //--------------------------------------------------------------------------
    // Instantiate Module 7: Sync Barrier Unit
    //--------------------------------------------------------------------------
    sync_barrier u_module7_barrier (
        .clk                    (clk),
        .rst                    (rst),
        .layer_start_in         (cfg_layer_start),
        .dma_rd_done_in         (dma_rd_done),
        .comp_done_in           (comp_done),
        .dma_wr_done_in         (dma_wr_done),
        .compute_buf_sel_out    (compute_buf_sel),
        .dma_buf_sel_out        (dma_buf_sel),
        .comp_start_pulse_out   (comp_start),
        .dma_rd_start_pulse_out (dma_rd_start),
        .layer_done_pulse_out   (layer_done)
    );

    //--------------------------------------------------------------------------
    // Instantiate Modules 1-5 Datapath Wrapper (im2col_mxu_top)
    //--------------------------------------------------------------------------
    im2col_mxu_top #(
        .AXI_ADDR_WIDTH   (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH   (AXI_DATA_WIDTH),
        .ADDR_WIDTH       (19),
        .LOCAL_ADDR_WIDTH (13),
        .DIM_WIDTH        (10),
        .CH_WIDTH         (10),
        .NUM_COLS         (NUM_COLS),
        .WIN_TAPS         (WIN_TAPS)
    ) u_datapath (
        .clk                (clk),
        .rst                (rst),
        .start              (comp_start),
        .busy               (comp_busy),
        .done               (comp_done),
        .dma_rd_start       (dma_rd_start),
        .dma_rd_base_addr   (cfg_dma_src),
        .dma_rd_total_beats (cfg_dma_beats),
        .cfg_xor_shift_en   (1'b1),
        .dma_rd_busy        (dma_rd_busy),
        .dma_rd_done        (dma_rd_done),
        .dma_wr_start       (cfg_layer_start),
        .dma_wr_base_addr   (cfg_dma_dst),
        .dma_wr_total_beats (16'd16),
        .dma_wr_busy        (dma_wr_busy),
        .dma_wr_done        (dma_wr_done),
        .compute_buf_sel    (compute_buf_sel),
        .dma_buf_sel        (dma_buf_sel),
        .base_addr          (19'd0),
        .plane_stride       (cfg_plane_stride),
        .h_in               (cfg_h_in),
        .w_in               (cfg_w_in),
        .h_out              (cfg_h_out),
        .w_out              (cfg_w_out),
        .c_in               (cfg_c_in),
        .k_size             (3'd3),
        .stride             (cfg_stride),
        .pad                (cfg_pad),
        .weight_wr_en       (weight_wr_en),
        .weight_wr_addr     (weight_wr_addr),
        .weight_wr_data     (weight_wr_data),
        .cfg_bias           (cfg_bias),
        .cfg_mult           (cfg_mult),
        .cfg_shift          (cfg_shift),
        .cfg_zero_point     (cfg_zp),
        .cfg_silu_bypass    (1'b0),
        .mxu_valid_out      (),
        .mxu_data_out       (),
        .vpu_req_out        (),
        .vpu_ack_out        (),
        .vpu_data_out       (),
        .ext_vpu_stall_n    (1'b1),
        .m_axi_araddr       (m_axi_araddr),
        .m_axi_arlen        (m_axi_arlen),
        .m_axi_arsize       (m_axi_arsize),
        .m_axi_arburst      (m_axi_arburst),
        .m_axi_arvalid      (m_axi_arvalid),
        .m_axi_arready      (m_axi_arready),
        .m_axi_rdata        (m_axi_rdata),
        .m_axi_rresp        (m_axi_rresp),
        .m_axi_rlast        (m_axi_rlast),
        .m_axi_rvalid       (m_axi_rvalid),
        .m_axi_rready       (m_axi_rready),
        .m_axi_awaddr       (m_axi_awaddr),
        .m_axi_awlen        (m_axi_awlen),
        .m_axi_awsize       (m_axi_awsize),
        .m_axi_awburst      (m_axi_awburst),
        .m_axi_awvalid      (m_axi_awvalid),
        .m_axi_awready      (m_axi_awready),
        .m_axi_wdata        (m_axi_wdata),
        .m_axi_wstrb        (m_axi_wstrb),
        .m_axi_wlast        (m_axi_wlast),
        .m_axi_wvalid       (m_axi_wvalid),
        .m_axi_wready       (m_axi_wready),
        .m_axi_bresp        (m_axi_bresp),
        .m_axi_bvalid       (m_axi_bvalid),
        .m_axi_bready       (m_axi_bready)
    );

endmodule
`default_nettype wire
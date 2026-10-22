//------------------------------------------------------------------------------
// File       : im2col_mxu_top.sv
// Module     : im2col_mxu_top
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Full 5-Module Elastic Pipeline Integration Wrapper:
//              DMA Read -> 18-Bank SRAM -> AGU -> MXU -> VPU -> DMA Writeback
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module im2col_mxu_top #(
    parameter int AXI_ADDR_WIDTH   = 32,
    parameter int AXI_DATA_WIDTH   = 64,
    parameter int ADDR_WIDTH       = 19,
    parameter int LOCAL_ADDR_WIDTH = 13,
    parameter int DIM_WIDTH        = 10,
    parameter int CH_WIDTH         = 10,
    parameter int NUM_COLS         = 8,
    parameter int WIN_TAPS         = 9
)(
    input  wire                                clk,
    input  wire                                rst,

    // Compute Pipeline Execution Control
    input  wire                                start,
    output wire                                busy,
    output wire                                done,

    // Module 5: DMA Descriptors & Status
    input  wire                                dma_rd_start,
    input  wire [AXI_ADDR_WIDTH-1:0]           dma_rd_base_addr,
    input  wire [15:0]                         dma_rd_total_beats,
    input  wire                                cfg_xor_shift_en,
    output wire                                dma_rd_busy,
    output wire                                dma_rd_done,

    input  wire                                dma_wr_start,
    input  wire [AXI_ADDR_WIDTH-1:0]           dma_wr_base_addr,
    input  wire [15:0]                         dma_wr_total_beats,
    output wire                                dma_wr_busy,
    output wire                                dma_wr_done,

    // Ping-Pong Buffer Ownership
    input  wire                                compute_buf_sel,
    input  wire                                dma_buf_sel,

    // Layer Geometry Configuration
    input  wire [ADDR_WIDTH-1:0]               base_addr,
    input  wire [ADDR_WIDTH-1:0]               plane_stride,
    input  wire [DIM_WIDTH-1:0]                h_in,
    input  wire [DIM_WIDTH-1:0]                w_in,
    input  wire [DIM_WIDTH-1:0]                h_out,
    input  wire [DIM_WIDTH-1:0]                w_out,
    input  wire [CH_WIDTH-1:0]                 c_in,
    input  wire [2:0]                          k_size,
    input  wire [1:0]                          stride,
    input  wire [1:0]                          pad,

    // Module 2: Weight Ring Buffer Preload Interface
    input  wire [NUM_COLS-1:0]                 weight_wr_en,
    input  wire [4:0]                          weight_wr_addr,
    input  wire signed [17:0][7:0]             weight_wr_data,

    // Module 4: VPU Quantization Parameters
    input  wire signed [NUM_COLS-1:0][31:0]    cfg_bias,
    input  wire signed [NUM_COLS-1:0][15:0]    cfg_mult,
    input  wire [NUM_COLS-1:0][4:0]            cfg_shift,
    input  wire signed [NUM_COLS-1:0][7:0]     cfg_zero_point,
    input  wire                                cfg_silu_bypass,

    // Intermediate Telemetry (Exposed for Verification)
    output wire                                mxu_valid_out,
    output wire signed [NUM_COLS-1:0][31:0]    mxu_data_out,
    output wire                                vpu_req_out,
    output wire                                vpu_ack_out,
    output wire signed [NUM_COLS-1:0][7:0]     vpu_data_out,

    // External VPU Stall Override (1 = Normal Operation, 0 = Force Backpressure)
    input  wire                                ext_vpu_stall_n,

    //--------------------------------------------------------------------------
    // AXI4-HP Master Interface (To Zynq PS DDR3 Controller)
    //--------------------------------------------------------------------------
    output wire [AXI_ADDR_WIDTH-1:0]           m_axi_araddr,
    output wire [7:0]                          m_axi_arlen,
    output wire [2:0]                          m_axi_arsize,
    output wire [1:0]                          m_axi_arburst,
    output wire                                m_axi_arvalid,
    input  wire                                m_axi_arready,

    input  wire [AXI_DATA_WIDTH-1:0]           m_axi_rdata,
    input  wire [1:0]                          m_axi_rresp,
    input  wire                                m_axi_rlast,
    input  wire                                m_axi_rvalid,
    output wire                                m_axi_rready,

    output wire [AXI_ADDR_WIDTH-1:0]           m_axi_awaddr,
    output wire [7:0]                          m_axi_awlen,
    output wire [2:0]                          m_axi_awsize,
    output wire [1:0]                          m_axi_awburst,
    output wire                                m_axi_awvalid,
    input  wire                                m_axi_awready,

    output wire [AXI_DATA_WIDTH-1:0]           m_axi_wdata,
    output wire [7:0]                          m_axi_wstrb,
    output wire                                m_axi_wlast,
    output wire                                m_axi_wvalid,
    input  wire                                m_axi_wready,

    input  wire [1:0]                          m_axi_bresp,
    input  wire                                m_axi_bvalid,
    output wire                                m_axi_bready
);

    //--------------------------------------------------------------------------
    // Interconnect Wires
    //--------------------------------------------------------------------------
    wire [17:0]                         dma_to_sram_wr_en;
    wire [LOCAL_ADDR_WIDTH-1:0]         dma_to_sram_wr_addr;
    wire signed [17:0][7:0]             dma_to_sram_wr_data;

    wire                                agu_to_sram_req;
    wire                                sram_to_agu_ack;
    wire                                agu_row_last;
    wire [WIN_TAPS-1:0]                 lane_valid;
    wire [WIN_TAPS-1:0]                 sram_rd_en;
    wire [WIN_TAPS-1:0]                 zero_pad;
    wire [WIN_TAPS-1:0][ADDR_WIDTH-1:0] sram_addr_a;
    wire [WIN_TAPS-1:0][ADDR_WIDTH-1:0] sram_addr_b;
    wire                                stream_b_val;

    wire                                sram_to_mxu_req;
    wire                                mxu_to_sram_ack;
    wire                                sram_to_mxu_row_last;
    wire [WIN_TAPS-1:0]                 sram_to_mxu_pad_s0;
    wire [WIN_TAPS-1:0]                 sram_to_mxu_pad_s1;
    wire signed [WIN_TAPS-1:0][7:0]     sram_to_mxu_act_s0;
    wire signed [WIN_TAPS-1:0][7:0]     sram_to_mxu_act_s1;

    wire                                vpu_to_mxu_ready;
    wire                                dma_to_vpu_ack;

    assign vpu_ack_out = dma_to_vpu_ack && ext_vpu_stall_n;

    //--------------------------------------------------------------------------
    // Instantiate Module 5: Disjoint AXI4-HP DMA Controller
    //--------------------------------------------------------------------------
    dma_controller #(
        .AXI_ADDR_WIDTH   (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH   (AXI_DATA_WIDTH),
        .LOCAL_ADDR_WIDTH (LOCAL_ADDR_WIDTH),
        .CANVAS_BITS      (6),
        .CHAN_BITS        (6)
    ) u_module5_dma (
        .clk                (clk),
        .rst                (rst),
        .dma_rd_start       (dma_rd_start),
        .dma_rd_base_addr   (dma_rd_base_addr),
        .dma_rd_total_beats (dma_rd_total_beats),
        .cfg_xor_shift_en   (cfg_xor_shift_en),
        .dma_rd_busy        (dma_rd_busy),
        .dma_rd_done        (dma_rd_done),
        .dma_wr_start       (dma_wr_start),
        .dma_wr_base_addr   (dma_wr_base_addr),
        .dma_wr_total_beats (dma_wr_total_beats),
        .dma_wr_busy        (dma_wr_busy),
        .dma_wr_done        (dma_wr_done),
        .sram_dma_wr_en     (dma_to_sram_wr_en),
        .sram_dma_wr_addr   (dma_to_sram_wr_addr),
        .sram_dma_wr_data   (dma_to_sram_wr_data),
        .vpu_req_in         (vpu_req_out && ext_vpu_stall_n),
        .vpu_ack_out        (dma_to_vpu_ack),
        .vpu_data_in        (vpu_data_out),
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

    //--------------------------------------------------------------------------
    // Instantiate Module 1: reshape_im2col_engine_async
    //--------------------------------------------------------------------------
    reshape_im2col_engine_async #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DIM_WIDTH  (DIM_WIDTH),
        .CH_WIDTH   (CH_WIDTH),
        .WIN_TAPS   (WIN_TAPS)
    ) u_module1_agu (
        .clk                (clk),
        .rst                (rst),
        .start              (start),
        .busy               (busy),
        .done               (done),
        .base_addr          (base_addr),
        .plane_stride       (plane_stride),
        .h_in               (h_in),
        .w_in               (w_in),
        .h_out              (h_out),
        .w_out              (w_out),
        .c_in               (c_in),
        .k_size             (k_size),
        .stride             (stride),
        .pad                (pad),
        .req_out            (agu_to_sram_req),
        .ack_in             (sram_to_agu_ack),
        .lane_valid_mask    (lane_valid),
        .sram_rd_en_mask    (sram_rd_en),
        .zero_pad_mask      (zero_pad),
        .sram_rd_addr_a_vec (sram_addr_a),
        .sram_rd_addr_b_vec (sram_addr_b),
        .stream_b_valid     (stream_b_val),
        .row_last           (agu_row_last)
    );

    //--------------------------------------------------------------------------
    // Instantiate Module 3: 18-Bank Ping-Pong SRAM Scratchpad
    //--------------------------------------------------------------------------
    sram_scratchpad #(
        .AGU_ADDR_WIDTH   (ADDR_WIDTH),
        .LOCAL_ADDR_WIDTH (LOCAL_ADDR_WIDTH),
        .WIN_TAPS         (WIN_TAPS)
    ) u_module3_sram (
        .clk                  (clk),
        .rst                  (rst),
        .compute_buf_sel      (compute_buf_sel),
        .dma_buf_sel          (dma_buf_sel),
        .dma_wr_en            (dma_to_sram_wr_en),
        .dma_wr_addr          (dma_to_sram_wr_addr),
        .dma_wr_data          (dma_to_sram_wr_data),
        .agu_req_in           (agu_to_sram_req),
        .agu_ack_out          (sram_to_agu_ack),
        .sram_rd_en_mask_in   (sram_rd_en),
        .zero_pad_mask_in     (zero_pad),
        .sram_rd_addr_a_in    (sram_addr_a),
        .sram_rd_addr_b_in    (sram_addr_b),
        .stream_b_valid_in    (stream_b_val),
        .agu_row_last_in      (agu_row_last),
        .mxu_req_out          (sram_to_mxu_req),
        .mxu_ack_in           (mxu_to_sram_ack),
        .mxu_row_last_out     (sram_to_mxu_row_last),
        .mxu_pad_mask_s0_out  (sram_to_mxu_pad_s0),
        .mxu_pad_mask_s1_out  (sram_to_mxu_pad_s1),
        .mxu_act_stream_0_out (sram_to_mxu_act_s0),
        .mxu_act_stream_1_out (sram_to_mxu_act_s1)
    );

    //--------------------------------------------------------------------------
    // Instantiate Module 2: 18-Tap Vector MXU
    //--------------------------------------------------------------------------
    matrix_engine_mxu_path_b #(
        .NUM_COLS (NUM_COLS),
        .TAPS     (18)
    ) u_module2_mxu (
        .clk             (clk),
        .rst_n           (~rst),
        .clear_acc       (rst),
        .agu_req_in      (sram_to_mxu_req),
        .agu_ack_out     (mxu_to_sram_ack),
        .agu_row_last_in (sram_to_mxu_row_last),
        .pad_mask_s0_in  (sram_to_mxu_pad_s0),
        .pad_mask_s1_in  (sram_to_mxu_pad_s1),
        .act_stream_0_in (sram_to_mxu_act_s0),
        .act_stream_1_in (sram_to_mxu_act_s1),
        .weight_wr_en    (weight_wr_en),
        .weight_wr_addr  (weight_wr_addr),
        .weight_wr_data  (weight_wr_data),
        .vpu_ready_in    (vpu_to_mxu_ready),
        .mxu_valid_out   (mxu_valid_out),
        .mxu_data_out    (mxu_data_out)
    );

    //--------------------------------------------------------------------------
    // Instantiate Module 4: Vector Processing Unit (VPU)
    //--------------------------------------------------------------------------
    vector_processing_unit #(
        .NUM_COLS (NUM_COLS)
    ) u_module4_vpu (
        .clk             (clk),
        .rst             (rst),
        .cfg_bias        (cfg_bias),
        .cfg_mult        (cfg_mult),
        .cfg_shift       (cfg_shift),
        .cfg_zero_point  (cfg_zero_point),
        .cfg_silu_bypass (cfg_silu_bypass),
        .mxu_valid_in    (mxu_valid_out),
        .vpu_ready_out   (vpu_to_mxu_ready),
        .mxu_psum_in     (mxu_data_out),
        .vpu_req_out     (vpu_req_out),
        .vpu_ack_in      (vpu_ack_out),
        .vpu_data_out    (vpu_data_out)
    );

endmodule
`default_nettype wire
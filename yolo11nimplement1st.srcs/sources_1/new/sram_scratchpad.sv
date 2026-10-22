//------------------------------------------------------------------------------
// File       : sram_scratchpad.sv
// Module     : sram_scratchpad
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Module 3 Multi-Banked Ping-Pong BRAM Scratchpad (18 Sub-Banks).
//              Provides 18 conflict-free parallel INT8 reads per cycle with an
//              integrated Elastic Skid Buffer to handle downstream backpressure
//              without losing in-flight BRAM read beats.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module sram_scratchpad #(
    parameter int AGU_ADDR_WIDTH   = 19,
    parameter int LOCAL_ADDR_WIDTH = 13, // 8 KB Ping + 8 KB Pong per bank (288 KB total)
    parameter int WIN_TAPS         = 9   // 9 spatial taps x 2 streams = 18 banks
)(
    input  wire                                  clk,
    input  wire                                  rst,

    //--------------------------------------------------------------------------
    // Ping-Pong Buffer Ownership Control
    //--------------------------------------------------------------------------
    input  wire                                  compute_buf_sel, // Active buffer for MXU reads
    input  wire                                  dma_buf_sel,     // Target buffer for DMA writes

    //--------------------------------------------------------------------------
    // DMA Write Interface (18 Banks Parallel / Selective Write)
    //--------------------------------------------------------------------------
    input  wire [17:0]                           dma_wr_en,
    input  wire [LOCAL_ADDR_WIDTH-1:0]           dma_wr_addr,
    input  wire signed [17:0][7:0]               dma_wr_data,

    //--------------------------------------------------------------------------
    // Upstream Interface: Module 1 (reshape_im2col_engine_async)
    //--------------------------------------------------------------------------
    input  wire                                  agu_req_in,
    output wire                                  agu_ack_out,
    input  wire [WIN_TAPS-1:0]                   sram_rd_en_mask_in,
    input  wire [WIN_TAPS-1:0]                   zero_pad_mask_in,
    input  wire [WIN_TAPS-1:0][AGU_ADDR_WIDTH-1:0] sram_rd_addr_a_in,
    input  wire [WIN_TAPS-1:0][AGU_ADDR_WIDTH-1:0] sram_rd_addr_b_in,
    input  wire                                  stream_b_valid_in,
    input  wire                                  agu_row_last_in,

    //--------------------------------------------------------------------------
    // Downstream Interface: Module 2 (matrix_engine_mxu_path_b)
    //--------------------------------------------------------------------------
    output wire                                  mxu_req_out,
    input  wire                                  mxu_ack_in,
    output wire                                  mxu_row_last_out,
    output wire [WIN_TAPS-1:0]                   mxu_pad_mask_s0_out,
    output wire [WIN_TAPS-1:0]                   mxu_pad_mask_s1_out,
    output wire signed [WIN_TAPS-1:0][7:0]       mxu_act_stream_0_out,
    output wire signed [WIN_TAPS-1:0][7:0]       mxu_act_stream_1_out
);

    //--------------------------------------------------------------------------
    // 1. Elastic Skid Control Signals
    //--------------------------------------------------------------------------
    logic skid_valid_q;
    logic s1_valid_q;

    // Ready to accept a new address from Module 1 whenever Skid Buffer is empty
    wire sram_ready   = !skid_valid_q;
    assign agu_ack_out = agu_req_in && sram_ready;
    wire bram_step_en = agu_req_in && sram_ready;

    //--------------------------------------------------------------------------
    // 2. 18 Independent Dual-Port BRAM Sub-Banks (Pool A: 0..8, Pool B: 9..17)
    //--------------------------------------------------------------------------
    logic signed [WIN_TAPS-1:0][7:0] bram_dout_a;
    logic signed [WIN_TAPS-1:0][7:0] bram_dout_b;

    genvar t;
    generate
        for (t = 0; t < WIN_TAPS; t++) begin : gen_bram_banks

            // Pool A Bank t (Stream 0: Lower Channel Half)
            bram_bank #(
                .LOCAL_ADDR_WIDTH (LOCAL_ADDR_WIDTH),
                .DATA_WIDTH       (8)
            ) u_bank_a (
                .clk        (clk),
                .wr_en      (dma_wr_en[t]),
                .wr_buf_sel (dma_buf_sel),
                .wr_addr    (dma_wr_addr),
                .wr_data    (dma_wr_data[t]),
                .rd_en      (bram_step_en && sram_rd_en_mask_in[t]),
                .rd_buf_sel (compute_buf_sel),
                .rd_addr    (sram_rd_addr_a_in[t][LOCAL_ADDR_WIDTH-1:0]),
                .rd_data    (bram_dout_a[t])
            );

            // Pool B Bank t (Stream 1: Upper Channel Half)
            bram_bank #(
                .LOCAL_ADDR_WIDTH (LOCAL_ADDR_WIDTH),
                .DATA_WIDTH       (8)
            ) u_bank_b (
                .clk        (clk),
                .wr_en      (dma_wr_en[t + 9]),
                .wr_buf_sel (dma_buf_sel),
                .wr_addr    (dma_wr_addr),
                .wr_data    (dma_wr_data[t + 9]),
                .rd_en      (bram_step_en && sram_rd_en_mask_in[t] && stream_b_valid_in),
                .rd_buf_sel (compute_buf_sel),
                .rd_addr    (sram_rd_addr_b_in[t][LOCAL_ADDR_WIDTH-1:0]),
                .rd_data    (bram_dout_b[t])
            );

        end
    endgenerate

    //--------------------------------------------------------------------------
    // 3. Stage 1 Sideband Alignment & Stage 2 Elastic Skid Buffer Registers
    //--------------------------------------------------------------------------
    // Stage 1 sideband registers (aligned with 1-cycle synchronous BRAM read)
    logic                s1_row_last_q;
    logic [WIN_TAPS-1:0] s1_pad_s0_q;
    logic [WIN_TAPS-1:0] s1_pad_s1_q;

    // Stage 2 Skid Buffer registers (catches in-flight BRAM read on MXU stall)
    logic                           skid_row_last_q;
    logic [WIN_TAPS-1:0]            skid_pad_s0_q;
    logic [WIN_TAPS-1:0]            skid_pad_s1_q;
    logic signed [WIN_TAPS-1:0][7:0] skid_act_s0_q;
    logic signed [WIN_TAPS-1:0][7:0] skid_act_s1_q;

    wire [WIN_TAPS-1:0] next_pad_s0 = zero_pad_mask_in;
    wire [WIN_TAPS-1:0] next_pad_s1 = zero_pad_mask_in | {WIN_TAPS{~stream_b_valid_in}};

    always_ff @(posedge clk) begin
        if (rst) begin
            s1_valid_q      <= 1'b0;
            s1_row_last_q   <= 1'b0;
            s1_pad_s0_q     <= '0;
            s1_pad_s1_q     <= '0;

            skid_valid_q    <= 1'b0;
            skid_row_last_q <= 1'b0;
            skid_pad_s0_q   <= '0;
            skid_pad_s1_q   <= '0;
            skid_act_s0_q   <= '0;
            skid_act_s1_q   <= '0;
        end else begin
            if (sram_ready) begin
                // If Stage 1 has valid BRAM data and downstream MXU stalls,
                // catch the current BRAM output and sideband into the Skid Buffer
                if (s1_valid_q && !mxu_ack_in) begin
                    skid_valid_q    <= 1'b1;
                    skid_row_last_q <= s1_row_last_q;
                    skid_pad_s0_q   <= s1_pad_s0_q;
                    skid_pad_s1_q   <= s1_pad_s1_q;
                    skid_act_s0_q   <= bram_dout_a;
                    skid_act_s1_q   <= bram_dout_b;
                end

                // Advance Stage 1 pipeline with new AGU request
                s1_valid_q <= agu_req_in;
                if (agu_req_in) begin
                    s1_row_last_q <= agu_row_last_in;
                    s1_pad_s0_q   <= next_pad_s0;
                    s1_pad_s1_q   <= next_pad_s1;
                end
            end else begin
                // Skid Buffer is full (sram_ready == 0): BRAM outputs are frozen.
                // Once downstream MXU acknowledges, drain the Skid Buffer.
                if (mxu_ack_in) begin
                    skid_valid_q <= 1'b0;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // 4. Output Multiplexer (Skid Buffer vs. Direct BRAM Output)
    //--------------------------------------------------------------------------
    assign mxu_req_out          = skid_valid_q ? 1'b1            : s1_valid_q;
    assign mxu_row_last_out     = skid_valid_q ? skid_row_last_q : s1_row_last_q;
    assign mxu_pad_mask_s0_out  = skid_valid_q ? skid_pad_s0_q   : s1_pad_s0_q;
    assign mxu_pad_mask_s1_out  = skid_valid_q ? skid_pad_s1_q   : s1_pad_s1_q;
    assign mxu_act_stream_0_out = skid_valid_q ? skid_act_s0_q   : bram_dout_a;
    assign mxu_act_stream_1_out = skid_valid_q ? skid_act_s1_q   : bram_dout_b;

endmodule
`default_nettype wire
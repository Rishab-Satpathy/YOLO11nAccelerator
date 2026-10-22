`timescale 1ns / 1ps
`default_nettype none

module matrix_engine_mxu_path_b #(
    parameter int NUM_COLS = 8, // 8 parallel output channels (144 MACs total)
    parameter int TAPS     = 18 // 9 spatial taps x 2 channel streams
)(
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               clear_acc,

    //--------------------------------------------------------------------------
    // Module 1 (AGU) Elastic Unidirectional Handshake & Control Interface
    //--------------------------------------------------------------------------
    input  wire                               agu_req_in,       // New batch valid from AGU
    output logic                              agu_ack_out,      // Direct 1-cycle step ack
    input  wire                               agu_row_last_in,  // Beat 31 flag
    input  wire [8:0]                         pad_mask_s0_in,   // Halo mask Stream 0
    input  wire [8:0]                         pad_mask_s1_in,   // Halo mask Stream 1

    //--------------------------------------------------------------------------
    // Module 3 (SRAM Scratchpad) 18-Tap Activation Data Bus
    //--------------------------------------------------------------------------
    input  wire signed [8:0][7:0]             act_stream_0_in,  // Ch 0..31 taps
    input  wire signed [8:0][7:0]             act_stream_1_in,  // Ch 32..63 taps

    //--------------------------------------------------------------------------
    // Weight Ring Buffer Configuration Interface
    //--------------------------------------------------------------------------
    input  wire [NUM_COLS-1:0]                weight_wr_en,
    input  wire [4:0]                         weight_wr_addr,
    input  wire signed [TAPS-1:0][7:0]        weight_wr_data,

    //--------------------------------------------------------------------------
    // Module 4 (VPU) Interface
    //--------------------------------------------------------------------------
    input  wire                               vpu_ready_in,     // Downstream backpressure
    output logic                              mxu_valid_out,    // 8 output pixels ready
    output logic signed [NUM_COLS-1:0][31:0]  mxu_data_out      // 8 finished 32-bit sums
);

    //--------------------------------------------------------------------------
    // 1. Elastic Handshake Dispatcher
    //--------------------------------------------------------------------------
    // Unidirectional Send-Ack: Step Module 1 whenever data is requested and VPU is ready
    assign agu_ack_out = agu_req_in && vpu_ready_in;

    wire step_active = agu_req_in && vpu_ready_in;

    //--------------------------------------------------------------------------
    // 2. Halo Zero-Padding Mask Application (Module 1 Pad Bits)
    //--------------------------------------------------------------------------
    logic signed [TAPS-1:0][7:0] act_masked_bus;

    always_comb begin
        // Stream 0: Lower 9 spatial taps
        for (int t = 0; t < 9; t++) begin
            act_masked_bus[t] = pad_mask_s0_in[t] ? 8'sd0 : act_stream_0_in[t];
        end
        // Stream 1: Upper 9 spatial taps
        for (int t = 0; t < 9; t++) begin
            act_masked_bus[t + 9] = pad_mask_s1_in[t] ? 8'sd0 : act_stream_1_in[t];
        end
    end

    //--------------------------------------------------------------------------
    // 3. Parallel Columns (8 Filter Columns x 18 Taps = 144 MACs)
    //--------------------------------------------------------------------------
    logic signed [TAPS-1:0][7:0] col_weights [NUM_COLS];
    logic [NUM_COLS-1:0]         col_valid_out;

    genvar c;
    generate
        for (c = 0; c < NUM_COLS; c++) begin : gen_columns

            // Local 32-entry LUTRAM Ring Buffer per column
            weight_ring_buffer #(
                .TAPS       (TAPS),
                .DEPTH      (32),
                .ADDR_WIDTH (5)
            ) u_weight_ring (
                .clk        (clk),
                .rst_n      (rst_n),
                .wr_en      (weight_wr_en[c]),
                .wr_addr    (weight_wr_addr),
                .wr_data    (weight_wr_data),
                .step_en    (step_active),
                .rewind     (agu_row_last_in && step_active),
                .weight_out (col_weights[c])
            );

            // 18-Tap Vector PE Column
            pe_column_18tap #(
                .TAPS       (TAPS)
            ) u_pe_col (
                .clk         (clk),
                .rst_n       (rst_n),
                .valid_in    (step_active),
                .row_last_in (agu_row_last_in),
                .clear_acc   (clear_acc),
                .act_in      (act_masked_bus),
                .weight_in   (col_weights[c]),
                .valid_out   (col_valid_out[c]),
                .out_data    (mxu_data_out[c])
            );

        end
    endgenerate

    // All columns operate in lockstep; col 0 valid provides array-wide strobe
    assign mxu_valid_out = col_valid_out[0];

endmodule
`default_nettype wire
//------------------------------------------------------------------------------
// File       : vector_processing_unit.sv
// Module     : vector_processing_unit
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Module 4 Top-Level Vector Processing Unit (8 Parallel Channels).
//              Consumes 8x 32-bit partial sums from Module 2 (MXU), applies
//              per-channel Bias, Dynamic Precision-Zoom Scaling, SiLU LUTRAM,
//              and INT8 Clamping, and streams 64-bit (8x INT8) packets via
//              an elastic unidirectional req/ack handshake.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module vector_processing_unit #(
    parameter int NUM_COLS = 8
)(
    input  wire                                clk,
    input  wire                                rst,

    //--------------------------------------------------------------------------
    // Quantization & Activation Configuration Registers
    //--------------------------------------------------------------------------
    input  wire signed [NUM_COLS-1:0][31:0]    cfg_bias,
    input  wire signed [NUM_COLS-1:0][15:0]    cfg_mult,
    input  wire [NUM_COLS-1:0][4:0]            cfg_shift,
    input  wire signed [NUM_COLS-1:0][7:0]     cfg_zero_point,
    input  wire                                cfg_silu_bypass,

    //--------------------------------------------------------------------------
    // Upstream Interface: Module 2 (MXU)
    //--------------------------------------------------------------------------
    input  wire                                mxu_valid_in,
    output wire                                vpu_ready_out,
    input  wire signed [NUM_COLS-1:0][31:0]    mxu_psum_in,

    //--------------------------------------------------------------------------
    // Downstream Elastic Interface: Output Buffer / AXI DMA
    //--------------------------------------------------------------------------
    output wire                                vpu_req_out,
    input  wire                                vpu_ack_in,
    output wire signed [NUM_COLS-1:0][7:0]     vpu_data_out
);

    //--------------------------------------------------------------------------
    // 1. Elastic Backpressure & Skid Buffer Control
    //--------------------------------------------------------------------------
    logic                           skid_valid_q;
    logic signed [NUM_COLS-1:0][7:0] skid_data_q;

    wire                            slice_valid_out [NUM_COLS];
    wire signed [NUM_COLS-1:0][7:0] slice_data_out;

    // Pipeline steps forward whenever the output Skid Buffer is empty
    wire pipe_step_en  = !skid_valid_q;
    assign vpu_ready_out = !skid_valid_q;

    //--------------------------------------------------------------------------
    // 2. 8 Parallel Channel Slices (Lockstep Execution)
    //--------------------------------------------------------------------------
    genvar c;
    generate
        for (c = 0; c < NUM_COLS; c++) begin : gen_vpu_slices
            vpu_channel_slice u_slice (
                .clk           (clk),
                .rst           (rst),
                .step_en       (pipe_step_en),
                .valid_in      (mxu_valid_in),
                .psum_in       (mxu_psum_in[c]),
                .bias_in       (cfg_bias[c]),
                .mult_in       (cfg_mult[c]),
                .shift_in      (cfg_shift[c]),
                .zero_point_in (cfg_zero_point[c]),
                .silu_bypass   (cfg_silu_bypass),
                .valid_out     (slice_valid_out[c]),
                .act_out       (slice_data_out[c])
            );
        end
    endgenerate

    wire pipe_valid = slice_valid_out[0];

    //--------------------------------------------------------------------------
    // 3. Elastic Output Skid Buffer
    //--------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            skid_valid_q <= 1'b0;
            skid_data_q  <= '0;
        end else begin
            if (pipe_step_en) begin
                // If pipeline emits a valid 8-byte vector while downstream stalls,
                // catch the vector in the Skid Buffer
                if (pipe_valid && !vpu_ack_in) begin
                    skid_valid_q <= 1'b1;
                    skid_data_q  <= slice_data_out;
                end
            end else begin
                // Drain Skid Buffer as soon as downstream acknowledges
                if (vpu_ack_in) begin
                    skid_valid_q <= 1'b0;
                end
            end
        end
    end

    assign vpu_req_out  = skid_valid_q ? 1'b1        : pipe_valid;
    assign vpu_data_out = skid_valid_q ? skid_data_q : slice_data_out;

endmodule
`default_nettype wire
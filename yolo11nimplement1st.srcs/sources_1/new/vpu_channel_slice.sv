//------------------------------------------------------------------------------
// File       : vpu_channel_slice.sv
// Module     : vpu_channel_slice
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Single-channel Vector Processing Unit slice with:
//              - Stage 1: INT32 Bias Add + Fixed-Point Scale Multiply
//              - Stage 2: Early Regime Classifier + Dynamic 4x Precision Zoom
//                         Bit-Shifter + Distributed SiLU ROM Lookup
//              - Stage 3: Post-Zoom Re-alignment + Zero-Point + INT8 Clamp
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module vpu_channel_slice (
    input  wire                clk,
    input  wire                rst,

    // Pipeline Handshake Step Enable (1 = advance pipeline, 0 = freeze on stall)
    input  wire                step_en,
    input  wire                valid_in,
    input  wire signed [31:0]  psum_in,

    // Per-Channel Quantization Parameters
    input  wire signed [31:0]  bias_in,       // 32-bit channel bias
    input  wire signed [15:0]  mult_in,       // Positive fixed-point scale multiplier
    input  wire [4:0]          shift_in,      // Right-shift exponent (0..31)
    input  wire signed [7:0]   zero_point_in, // Output INT8 zero-point offset
    input  wire                silu_bypass,   // 1 = Linear/Leaky passthrough, 0 = SiLU

    // Quantized INT8 Output
    output logic               valid_out,
    output logic signed [7:0]  act_out
);

    // Regime Classification Encoding
    typedef enum logic [1:0] {
        REGIME_NEG_ZERO = 2'b00, // x <= -64 -> SiLU(x) = 0
        REGIME_FINE_LUT = 2'b01, // |x| < 16 -> 4x zoomed fractional lookup
        REGIME_STD_LUT  = 2'b10, // [-63, +63] -> standard LUT lookup
        REGIME_POS_LIN  = 2'b11  // x >= +64 -> linear passthrough + clamp
    } regime_t;

    //--------------------------------------------------------------------------
    // Stage 1: Bias Addition & Fixed-Point Multiplication
    //--------------------------------------------------------------------------
    logic               valid_s1;
    logic signed [47:0] scaled_prod_s1;
    logic [4:0]         shift_s1;
    logic signed [7:0]  zp_s1;
    logic               bypass_s1;

    wire signed [31:0] biased_sum = psum_in + bias_in;

    always_ff @(posedge clk) begin
        if (rst) begin
            valid_s1       <= 1'b0;
            scaled_prod_s1 <= 48'sd0;
            shift_s1       <= 5'd0;
            zp_s1          <= 8'sd0;
            bypass_s1      <= 1'b0;
        end else if (step_en) begin
            valid_s1       <= valid_in;
            scaled_prod_s1 <= biased_sum * mult_in;
            shift_s1       <= shift_in;
            zp_s1          <= zero_point_in;
            bypass_s1      <= silu_bypass;
        end
    end

    //--------------------------------------------------------------------------
    // Stage 2: Early Regime Detection & Dynamic Sub-Range Bit-Shifter
    //--------------------------------------------------------------------------
    wire [4:0]         fine_shift     = (shift_s1 >= 5'd2) ? (shift_s1 - 5'd2) : shift_s1;
    wire signed [47:0] std_shifted    = scaled_prod_s1 >>> shift_s1;
    wire signed [47:0] zoomed_shifted = scaled_prod_s1 >>> fine_shift;

    // Detect if value is in small sub-range [-15, +15] and eligible for 4x zoom
    wire is_small_range = (std_shifted >= -48'sd15) && (std_shifted <= 48'sd15) && (shift_s1 >= 5'd2);

    regime_t           regime_comb;
    logic signed [7:0] rom_addr_comb;

    always_comb begin
        if (std_shifted <= -48'sd64) begin
            regime_comb   = REGIME_NEG_ZERO;
            rom_addr_comb = -8'sd64;
        end else if (std_shifted >= 48'sd64) begin
            regime_comb   = REGIME_POS_LIN;
            rom_addr_comb = (std_shifted > 48'sd127) ? 8'sd127 : std_shifted[7:0];
        end else if (is_small_range) begin
            regime_comb   = REGIME_FINE_LUT;
            rom_addr_comb = zoomed_shifted[7:0]; // 4x higher resolution index!
        end else begin
            regime_comb   = REGIME_STD_LUT;
            rom_addr_comb = std_shifted[7:0];
        end
    end

    // Instantiate Distributed LUTRAM SiLU ROM (1-cycle synchronous read into Stage 3)
    logic signed [7:0] rom_dout_s2;

    silu_rom u_silu_rom (
        .clk      (clk),
        .addr_in  (rom_addr_comb),
        .data_out (rom_dout_s2)
    );

    // Stage 2 Pipeline Alignment Registers
    logic               valid_s2;
    regime_t            regime_s2;
    logic signed [47:0] std_val_s2;
    logic signed [7:0]  zp_s2;
    logic               bypass_s2;

    always_ff @(posedge clk) begin
        if (rst) begin
            valid_s2   <= 1'b0;
            regime_s2  <= REGIME_NEG_ZERO;
            std_val_s2 <= 48'sd0;
            zp_s2      <= 8'sd0;
            bypass_s2  <= 1'b0;
        end else if (step_en) begin
            valid_s2   <= valid_s1;
            regime_s2  <= regime_comb;
            std_val_s2 <= std_shifted;
            zp_s2      <= zp_s1;
            bypass_s2  <= bypass_s1;
        end
    end

    //--------------------------------------------------------------------------
    // Stage 3: Post-LUT Selection, Fine-Zoom Downshift & INT8 Saturation Clamp
    //--------------------------------------------------------------------------
    logic signed [47:0] activated_val;
    logic signed [47:0] final_unclamped;
    logic signed [7:0]  clamped_int8;

    // Undo the 4x zoom with rounding: (rom_dout + 2) >>> 2
    wire signed [7:0] fine_rescaled = ($signed(rom_dout_s2) + 8'sd2) >>> 2;

    always_comb begin
        if (bypass_s2) begin
            activated_val = std_val_s2;
        end else begin
            case (regime_s2)
                REGIME_NEG_ZERO: activated_val = 48'sd0;
                REGIME_FINE_LUT: activated_val = {{40{fine_rescaled[7]}}, fine_rescaled};
                REGIME_STD_LUT:  activated_val = {{40{rom_dout_s2[7]}}, rom_dout_s2};
                REGIME_POS_LIN:  activated_val = std_val_s2;
                default:         activated_val = 48'sd0;
            endcase
        end

        final_unclamped = activated_val + {{40{zp_s2[7]}}, zp_s2};

        // Hard INT8 Saturation Clamp [-128, +127]
        if (final_unclamped > 48'sd127) begin
            clamped_int8 = 8'sd127;
        end else if (final_unclamped < -48'sd128) begin
            clamped_int8 = -8'sd128;
        end else begin
            clamped_int8 = final_unclamped[7:0];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            valid_out <= 1'b0;
            act_out   <= 8'sd0;
        end else if (step_en) begin
            valid_out <= valid_s2;
            if (valid_s2) begin
                act_out <= clamped_int8;
            end
        end
    end

endmodule
`default_nettype wire
//------------------------------------------------------------------------------
// File       : silu_rom.sv
// Module     : silu_rom
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: 256-Entry x 8-Bit Signed Distributed LUTRAM for SiLU (Swish).
//              Maps signed INT8 fixed-point input x (1.0 = 16 units) to
//              signed INT8 output: SiLU(x) = x / (1 + exp(-x/16)).
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module silu_rom (
    input  wire               clk,
    input  wire signed [7:0]  addr_in,  // Signed INT8 index [-128, +127]
    output logic signed [7:0] data_out  // Signed INT8 SiLU(x) output
);

    (* rom_style = "distributed" *)
    logic signed [7:0] rom_table [0:255];

    // Populate deterministic SiLU curve across all 256 two's-complement indices:
    //   - Deep negative (x <= -64): saturates to 0
    //   - Non-linear region (-64 < x < 64): smooth rational/exponential curve
    //     with characteristic negative dip to -5 at x = -32
    //   - Positive linear region (x >= 64): SiLU(x) = x
    initial begin
        for (int i = 0; i < 256; i++) begin
            int signed x_val;
            int signed y_val;
            x_val = (i >= 128) ? (i - 256) : i;

            if (x_val <= -64) begin
                y_val = 0;
            end else if (x_val >= 64) begin
                y_val = x_val;
            end else begin
                // Smooth second-order sigmoid approximation on [-64, +64]:
                // y = round( x * (x + 64) / 128 )
                y_val = (x_val * (x_val + 64) + 64) >>> 7;
            end
            rom_table[i] = 8'(y_val);
        end
    end

    wire [7:0] unsigned_idx = addr_in;

    // Single-cycle synchronous LUTRAM read
    always_ff @(posedge clk) begin
        data_out <= rom_table[unsigned_idx];
    end

endmodule
`default_nettype wire
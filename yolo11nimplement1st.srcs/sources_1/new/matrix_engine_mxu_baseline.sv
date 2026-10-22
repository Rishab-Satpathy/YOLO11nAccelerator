`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.10.2026 23:34:39
// Design Name: 
// Module Name: matrix_engine_mxu_baseline
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps
`default_nettype none

module matrix_engine_mxu_baseline #(
    parameter int ROWS = 12, // Number of activation rows (output pixels)
    parameter int COLS = 12  // Number of weight columns (output channels)
)(
    input  wire                                clk,
    input  wire                                rst_n,

    // Control signals from AGU / Sequencer
    input  wire                                valid_in,
    input  wire                                row_last_in,

    // Streamed inputs at time-step k
    input  wire signed [7:0]                   act_in   [ROWS], // 12 activation scalars
    input  wire signed [7:0]                   weight_in[COLS], // 12 weight scalars

    // Output matrix of completed 32-bit partial sums
    output logic                               valid_out,
    output logic signed [31:0]                 data_out [ROWS][COLS]
);

    // 2D grid of completion flags from each individual MAC cell
    logic pe_valid [ROWS][COLS];

    //--------------------------------------------------------------------------
    // 2D Array Instantiation (144 Processing Elements)
    //--------------------------------------------------------------------------
    genvar r, c;
    generate
        for (r = 0; r < ROWS; r = r + 1) begin : gen_row
            for (c = 0; c < COLS; c = c + 1) begin : gen_col

                sequential_mac u_mac (
                    .clk         (clk),
                    .rst_n       (rst_n),
                    .valid_in    (valid_in),
                    .row_last_in (row_last_in),
                    .act_in      (act_in[r]),      // Horizontal broadcast along row r
                    .weight_in   (weight_in[c]),   // Vertical broadcast down column c
                    .valid_out   (pe_valid[r][c]), // Cell completion flag
                    .data_out    (data_out[r][c])  // Cell 32-bit latched result
                );

            end
        end
    endgenerate

    // All PEs execute in lockstep on the identical valid_in and row_last_in.
    // Tapping PE (0,0) provides the array-wide output valid pulse.
    assign valid_out = pe_valid[0][0];

endmodule
`default_nettype wire

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.10.2026 23:12:43
// Design Name: 
// Module Name: sequential_mac
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

module sequential_mac (
    input  wire                clk,
    input  wire                rst_n,

    input  wire                valid_in,
    input  wire                row_last_in,
    input  wire signed [7:0]   act_in,
    input  wire signed [7:0]   weight_in,

    output logic               valid_out,
    output logic signed [31:0] data_out
);

    // 1. Combinational math: signed multiplier & 32-bit adder
    logic signed [15:0] prod;
    logic signed [31:0] next_sum;
    logic signed [31:0] acc_reg;

    assign prod     = act_in * weight_in;
    assign next_sum = acc_reg + prod;

    // 2. Sequential state register
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            acc_reg   <= 32'sd0;
            data_out  <= 32'sd0;
            valid_out <= 1'b0;
        end else begin
            // Default: valid_out is a 1-cycle pulse
            valid_out <= 1'b0;

            if (valid_in) begin
                if (row_last_in) begin
                    // Final tap arrived: emit sum, pulse valid, clear accumulator for next pixel
                    data_out  <= next_sum;
                    valid_out <= 1'b1;
                    acc_reg   <= 32'sd0;
                end else begin
                    // Intermediate tap: accumulate and hold
                    acc_reg   <= next_sum;
                end
            end
        end
    end

endmodule
`default_nettype wire 

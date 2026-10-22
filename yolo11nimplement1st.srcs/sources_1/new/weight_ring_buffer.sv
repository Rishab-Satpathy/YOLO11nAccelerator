`timescale 1ns / 1ps
`default_nettype none

module weight_ring_buffer #(
    parameter int DEPTH      = 32,
    parameter int ADDR_WIDTH = 5,
    parameter int TAPS       = 18,
    parameter int DATA_BITS  = 8
)(
    input  wire                                     clk,
    input  wire                                     rst_n,      // Active-low reset
    input  wire                                     wr_en,
    input  wire [ADDR_WIDTH-1:0]                    wr_addr,
    input  wire signed [TAPS-1:0][DATA_BITS-1:0]   wr_data,
    input  wire                                     step_en,
    input  wire                                     rewind,     // Rewinds pointer to index 0 on pixel boundary
    output logic signed [TAPS-1:0][DATA_BITS-1:0]   weight_out  // Matched output port name
);

    localparam int WORD_WIDTH = TAPS * DATA_BITS; // 18 * 8 = 144 bits

    // 144-bit flat word array infers SLICEM Distributed RAM (LUTRAM)
    (* ram_style = "distributed" *)
    logic [WORD_WIDTH-1:0] mem [0:DEPTH-1];

    logic [ADDR_WIDTH-1:0] rd_ptr_q;

    // Synchronous Write Port
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    // Ring Pointer Counter with Synchronous Rewind & Step
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr_q <= '0;
        end else if (rewind) begin
            rd_ptr_q <= '0; // Reset read pointer to start of weight set for next spatial pixel
        end else if (step_en) begin
            rd_ptr_q <= (rd_ptr_q == DEPTH - 1) ? '0 : (rd_ptr_q + 1'b1);
        end
    end

    // Asynchronous Read Port for Zero-Latency Execution
    assign weight_out = mem[rd_ptr_q];

endmodule
`default_nettype wire
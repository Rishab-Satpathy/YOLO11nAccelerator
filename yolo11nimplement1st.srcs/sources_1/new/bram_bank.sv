//------------------------------------------------------------------------------
// File       : bram_bank.sv
// Module     : bram_bank
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Simple Dual-Port Synchronous Block RAM sub-bank with hardware
//              Ping-Pong buffer partitioning via MSB address bit.
//              18 instances form the conflict-free Module 3 Scratchpad.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module bram_bank #(
    parameter int LOCAL_ADDR_WIDTH = 13, // 8 KB per Ping/Pong half (16 KB total/bank)
    parameter int DATA_WIDTH       = 8   // INT8 activation byte
)(
    input  wire                        clk,

    // Port A: DMA Write Port (Access / Prefetch Side)
    input  wire                        wr_en,
    input  wire                        wr_buf_sel, // 0 = Ping, 1 = Pong
    input  wire [LOCAL_ADDR_WIDTH-1:0] wr_addr,
    input  wire signed [DATA_WIDTH-1:0] wr_data,

    // Port B: AGU/MXU Read Port (Compute / Execute Side)
    input  wire                        rd_en,
    input  wire                        rd_buf_sel, // 0 = Ping, 1 = Pong
    input  wire [LOCAL_ADDR_WIDTH-1:0] rd_addr,
    output logic signed [DATA_WIDTH-1:0] rd_data
);

    // Total depth = 2 * (2^LOCAL_ADDR_WIDTH) = 16,384 bytes (4 RAMB36E1 slices)
    localparam int TOTAL_DEPTH = 1 << (LOCAL_ADDR_WIDTH + 1);

    (* ram_style = "block" *)
    logic signed [DATA_WIDTH-1:0] mem [0:TOTAL_DEPTH-1];

    // Simulation initialization to prevent X-propagation on untouched addresses
    initial begin
        for (int i = 0; i < TOTAL_DEPTH; i++) begin
            mem[i] = '0;
        end
    end

    wire [LOCAL_ADDR_WIDTH:0] full_wr_addr = {wr_buf_sel, wr_addr};
    wire [LOCAL_ADDR_WIDTH:0] full_rd_addr = {rd_buf_sel, rd_addr};

    // Port A: Synchronous DMA Write
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[full_wr_addr] <= wr_data;
        end
    end

    // Port B: Synchronous Compute Read with Read-Enable Freeze for Elastic Stalls
    always_ff @(posedge clk) begin
        if (rd_en) begin
            rd_data <= mem[full_rd_addr];
        end
    end

endmodule
`default_nettype wire
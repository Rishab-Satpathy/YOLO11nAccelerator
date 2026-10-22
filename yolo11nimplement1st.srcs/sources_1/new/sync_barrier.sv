//------------------------------------------------------------------------------
// File       : sync_barrier.sv
// Module     : sync_barrier
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Module 7 Decoupled Access/Execute (DAE) Barrier Coordinator.
//              - State S_PRELOAD: Runs initial DMA load into Ping buffer.
//              - When initial load completes, immediately kicks off Compute.
//              - State S_RUNNING: Tracks compute execution and DDR writeback,
//                eliminating circular barrier deadlock.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module sync_barrier (
    input  wire  clk,
    input  wire  rst,

    // Layer Start Strobe (From Module 6 Dispatcher)
    input  wire  layer_start_in,

    // Completion Tokens
    input  wire  dma_rd_done_in,    // Token A: Ingest DMA finished loading BRAM
    input  wire  comp_done_in,      // Token B: Compute finished reading BRAM
    input  wire  dma_wr_done_in,    // Token C: Drain DMA finished writing DDR

    // Ping-Pong Buffer Ownership
    output logic compute_buf_sel_out,
    output logic dma_buf_sel_out,

    // Step Triggers to Data Engines
    output logic comp_start_pulse_out,
    output logic dma_rd_start_pulse_out,

    // Layer Done Pulse to Module 6 Dispatcher
    output logic layer_done_pulse_out
);

    typedef enum logic [1:0] {
        S_IDLE    = 2'b00,
        S_PRELOAD = 2'b01, // Initial DMA burst into Ping buffer
        S_RUNNING = 2'b10, // Compute and Drain active
        S_DONE    = 2'b11
    } barrier_fsm_t;

    barrier_fsm_t state_q;

    always_ff @(posedge clk) begin
        if (rst) begin
            state_q                <= S_IDLE;
            compute_buf_sel_out    <= 1'b0;
            dma_buf_sel_out        <= 1'b0;
            comp_start_pulse_out   <= 1'b0;
            dma_rd_start_pulse_out <= 1'b0;
            layer_done_pulse_out   <= 1'b0;
        end else begin
            comp_start_pulse_out   <= 1'b0;
            dma_rd_start_pulse_out <= 1'b0;
            layer_done_pulse_out   <= 1'b0;

            case (state_q)
                S_IDLE: begin
                    if (layer_start_in) begin
                        compute_buf_sel_out    <= 1'b0;
                        dma_buf_sel_out        <= 1'b0; // Target Ping buffer first
                        dma_rd_start_pulse_out <= 1'b1; // Trigger initial DMA preload
                        state_q                <= S_PRELOAD;
                    end
                end

                S_PRELOAD: begin
                    // Once initial DMA read completes, trigger compute immediately!
                    if (dma_rd_done_in) begin
                        comp_start_pulse_out <= 1'b1; // START COMPUTE ENGINE
                        dma_buf_sel_out      <= 1'b1; // Point next DMA prefetch to Pong
                        state_q              <= S_RUNNING;
                    end
                end

                S_RUNNING: begin
                    // Layer completes once all quantized VPU pixels are written to DDR
                    if (dma_wr_done_in) begin
                        layer_done_pulse_out <= 1'b1;
                        state_q              <= S_IDLE;
                    end
                end

                default: state_q <= S_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
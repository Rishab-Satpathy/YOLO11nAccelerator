`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.10.2026 21:54:50
// Design Name: 
// Module Name: reshape_im2col_engine_async
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

module reshape_im2col_engine_async #(
    parameter int ADDR_WIDTH = 19, // 512 KB addressable space (covers 448 KB SRAM)
    parameter int DIM_WIDTH  = 10, // Max spatial dimension = 640
    parameter int CH_WIDTH   = 10, // Max channels = 512
    parameter int WIN_TAPS   = 9   // 3x3 window computed in 1 parallel batch
) (
    // System Synchronization & DAE Trigger
    input  logic                                clk,
    input  logic                                rst,
    input  logic                                start,
    output logic                                busy,
    output logic                                done,

    // Layer Geometry & Pointers
    input  logic [ADDR_WIDTH-1:0]               base_addr,
    input  logic [ADDR_WIDTH-1:0]               plane_stride,
    input  logic [DIM_WIDTH-1:0]                h_in,
    input  logic [DIM_WIDTH-1:0]                w_in,
    input  logic [DIM_WIDTH-1:0]                h_out,
    input  logic [DIM_WIDTH-1:0]                w_out,
    input  logic [CH_WIDTH-1:0]                 c_in,
    input  logic [2:0]                          k_size, // 1 (1x1 conv) or 3 (3x3 conv)
    input  logic [1:0]                          stride,
    input  logic [1:0]                          pad,

    // Unidirectional Send-Ack Handshake to Downstream Elastic Buffer / SRAM
    output logic                                req_out, // High when 18-tap bundle is valid
    input  logic                                ack_in,  // High when downstream buffer accepts bundle

    // Parallel Batch Outputs (9 Spatial Taps x 2 Channel Streams = 18 Addresses/Batch)
    output logic [WIN_TAPS-1:0]                 lane_valid_mask,    // Active taps (9'b111_111_111 for 3x3)
    output logic [WIN_TAPS-1:0]                 sram_rd_en_mask,    // 1 = Read SRAM, 0 = Skip (Zero-Pad)
    output logic [WIN_TAPS-1:0]                 zero_pad_mask,      // 1 = Inject 8'sd0 into MAC
    output logic [WIN_TAPS-1:0][ADDR_WIDTH-1:0] sram_rd_addr_a_vec, // Stream A: Lower channels 0 .. (C_in/2)-1
    output logic [WIN_TAPS-1:0][ADDR_WIDTH-1:0] sram_rd_addr_b_vec, // Stream B: Upper channels (C_in/2) .. C_in-1
    output logic                                stream_b_valid,     // 1 if Stream B has an active channel
    output logic                                row_last            // High on final channel batch of a pixel
);

    // =========================================================================
    // 1. Unidirectional Handshake FSM States (Zero Dead Cycles)
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE       = 2'd0, // Waiting for 'start' pulse
        S_REQ_ASSERT = 2'd1, // Present 18-tap bundle (req_out=1); step immediately on ack_in=1
        S_DONE       = 2'd2  // Single-cycle completion pulse
    } hs_state_t;

    hs_state_t state_q, state_d;

    // =========================================================================
    // 2. Dual-Channel Splitting & Window Token Counters
    // =========================================================================
    logic [CH_WIDTH-1:0]   half_c;
    logic [ADDR_WIDTH-1:0] half_vol_stride;

    logic [CH_WIDTH-1:0]   c_q,  c_d;
    logic [DIM_WIDTH-1:0]  ox_q, ox_d;
    logic [DIM_WIDTH-1:0]  oy_q, oy_d;

    logic [ADDR_WIDTH-1:0] c_offset_a_q, c_offset_a_d; // Stream A channel byte offset
    logic [ADDR_WIDTH-1:0] c_offset_b_q, c_offset_b_d; // Stream B channel byte offset
    logic [ADDR_WIDTH-1:0] oy_offset_q,  oy_offset_d;  // Output row byte offset

    // Number of channel-pair steps: ceil(c_in / 2)
    assign half_c = (c_in + 1'b1) >> 1;

    // Multiplier-free halfway volume offset (half_c * plane_stride) via bit shifts
    always_comb begin
        case (c_in)
            10'd2:   half_vol_stride = plane_stride;      // half_c = 1
            10'd3:   half_vol_stride = plane_stride << 1; // RGB Layer 0: half_c = 2
            10'd4:   half_vol_stride = plane_stride << 1; // Testbench: half_c = 2
            10'd8:   half_vol_stride = plane_stride << 2; // half_c = 4
            10'd16:  half_vol_stride = plane_stride << 3; // half_c = 8
            10'd32:  half_vol_stride = plane_stride << 4; // half_c = 16
            10'd64:  half_vol_stride = plane_stride << 5; // half_c = 32
            10'd128: half_vol_stride = plane_stride << 6; // half_c = 64
            10'd256: half_vol_stride = plane_stride << 7; // half_c = 128
            10'd512: half_vol_stride = plane_stride << 8; // half_c = 256
            default: half_vol_stride = ADDR_WIDTH'(plane_stride * half_c);
        endcase
    end

    // Terminal rollover flags
    logic c_end, ox_end, oy_end;
    assign c_end  = (c_q  == (half_c - 1'b1));
    assign ox_end = (ox_q == (w_out  - 1'b1)) && c_end;
    assign oy_end = (oy_q == (h_out  - 1'b1)) && ox_end;

    // Stream B is valid unless c_in is odd (c_in[0]==1) on the final channel step (c_end)
    assign stream_b_valid = (state_q == S_REQ_ASSERT) && !(c_in[0] && c_end);

    // =========================================================================
    // 3. Parallel 3x3 Spatial Coordinate Datapath (Shared by Stream A & B)
    // =========================================================================
    logic signed [DIM_WIDTH+1:0] ox_strided, oy_strided;
    logic signed [DIM_WIDTH+1:0] ix_base, iy_base;
    logic signed [DIM_WIDTH+1:0] ix_tap [3];
    logic signed [DIM_WIDTH+1:0] iy_tap [3];

    logic [ADDR_WIDTH-1:0] pad_offset;
    logic [ADDR_WIDTH-1:0] row_base_addr [3];
    logic [ADDR_WIDTH-1:0] spatial_addr  [3][3];

    // Stride multiplication via wire shift (x1 or x2)
    assign ox_strided = (stride == 2'd2) ?  $signed({2'b00, ox_q, 1'b0}) : 
                                             $signed({2'b00, ox_q});
    assign oy_strided = (stride == 2'd2) ? $signed({1'b0, oy_q, 1'b0}) : $signed({2'b00, oy_q});

    // Top-left anchor of the 3x3 window (kx = 0, ky = 0)
    assign ix_base = ox_strided - $signed({10'd0, pad});
    assign iy_base = oy_strided - $signed({10'd0, pad});

    // 3 parallel X and Y coordinates across the 3x3 window
    assign ix_tap[0] = ix_base;
    assign ix_tap[1] = ix_base + 12'sd1;
    assign ix_tap[2] = ix_base + 12'sd2;

    assign iy_tap[0] = iy_base;
    assign iy_tap[1] = iy_base + 12'sd1;
    assign iy_tap[2] = iy_base + 12'sd2;

    // Padding row byte offset: pad * w_in
    assign pad_offset = (pad == 2'd2) ? {{(ADDR_WIDTH-DIM_WIDTH-1){1'b0}}, w_in, 1'b0} :
                        (pad == 2'd1) ? {{(ADDR_WIDTH-DIM_WIDTH){1'b0}},   w_in}       : '0;

    // 3 parallel row base addresses (ky = 0, 1, 2)
    assign row_base_addr[0] = ADDR_WIDTH'(base_addr + oy_offset_q - pad_offset);
    assign row_base_addr[1] = ADDR_WIDTH'(row_base_addr[0] + {{(ADDR_WIDTH-DIM_WIDTH){1'b0}}, w_in});
    assign row_base_addr[2] = ADDR_WIDTH'(row_base_addr[0] + {{(ADDR_WIDTH-DIM_WIDTH-1){1'b0}}, w_in, 1'b0});

    // Unroll all 9 window taps in parallel
    genvar ky_idx, kx_idx;
    generate
        for (ky_idx = 0; ky_idx < 3; ky_idx++) begin : gen_ky
            for (kx_idx = 0; kx_idx < 3; kx_idx++) begin : gen_kx
                localparam int LANE = ky_idx * 3 + kx_idx;

                logic x_ok, y_ok, tap_in_bounds, lane_active;

                // 1x1 conv activates only Lane 0; 3x3 conv activates all 9 lanes
                assign lane_active = (k_size == 3'd1) ? (LANE == 0) : 1'b1;

                assign x_ok = (ix_tap[kx_idx] >= 0) && (ix_tap[kx_idx] < $signed({2'b00, w_in}));
                assign y_ok = (iy_tap[ky_idx] >= 0) && (iy_tap[ky_idx] < $signed({2'b00, h_in}));
                assign tap_in_bounds = x_ok && y_ok;

                assign lane_valid_mask[LANE] = (state_q == S_REQ_ASSERT) && lane_active;
                assign zero_pad_mask[LANE]   = (state_q == S_REQ_ASSERT) && lane_active && !tap_in_bounds;
                assign sram_rd_en_mask[LANE] = (state_q == S_REQ_ASSERT) && lane_active &&  tap_in_bounds;

                // 2D spatial offset within a plane
                assign spatial_addr[ky_idx][kx_idx] = ADDR_WIDTH'(
                    row_base_addr[ky_idx] +
                    {{(ADDR_WIDTH-DIM_WIDTH-2){ix_tap[kx_idx][DIM_WIDTH+1]}}, ix_tap[kx_idx]}
                );

                // Final Dual-Stream SRAM Addresses (Stream A = Lower Half, Stream B = Upper Half)
                assign sram_rd_addr_a_vec[LANE] = ADDR_WIDTH'(spatial_addr[ky_idx][kx_idx] + c_offset_a_q);
                assign sram_rd_addr_b_vec[LANE] = ADDR_WIDTH'(spatial_addr[ky_idx][kx_idx] + c_offset_b_q);
            end
        end
    endgenerate

    // =========================================================================
    // 4. Handshake & Status Outputs
    // =========================================================================
    assign req_out  = (state_q == S_REQ_ASSERT);
    assign busy     = (state_q == S_REQ_ASSERT);
    assign done     = (state_q == S_DONE);
    assign row_last = (state_q == S_REQ_ASSERT) && c_end;

    // =========================================================================
    // 5. Unidirectional Send-Ack Control & Token Progression
    // =========================================================================
    always_comb begin
        state_d      = state_q;
        c_d          = c_q;
        ox_d         = ox_q;
        oy_d         = oy_q;
        c_offset_a_d = c_offset_a_q;
        c_offset_b_d = c_offset_b_q;
        oy_offset_d  = oy_offset_q;

        case (state_q)
            S_IDLE: begin
                c_d          = '0;
                ox_d         = '0;
                oy_d         = '0;
                c_offset_a_d = '0;
                c_offset_b_d = half_vol_stride; // Stream B starts at halfway channel volume
                oy_offset_d  = '0;

                if (start) begin
                    state_d = S_REQ_ASSERT;
                end
            end

            S_REQ_ASSERT: begin
                // Unidirectional Handshake: As soon as downstream asserts ack_in = 1,
                // immediately advance to the next window batch with zero dead cycles.
                if (ack_in) begin
                    if (oy_end) begin
                        state_d = S_DONE;
                    end else begin
                        state_d = S_REQ_ASSERT;

                        // 1. Advance Channel Pair Counter & Dual Stride Accumulators
                        if (c_end) begin
                            c_d          = '0;
                            c_offset_a_d = '0;
                            c_offset_b_d = half_vol_stride;
                        end else begin
                            c_d          = c_q + 1'b1;
                            c_offset_a_d = ADDR_WIDTH'(c_offset_a_q + plane_stride);
                            c_offset_b_d = ADDR_WIDTH'(c_offset_b_q + plane_stride);
                        end

                        // 2. Advance Output X when all channel pairs complete for this pixel
                        if (c_end) begin
                            ox_d = (ox_q == (w_out - 1'b1)) ? '0 : (ox_q + 1'b1);
                        end

                        // 3. Advance Output Y & Row Accumulator when output row completes
                        if (ox_end) begin
                            oy_d        = (oy_q == (h_out - 1'b1)) ? '0 : (oy_q + 1'b1);
                            oy_offset_d = (stride == 2'd2) ?
                                          ADDR_WIDTH'(oy_offset_q + {{(ADDR_WIDTH-DIM_WIDTH-1){1'b0}}, w_in, 1'b0}) :
                                          ADDR_WIDTH'(oy_offset_q + {{(ADDR_WIDTH-DIM_WIDTH){1'b0}},   w_in});
                        end
                    end
                end
            end

            S_DONE: begin
                state_d = S_IDLE;
            end

            default: state_d = S_IDLE;
        endcase
    end

    // =========================================================================
    // 6. Sequential Register Storage
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state_q      <= S_IDLE;
            c_q          <= '0;
            ox_q         <= '0;
            oy_q         <= '0;
            c_offset_a_q <= '0;
            c_offset_b_q <= '0;
            oy_offset_q  <= '0;
        end else begin
            state_q      <= state_d;
            c_q          <= c_d;
            ox_q         <= ox_d;
            oy_q         <= oy_d;
            c_offset_a_q <= c_offset_a_d;
            c_offset_b_q <= c_offset_b_d;
            oy_offset_q  <= oy_offset_d;
        end
    end

endmodule
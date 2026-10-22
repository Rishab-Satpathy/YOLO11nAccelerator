`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.10.2026 23:50:41
// Design Name: 
// Module Name: reshape_im2col_engine
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

module reshape_im2col_engine #(
    parameter int ADDR_WIDTH = 19, // 512 KB addressable space (covers 448 KB SRAM)
    parameter int DIM_WIDTH  = 10, // Max spatial dimension = 640 (2^10 = 1024)
    parameter int CH_WIDTH   = 10  // Max channels = 512 (2^10 = 1024)
) (
    // DAE Control
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  start,
    output logic                  busy,
    output logic                  done,

    // Layer Geometry & Pointers
    input  logic [ADDR_WIDTH-1:0] base_addr,
    input  logic [ADDR_WIDTH-1:0] plane_stride,
    input  logic [DIM_WIDTH-1:0]  h_in,
    input  logic [DIM_WIDTH-1:0]  w_in,
    input  logic [DIM_WIDTH-1:0]  h_out,
    input  logic [DIM_WIDTH-1:0]  w_out,
    input  logic [CH_WIDTH-1:0]   c_in,
    input  logic [2:0]            k_size,
    input  logic [1:0]            stride,
    input  logic [1:0]            pad,

    // SRAM Interface
    output logic                  sram_rd_en,
    output logic [ADDR_WIDTH-1:0] sram_rd_addr,

    // Downstream Datapath Interface
    output logic                  valid_out,
    output logic                  zero_pad_out,
    output logic                  row_last
);

typedef enum logic [1:0] {
    S_IDLE = 2'b00,
    S_RUN  = 2'b01,
    S_DONE = 2'b10
} state_t;

state_t state_q, state_d;

logic [ADDR_WIDTH-1:0] c_offset_q, c_offset_d;
logic [ADDR_WIDTH-1:0] oy_offset_q, oy_offset_d;
logic [ADDR_WIDTH-1:0] ky_offset_q, ky_offset_d;

logic in_bounds; //for padding srtuff

logic [2:0] kx_q, ky_q; //q symbolise the output registers the current state
logic [CH_WIDTH - 1 : 0] c_q;
logic [DIM_WIDTH-1:0] ox_q,oy_q;

logic [2:0] kx_d, ky_d;
logic [CH_WIDTH - 1 : 0] c_d; //d symbolises the input registers the next state
logic [DIM_WIDTH-1:0] ox_d,oy_d;

logic kx_end, ky_end, c_end, ox_end, oy_end;
logic signed [DIM_WIDTH+1:0] ix, iy;
logic signed [DIM_WIDTH+1:0] ox_strided, oy_strided;

assign ox_strided = (stride == 2'd2) ? $signed({1'b0, ox_q, 1'b0}) : $signed({2'b00, ox_q});
assign oy_strided = (stride == 2'd2) ? $signed({1'b0, oy_q, 1'b0}) : $signed({2'b00, oy_q});

assign ix = ox_strided + $signed({9'd0, kx_q}) - $signed({10'd0, pad});
assign iy = oy_strided + $signed({9'd0, ky_q}) - $signed({10'd0, pad});

// Sign-extend 12-bit ix to 19-bit ADDR_WIDTH:
assign sram_rd_addr = base_addr + c_offset_q + oy_offset_q + ky_offset_q - pad_offset + {{7{ix[DIM_WIDTH+1]}}, ix};

logic [ADDR_WIDTH-1:0] pad_offset;

assign pad_offset = (pad == 2'd2) ? {w_in, 1'b0} :
                    (pad == 2'd1) ? {{1'b0}, w_in} : '0;
                    
assign kx_end = (kx_q == (k_size - 1'b1));
assign ky_end = (ky_q == (k_size - 1'b1)) && kx_end;
assign c_end  = (c_q  == (c_in   - 1'b1)) && ky_end;
assign ox_end = (ox_q == (w_out  - 1'b1)) && c_end;
assign oy_end = (oy_q == (h_out  - 1'b1)) && ox_end;


always_ff @(posedge clk) begin
    if (rst) begin
        state_q     <= S_IDLE;
        kx_q        <= '0;
        ky_q        <= '0;
        c_q         <= '0;
        ox_q        <= '0;
        oy_q        <= '0;
        c_offset_q  <= '0;
        ky_offset_q <= '0;
        oy_offset_q <= '0;
    end else begin
        state_q     <= state_d;
        kx_q        <= kx_d;
        ky_q        <= ky_d;
        c_q         <= c_d;
        ox_q        <= ox_d;
        oy_q        <= oy_d;
        c_offset_q  <= c_offset_d;
        ky_offset_q <= ky_offset_d;
        oy_offset_q <= oy_offset_d;
    end
end

assign zero_pad_out = (state_q == S_RUN) && !in_bounds;
assign sram_rd_en   = (state_q == S_RUN) &&  in_bounds;
assign busy      = (state_q == S_RUN);
assign done      = (state_q == S_DONE);
assign valid_out = (state_q == S_RUN);
assign row_last  = (state_q == S_RUN) && c_end; // Marks the end of length K (full dot-product)


assign in_bounds = (ix >= 0) && (ix < $signed({1'b0, w_in})) &&
                   (iy >= 0) && (iy < $signed({1'b0, h_in}));
                   
always_comb begin
    // 1. Default assignments (prevent latches)
    state_d     = state_q;
    kx_d        = kx_q;
    ky_d        = ky_q;
    c_d         = c_q;
    ox_d        = ox_q;
    oy_d        = oy_q;
    c_offset_d  = c_offset_q;
    ky_offset_d = ky_offset_q;
    oy_offset_d = oy_offset_q;

    case (state_q)
        S_IDLE: begin
            kx_d        = '0;
            ky_d        = '0;
            c_d         = '0;
            ox_d        = '0;
            oy_d        = '0;
            c_offset_d  = '0;
            ky_offset_d = '0;
            oy_offset_d = '0;

            if (start) begin
                state_d = S_RUN;
            end
        end

        S_RUN: begin
            // --- Counters Progression (kx -> ky -> c -> ox -> oy) ---
            kx_d = kx_end ? '0 : (kx_q + 1'b1);

            if (kx_end) begin
                ky_d = (ky_q == (k_size - 1'b1)) ? '0 : (ky_q + 1'b1);
            end

            if (ky_end) begin
                c_d = (c_q == (c_in - 1'b1)) ? '0 : (c_q + 1'b1);
            end

            if (c_end) begin
                ox_d = (ox_q == (w_out - 1'b1)) ? '0 : (ox_q + 1'b1);
            end

            if (ox_end) begin
                oy_d = (oy_q == (h_out - 1'b1)) ? '0 : (oy_q + 1'b1);
            end

            // --- Multiplier-Free Stride Accumulators ---
            // 1. Channel Accumulator (c_offset)
            if (c_end) begin
                c_offset_d = '0;
            end else if (ky_end) begin
                c_offset_d = c_offset_q + plane_stride;
            end

            // 2. Kernel Tap Row Accumulator (ky_offset)
            if (ky_end) begin
                ky_offset_d = '0;
            end else if (kx_end) begin
                ky_offset_d = ky_offset_q + {{(ADDR_WIDTH-DIM_WIDTH){1'b0}}, w_in};
            end

            // 3. Output Row Accumulator (oy_offset)
            if (oy_end) begin
                oy_offset_d = '0;
            end else if (ox_end) begin
                oy_offset_d = (stride == 2'd2) ? 
                              (oy_offset_q + {{(ADDR_WIDTH-DIM_WIDTH-1){1'b0}}, w_in, 1'b0}) :
                              (oy_offset_q + {{(ADDR_WIDTH-DIM_WIDTH){1'b0}}, w_in});
            end

            // --- Termination Check ---
            if (oy_end) begin
                state_d = S_DONE;
            end
        end

        S_DONE: begin
            state_d = S_IDLE;
        end

        default: begin
            state_d = S_IDLE;
        end
    endcase
end

endmodule

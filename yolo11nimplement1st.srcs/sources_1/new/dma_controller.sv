//------------------------------------------------------------------------------
// File       : dma_controller.sv
// Module     : dma_controller
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Module 5 Disjoint AXI4-HP Master DMA Engine.
//              Features:
//              1. Disjoint AXI4 Read Address (AR) & Read Data (R) engines.
//              2. Bitwise 4KB Boundary Burst Slicer (zero DSPs).
//              3. Zero-Comparator MSB Credit Guard + 64-Beat Elastic Queue.
//              4. Layered {Channel_Layer, Canvas_XY} Address Generator.
//              5. Set-Associative XOR + 1-Bit Shift 18-Bank Window Unpacker.
//              6. Disjoint AXI4 Write Master draining Module 4 (VPU) to DDR3.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module dma_controller #(
    parameter int AXI_ADDR_WIDTH   = 32,
    parameter int AXI_DATA_WIDTH   = 64,
    parameter int LOCAL_ADDR_WIDTH = 13,
    parameter int CANVAS_BITS      = 6,  // 2^6 = 64 spatial pixels per 8x8 plane
    parameter int CHAN_BITS        = 6   // 2^6 = 64 channel layers
)(
    input  wire                        clk,
    input  wire                        rst,

    //--------------------------------------------------------------------------
    // Control & Layer Transfer Descriptors
    //--------------------------------------------------------------------------
    input  wire                        dma_rd_start,       // Launch DDR -> SRAM preload
    input  wire [AXI_ADDR_WIDTH-1:0]   dma_rd_base_addr,   // DDR3 source base address
    input  wire [15:0]                 dma_rd_total_beats, // Total 64-bit beats to fetch
    input  wire                        cfg_xor_shift_en,   // 1 = XOR + 1-bit shift unpack
    output logic                       dma_rd_busy,
    output logic                       dma_rd_done,

    input  wire                        dma_wr_start,       // Arm VPU -> DDR writeback
    input  wire [AXI_ADDR_WIDTH-1:0]   dma_wr_base_addr,   // DDR3 destination base addr
    input  wire [15:0]                 dma_wr_total_beats, // Total 64-bit VPU vectors
    output logic                       dma_wr_busy,
    output logic                       dma_wr_done,

    //--------------------------------------------------------------------------
    // Module 3 (18-Bank SRAM Scratchpad) Write Interface
    //--------------------------------------------------------------------------
    output logic [17:0]                 sram_dma_wr_en,
    output logic [LOCAL_ADDR_WIDTH-1:0] sram_dma_wr_addr,
    output logic signed [17:0][7:0]     sram_dma_wr_data,

    //--------------------------------------------------------------------------
    // Module 4 (VPU) Elastic Drain Interface (8x INT8 = 64-bit Vector)
    //--------------------------------------------------------------------------
    input  wire                        vpu_req_in,
    output wire                        vpu_ack_out,
    input  wire signed [7:0][7:0]      vpu_data_in,

    //--------------------------------------------------------------------------
    // AXI4-HP Master Read Interface (DDR3 -> DMA)
    //--------------------------------------------------------------------------
    output logic [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,
    output logic [7:0]                 m_axi_arlen,
    output wire  [2:0]                 m_axi_arsize,
    output wire  [1:0]                 m_axi_arburst,
    output logic                       m_axi_arvalid,
    input  wire                        m_axi_arready,

    input  wire  [AXI_DATA_WIDTH-1:0]  m_axi_rdata,
    input  wire  [1:0]                 m_axi_rresp,
    input  wire                        m_axi_rlast,
    input  wire                        m_axi_rvalid,
    output wire                        m_axi_rready,

    //--------------------------------------------------------------------------
    // AXI4-HP Master Write Interface (DMA -> DDR3)
    //--------------------------------------------------------------------------
    output logic [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
    output logic [7:0]                 m_axi_awlen,
    output wire  [2:0]                 m_axi_awsize,
    output wire  [1:0]                 m_axi_awburst,
    output logic                       m_axi_awvalid,
    input  wire                        m_axi_awready,

    output wire  [AXI_DATA_WIDTH-1:0]  m_axi_wdata,
    output wire  [7:0]                 m_axi_wstrb,
    output wire                        m_axi_wlast,
    output wire                        m_axi_wvalid,
    input  wire                        m_axi_wready,

    input  wire  [1:0]                 m_axi_bresp,
    input  wire                        m_axi_bvalid,
    output wire                        m_axi_bready
);

    // Fixed AXI4 Burst Attributes: 8 bytes/beat (2^3), INCR burst type
    assign m_axi_arsize  = 3'b011;
    assign m_axi_arburst = 2'b01;
    assign m_axi_awsize  = 3'b011;
    assign m_axi_awburst = 2'b01;
    assign m_axi_wstrb   = 8'hFF;
    assign m_axi_bready  = 1'b1;

    //==========================================================================
    // PART 1: DISJOINT AXI4 READ ENGINE (DDR3 -> INGEST QUEUE -> 18-BANK SRAM)
    //==========================================================================

    // 64-Entry Elastic Ingest FIFO + Credit Counter
    logic [AXI_DATA_WIDTH-1:0] ing_fifo_mem [0:63];
    logic [5:0]                ing_wr_ptr, ing_rd_ptr;
    logic [6:0]                ing_fifo_cnt;
    logic [6:0]                ing_credit_cnt; // Tracks FIFO + in-flight AR beats

    wire ing_fifo_full  = ing_fifo_cnt[6]; // 64 entries full
    wire ing_fifo_empty = (ing_fifo_cnt == 7'd0);

    // Single-Wire Mid-Burst Backpressure to AXI Slave
    assign m_axi_rready = !ing_fifo_full;

    wire ing_push = m_axi_rvalid && m_axi_rready;
    wire ing_pop  = !ing_fifo_empty;

    // Zero-Comparator MSB Credit Check:
    // True when reserved credits < 32 (both bit 6 and bit 5 are 0)
    wire credit_ok = ~(ing_credit_cnt[6] | ing_credit_cnt[5]);

    //--------------------------------------------------------------------------
    // 1A. Disjoint Read Address Launcher & 4KB Boundary Slicer
    //--------------------------------------------------------------------------
    logic [AXI_ADDR_WIDTH-1:0] ar_curr_addr;
    logic [15:0]               ar_rem_beats;
    logic [15:0]               r_rem_beats;

    // Bitwise 4KB Boundary Distance Calculation:
    // Remaining bytes in current 4KB page = 4096 - addr[11:0]
    // Remaining 8-byte beats in 4KB page  = (4096 - addr[11:0]) >> 3
    wire [12:0] bytes_to_4k = 13'd4096 - {1'b0, ar_curr_addr[11:0]};
    wire [9:0]  beats_to_4k = bytes_to_4k[12:3];

    // Cap burst size to min(ar_rem_beats, beats_to_4k, 32)
    logic [8:0] next_burst_beats;
    always_comb begin
        next_burst_beats = 9'd32;
        if ({7'd0, beats_to_4k} < 16'd32) begin
            next_burst_beats = beats_to_4k[8:0];
        end
        if (ar_rem_beats < {7'd0, next_burst_beats}) begin
            next_burst_beats = ar_rem_beats[8:0];
        end
    end

    wire ar_launch_fire = m_axi_arvalid && m_axi_arready;

    always_ff @(posedge clk) begin
        if (rst) begin
            ar_curr_addr  <= '0;
            ar_rem_beats  <= '0;
            m_axi_arvalid <= 1'b0;
            m_axi_araddr  <= '0;
            m_axi_arlen   <= '0;
        end else begin
            if (dma_rd_start && !dma_rd_busy) begin
                ar_curr_addr  <= dma_rd_base_addr;
                ar_rem_beats  <= dma_rd_total_beats;
                m_axi_arvalid <= 1'b0;
            end else if (m_axi_arvalid) begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                end
            end else if (ar_rem_beats > 16'd0 && credit_ok) begin
                m_axi_arvalid <= 1'b1;
                m_axi_araddr  <= ar_curr_addr;
                m_axi_arlen   <= 8'(next_burst_beats - 9'd1);
                ar_curr_addr  <= ar_curr_addr + ({23'd0, next_burst_beats} << 3);
                ar_rem_beats  <= ar_rem_beats - {7'd0, next_burst_beats};
            end
        end
    end

    //--------------------------------------------------------------------------
    // 1B. Elastic Ingest Queue & Credit Counter
    //--------------------------------------------------------------------------
    wire [6:0] ar_beats_added = ( (!m_axi_arvalid) && (ar_rem_beats > 16'd0) && credit_ok && !(dma_rd_start && !dma_rd_busy) )
                                ? next_burst_beats[6:0] : 7'd0;

    always_ff @(posedge clk) begin
        if (rst) begin
            ing_wr_ptr     <= '0;
            ing_rd_ptr     <= '0;
            ing_fifo_cnt   <= '0;
            ing_credit_cnt <= '0;
        end else begin
            if (ing_push) begin
                ing_fifo_mem[ing_wr_ptr] <= m_axi_rdata;
                ing_wr_ptr               <= ing_wr_ptr + 6'd1;
            end
            if (ing_pop) begin
                ing_rd_ptr <= ing_rd_ptr + 6'd1;
            end

            case ({ing_push, ing_pop})
                2'b10:   ing_fifo_cnt <= ing_fifo_cnt + 7'd1;
                2'b01:   ing_fifo_cnt <= ing_fifo_cnt - 7'd1;
                default: ing_fifo_cnt <= ing_fifo_cnt;
            endcase

            // Credit counter adds burst beats on AR issue and subtracts on FIFO pop
            ing_credit_cnt <= ing_credit_cnt + ar_beats_added - {6'd0, ing_pop};
        end
    end

    //--------------------------------------------------------------------------
    // 1C. Layered {Channel_Layer, Canvas_XY} Address & XOR+Shift Unpacker
    //--------------------------------------------------------------------------
    logic [CANVAS_BITS-1:0] wr_canvas_idx; // 0..63 (8x8 spatial plane)
    logic [CHAN_BITS-1:0]   wr_chan_layer; // 0..63 (64 channels)

    wire [AXI_DATA_WIDTH-1:0] pop_word = ing_fifo_mem[ing_rd_ptr];

    // Extract 9 Pool-A bytes using XOR swizzle for the 9th spatial tap
    logic signed [8:0][7:0] unpacked_pool_a;
    logic signed [8:0][7:0] unpacked_pool_b;

    always_comb begin
        for (int t = 0; t < 8; t++) begin
            unpacked_pool_a[t] = pop_word[t*8 +: 8];
        end
        // 9th spatial tap (index 8) formed via XOR of boundary lanes (1 ^ 8 = 9)
        unpacked_pool_a[8] = cfg_xor_shift_en ? (pop_word[7:0] ^ pop_word[63:56])
                                              : pop_word[7:0];

        // Pool B (Banks 9..17) formed via 1-bit left barrel shift (<< 1)
        for (int t = 0; t < 9; t++) begin
            unpacked_pool_b[t] = cfg_xor_shift_en ? (unpacked_pool_a[t] <<< 1)
                                                  : unpacked_pool_a[t];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_canvas_idx    <= '0;
            wr_chan_layer    <= '0;
            r_rem_beats      <= '0;
            dma_rd_busy      <= 1'b0;
            dma_rd_done      <= 1'b0;
            sram_dma_wr_en   <= '0;
            sram_dma_wr_addr <= '0;
            sram_dma_wr_data <= '0;
        end else begin
            dma_rd_done <= 1'b0;

            if (dma_rd_start && !dma_rd_busy) begin
                wr_canvas_idx  <= '0;
                wr_chan_layer  <= '0;
                r_rem_beats    <= dma_rd_total_beats;
                dma_rd_busy    <= 1'b1;
                sram_dma_wr_en <= '0;
            end else if (ing_pop) begin
                // Drive all 18 BRAM sub-banks with the unpacked window slice
                sram_dma_wr_en   <= 18'h3FFFF;
                sram_dma_wr_addr <= {{(LOCAL_ADDR_WIDTH - CHAN_BITS - CANVAS_BITS){1'b0}},
                                     wr_chan_layer, wr_canvas_idx};

                for (int t = 0; t < 9; t++) begin
                    sram_dma_wr_data[t]     <= unpacked_pool_a[t];
                    sram_dma_wr_data[t + 9] <= unpacked_pool_b[t];
                end

                // Step Canvas coordinate first, then Channel Layer on wrap-around
                if (wr_canvas_idx == {CANVAS_BITS{1'b1}}) begin
                    wr_canvas_idx <= '0;
                    wr_chan_layer <= wr_chan_layer + 1'b1;
                end else begin
                    wr_canvas_idx <= wr_canvas_idx + 1'b1;
                end

                if (r_rem_beats == 16'd1) begin
                    r_rem_beats <= 16'd0;
                    dma_rd_busy <= 1'b0;
                    dma_rd_done <= 1'b1;
                end else begin
                    r_rem_beats <= r_rem_beats - 16'd1;
                end
            end else begin
                sram_dma_wr_en <= '0;
            end
        end
    end

    //==========================================================================
    // PART 2: DISJOINT AXI4 WRITE ENGINE (MODULE 4 VPU -> DRAIN QUEUE -> DDR3)
    //==========================================================================
    logic [AXI_DATA_WIDTH-1:0] drn_fifo_mem [0:31];
    logic [4:0]                drn_wr_ptr, drn_rd_ptr;
    logic [5:0]                drn_fifo_cnt;

    wire drn_fifo_full  = drn_fifo_cnt[5];
    wire drn_fifo_empty = (drn_fifo_cnt == 6'd0);

    assign vpu_ack_out = !drn_fifo_full && dma_wr_busy;

    wire drn_push = vpu_req_in && vpu_ack_out;
    wire drn_pop  = m_axi_wvalid && m_axi_wready;

    logic [AXI_ADDR_WIDTH-1:0] aw_curr_addr;
    logic [15:0]               aw_rem_beats;
    logic                      aw_issued_q;
    logic                      w_active_q;

    always_ff @(posedge clk) begin
        if (rst) begin
            drn_wr_ptr    <= '0;
            drn_rd_ptr    <= '0;
            drn_fifo_cnt  <= '0;
            aw_curr_addr  <= '0;
            aw_rem_beats  <= '0;
            aw_issued_q   <= 1'b0;
            w_active_q    <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_awaddr  <= '0;
            m_axi_awlen   <= '0;
            dma_wr_busy   <= 1'b0;
            dma_wr_done   <= 1'b0;
        end else begin
            dma_wr_done <= 1'b0;

            if (dma_wr_start && !dma_wr_busy) begin
                aw_curr_addr  <= dma_wr_base_addr;
                aw_rem_beats  <= dma_wr_total_beats;
                dma_wr_busy   <= 1'b1;
                aw_issued_q   <= 1'b0;
                w_active_q    <= 1'b0;
                m_axi_awvalid <= 1'b0;
            end else if (dma_wr_busy) begin
                // Launch single-beat or burst AW request whenever a VPU packet is queued
                if (!aw_issued_q && !w_active_q && !drn_fifo_empty && (aw_rem_beats > 16'd0)) begin
                    m_axi_awvalid <= 1'b1;
                    m_axi_awaddr  <= aw_curr_addr;
                    m_axi_awlen   <= 8'd0; // 1-beat low-latency writeback per pixel
                    aw_issued_q   <= 1'b1;
                    w_active_q    <= 1'b1;
                end

                if (m_axi_awvalid && m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                end

                if (drn_pop) begin
                    w_active_q   <= 1'b0;
                    aw_curr_addr <= aw_curr_addr + 32'd8;
                    aw_rem_beats <= aw_rem_beats - 16'd1;
                end

                if (m_axi_bvalid && m_axi_bready) begin
                    aw_issued_q <= 1'b0;
                    if (aw_rem_beats == 16'd0 && !w_active_q) begin
                        dma_wr_busy <= 1'b0;
                        dma_wr_done <= 1'b1;
                    end
                end
            end

            // Drain Queue Pointers
            if (drn_push) begin
                drn_fifo_mem[drn_wr_ptr] <= vpu_data_in;
                drn_wr_ptr               <= drn_wr_ptr + 5'd1;
            end
            if (drn_pop) begin
                drn_rd_ptr <= drn_rd_ptr + 5'd1;
            end

            case ({drn_push, drn_pop})
                2'b10:   drn_fifo_cnt <= drn_fifo_cnt + 6'd1;
                2'b01:   drn_fifo_cnt <= drn_fifo_cnt - 6'd1;
                default: drn_fifo_cnt <= drn_fifo_cnt;
            endcase
        end
    end

    assign m_axi_wvalid = w_active_q && !drn_fifo_empty;
    assign m_axi_wdata  = drn_fifo_mem[drn_rd_ptr];
    assign m_axi_wlast  = m_axi_wvalid;

endmodule
`default_nettype wire
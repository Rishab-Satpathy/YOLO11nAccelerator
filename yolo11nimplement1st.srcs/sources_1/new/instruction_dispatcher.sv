//------------------------------------------------------------------------------
// File       : instruction_dispatcher.sv
// Module     : instruction_dispatcher
// Project    : YOLO11n Custom INT8 Accelerator (Zynq-7020 / Zynq UltraScale+)
// Description: Module 6 Command Processor & Layer Descriptor Queue.
//              Accepts memory-mapped AXI4-Lite configuration packets from the
//              ARM CPU, stages multi-layer parameters, and broadcasts them
//              to Modules 1, 4, and 5 in a single clock cycle.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module instruction_dispatcher #(
    parameter int AXI_ADDR_WIDTH = 32,
    parameter int AXI_DATA_WIDTH = 32,
    parameter int NUM_COLS       = 8
)(
    input  wire                               clk,
    input  wire                               rst,

    //--------------------------------------------------------------------------
    // AXI4-Lite Slave Interface (From ARM Processing System)
    //--------------------------------------------------------------------------
    input  wire [AXI_ADDR_WIDTH-1:0]          s_axi_awaddr,
    input  wire                               s_axi_awvalid,
    output logic                              s_axi_awready,

    input  wire [AXI_DATA_WIDTH-1:0]          s_axi_wdata,
    input  wire [3:0]                         s_axi_wstrb,
    input  wire                               s_axi_wvalid,
    output logic                              s_axi_wready,

    output logic [1:0]                        s_axi_bresp,
    output logic                              s_axi_bvalid,
    input  wire                               s_axi_bready,

    input  wire [AXI_ADDR_WIDTH-1:0]          s_axi_araddr,
    input  wire                               s_axi_arvalid,
    output logic                              s_axi_arready,

    output logic [AXI_DATA_WIDTH-1:0]         s_axi_rdata,
    output logic [1:0]                        s_axi_rresp,
    output logic                              s_axi_rvalid,
    input  wire                               s_axi_rready,

    //--------------------------------------------------------------------------
    // Layer Transition & Barrier Tokens (To/From Module 7)
    //--------------------------------------------------------------------------
    input  wire                               layer_complete_pulse,
    output logic                              accel_busy,
    output logic                              accel_irq,

    //--------------------------------------------------------------------------
    // Broadcast Layer Configuration Bus (To Modules 1, 4, 5)
    //--------------------------------------------------------------------------
    output logic                              cfg_layer_start_pulse,
    output logic [31:0]                       cfg_dma_src_addr,
    output logic [31:0]                       cfg_dma_dst_addr,
    output logic [15:0]                       cfg_dma_beats,
    output logic [18:0]                       cfg_plane_stride,
    output logic [9:0]                        cfg_h_in,
    output logic [9:0]                        cfg_w_in,
    output logic [9:0]                        cfg_h_out,
    output logic [9:0]                        cfg_w_out,
    output logic [9:0]                        cfg_c_in,
    output logic [1:0]                        cfg_stride,
    output logic [1:0]                        cfg_pad,
    output logic signed [NUM_COLS-1:0][31:0]  cfg_bias,
    output logic signed [NUM_COLS-1:0][15:0]  cfg_mult,
    output logic [NUM_COLS-1:0][4:0]          cfg_shift,
    output logic signed [NUM_COLS-1:0][7:0]   cfg_zero_point
);

    // Register Map Offsets
    localparam [7:0] REG_CONTROL    = 8'h00; // [0] = Run/Start, [1] = Soft Reset
    localparam [7:0] REG_STATUS     = 8'h04; // [0] = Busy, [1] = Queue Full, [2] = Done
    localparam [7:0] REG_SRC_ADDR   = 8'h10;
    localparam [7:0] REG_DST_ADDR   = 8'h14;
    localparam [7:0] REG_BEATS      = 8'h18;
    localparam [7:0] REG_GEO_IN     = 8'h20; // [9:0] = H_in, [25:16] = W_in
    localparam [7:0] REG_GEO_OUT    = 8'h24; // [9:0] = H_out, [25:16] = W_out
    localparam [7:0] REG_CHANNELS   = 8'h28; // [9:0] = C_in, [13:12] = Stride, [15:14] = Pad
    localparam [7:0] REG_QUANT_CFG  = 8'h30; // [15:0] = Mult, [20:16] = Shift, [31:24] = ZP
    localparam [7:0] REG_BIAS_BASE  = 8'h40; // 0x40..0x5C: 8x 32-bit Column Biases

    // Staging Registers Written via AXI-Lite
    logic [31:0] reg_src_addr_q;
    logic [31:0] reg_dst_addr_q;
    logic [15:0] reg_beats_q;
    logic [9:0]  reg_h_in_q, reg_w_in_q;
    logic [9:0]  reg_h_out_q, reg_w_out_q;
    logic [9:0]  reg_c_in_q;
    logic [1:0]  reg_stride_q, reg_pad_q;
    logic signed [15:0] reg_mult_q;
    logic [4:0]  reg_shift_q;
    logic signed [7:0] reg_zp_q;
    logic signed [NUM_COLS-1:0][31:0] reg_bias_q;

    // AXI-Lite Write FSM
    always_ff @(posedge clk) begin
        if (rst) begin
            s_axi_awready         <= 1'b0;
            s_axi_wready          <= 1'b0;
            s_axi_bvalid          <= 1'b0;
            s_axi_bresp           <= 2'b00;
            cfg_layer_start_pulse <= 1'b0;
            accel_busy            <= 1'b0;
            accel_irq             <= 1'b0;
            reg_src_addr_q        <= '0;
            reg_dst_addr_q        <= '0;
            reg_beats_q           <= '0;
            reg_h_in_q            <= 10'd8;
            reg_w_in_q            <= 10'd8;
            reg_h_out_q           <= 10'd4;
            reg_w_out_q           <= 10'd4;
            reg_c_in_q            <= 10'd64;
            reg_stride_q          <= 2'd2;
            reg_pad_q             <= 2'd1;
            reg_mult_q            <= 16'sd1;
            reg_shift_q           <= 5'd8;
            reg_zp_q              <= 8'sd0;
            reg_bias_q            <= '0;
        end else begin
            cfg_layer_start_pulse <= 1'b0;

            if (layer_complete_pulse) begin
                accel_busy <= 1'b0;
                accel_irq  <= 1'b1;
            end

            // AXI-Lite Handshake
            if (!s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                s_axi_bvalid  <= 1'b1;

                case (s_axi_awaddr[7:0])
                    REG_CONTROL: begin
                        if (s_axi_wdata[0]) begin // Start Command
                            cfg_layer_start_pulse <= 1'b1;
                            accel_busy            <= 1'b1;
                            accel_irq             <= 1'b0;
                        end
                    end
                    REG_SRC_ADDR:  reg_src_addr_q <= s_axi_wdata;
                    REG_DST_ADDR:  reg_dst_addr_q <= s_axi_wdata;
                    REG_BEATS:     reg_beats_q    <= s_axi_wdata[15:0];
                    REG_GEO_IN:    begin
                        reg_h_in_q <= s_axi_wdata[9:0];
                        reg_w_in_q <= s_axi_wdata[25:16];
                    end
                    REG_GEO_OUT:   begin
                        reg_h_out_q <= s_axi_wdata[9:0];
                        reg_w_out_q <= s_axi_wdata[25:16];
                    end
                    REG_CHANNELS:  begin
                        reg_c_in_q   <= s_axi_wdata[9:0];
                        reg_stride_q <= s_axi_wdata[13:12];
                        reg_pad_q    <= s_axi_wdata[15:14];
                    end
                    REG_QUANT_CFG: begin
                        reg_mult_q  <= s_axi_wdata[15:0];
                        reg_shift_q <= s_axi_wdata[20:16];
                        reg_zp_q    <= s_axi_wdata[31:24];
                    end
                    default: begin
                        if (s_axi_awaddr[7:0] >= REG_BIAS_BASE && s_axi_awaddr[7:0] <= (REG_BIAS_BASE + 8'h1C)) begin
                            int col_idx;
                            col_idx = (s_axi_awaddr[7:0] - REG_BIAS_BASE) >> 2;
                            if (col_idx < NUM_COLS) begin
                                reg_bias_q[col_idx] <= s_axi_wdata;
                            end
                        end
                    end
                endcase
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bvalid && s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                end
            end
        end
    end

    // AXI-Lite Read Responder
    always_ff @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= '0;
            s_axi_rresp   <= 2'b00;
        end else begin
            if (!s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[7:0])
                    REG_STATUS:  s_axi_rdata <= {29'd0, accel_irq, 1'b0, accel_busy};
                    REG_SRC_ADDR: s_axi_rdata <= reg_src_addr_q;
                    REG_DST_ADDR: s_axi_rdata <= reg_dst_addr_q;
                    default:     s_axi_rdata <= 32'hDEAD_BEEF;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid && s_axi_rready) begin
                    s_axi_rvalid <= 1'b0;
                end
            end
        end
    end

    // Broadcast Active Layer Parameters
    assign cfg_dma_src_addr = reg_src_addr_q;
    assign cfg_dma_dst_addr = reg_dst_addr_q;
    assign cfg_dma_beats    = reg_beats_q;
    assign cfg_plane_stride = 19'(reg_h_in_q * reg_w_in_q);
    assign cfg_h_in         = reg_h_in_q;
    assign cfg_w_in         = reg_w_in_q;
    assign cfg_h_out        = reg_h_out_q;
    assign cfg_w_out        = reg_w_out_q;
    assign cfg_c_in         = reg_c_in_q;
    assign cfg_stride       = reg_stride_q;
    assign cfg_pad          = reg_pad_q;
    assign cfg_bias         = reg_bias_q;

    genvar c;
    generate
        for (c = 0; c < NUM_COLS; c++) begin : gen_quant_broadcast
            assign cfg_mult[c]       = reg_mult_q;
            assign cfg_shift[c]      = reg_shift_q;
            assign cfg_zero_point[c] = reg_zp_q;
        end
    endgenerate

endmodule
`default_nettype wire
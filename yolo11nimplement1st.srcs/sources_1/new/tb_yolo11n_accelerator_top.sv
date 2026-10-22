//------------------------------------------------------------------------------
// File       : tb_yolo11n_accelerator_top.sv
// Description: Complete Bit-Exact Co-Simulation Testbench for YOLO11n INT8
//              Hardware Accelerator (Integrated 7-Module Architecture).
//              - Preloads real PyTorch INT8 weights via $readmemh into ring buffers.
//              - Preloads input activation tensor into simulated DDR3 RAM.
//              - Dispatches layer configuration descriptors via AXI4-Lite.
//              - Captures live AGU, MXU, and VPU execution telemetry.
//              - Captures 16 64-bit DDR3 writeback beats to 'hw_output.hex'.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module tb_yolo11n_accelerator_top;

    localparam int AXI_ADDR_WIDTH = 32;
    localparam int AXI_DATA_WIDTH = 64;
    localparam int NUM_COLS       = 8;
    localparam int WIN_TAPS       = 9;

    // Clock and Reset Signals
    logic clk;
    logic rst;
    logic sim_done;
    logic accel_irq;

    // AXI4-Lite Control Signals (From CPU Master)
    logic [AXI_ADDR_WIDTH-1:0] s_axi_awaddr;
    logic                      s_axi_awvalid;
    logic                      s_axi_awready;
    logic [31:0]               s_axi_wdata;
    logic [3:0]                s_axi_wstrb;
    logic                      s_axi_wvalid;
    logic                      s_axi_wready;
    logic [1:0]                s_axi_bresp;
    logic                      s_axi_bvalid;
    logic                      s_axi_bready;
    logic [AXI_ADDR_WIDTH-1:0] s_axi_araddr;
    logic                      s_axi_arvalid;
    logic                      s_axi_arready;
    logic [31:0]               s_axi_rdata;
    logic [1:0]                s_axi_rresp;
    logic                      s_axi_rvalid;
    logic                      s_axi_rready;

    // AXI4-HP Master Bus (To Simulated DDR3 Memory)
    logic [AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    logic [7:0]                m_axi_arlen;
    logic [2:0]                m_axi_arsize;
    logic [1:0]                m_axi_arburst;
    logic                      m_axi_arvalid;
    logic                      m_axi_arready;
    logic [AXI_DATA_WIDTH-1:0] m_axi_rdata;
    logic [1:0]                m_axi_rresp;
    logic                      m_axi_rlast;
    logic                      m_axi_rvalid;
    logic                      m_axi_rready;

    logic [AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    logic [7:0]                m_axi_awlen;
    logic [2:0]                m_axi_awsize;
    logic [1:0]                m_axi_awburst;
    logic                      m_axi_awvalid;
    logic                      m_axi_awready;
    logic [AXI_DATA_WIDTH-1:0] m_axi_wdata;
    logic [7:0]                m_axi_wstrb;
    logic                      m_axi_wlast;
    logic                      m_axi_wvalid;
    logic                      m_axi_wready;
    logic [1:0]                m_axi_bresp;
    logic                      m_axi_bvalid;
    logic                      m_axi_bready;

    // Direct Weight Ring Preload Interface
    logic [NUM_COLS-1:0]       weight_wr_en;
    logic [4:0]                weight_wr_addr;
    logic signed [17:0][7:0]   weight_wr_data;

    int ddr_wr_count;

    //--------------------------------------------------------------------------
    // DUT Instantiation
    //--------------------------------------------------------------------------
    yolo11n_accelerator_top #(
        .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
        .NUM_COLS       (NUM_COLS),
        .WIN_TAPS       (WIN_TAPS)
    ) dut (.*);

    // 125 MHz Differential Clock Generation (8.0 ns Period)
    initial begin
        clk      = 1'b0;
        sim_done = 1'b0;
    end
    always #4.0 if (!sim_done) clk = ~clk;

    //--------------------------------------------------------------------------
    // Simulated DDR3 Memory Slave Model
    //--------------------------------------------------------------------------
logic [AXI_DATA_WIDTH-1:0] ddr3_input_ram [0:4095];
    logic [AXI_DATA_WIDTH-1:0] ddr3_out_buffer [0:15];
    logic [7:0]                axi_rd_beats_left;
    logic                      axi_rd_active;
    logic [11:0]               axi_rd_idx;      // Proper beat index (0..4095)
    logic [3:0]                ddr_wr_idx;

    initial begin
        for (int i = 0; i < 4096; i++) ddr3_input_ram[i] = 64'd0;
        for (int i = 0; i < 16; i++)   ddr3_out_buffer[i] = 64'd0;
        $readmemh("C:/Users/risha/OneDrive/Documents/mdmSirResearch/input_image.hex", ddr3_input_ram);
    end

    assign m_axi_arready = !axi_rd_active;
    assign m_axi_rresp   = 2'b00;
    assign m_axi_rdata   = ddr3_input_ram[axi_rd_idx];
    assign m_axi_rvalid  = axi_rd_active;
    assign m_axi_rlast   = axi_rd_active && (axi_rd_beats_left == 8'd0);

    assign m_axi_awready = 1'b1;
    assign m_axi_wready  = 1'b1;
    assign m_axi_bresp   = 2'b00;

    always_ff @(posedge clk) begin
        if (rst) begin
            axi_rd_active     <= 1'b0;
            axi_rd_beats_left <= 8'd0;
            axi_rd_idx        <= 12'd0;
            m_axi_bvalid      <= 1'b0;
            ddr_wr_idx        <= 4'd0;
            ddr_wr_count      <= 0;
        end else begin
            // AXI Read Engine (Properly increments axi_rd_idx on every R-channel beat!)
            if (!axi_rd_active) begin
                if (m_axi_arvalid && m_axi_arready) begin
                    axi_rd_active     <= 1'b1;
                    axi_rd_beats_left <= m_axi_arlen;
                    axi_rd_idx        <= m_axi_araddr[14:3];
                end
            end else if (m_axi_rvalid && m_axi_rready) begin
                axi_rd_idx <= axi_rd_idx + 12'd1;
                if (axi_rd_beats_left == 8'd0) begin
                    axi_rd_active <= 1'b0;
                end else begin
                    axi_rd_beats_left <= axi_rd_beats_left - 8'd1;
                end
            end

            // AXI Write Engine (DDR Writeback Capture)
            if (m_axi_wvalid && m_axi_wready) begin
                ddr3_out_buffer[ddr_wr_idx] <= m_axi_wdata;
                $display("[TIME %0t ns] [DDR3 WR] Pixel %02d | Data=%016h | Col 0..7=[%0d, %0d, %0d, %0d, %0d, %0d, %0d, %0d]",
                    $time, ddr_wr_idx, m_axi_wdata,
                    $signed(m_axi_wdata[7:0]),   $signed(m_axi_wdata[15:8]),
                    $signed(m_axi_wdata[23:16]), $signed(m_axi_wdata[31:24]),
                    $signed(m_axi_wdata[39:32]), $signed(m_axi_wdata[47:40]),
                    $signed(m_axi_wdata[55:48]), $signed(m_axi_wdata[63:56])
                );
                ddr_wr_idx   <= ddr_wr_idx + 4'd1;
                ddr_wr_count <= ddr_wr_count + 1;
                m_axi_bvalid <= 1'b1;
            end else if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // AXI-Lite Single Master Write Task
    //--------------------------------------------------------------------------
    task automatic cpu_axi_lite_write(input logic [31:0] addr, input logic [31:0] data);
        begin
            @(posedge clk);
            s_axi_awaddr  <= addr;
            s_axi_awvalid <= 1'b1;
            s_axi_wdata   <= data;
            s_axi_wstrb   <= 4'hF;
            s_axi_wvalid  <= 1'b1;
            s_axi_bready  <= 1'b1;

            wait (s_axi_awready && s_axi_wready);
            @(posedge clk);
            s_axi_awvalid <= 1'b0;
            s_axi_wvalid  <= 1'b0;

            wait (s_axi_bvalid);
            @(posedge clk);
            s_axi_bready  <= 1'b0;
        end
    endtask

    // Weight File Storage Array
    logic [143:0] weight_file_mem [0:255];
    initial begin
        for (int i = 0; i < 256; i++) weight_file_mem[i] = 144'd0;
        $readmemh("C:/Users/risha/OneDrive/Documents/mdmSirResearch/weights_layer.hex", weight_file_mem);
    end

    //--------------------------------------------------------------------------
    // Cycle-Aligned MXU Bundle Dump (All 32 Beats of P02 & P03) + SRAM Shadow
    //--------------------------------------------------------------------------
    int mxu_pix;
    int mxu_beat;
    int bundle_fd;
    logic signed [17:0][7:0] sram_shadow [0:1][0:4095]; // [buf_sel][addr][bank]

    initial begin
        bundle_fd = $fopen("C:/Users/risha/OneDrive/Documents/mdmSirResearch/p02_p03_bundle_dump.txt", "w");
        for (int b = 0; b < 2; b++)
            for (int a = 0; a < 4096; a++)
                sram_shadow[b][a] = '0;
    end

    // Dump active SRAM buffer contents to file on compute start pulse
    always_ff @(posedge clk) begin
        if (!rst) begin
            // Record every DMA write into the 18 SRAM banks
            for (int bank = 0; bank < 18; bank++) begin
                if (dut.u_datapath.u_module3_sram.dma_wr_en[bank]) begin
                    sram_shadow[dut.u_datapath.u_module3_sram.dma_buf_sel]
                               [dut.u_datapath.u_module3_sram.dma_wr_addr[11:0]]
                               [bank] <= dut.u_datapath.u_module3_sram.dma_wr_data[bank];
                end
            end

            // When compute starts, dump first 64 spatial addresses of Channel 0 & 1 across all 18 banks
            if (dut.u_datapath.start) begin
                int sram_fd;
                logic c_buf;
                c_buf = dut.u_datapath.u_module3_sram.compute_buf_sel;
                sram_fd = $fopen("C:/Users/risha/OneDrive/Documents/mdmSirResearch/sram_banks_dump.txt", "w");
                $fdisplay(sram_fd, "=== SRAM SHADOW DUMP (compute_buf_sel = %0d) ===", c_buf);
                for (int addr = 0; addr < 128; addr++) begin
                    $fdisplay(sram_fd, "Addr %04d (ch=%02d, y=%0d, x=%0d) | PoolA[0..8]=[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d] | PoolB[9..17]=[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d]",
                        addr, addr >> 6, (addr & 63) >> 3, addr & 7,
                        $signed(sram_shadow[c_buf][addr][0]),  $signed(sram_shadow[c_buf][addr][1]),  $signed(sram_shadow[c_buf][addr][2]),
                        $signed(sram_shadow[c_buf][addr][3]),  $signed(sram_shadow[c_buf][addr][4]),  $signed(sram_shadow[c_buf][addr][5]),
                        $signed(sram_shadow[c_buf][addr][6]),  $signed(sram_shadow[c_buf][addr][7]),  $signed(sram_shadow[c_buf][addr][8]),
                        $signed(sram_shadow[c_buf][addr][9]),  $signed(sram_shadow[c_buf][addr][10]), $signed(sram_shadow[c_buf][addr][11]),
                        $signed(sram_shadow[c_buf][addr][12]), $signed(sram_shadow[c_buf][addr][13]), $signed(sram_shadow[c_buf][addr][14]),
                        $signed(sram_shadow[c_buf][addr][15]), $signed(sram_shadow[c_buf][addr][16]), $signed(sram_shadow[c_buf][addr][17])
                    );
                end
                $fclose(sram_fd);
                $display("[TIME %0t] >> Dumped SRAM bank contents to sram_banks_dump.txt", $time);
            end
        end
    end

    // MXU-side beat & pixel tracking (100% aligned with act_masked_bus and col_weights)
    always_ff @(posedge clk) begin
        if (rst) begin
            mxu_pix  <= 0;
            mxu_beat <= 0;
        end else begin
            if (dut.u_datapath.u_module2_mxu.step_active) begin
                // Dump all 32 beats for P02 (mxu_pix == 2) and P03 (mxu_pix == 3)
                if (mxu_pix == 2 || mxu_pix == 3) begin
                    $fdisplay(bundle_fd, "P%02d Beat %02d | row_last=%b | pad_s0=%09b pad_s1=%09b | act_s0=[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d] | act_s1=[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d] | w_col0_s0=[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d]",
                        mxu_pix, mxu_beat,
                        dut.u_datapath.u_module2_mxu.agu_row_last_in,
                        dut.u_datapath.u_module2_mxu.pad_mask_s0_in,
                        dut.u_datapath.u_module2_mxu.pad_mask_s1_in,
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[0]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[1]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[2]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[3]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[4]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[5]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[6]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[7]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[8]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[9]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[10]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[11]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[12]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[13]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[14]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[15]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[16]),
                        $signed(dut.u_datapath.u_module2_mxu.act_masked_bus[17]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][0]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][1]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][2]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][3]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][4]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][5]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][6]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][7]),
                        $signed(dut.u_datapath.u_module2_mxu.col_weights[0][8])
                    );
                    if (mxu_pix == 3 && dut.u_datapath.u_module2_mxu.agu_row_last_in) begin
                        $fflush(bundle_fd);
                    end
                end

                if (dut.u_datapath.u_module2_mxu.agu_row_last_in) begin
                    mxu_beat <= 0;
                    mxu_pix  <= mxu_pix + 1;
                end else begin
                    mxu_beat <= mxu_beat + 1;
                end
            end

            if (dut.u_datapath.u_module2_mxu.mxu_valid_out) begin
                $display("[TIME %0t] [MXU PSum] Vector Data: [%0d, %0d, %0d, %0d, %0d, %0d, %0d, %0d]",
                    $time,
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[0]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[1]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[2]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[3]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[4]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[5]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[6]),
                    $signed(dut.u_datapath.u_module2_mxu.mxu_data_out[7])
                );
            end
        end
    end
    //--------------------------------------------------------------------------
    // Main Stimulus Orchestrator
    //--------------------------------------------------------------------------
    initial begin
        rst            = 1'b1;
        s_axi_awaddr   = '0;
        s_axi_awvalid  = 1'b0;
        s_axi_wdata    = '0;
        s_axi_wstrb    = '0;
        s_axi_wvalid   = 1'b0;
        s_axi_bready   = 1'b0;
        s_axi_araddr   = '0;
        s_axi_arvalid  = 1'b0;
        s_axi_rready   = 1'b0;
        weight_wr_en   = '0;
        weight_wr_addr = '0;
        weight_wr_data = '0;

        #40;
        @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Preload weights into local LUTRAM ring buffers
        $display("[TIME %0t ns] Preloading Real YOLO11n Weights...", $time);
        for (int c = 0; c < NUM_COLS; c++) begin
            for (int addr = 0; addr < 32; addr++) begin
                @(posedge clk);
                weight_wr_en   <= (1'b1 << c);
                weight_wr_addr <= addr[4:0];
                weight_wr_data <= weight_file_mem[c * 32 + addr];
            end
        end
        @(posedge clk);
        weight_wr_en <= '0;

        // CPU writes Layer Descriptors via AXI-Lite
        $display("[TIME %0t ns] CPU writing Layer Descriptors via AXI-Lite...", $time);
        cpu_axi_lite_write(32'h10, 32'h1000_0000);                // SRC ADDR
        cpu_axi_lite_write(32'h14, 32'h2000_0000);                // DST ADDR
        cpu_axi_lite_write(32'h18, 32'd4096);                     // BEATS = 4096
        cpu_axi_lite_write(32'h20, 32'h0008_0008);                // W_in=8 [25:16], H_in=8 [9:0]
        cpu_axi_lite_write(32'h24, 32'h0004_0004);                // W_out=4 [25:16], H_out=4 [9:0]
        cpu_axi_lite_write(32'h28, 32'h0000_6040);                // Pad=1 [15:14], Stride=2 [13:12], Cin=64 [9:0]
        cpu_axi_lite_write(32'h30, 32'h0006_0001);                // ZP=0 [31:24], Shift=6 [20:16], Mult=1 [15:0]

        // Clear channel biases
        for (int c = 0; c < 8; c++) begin
            cpu_axi_lite_write(32'h40 + (c * 4), 32'sd0);
        end

        // Launch Inference
        $display("[TIME %0t ns] Launching Hardware Inference (REG_CONTROL = 1)...", $time);
        cpu_axi_lite_write(32'h00, 32'h0000_0001);

        // Wait for accelerator completion
        wait (accel_irq == 1'b1);

        $display("[TIME %0t ns] >> HARDWARE INTERRUPT (accel_irq) RECEIVED!", $time);
        repeat (10) @(posedge clk);

        // Write finished DDR3 outputs to hex dump file
        begin : dump_block
            int fd;
            fd = $fopen("C:/Users/risha/OneDrive/Documents/mdmSirResearch/hw_output.hex", "w");
            for (int p = 0; p < 16; p++) begin
                $fdisplay(fd, "%016h", ddr3_out_buffer[p]);
            end
            $fclose(fd);
            $display("[TIME %0t ns] >> Dumped 16 valid output pixels to 'hw_output.hex'.", $time);
        end

        $display("\n=======================================================");
        $display("     METHOD B: REAL YOLO11n HARDWARE EXECUTION DONE    ");
        $display("=======================================================");
        $display("  Total DDR3 Pixels Written : %0d / 16", ddr_wr_count);
        $display("  Hardware IRQ Fired        : %b", accel_irq);
        $display("=======================================================\n");

        sim_done = 1'b1;
        $finish;
    end

endmodule
`default_nettype wire
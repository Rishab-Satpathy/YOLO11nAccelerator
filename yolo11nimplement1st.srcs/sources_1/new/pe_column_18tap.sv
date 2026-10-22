
`timescale 1ns / 1ps
`default_nettype none

module pe_column_18tap #(
    parameter int TAPS = 18
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // Elastic Handshake & Sequence Flags
    input  wire                               valid_in,     // req_in && ack_out
    input  wire                               row_last_in,  // Beat 31 (terminal tap)
    input  wire                               clear_acc,

    // 18-Tap Inputs
    input  wire signed [TAPS-1:0][7:0]        act_in,       // From Module 3 Shared SRAM
    input  wire signed [TAPS-1:0][7:0]        weight_in,    // From local LUTRAM Ring Buffer

    // Output Interface to VPU
    output logic                              valid_out,
    output logic signed [31:0]                out_data
);

    //--------------------------------------------------------------------------
    // Stage 1: 18 Parallel Multipliers
    //--------------------------------------------------------------------------
    logic signed [15:0] prod_s1 [TAPS];
    logic               valid_s1;
    logic               row_last_s1;

    always_ff @(posedge clk) begin
        if (!rst_n || clear_acc) begin
            valid_s1    <= 1'b0;
            row_last_s1 <= 1'b0;
            for (int i = 0; i < TAPS; i++) prod_s1[i] <= 16'sd0;
        end else begin
            valid_s1    <= valid_in;
            row_last_s1 <= row_last_in;
            if (valid_in) begin
                for (int i = 0; i < TAPS; i++) begin
                    prod_s1[i] <= $signed(act_in[i]) * $signed(weight_in[i]);
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Stage 2: 18-to-1 Pipelined Adder Tree (Level A: 18 -> 9 -> 5)
    //--------------------------------------------------------------------------
    logic signed [16:0] sum_l1 [9];
    logic signed [17:0] sum_l2 [5];
    logic               valid_s2;
    logic               row_last_s2;

    always_comb begin
        for (int i = 0; i < 9; i++) begin
            sum_l1[i] = $signed(prod_s1[2*i]) +$signed(prod_s1[2*i + 1]);
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n || clear_acc) begin
            valid_s2    <= 1'b0;
            row_last_s2 <= 1'b0;
            for (int i = 0; i < 5; i++) sum_l2[i] <= 18'sd0;
        end else begin
            valid_s2    <= valid_s1;
            row_last_s2 <= row_last_s1;
            if (valid_s1) begin
                sum_l2[0] <= $signed(sum_l1[0]) +$signed(sum_l1[1]);
                sum_l2[1] <= $signed(sum_l1[2]) +$signed(sum_l1[3]);
                sum_l2[2] <= $signed(sum_l1[4]) +$signed(sum_l1[5]);
                sum_l2[3] <= $signed(sum_l1[6]) +$signed(sum_l1[7]);
                sum_l2[4] <= {{1{sum_l1[8][16]}}, sum_l1[8]}; // Carry lone 9th branch
            end
        end
    end

    //--------------------------------------------------------------------------
    // Stage 3: Adder Tree Final Reduction (5 -> 1) into 21-Bit Vector Sum
    //--------------------------------------------------------------------------
    logic signed [20:0] beat_sum_s3;
    logic               valid_s3;
    logic               row_last_s3;

    always_ff @(posedge clk) begin
        if (!rst_n || clear_acc) begin
            valid_s3    <= 1'b0;
            row_last_s3 <= 1'b0;
            beat_sum_s3 <= 21'sd0;
        end else begin
            valid_s3    <= valid_s2;
            row_last_s3 <= row_last_s2;
            if (valid_s2) begin
                beat_sum_s3 <= ($signed(sum_l2[0]) + $signed(sum_l2[1])) + ($signed(sum_l2[2]) + $signed(sum_l2[3])) +$signed({{3{sum_l2[4][17]}}, sum_l2[4]});
            end
        end
    end

    //--------------------------------------------------------------------------
    // Stage 4: 32-Bit Output-Stationary Accumulator
    //--------------------------------------------------------------------------
    logic signed [31:0] acc_reg;
    logic signed [31:0] next_acc;

    assign next_acc = acc_reg + {{11{beat_sum_s3[20]}}, beat_sum_s3};

    always_ff @(posedge clk) begin
        if (!rst_n || clear_acc) begin
            acc_reg   <= 32'sd0;
            out_data  <= 32'sd0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= 1'b0; // Default: 1-cycle output strobe

            if (valid_s3) begin
                if (row_last_s3) begin
                    // Beat 31: latch completed 576-tap dot product for VPU
                    out_data  <= next_acc;
                    valid_out <= 1'b1;
                    acc_reg   <= 32'sd0; // Zero-stall reset for next pixel
                end else begin
                    // Beats 0..30: accumulate running partial sum
                    acc_reg   <= next_acc;
                end
            end
        end
    end

endmodule
`default_nettype wire

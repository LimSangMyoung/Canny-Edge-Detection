`timescale 1ns / 1ps
// =====================================================================
//  Adaptive threshold (Otsu, Eq.14) — argmax of 2*log2(sigma_k)
//
//  Fix vs. original: numerator = |u_L1*w_k - u_k|.  The unsigned
//  subtraction (mul_r2 - uk_r2) wraps to a huge value when uk_r2 is the
//  larger term, which then feeds garbage into the (unsigned) log unit.
//  Eq.11 squares this term, so only its magnitude matters -> take abs.
//
//  Fix 2: inputs are 8.24 fixed point (see otsu_accum), so u_L1 * w_k
//  is a 16.48 product.  Keeping only the low 32 bits of it is wrong;
//  the full 2W-bit product must be taken and shifted right by SCALE
//  (= 24) to return to 8.24 before subtracting u_k.
// =====================================================================
module threshold_calc #(
    parameter W       = 32,
    parameter FRAC    = 26,
    parameter LATENCY = 6,    // mult 2 + log 3 + adder 1
    parameter SCALE   = 24    // fractional bits of the 8.24 operands
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          dout_valid,
    input  wire [W-1:0]  w_k,
    input  wire [W-1:0]  sub_wk,
    input  wire [W-1:0]  u_k,
    input  wire [W-1:0]  u_L1,
    output reg  [W-1:0]  threshold,
    output reg           thresh_valid
);

    // ---- Step 1: pipelined multiplier (2-cycle) ----
    reg [2*W-1:0] mul_r1;             // full 16.48 product
    reg [W-1:0]   mul_r2;             // >> SCALE, back to 8.24
    reg [W-1:0] uk_r1,    uk_r2;
    reg [W-1:0] wk_r1,    wk_r2;
    reg [W-1:0] subwk_r1, subwk_r2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_r1   <= {(2*W){1'b0}}; mul_r2 <= {W{1'b0}};
            uk_r1    <= {W{1'b0}}; uk_r2    <= {W{1'b0}};
            wk_r1    <= {W{1'b0}}; wk_r2    <= {W{1'b0}};
            subwk_r1 <= {W{1'b0}}; subwk_r2 <= {W{1'b0}};
        end else begin
            mul_r1   <= u_L1 * w_k; mul_r2   <= mul_r1 >> SCALE;
            uk_r1    <= u_k;        uk_r2    <= uk_r1;
            wk_r1    <= w_k;        wk_r2    <= wk_r1;
            subwk_r1 <= sub_wk;     subwk_r2 <= subwk_r1;
        end
    end

    // numerator = | mul_r2 - uk_r2 |  (W+1-bit signed diff, then abs)
    wire signed [W:0] num_s = $signed({1'b0, mul_r2}) - $signed({1'b0, uk_r2});
    wire [W-1:0] numerator  = num_s[W] ? (~num_s[W-1:0] + 1'b1) : num_s[W-1:0];

    // ---- Step 2: log units (3-cycle each) ----
    wire [W-1:0] log_wk, log_sub_wk, log_num;
    log_unit #(.W(W), .FRAC(FRAC)) u_log_wk  (.clk(clk), .rst_n(rst_n), .x(wk_r2),     .log2_out(log_wk));
    log_unit #(.W(W), .FRAC(FRAC)) u_log_sub (.clk(clk), .rst_n(rst_n), .x(subwk_r2),  .log2_out(log_sub_wk));
    log_unit #(.W(W), .FRAC(FRAC)) u_log_num (.clk(clk), .rst_n(rst_n), .x(numerator), .log2_out(log_num));

    // ---- Step 3: Eq.14 adder/subtractor (1-cycle) ----
    reg signed [W-1:0] sigma2_k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sigma2_k <= {W{1'b0}};
        else        sigma2_k <= $signed(log_num << 1) - $signed(log_wk + log_sub_wk);
    end

    // ---- Step 4: k counter + LATENCY-deep alignment shift reg ----
    reg [7:0] k_raw;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            k_raw <= 8'd0;
        else if (dout_valid)   k_raw <= k_raw + 1;
        else                   k_raw <= 8'd0;
    end

    reg [7:0] k_pipe [0:LATENCY-1];
    reg       v_pipe [0:LATENCY-1];
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (j = 0; j < LATENCY; j = j + 1) begin
                k_pipe[j] <= 8'd0;
                v_pipe[j] <= 1'b0;
            end
        end else begin
            k_pipe[0] <= k_raw;
            v_pipe[0] <= dout_valid;
            for (j = 1; j < LATENCY; j = j + 1) begin
                k_pipe[j] <= k_pipe[j-1];
                v_pipe[j] <= v_pipe[j-1];
            end
        end
    end

    wire [7:0] k_aligned = k_pipe[LATENCY-1];
    wire       v_aligned = v_pipe[LATENCY-1];

    reg v_aligned_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) v_aligned_prev <= 1'b0;
        else        v_aligned_prev <= v_aligned;
    end
    wire v_aligned_fall = v_aligned_prev & ~v_aligned;

    // ---- Step 5: max-hold (argmax) threshold search ----
    localparam SIGNED_MIN = {1'b1, {(W-1){1'b0}}};
    reg signed [W-1:0] max_sigma2;
    reg        [7:0]   best_k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            max_sigma2   <= SIGNED_MIN;
            best_k       <= 8'd0;
            threshold    <= {W{1'b0}};
            thresh_valid <= 1'b0;
        end else if (v_aligned) begin
            thresh_valid <= 1'b0;
            if ($signed(sigma2_k) > $signed(max_sigma2)) begin
                max_sigma2 <= sigma2_k;
                best_k     <= k_aligned;
            end
        end else if (v_aligned_fall) begin
            threshold    <= best_k;
            thresh_valid <= 1'b1;
            max_sigma2   <= SIGNED_MIN;
            best_k       <= 8'd0;
        end else begin
            thresh_valid <= 1'b0;
        end
    end
endmodule

`timescale 1ns / 1ps
// =====================================================================
//  32-bit logarithmic arithmetic unit  (Eq.14 building block)
//  log2(x) in Q6.26 fixed-point  (Mitchell-style linear approximation)
//
//  3-stage pipeline (3-cycle latency):
//    stage 1 : CLZ
//    stage 2 : barrel shift (normalize) + characteristic
//    stage 3 : fractional part + combine
//  This latency matches LATENCY=6 in threshold_calc
//  (mult 2 + log 3 + adder 1).
// =====================================================================
module log_unit #(
    parameter W    = 32,
    parameter FRAC = 26
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire [W-1:0]  x,
    output reg  [W-1:0]  log2_out
);
    localparam CHAR  = W - FRAC;        // characteristic width = 6
    localparam CLZ_W = $clog2(W) + 1;   // CLZ output width = 6 (covers 0..32)

    // -------- stage 1 : count leading zeros --------------------------
    wire [CLZ_W-1:0] clz_w;
    clz #(.W(W)) u_clz (.x(x), .clz_out(clz_w));

    reg [CLZ_W-1:0] clz_s1;
    reg [W-1:0]     x_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clz_s1 <= {CLZ_W{1'b0}};
            x_s1   <= {W{1'b0}};
        end else begin
            clz_s1 <= clz_w;
            x_s1   <= x;
        end
    end

    // -------- stage 2 : normalize + characteristic -------------------
    // shift amount is 0..31 for x!=0, so the low 5 bits are enough.
    wire [W-1:0]    xnorm_w;
    wire [CHAR-1:0] char_w;
    bsh  #(.W(W))              u_bsh  (.x(x_s1), .shift(clz_s1[4:0]), .x_norm(xnorm_w));
    cgen #(.W(W), .CHAR(CHAR)) u_cgen (.clz_out(clz_s1), .x(x_s1),    .char_out(char_w));

    reg [W-1:0]    xnorm_s2;
    reg [CHAR-1:0] char_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xnorm_s2 <= {W{1'b0}};
            char_s2  <= {CHAR{1'b0}};
        end else begin
            xnorm_s2 <= xnorm_w;
            char_s2  <= char_w;
        end
    end

    // -------- stage 3 : fractional part + combine --------------------
    wire [FRAC-1:0] frac_w;
    fpgen #(.W(W), .FRAC(FRAC)) u_fp (.x_norm(xnorm_s2), .frac_out(frac_w));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) log2_out <= {W{1'b0}};
        else        log2_out <= {char_s2, frac_w};
    end
endmodule

// =====================================================================
//  Count Leading Zeros (recursive)  output width = clog2(W)+1
// =====================================================================
module clz #(parameter W = 32) (
    input  wire [W-1:0]        x,
    output wire [$clog2(W):0]  clz_out
);
    generate
        if (W == 1) begin : g_base
            assign clz_out = ~x[0];
        end else begin : g_rec
            wire [$clog2(W/2):0] upper_clz, lower_clz;
            wire                 upper_zero;
            clz #(.W(W/2)) u_upper (.x(x[W-1 : W/2]),  .clz_out(upper_clz));
            clz #(.W(W/2)) u_lower (.x(x[W/2-1 : 0]),  .clz_out(lower_clz));
            assign upper_zero = (x[W-1 : W/2] == {(W/2){1'b0}});
            assign clz_out = upper_zero ? ({1'b0, lower_clz} + (W/2))
                                        : {1'b0, upper_clz};
        end
    endgenerate
endmodule

// =====================================================================
//  Barrel shifter : left-align the leading 1 to the MSB
// =====================================================================
module bsh #(parameter W = 32) (
    input  wire [W-1:0] x,
    input  wire [4:0]   shift,
    output wire [W-1:0] x_norm
);
    assign x_norm = x << shift;
endmodule

// =====================================================================
//  Characteristic generator : integer part of log2(x) = MSB position
//  CHAR-bit output (was 5-bit before -> truncated the 6th bit)
// =====================================================================
module cgen #(
    parameter W    = 32,
    parameter CHAR = 6
) (
    input  wire [$clog2(W):0] clz_out,   // 6-bit, matches clz
    input  wire [W-1:0]       x,
    output reg  [CHAR-1:0]    char_out
);
    always @(*) begin
        if (x == {W{1'b0}})
            char_out = {CHAR{1'b0}};         // log2(0) sentinel
        else
            char_out = (W - 1) - clz_out;    // position of leading 1
    end
endmodule

// =====================================================================
//  Fractional part generator : drop the implicit MSB, keep upper FRAC bits
// =====================================================================
module fpgen #(
    parameter W    = 32,
    parameter FRAC = 26
) (
    input  wire [W-1:0]    x_norm,
    output wire [FRAC-1:0] frac_out
);
    assign frac_out = x_norm[W-2 : W-1-FRAC];
endmodule

`timescale 1ns / 1ps
// =====================================================================
//  Hysteresis thresholding (Fig.17)
//
//  gh = strong edge (din >= t_high), gl = weak edge (t_low<=din<t_high).
//  A weak center pixel is kept only if any of its 8 neighbors is strong.
//
//  Fixes vs. original:
//    * window instantiated with .DW(2)/.IMG_W(W)  (was .W(2))
//    * valid delay = center-tap latency = IMG_W + 2  (was W+1, off by one)
// =====================================================================
module hysteresis #(
    parameter W  = 32,   // image width
    parameter DW = 8     // pixel data width
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          data_valid,
    input  wire [DW-1:0] din,
    input  wire [DW-1:0] t_high,
    input  wire [DW-1:0] t_low,
    output reg           edge_out,
    output reg           dout_valid
);
    // -------- binarize BEFORE the window (gh,gl move together) -------
    wire       gh_in = (din >= t_high);
    wire       gl_in = (din >= t_low) && (din < t_high);
    wire [1:0] bin_in = {gh_in, gl_in};   // [1]=gh, [0]=gl

    // -------- 2-bit 3x3 window ---------------------------------------
    wire [1:0] p11, p12, p13;
    wire [1:0] p21, p22, p23;
    wire [1:0] p31, p32, p33;

    window_3x3 #(.DW(2), .IMG_W(W)) u_win (
        .clk(clk), .rst_n(rst_n), .data_valid(data_valid),
        .din(bin_in),
        .p11(p11), .p12(p12), .p13(p13),
        .p21(p21), .p22(p22), .p23(p23),
        .p31(p31), .p32(p32), .p33(p33)
    );

    wire gh_center = p22[1];
    wire gl_center = p22[0];

    // any strong edge among the 8 neighbors
    wire strong_neighbor = p11[1] | p12[1] | p13[1] |
                           p21[1] |           p23[1] |
                           p31[1] | p32[1] | p33[1];

    // -------- valid delay = center-tap latency = IMG_W + 2 -----------
    localparam DELAY = W + 2;
    reg [DELAY-1:0] valid_sr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_sr <= {DELAY{1'b0}};
        else        valid_sr <= {valid_sr[DELAY-2:0], data_valid};
    end
    wire valid_aligned = valid_sr[DELAY-1];

    // -------- hysteresis decision ------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            edge_out   <= 1'b0;
            dout_valid <= 1'b0;
        end else begin
            dout_valid <= valid_aligned;
            if (valid_aligned)
                edge_out <= gh_center | (gl_center & strong_neighbor);
            else
                edge_out <= 1'b0;
        end
    end
endmodule

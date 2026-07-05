`timescale 1ns / 1ps

// 5x5 Gaussian filter with a one-entry elastic output register.
//
// s_window is row-major, with the top-left pixel in bits [7:0]:
//   p00 = s_window[  7:  0], ... p04 = s_window[ 39: 32]
//   p10 = s_window[ 47: 40], ... p44 = s_window[199:192]
//
// Kernel:
//    1   4   6   4  1
//    4  16  24  16  4
//    6  24  36  24  6       sum = 256
//    4  16  24  16  4
//    1   4   6   4  1
module gaussian_filter (
    input  wire         clk,
    input  wire         rst_n,

    input  wire [199:0] s_window,
    input  wire         s_valid,
    output wire         s_ready,
    input  wire         s_sof,
    input  wire         s_eol,

    output reg  [7:0]   m_data,
    output reg          m_valid,
    input  wire         m_ready,
    output reg          m_sof,
    output reg          m_eol
);

    wire [7:0] p00 = s_window[  7:  0];
    wire [7:0] p01 = s_window[ 15:  8];
    wire [7:0] p02 = s_window[ 23: 16];
    wire [7:0] p03 = s_window[ 31: 24];
    wire [7:0] p04 = s_window[ 39: 32];
    wire [7:0] p10 = s_window[ 47: 40];
    wire [7:0] p11 = s_window[ 55: 48];
    wire [7:0] p12 = s_window[ 63: 56];
    wire [7:0] p13 = s_window[ 71: 64];
    wire [7:0] p14 = s_window[ 79: 72];
    wire [7:0] p20 = s_window[ 87: 80];
    wire [7:0] p21 = s_window[ 95: 88];
    wire [7:0] p22 = s_window[103: 96];
    wire [7:0] p23 = s_window[111:104];
    wire [7:0] p24 = s_window[119:112];
    wire [7:0] p30 = s_window[127:120];
    wire [7:0] p31 = s_window[135:128];
    wire [7:0] p32 = s_window[143:136];
    wire [7:0] p33 = s_window[151:144];
    wire [7:0] p34 = s_window[159:152];
    wire [7:0] p40 = s_window[167:160];
    wire [7:0] p41 = s_window[175:168];
    wire [7:0] p42 = s_window[183:176];
    wire [7:0] p43 = s_window[191:184];
    wire [7:0] p44 = s_window[199:192];

    // Maximum is 255 * 256 = 65280, so 16 bits are sufficient.
    // Explicit multiplication also prevents Verilog from keeping a shifted
    // 8-bit operand at only 8 bits and silently discarding its upper bits.
    wire [15:0] gaussian_sum =
          p00       + (p01 * 4)  + (p02 * 6)  + (p03 * 4)  + p04
        + (p10 * 4)  + (p11 * 16) + (p12 * 24) + (p13 * 16) + (p14 * 4)
        + (p20 * 6)  + (p21 * 24) + (p22 * 36) + (p23 * 24) + (p24 * 6)
        + (p30 * 4)  + (p31 * 16) + (p32 * 24) + (p33 * 16) + (p34 * 4)
        + p40       + (p41 * 4)  + (p42 * 6)  + (p43 * 4)  + p44;

    // The stage may accept data when its output register is empty or consumed.
    assign s_ready = ~m_valid | m_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            m_data  <= 8'd0;
            m_valid <= 1'b0;
            m_sof   <= 1'b0;
            m_eol   <= 1'b0;
        end else if (s_ready) begin
            m_valid <= s_valid;
            if (s_valid) begin
                // Divide the kernel sum by 256. Truncation matches integer RTL.
              m_data <= gaussian_sum[15:8]; //256으로 나누는것을 상위 8비트만 가져와서 >>8의 효과를 냄 
                m_sof  <= s_sof;
                m_eol  <= s_eol;

            end
        end
    end

endmodule

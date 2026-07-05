`timescale 1ns / 1ps

// 3x3 Sobel gradient with a one-entry elastic output register.
//
// s_window is row-major, with the top-left pixel in bits [7:0].
// Magnitude uses |Gx| + |Gy| to avoid square-root hardware.
//
// Direction encoding for NMS:
//   2'b00:   0 degrees (compare left/right)
//   2'b01:  45 degrees (compare NW/SE)
//   2'b10:  90 degrees (compare up/down)
//   2'b11: 135 degrees (compare NE/SW)
module gradient_sobel (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [71:0] s_window,
    input  wire        s_valid,
    output wire        s_ready,
    input  wire        s_sof,
    input  wire        s_eol,

    output reg  [11:0] m_magnitude,
    output reg  [1:0]  m_direction,
    output reg         m_valid,
    input  wire        m_ready,
    output reg         m_sof,
    output reg         m_eol
);

    wire [7:0] p00 = s_window[ 7: 0];
    wire [7:0] p01 = s_window[15: 8];
    wire [7:0] p02 = s_window[23:16];
    wire [7:0] p10 = s_window[31:24];
    wire [7:0] p11 = s_window[39:32];
    wire [7:0] p12 = s_window[47:40];
    wire [7:0] p20 = s_window[55:48];
    wire [7:0] p21 = s_window[63:56];
    wire [7:0] p22 = s_window[71:64];

    // Sobel kernels:
    // Gx = [-1 0 1; -2 0 2; -1 0 1]
    // Gy = [-1 -2 -1; 0 0 0; 1 2 1]
    // Range of each component is -1020 to +1020.
    wire signed [11:0] gx =
          $signed({4'b0000, p02})
        + ($signed({4'b0000, p12}) <<< 1)
        + $signed({4'b0000, p22})
        - $signed({4'b0000, p00})
        - ($signed({4'b0000, p10}) <<< 1)
        - $signed({4'b0000, p20});

    wire signed [11:0] gy =
          $signed({4'b0000, p20})
        + ($signed({4'b0000, p21}) <<< 1)
        + $signed({4'b0000, p22})
        - $signed({4'b0000, p00})
        - ($signed({4'b0000, p01}) <<< 1)
        - $signed({4'b0000, p02});

    wire [10:0] abs_gx = gx[11] ? -gx : gx;
    wire [10:0] abs_gy = gy[11] ? -gy : gy;
    wire [11:0] magnitude_next = {1'b0, abs_gx} + {1'b0, abs_gy}; //이건 언사인드

    reg [1:0] direction_next;
    always @* begin
        // Low-cost approximation of 22.5/67.5-degree boundaries.
        if (abs_gy <= (abs_gx >> 1))
            direction_next = 2'b00;
        else if (abs_gx <= (abs_gy >> 1))
            direction_next = 2'b10;
        else if (gx[11] == gy[11])
            direction_next = 2'b01;
        else
            direction_next = 2'b11;
    end

    assign s_ready = ~m_valid | m_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            m_magnitude <= 12'd0;
            m_direction <= 2'b00;
            m_valid     <= 1'b0;
            m_sof       <= 1'b0;
            m_eol       <= 1'b0;
        end else if (s_ready) begin
            m_valid <= s_valid;
            if (s_valid) begin
                m_magnitude <= magnitude_next;
                m_direction <= direction_next;
                m_sof       <= s_sof;
                m_eol       <= s_eol;
            end
        end
    end

endmodule

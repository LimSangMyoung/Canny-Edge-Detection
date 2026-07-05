`timescale 1ns / 1ps
// =====================================================================
//  Otsu accumulator  (Fig.13 / Eq.15 of the paper)
//
//  Consumes the histogram readout stream (bin 0..NBINS-1 in order) and
//  produces, for every candidate threshold k, the four operands of
//  threshold_calc:
//      w_k    = sum_{i<=k} p_i          (class-0 probability)
//      sub_wk = w_total - w_k           (= 1 - w_k, class-1 probability)
//      u_k    = sum_{i<=k} i * p_i      (class-0 cumulative mean)
//      u_L1   = u_{NBINS-1}             (global mean, same every cycle)
//
//  Normalization (Eq.15): p_i = count_i * 2^24 / total_pixels.
//  For a power-of-two pixel count this is a pure shift:
//      P_SHIFT = 24 - log2(IMG_SIZE)     (512x512 -> P_SHIFT = 6)
//  All values are 8.24 unsigned fixed point; the 2^24 scale cancels in
//  Eq.14 (2*24 - 24 - 24 = 0), so threshold_calc needs no correction.
//
//  Why two phases with RAM buffering: u_L1 is only known after the LAST
//  bin has been accumulated, but threshold_calc needs it next to every
//  k starting from k = 0. So phase A stores the running sums w_k / u_k
//  into two small RAMs while accumulating, and phase B replays them as
//  an uninterrupted NBINS-cycle burst once u_L1 / w_total are latched.
//
//  Overflow: p_i <= 2^24 and sum(p_i) = 2^24, so w fits in 25 bits and
//  u <= 255 * 2^24 < 2^32 - all safely inside W = 32.
//
//  Endpoint bins (w_k = 0 or sub_wk = 0) make Eq.14 use log2(0); the
//  log unit returns its 0-sentinel there, which yields a strongly
//  negative sigma^2 that can never win the argmax - no special case.
// =====================================================================
module otsu_accum #(
    parameter DW      = 8,    // bin address width (NBINS = 2^DW)
    parameter CW      = 32,   // histogram count width
    parameter W       = 32,   // operand width (8.24 fixed point)
    parameter P_SHIFT = 6     // p_i = count_i << P_SHIFT (24 - log2(IMG_SIZE))
) (
    input  wire          clk,
    input  wire          rst_n,
    // from histogram readout
    input  wire          hist_valid,   // histogram dout_valid
    input  wire [DW-1:0] bin_addr,
    input  wire [CW-1:0] count,
    // to threshold_calc, k = 0 .. NBINS-1 on consecutive cycles
    output reg           dout_valid,
    output reg  [W-1:0]  w_k,
    output reg  [W-1:0]  sub_wk,
    output reg  [W-1:0]  u_k,
    output reg  [W-1:0]  u_L1
);
    localparam NBINS = (1 << DW);

    reg [W-1:0] w_ram [0:NBINS-1];   // w_k per bin
    reg [W-1:0] u_ram [0:NBINS-1];   // u_k per bin

    reg           streaming;         // 0 = accumulate, 1 = replay
    reg [DW:0]    k;                 // replay index
    reg [W-1:0]   w_acc, u_acc;      // running sums
    reg [W-1:0]   w_tot, u_L1_r;     // latched totals

    // p_i = count << P_SHIFT ; term i*p_i <= 255 * 2^24 < 2^32
    wire [W-1:0] p    = count << P_SHIFT;
    wire [W-1:0] ip   = bin_addr * p;
    wire [W-1:0] wnew = w_acc + p;
    wire [W-1:0] unew = u_acc + ip;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            streaming  <= 1'b0;
            k          <= {(DW+1){1'b0}};
            w_acc      <= {W{1'b0}};
            u_acc      <= {W{1'b0}};
            w_tot      <= {W{1'b0}};
            u_L1_r     <= {W{1'b0}};
            dout_valid <= 1'b0;
            w_k        <= {W{1'b0}};
            sub_wk     <= {W{1'b0}};
            u_k        <= {W{1'b0}};
            u_L1       <= {W{1'b0}};
        end else if (!streaming) begin
            // ---------------- phase A : accumulate ----------------
            dout_valid <= 1'b0;
            if (hist_valid) begin
                w_ram[bin_addr] <= wnew;   // cumulative INCLUDING bin i
                u_ram[bin_addr] <= unew;
                w_acc <= wnew;
                u_acc <= unew;
                if (bin_addr == NBINS-1) begin
                    w_tot     <= wnew;     // = 2^24 (sum of all p_i)
                    u_L1_r    <= unew;     // global mean u_{L-1}
                    w_acc     <= {W{1'b0}};   // re-arm for next frame
                    u_acc     <= {W{1'b0}};
                    k         <= {(DW+1){1'b0}};
                    streaming <= 1'b1;
                end
            end
        end else begin
            // ---------------- phase B : replay to threshold_calc --
            dout_valid <= 1'b1;
            w_k    <= w_ram[k[DW-1:0]];
            u_k    <= u_ram[k[DW-1:0]];
            sub_wk <= w_tot - w_ram[k[DW-1:0]];
            u_L1   <= u_L1_r;
            if (k == NBINS-1) begin
                streaming <= 1'b0;
                k         <= {(DW+1){1'b0}};
            end else begin
                k <= k + 1'b1;
            end
        end
    end
endmodule

// output_buffer - hysteresis 픽셀 스트림 -> DDR 쓰기 워드
//
// 8비트 픽셀을 모아 64비트로 패킹, ddr_interface 쓰기 엔진에 FWFT로 전달
//
// 전제: 784픽셀(28x28) = 98워드, partial beat 없음, WSTRB 항상 0xFF
//   크롭 프레임 쓰게 되면 TotalPix + ddr_interface 마지막 비트 WSTRB만 손보면 됨
//
// 패킹 순서는 input_buffer.sv와 동일해야 함 (픽셀 k = 바이트 k, LSB first)

`default_nettype none

module output_buffer #(
  parameter int unsigned AxiDataW   = 64,
  parameter int unsigned PixW       = 8,
  parameter int unsigned Depth      = 128,          // 2의 거듭제곱, ceil(TotalPix/PixPerBeat) 이상
  localparam int unsigned PixPerBeat = AxiDataW / PixW,   // 8
  localparam int unsigned DepthLog2  = $clog2(Depth),
  localparam int unsigned PtrW       = DepthLog2 + 1,
  localparam int unsigned OccW       = DepthLog2 + 1,
  localparam int unsigned SubW       = $clog2(PixPerBeat)
) (
  input  wire                   clk,
  input  wire                   rst_ni,

  // 픽셀 입력 (hysteresis_threshold로부터: valid + data)
  input  wire                   pix_valid_i,
  input  wire [PixW-1:0]        pix_data_i,

  // 팝 쪽 (ddr_interface의 AXI W로)
  output wire                   pop_valid_o,
  output wire [AxiDataW-1:0]    pop_data_o,
  input  wire                   pop_ready_i,

  // 상태
  output wire [OccW-1:0]        occupancy_o          // 현재 쌓여있는 64비트 워드 수
);

  // 패커: PixPerBeat개 픽셀(LSB부터)을 64비트 워드 하나로 모아서 lane 7에서 푸시
  logic [SubW-1:0]        sub_q;                     // 바이트 위치 0..7
  logic [AxiDataW-1:0]    word_q;                    // 조립 중인 워드

  wire word_last_w = (sub_q == SubW'(PixPerBeat - 1));
  wire do_push     = pix_valid_i & word_last_w;      // 완성된 워드를 푸시

  // word_d: 들어온 픽셀을 sub_q 위치에 얹은 현재 워드
  logic [AxiDataW-1:0] word_d;
  always_comb begin
    word_d              = word_q;
    word_d[sub_q*PixW +: PixW] = pix_data_i;
  end

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      sub_q  <= '0;
      word_q <= '0;
    end else if (pix_valid_i) begin
      word_q <= word_d;
      sub_q  <= sub_q + SubW'(1);   // PixPerBeat가 2의 거듭제곱이라 자동으로 랩됨
    end
  end

  // lane 7 클럭에 FIFO로 들어가는 데이터 = word_d (앞쪽 레인은 word_q에서,
  // 이번 클럭 픽셀은 lane 7에서)
  wire [AxiDataW-1:0] push_word_w = word_d;

  // 64비트 FWFT FIFO (ofmap_fifo.sv의 rp_d 읽기 + bypass-to-mem 뼈대)
  (* ram_style = "block" *)
  logic [AxiDataW-1:0] mem_q [Depth];

  logic [PtrW-1:0]     wp_q, wp_d;
  logic [PtrW-1:0]     rp_q, rp_d;

  logic [AxiDataW-1:0] mem_read_q;
  logic                bypass_q;
  logic [AxiDataW-1:0] bypass_data_q;

  logic [PtrW-1:0]     occ_w;
  assign occ_w       = wp_q - rp_q;
  assign occupancy_o = occ_w[OccW-1:0];

  wire fifo_empty_w = (wp_q == rp_q);
  wire fifo_full_w  = (occ_w == PtrW'(Depth));

  // 패커에서 오는 do_push는 항상 성공한다: Depth가 이미지 한 장의 워드 수보다
  // 크니까 한 프레임 처리 중엔 FIFO가 절대 안 찬다. (파이프라인 쪽으로는
  // push_ready가 없음. 엣지맵을 만드는 쪽이 원래 백프레셔가 없으니 line_buffer와
  // 같은 조건이다.)
  wire do_push_fifo = do_push & ~fifo_full_w;
  wire do_pop       = ~fifo_empty_w & pop_ready_i;

  assign pop_valid_o = ~fifo_empty_w;
  assign pop_data_o  = bypass_q ? bypass_data_q : mem_read_q;

  always_comb begin
    wp_d = wp_q;
    if (do_push_fifo) wp_d = wp_q + PtrW'(1);
  end

  always_comb begin
    rp_d = rp_q;
    if (do_pop) rp_d = rp_q + PtrW'(1);
  end

  wire read_bypass_w = do_push_fifo & (rp_d[DepthLog2-1:0] == wp_q[DepthLog2-1:0]);

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      wp_q          <= '0;
      rp_q          <= '0;
      bypass_q      <= 1'b0;
      bypass_data_q <= '0;
    end else begin
      wp_q          <= wp_d;
      rp_q          <= rp_d;
      bypass_q      <= read_bypass_w;
      bypass_data_q <= push_word_w;
    end
  end

  // BRAM SDP: 동기 쓰기 + rp_d에서 항상 동작하는 동기 읽기 (ram_style="block")
  always_ff @(posedge clk) begin
    if (do_push_fifo) begin
      mem_q[wp_q[DepthLog2-1:0]] <= push_word_w;
    end
    mem_read_q <= mem_q[rp_d[DepthLog2-1:0]];
  end

endmodule

`default_nettype wire

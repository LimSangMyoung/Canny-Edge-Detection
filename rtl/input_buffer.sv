// input_buffer - DDR 읽기 워드 -> 8비트 픽셀 스트림 (line_buffer용)
//
// 64비트 AXI FIFO + 언패커
//   푸시: ddr_interface가 비트마다 씀, 깊이 128 (이미지 한 장보다 큼, 백프레셔 안 걸림)
//   팝: stream_start_i 뜨면 TotalPix개 바이트를 LSB부터 순서대로 송출
//
// 패킹 순서는 output_buffer.sv / PS 이미지 레이아웃과 동일해야 함
// (픽셀 k = 바이트 k = 비트[8k+7:8k], LSB first)

`default_nettype none

module input_buffer #(
  parameter int unsigned AxiDataW   = 64,
  parameter int unsigned PixW       = 8,
  parameter int unsigned TotalPix   = 784,          // 28*28
  parameter int unsigned Depth      = 128,          // 2의 거듭제곱, ceil(TotalPix/PixPerBeat) 이상
  localparam int unsigned PixPerBeat = AxiDataW / PixW,   // 8
  localparam int unsigned DepthLog2  = $clog2(Depth),
  localparam int unsigned PtrW       = DepthLog2 + 1,     // +1은 랩 여부 비트
  localparam int unsigned PixCntW    = $clog2(TotalPix) + 1,
  localparam int unsigned SubW       = $clog2(PixPerBeat) // 3
) (
  input  wire                   clk,
  input  wire                   rst_ni,

  // 푸시 쪽 (ddr_interface의 AXI R로부터)
  input  wire                   wr_valid_i,
  input  wire [AxiDataW-1:0]    wr_data_i,
  output wire                   wr_ready_o,

  // 스트림 제어 (ddr_interface로부터)
  // 이미지 한 장이 전부 들어간 뒤 펄스로 뜸. 픽셀 스트림 송출을 시작한다.
  input  wire                   stream_start_i,

  // 픽셀 출력 (line_buffer로: data_valid / din에 해당)
  output wire                   pix_valid_o,
  output wire [PixW-1:0]        pix_data_o,

  // 상태
  output wire                   frame_done_o,        // 마지막 픽셀 다음 클럭에 1펄스
  output wire                   streaming_o          // 프레임을 내보내는 동안 계속 1
);

  // 64비트 FWFT FIFO (ifmap_fifo.sv의 rp_d 읽기 + bypass-to-mem 뼈대)
  (* ram_style = "block" *)
  logic [AxiDataW-1:0] mem_q [Depth];

  logic [PtrW-1:0]     wp_q, wp_d;
  logic [PtrW-1:0]     rp_q, rp_d;

  logic [AxiDataW-1:0] mem_read_q;
  logic                bypass_q;
  logic [AxiDataW-1:0] bypass_data_q;

  logic                do_push;
  logic                do_pop;

  logic [PtrW-1:0]     occ_w;
  assign occ_w = wp_q - rp_q;

  wire fifo_empty_w = (wp_q == rp_q);
  wire fifo_full_w  = (occ_w == PtrW'(Depth));

  assign wr_ready_o = ~fifo_full_w;

  wire                 fifo_pop_valid_w = ~fifo_empty_w;
  wire [AxiDataW-1:0]  fifo_pop_data_w  = bypass_q ? bypass_data_q : mem_read_q;

  assign do_push = wr_valid_i & ~fifo_full_w;

  // 쓰기 포인터
  always_comb begin
    wp_d = wp_q;
    if (do_push) wp_d = wp_q + PtrW'(1);
  end

  // 읽기 포인터 (팝할 때만 전진 -> 정지 중엔 헤드가 그대로 유지됨)
  always_comb begin
    rp_d = rp_q;
    if (do_pop) rp_d = rp_q + PtrW'(1);
  end

  // read_bypass: do_push & rp_d == wp_q (Depth로 mod) 상황이면, empty 직후 첫
  // 푸시에서 BRAM read-during-write 경쟁이 생기니 그걸 피하려고 헤드로 바로
  // 우회해서 넘겨준다. full일 땐 do_push=0이라 오탐은 안 난다.
  wire read_bypass_w = do_push & (rp_d[DepthLog2-1:0] == wp_q[DepthLog2-1:0]);

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
      bypass_data_q <= wr_data_i;
    end
  end

  // BRAM SDP: 동기 쓰기 + rp_d에서 항상 동작하는 동기 읽기 (ram_style="block")
  always_ff @(posedge clk) begin
    if (do_push) begin
      mem_q[wp_q[DepthLog2-1:0]] <= wr_data_i;
    end
    mem_read_q <= mem_q[rp_d[DepthLog2-1:0]];
  end

  // 언패커: 64비트 워드 -> PixPerBeat개의 바이트로, 클럭당 하나씩 LSB부터
  logic                streaming_q;
  logic [PixCntW-1:0]  pix_cnt_q;     // 이번 프레임에서 내보낸 픽셀 수 (0..TotalPix)
  logic [SubW-1:0]     sub_q;         // 현재 워드 안에서의 바이트 위치 (0..7)

  wire last_pixel_w = streaming_q & (pix_cnt_q == PixCntW'(TotalPix - 1));
  wire word_last_w  = (sub_q == SubW'(PixPerBeat - 1));

  // 현재 워드의 마지막 바이트를 다 쓰면(또는 프레임 끝에서 FIFO를 비우려고) 팝
  assign do_pop = streaming_q & fifo_pop_valid_w & word_last_w;

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      streaming_q <= 1'b0;
      pix_cnt_q   <= '0;
      sub_q       <= '0;
    end else if (stream_start_i) begin
      streaming_q <= 1'b1;
      pix_cnt_q   <= '0;
      sub_q       <= '0;
    end else if (streaming_q & fifo_pop_valid_w) begin
      if (last_pixel_w) begin
        streaming_q <= 1'b0;
        pix_cnt_q   <= '0;
        sub_q       <= '0;
      end else begin
        pix_cnt_q <= pix_cnt_q + PixCntW'(1);
        sub_q     <= sub_q + SubW'(1);   // PixPerBeat가 2의 거듭제곱이라 자동으로 랩됨
      end
    end
  end

  // 현재 픽셀 = FIFO 헤드 워드의 sub_q번째 바이트
  wire [PixW-1:0] pix_byte_w = fifo_pop_data_w[sub_q*PixW +: PixW];

  assign pix_valid_o  = streaming_q & fifo_pop_valid_w;
  assign pix_data_o   = pix_byte_w;
  assign streaming_o  = streaming_q;
  assign frame_done_o = streaming_q & fifo_pop_valid_w & last_pixel_w;

endmodule

`default_nettype wire

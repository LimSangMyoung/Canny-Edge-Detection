// ddr_interface - Canny 버퍼용 AXI 마스터 + 데이터 이동 컨트롤러
//
// start_i 펄스마다:
//   1. 이미지 한 장(28x28, 98비트)을 in_base_i에서 읽어 input_buffer로, 다 채우면 stream_start
//   2. output_buffer 결과를 out_base_i에 씀
// 읽기/쓰기 동시 진행 (쓰기가 파이프라인 지연만큼 뒤처짐), 둘 다 끝나면 done_o 펄스
//
// AXI: 64비트, INCR, ID 고정
//   읽기: 최대 16비트 버스트 (4KB 경계 안전)
//   쓰기: 1비트 버스트, AW/W 독립 핸드셰이크(HP3 스타일), WSTRB 항상 0xFF
//
// CNN 전용 로직(레이어 타입/replay/bias/부트 fetch 등)은 제거

`default_nettype none

module ddr_interface #(
  parameter int unsigned AxiAddrW    = 32,
  parameter int unsigned AxiDataW    = 64,
  parameter int unsigned AxiIdW      = 6,
  parameter int unsigned Beats       = 98,      // ceil(28*28 / 8)
  parameter int unsigned OutstandMax = 4,       // Zynq HP 발행 깊이 이하로
  localparam int unsigned BeatCntW   = 10       // 0..Beats 표현 (4KB 비트 스팬 512도 포함)
) (
  input  wire                   clk,
  input  wire                   rst_ni,

  // 제어 (나중에 붙는 AXI-Lite 제어 레지스터 슬레이브로부터)
  input  wire                   start_i,        // 1클럭 펄스: 이미지 한 장 시작
  input  wire [AxiAddrW-1:0]    in_base_i,      // DDR 소스 (8바이트 정렬)
  input  wire [AxiAddrW-1:0]    out_base_i,     // DDR 목적지 (8바이트 정렬)
  output wire                   busy_o,
  output wire                   done_o,         // 1클럭 펄스: 이미지 처리 완료

  // input_buffer 푸시 쪽
  output wire                   ib_wr_valid_o,
  output wire [AxiDataW-1:0]    ib_wr_data_o,
  input  wire                   ib_wr_ready_i,
  output wire                   ib_stream_start_o,

  // output_buffer 팝 쪽
  input  wire                   ob_pop_valid_i,
  input  wire [AxiDataW-1:0]    ob_pop_data_i,
  output wire                   ob_pop_ready_o,

  // AXI 마스터 읽기 채널 (HP 포트)
  output wire                   arvalid_o,
  input  wire                   arready_i,
  output wire [AxiAddrW-1:0]    araddr_o,
  output wire [7:0]             arlen_o,
  output wire [2:0]             arsize_o,
  output wire [1:0]             arburst_o,
  output wire [AxiIdW-1:0]      arid_o,
  input  wire                   rvalid_i,
  output wire                   rready_o,
  input  wire [AxiDataW-1:0]    rdata_i,
  input  wire                   rlast_i,
  input  wire [1:0]             rresp_i,

  // AXI 마스터 쓰기 채널 (HP 포트)
  output wire                   awvalid_o,
  input  wire                   awready_i,
  output wire [AxiAddrW-1:0]    awaddr_o,
  output wire [7:0]             awlen_o,
  output wire [2:0]             awsize_o,
  output wire [1:0]             awburst_o,
  output wire [AxiIdW-1:0]      awid_o,
  output wire                   wvalid_o,
  input  wire                   wready_i,
  output wire [AxiDataW-1:0]    wdata_o,
  output wire [AxiDataW/8-1:0]  wstrb_o,
  output wire                   wlast_o,
  input  wire                   bvalid_i,
  output wire                   bready_o,
  input  wire [1:0]             bresp_i
);

  localparam logic [1:0] BurstIncr = 2'b01;
  localparam logic [2:0] Size8B     = 3'b011;

  // 읽기 엔진 (AR/R) -> input_buffer
  logic [AxiAddrW-1:0]   ar_addr_q;
  logic [BeatCntW-1:0]   ar_rem_q;     // 아직 요청 안 한 비트 수 (0이면 AR 쪽은 끝)
  logic [BeatCntW-1:0]   r_rem_q;      // 아직 못 받은 비트 수 (0이면 읽기 완료)
  logic [3:0]            ar_outstand_q;

  // 버스트 분할: min(16, ar_rem, 다음 4KB 경계까지 남은 비트). 주소는 8바이트
  // 정렬이라 addr[11:3]이 4KB(512비트) 페이지 안에서의 비트 인덱스가 된다.
  logic [BeatCntW-1:0]   beats_to_4kb_w;
  logic [BeatCntW-1:0]   cap16_w;
  logic [BeatCntW-1:0]   burst_beats_w;
  assign beats_to_4kb_w = BeatCntW'(10'd512) - {1'b0, ar_addr_q[11:3]};
  assign cap16_w        = (ar_rem_q < BeatCntW'(16)) ? ar_rem_q : BeatCntW'(16);
  assign burst_beats_w  = (beats_to_4kb_w < cap16_w) ? beats_to_4kb_w : cap16_w;

  wire ar_can_fire_w = (ar_rem_q != '0) & (ar_outstand_q < 4'(OutstandMax));
  wire ar_fire_w     = ar_can_fire_w & arready_i;
  wire r_fire_w      = rvalid_i & rready_o;
  wire r_last_w      = r_fire_w & rlast_i;
  wire last_r_w      = r_fire_w & (r_rem_q == BeatCntW'(1));

  assign arvalid_o = ar_can_fire_w;
  assign araddr_o  = ar_addr_q;
  assign arlen_o   = 8'(burst_beats_w - BeatCntW'(1));
  assign arsize_o  = Size8B;
  assign arburst_o = BurstIncr;
  assign arid_o    = '0;

  // RREADY는 input_buffer의 백프레셔를 그대로 따른다(무손실). FIFO가 이미지 한
  // 장보다 크게 잡혀 있어서 실제로 이미지 한 장 처리 중에 멈출 일은 없다.
  assign rready_o       = ib_wr_ready_i;
  assign ib_wr_valid_o  = r_fire_w;
  assign ib_wr_data_o   = rdata_i;

  logic stream_start_q;
  assign ib_stream_start_o = stream_start_q;

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      ar_addr_q     <= '0;
      ar_rem_q      <= '0;
      r_rem_q       <= '0;
      ar_outstand_q <= '0;
      stream_start_q<= 1'b0;
    end else begin
      stream_start_q <= last_r_w;     // 마지막 R 비트 다음 클럭에 펄스

      if (start_i) begin
        ar_addr_q <= in_base_i;
        ar_rem_q  <= BeatCntW'(Beats);
        r_rem_q   <= BeatCntW'(Beats);
      end else begin
        if (ar_fire_w) begin
          ar_addr_q <= ar_addr_q + AxiAddrW'({burst_beats_w, 3'b000});
          ar_rem_q  <= ar_rem_q - burst_beats_w;
        end
        if (r_fire_w) begin
          r_rem_q <= r_rem_q - BeatCntW'(1);
        end
      end

      // 미완료 읽기 버스트 수: AR fire에 +1, RLAST에 -1
      unique case ({ar_fire_w, r_last_w})
        2'b10:   ar_outstand_q <= ar_outstand_q + 4'd1;
        2'b01:   ar_outstand_q <= ar_outstand_q - 4'd1;
        default: ar_outstand_q <= ar_outstand_q;
      endcase
    end
  end

  // 쓰기 엔진 (AW/W/B) <- output_buffer (1비트 버스트, HP3 스타일 FSM)
  typedef enum logic [1:0] { WR_IDLE, WR_AW_DONE, WR_W_DONE } wr_state_t;
  wr_state_t            wr_state_q;
  logic [AxiAddrW-1:0]  wr_addr_q;
  logic [BeatCntW-1:0]  w_rem_q;       // 아직 못 쓴 비트 수 (0이면 AW/W 완료)
  logic [3:0]           aw_outstand_q;
  logic                 write_active_q;
  logic [AxiAddrW-1:0]  pend_addr_q;
  logic [AxiDataW-1:0]  pend_data_q;

  wire wr_can_emit_w   = (aw_outstand_q < 4'(OutstandMax));
  wire wr_emit_intent_w = (wr_state_q == WR_IDLE) & write_active_q &
                          (w_rem_q != '0) & ob_pop_valid_i & wr_can_emit_w;

  assign awvalid_o = (wr_state_q == WR_W_DONE) | wr_emit_intent_w;
  assign wvalid_o  = (wr_state_q == WR_AW_DONE) | wr_emit_intent_w;
  assign awaddr_o  = (wr_state_q == WR_W_DONE) ? pend_addr_q : wr_addr_q;
  assign awlen_o   = 8'd0;
  assign awsize_o  = Size8B;
  assign awburst_o = BurstIncr;
  assign awid_o    = '0;
  assign wdata_o   = (wr_state_q == WR_AW_DONE) ? pend_data_q : ob_pop_data_i;
  assign wstrb_o   = '1;             // 0xFF: 28x28은 정확히 8바이트 비트로 나눠짐
  assign wlast_o   = 1'b1;
  assign bready_o  = 1'b1;

  wire aw_fire_w = awvalid_o & awready_i;
  wire w_fire_w  = wvalid_o  & wready_i;
  wire b_fire_w  = bvalid_i  & bready_o;

  wire beat_done_w =
       ((wr_state_q == WR_IDLE)    & wr_emit_intent_w & aw_fire_w & w_fire_w)
     | ((wr_state_q == WR_AW_DONE) & w_fire_w)
     | ((wr_state_q == WR_W_DONE)  & aw_fire_w);

  // 쓰기 비트 하나가 끝날 때마다 output_buffer를 딱 한 번 팝
  assign ob_pop_ready_o = beat_done_w;

  wire write_complete_w = write_active_q & (w_rem_q == '0) &
                          (wr_state_q == WR_IDLE) & (aw_outstand_q == 4'd0);

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      wr_state_q     <= WR_IDLE;
      wr_addr_q      <= '0;
      w_rem_q        <= '0;
      aw_outstand_q  <= '0;
      write_active_q <= 1'b0;
      pend_addr_q    <= '0;
      pend_data_q    <= '0;
    end else begin
      if (start_i) begin
        wr_state_q     <= WR_IDLE;
        wr_addr_q      <= out_base_i;
        w_rem_q        <= BeatCntW'(Beats);
        write_active_q <= 1'b1;
      end else begin
        // FSM 전이
        unique case (wr_state_q)
          WR_IDLE: begin
            if (wr_emit_intent_w) begin
              if (aw_fire_w & ~w_fire_w) begin
                wr_state_q  <= WR_AW_DONE;   // AW는 받았고 W는 아직
                pend_data_q <= ob_pop_data_i;
              end else if (~aw_fire_w & w_fire_w) begin
                wr_state_q  <= WR_W_DONE;    // W는 받았고 AW는 아직
                pend_addr_q <= wr_addr_q;
              end
            end
          end
          WR_AW_DONE: if (w_fire_w)  wr_state_q <= WR_IDLE;
          WR_W_DONE : if (aw_fire_w) wr_state_q <= WR_IDLE;
          default   : wr_state_q <= WR_IDLE;
        endcase

        // 비트 하나 완료: 주소와 남은 개수 갱신
        if (beat_done_w) begin
          wr_addr_q <= wr_addr_q + AxiAddrW'(AxiDataW/8);  // +8바이트
          w_rem_q   <= w_rem_q - BeatCntW'(1);
        end

        // 마지막 B 응답이 오면 종료 처리
        if (write_complete_w) begin
          write_active_q <= 1'b0;
        end
      end

      // 미완료 쓰기 버스트 수: AW fire에 +1, B에 -1
      unique case ({aw_fire_w, b_fire_w})
        2'b10:   aw_outstand_q <= aw_outstand_q + 4'd1;
        2'b01:   aw_outstand_q <= aw_outstand_q - 4'd1;
        default: aw_outstand_q <= aw_outstand_q;
      endcase
    end
  end

  // 상태 신호
  wire read_active_w = (r_rem_q != '0);
  assign busy_o = read_active_w | write_active_q;
  // 두 방향 중 나중에 끝나는 쪽인 쓰기 엔진이 정리되는 클럭에 done 펄스
  assign done_o = write_complete_w;

  // 사용 안 함 표시 (이 단계에서는 응답이 항상 OKAY라고 가정)
  wire _unused = &{1'b0, rresp_i, bresp_i};

endmodule

`default_nettype wire

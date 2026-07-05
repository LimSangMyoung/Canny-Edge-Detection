// tb_ddr_path - DDR 경로 테스트벤치 (ddr_interface + input_buffer + output_buffer)
//
// 파이프라인 중간(gaussian~hysteresis)은 아직 스텁 -> 루프백으로 대체
// in_base 이미지가 out_base에 그대로 써지는지만 확인
//
// 실행 (Vivado xsim):
//   xvlog -sv tb/tb_ddr_path.sv rtl/ddr_interface.sv rtl/input_buffer.sv \
//             rtl/output_buffer.sv
//   xelab -debug typical tb_ddr_path -s sim
//   xsim sim -runall

`timescale 1ns / 1ps
`default_nettype none

module tb_ddr_path;

  localparam int unsigned AxiAddrW = 32;
  localparam int unsigned AxiDataW = 64;
  localparam int unsigned AxiIdW   = 6;
  localparam int unsigned Beats    = 98;       // 28*28 / 8
  localparam int unsigned TotalPix = 784;

  localparam logic [AxiAddrW-1:0] InBase  = 32'h0010_0000;
  localparam logic [AxiAddrW-1:0] OutBase = 32'h0020_0000;

  // DDR 백킹 스토어 (바이트 주소, sparse한 구간을 배열로 모델링)
  // 인덱스 0이 각각 InBase / OutBase에 대응한다.
  logic [7:0] ddr_in  [0:TotalPix-1];
  logic [7:0] ddr_out [0:TotalPix-1];

  logic clk = 1'b0;
  logic rst_ni = 1'b0;
  always #5 clk = ~clk;   // 100 MHz

  // 제어: ddr_interface 입력은 이제 axil_ctrl이 구동함
  logic                start;                 // axil_ctrl.start_o
  logic [31:0]         in_base, out_base;     // axil_ctrl의 base 출력
  logic                busy, done;            // ddr_interface 상태
  logic                irq;                   // axil_ctrl.irq_o (sticky DONE)

  // AXI4-Lite 마스터 (tb -> axil_ctrl)
  localparam int unsigned LiteAddrW = 4;
  logic [LiteAddrW-1:0] l_awaddr;
  logic                 l_awvalid, l_awready;
  logic [31:0]          l_wdata;
  logic [3:0]           l_wstrb;
  logic                 l_wvalid,  l_wready;
  logic [1:0]           l_bresp;
  logic                 l_bvalid,  l_bready;
  logic [LiteAddrW-1:0] l_araddr;
  logic                 l_arvalid, l_arready;
  logic [31:0]          l_rdata;
  logic [1:0]           l_rresp;
  logic                 l_rvalid,  l_rready;

  // 레지스터 바이트 오프셋
  localparam logic [3:0] REG_CTRL = 4'h0, REG_STATUS = 4'h4,
                         REG_IN = 4'h8, REG_OUT = 4'hC;

  // ddr_interface <-> 버퍼들
  logic                ib_wr_valid, ib_wr_ready, ib_stream_start;
  logic [AxiDataW-1:0] ib_wr_data;
  logic                ob_pop_valid, ob_pop_ready;
  logic [AxiDataW-1:0] ob_pop_data;

  // AXI 읽기
  logic                arvalid, arready;
  logic [AxiAddrW-1:0] araddr;
  logic [7:0]          arlen;
  logic [2:0]          arsize;
  logic [1:0]          arburst;
  logic [AxiIdW-1:0]   arid;
  logic                rvalid, rready, rlast;
  logic [AxiDataW-1:0] rdata;
  logic [1:0]          rresp;

  // AXI 쓰기
  logic                awvalid, awready;
  logic [AxiAddrW-1:0] awaddr;
  logic [7:0]          awlen;
  logic [2:0]          awsize;
  logic [1:0]          awburst;
  logic [AxiIdW-1:0]   awid;
  logic                wvalid, wready, wlast;
  logic [AxiDataW-1:0] wdata;
  logic [7:0]          wstrb;
  logic                bvalid, bready;
  logic [1:0]          bresp;

  // 픽셀 스트림 루프백 (input_buffer -> output_buffer)
  logic                pix_valid;
  logic [7:0]          pix_data;
  logic                frame_done, streaming;
  logic [7:0]          occupancy;

  // DUT들
  axil_ctrl #(
    .AddrW(LiteAddrW), .DataW(32)
  ) u_ctrl (
    .clk(clk), .rst_ni(rst_ni),
    .s_axil_awaddr(l_awaddr), .s_axil_awprot(3'b000), .s_axil_awvalid(l_awvalid),
    .s_axil_awready(l_awready),
    .s_axil_wdata(l_wdata), .s_axil_wstrb(l_wstrb), .s_axil_wvalid(l_wvalid),
    .s_axil_wready(l_wready),
    .s_axil_bresp(l_bresp), .s_axil_bvalid(l_bvalid), .s_axil_bready(l_bready),
    .s_axil_araddr(l_araddr), .s_axil_arprot(3'b000), .s_axil_arvalid(l_arvalid),
    .s_axil_arready(l_arready),
    .s_axil_rdata(l_rdata), .s_axil_rresp(l_rresp), .s_axil_rvalid(l_rvalid),
    .s_axil_rready(l_rready),
    .start_o(start), .in_base_o(in_base), .out_base_o(out_base),
    .busy_i(busy), .done_i(done), .irq_o(irq)
  );

  ddr_interface #(
    .AxiAddrW(AxiAddrW), .AxiDataW(AxiDataW), .AxiIdW(AxiIdW), .Beats(Beats)
  ) u_ddr (
    .clk(clk), .rst_ni(rst_ni),
    .start_i(start), .in_base_i(in_base), .out_base_i(out_base),
    .busy_o(busy), .done_o(done),
    .ib_wr_valid_o(ib_wr_valid), .ib_wr_data_o(ib_wr_data),
    .ib_wr_ready_i(ib_wr_ready), .ib_stream_start_o(ib_stream_start),
    .ob_pop_valid_i(ob_pop_valid), .ob_pop_data_i(ob_pop_data),
    .ob_pop_ready_o(ob_pop_ready),
    .arvalid_o(arvalid), .arready_i(arready), .araddr_o(araddr), .arlen_o(arlen),
    .arsize_o(arsize), .arburst_o(arburst), .arid_o(arid),
    .rvalid_i(rvalid), .rready_o(rready), .rdata_i(rdata), .rlast_i(rlast),
    .rresp_i(rresp),
    .awvalid_o(awvalid), .awready_i(awready), .awaddr_o(awaddr), .awlen_o(awlen),
    .awsize_o(awsize), .awburst_o(awburst), .awid_o(awid),
    .wvalid_o(wvalid), .wready_i(wready), .wdata_o(wdata), .wstrb_o(wstrb),
    .wlast_o(wlast), .bvalid_i(bvalid), .bready_o(bready), .bresp_i(bresp)
  );

  input_buffer #(
    .AxiDataW(AxiDataW), .TotalPix(TotalPix)
  ) u_ib (
    .clk(clk), .rst_ni(rst_ni),
    .wr_valid_i(ib_wr_valid), .wr_data_i(ib_wr_data), .wr_ready_o(ib_wr_ready),
    .stream_start_i(ib_stream_start),
    .pix_valid_o(pix_valid), .pix_data_o(pix_data),
    .frame_done_o(frame_done), .streaming_o(streaming)
  );

  output_buffer #(
    .AxiDataW(AxiDataW)
  ) u_ob (
    .clk(clk), .rst_ni(rst_ni),
    .pix_valid_i(pix_valid), .pix_data_i(pix_data),
    .pop_valid_o(ob_pop_valid), .pop_data_o(ob_pop_data),
    .pop_ready_i(ob_pop_ready), .occupancy_o(occupancy)
  );

  // AXI 슬레이브 (DDR) BFM
  // 읽기 채널: AR을 받으면 ddr_in에서 arlen+1개 비트를 스트리밍
  int unsigned r_beat_idx;     // 현재 버스트에서 ddr_in을 가리키는 byte/8 인덱스
  int unsigned r_beats_left;
  logic        r_busy;

  assign arready = ~r_busy;
  assign rresp   = 2'b00;

  always_ff @(posedge clk or negedge rst_ni) begin
    if (!rst_ni) begin
      r_busy <= 1'b0; rvalid <= 1'b0; rlast <= 1'b0; rdata <= '0;
      r_beat_idx <= 0; r_beats_left <= 0;
    end else begin
      // 새 AR을 받음
      if (arvalid && arready) begin
        r_busy       <= 1'b1;
        r_beat_idx   <= (araddr - InBase) >> 3;   // 8바이트/비트
        r_beats_left <= arlen + 1;
      end
      // R 비트 송출
      if (r_busy) begin
        if (!rvalid || (rvalid && rready)) begin
          if (r_beats_left != 0) begin
            for (int b = 0; b < 8; b++)
              rdata[b*8 +: 8] <= ddr_in[r_beat_idx*8 + b];
            rvalid <= 1'b1;
            rlast  <= (r_beats_left == 1);
            r_beat_idx   <= r_beat_idx + 1;
            r_beats_left <= r_beats_left - 1;
          end else begin
            rvalid <= 1'b0;
            rlast  <= 1'b0;
            r_busy <= 1'b0;
          end
        end
      end else begin
        if (rvalid && rready) rvalid <= 1'b0;
      end
    end
  end

  // 쓰기 채널: AW를 받고 W 비트들을 받아서 B로 응답
  // 1비트 버스트. 이 설계에서는 AW와 W가 같이 fire되거나 AW가 먼저 오는데,
  // 실제로 저장에 쓰는 주소는 AW가 같이 왔으면 awaddr, 아니면 마지막에
  // 잡아둔 AW 주소를 쓴다.
  logic [AxiAddrW-1:0] aw_addr_reg;
  logic [AxiAddrW-1:0] waddr_eff;
  int unsigned         w_beat_idx;
  int unsigned         b_pending;

  assign awready   = 1'b1;        // AW는 항상 받음
  assign wready    = 1'b1;        // W도 항상 받음
  assign bresp     = 2'b00;
  assign waddr_eff = (awvalid && awready) ? awaddr : aw_addr_reg;

  always_ff @(posedge clk or negedge rst_ni) begin
    if (!rst_ni) begin
      aw_addr_reg <= '0; w_beat_idx <= 0; b_pending <= 0; bvalid <= 1'b0;
    end else begin
      // AW 주소를 잡아둠 (AW가 W보다 먼저 오는 경우까지 커버)
      if (awvalid && awready)
        aw_addr_reg <= awaddr;

      // WSTRB를 지키면서 유효 주소에 W 비트를 저장
      if (wvalid && wready) begin
        w_beat_idx = (waddr_eff - OutBase) >> 3;
        for (int b = 0; b < 8; b++)
          if (wstrb[b]) ddr_out[w_beat_idx*8 + b] <= wdata[b*8 +: 8];
        b_pending <= b_pending + 1;
      end

      // B 응답 송출
      if (bvalid && bready) begin
        bvalid <= 1'b0;
        if (!(wvalid && wready)) b_pending <= b_pending - 1;
      end
      if (!bvalid && (b_pending != 0)) begin
        bvalid <= 1'b1;
        if (!(wvalid && wready)) b_pending <= b_pending - 1;
      end
    end
  end

  // AXI4-Lite 마스터 태스크 (tb -> axil_ctrl)
  task automatic axil_write(input [LiteAddrW-1:0] addr, input [31:0] data);
    begin
      @(posedge clk);
      l_awaddr  <= addr;  l_awvalid <= 1'b1;
      l_wdata   <= data;  l_wstrb   <= 4'hF; l_wvalid <= 1'b1;
      l_bready  <= 1'b1;
      // AW와 W가 둘 다 받아들여질 때까지 대기
      fork
        begin wait (l_awready); @(posedge clk); l_awvalid <= 1'b0; end
        begin wait (l_wready);  @(posedge clk); l_wvalid  <= 1'b0; end
      join
      wait (l_bvalid);
      @(posedge clk);
      l_bready <= 1'b0;
    end
  endtask

  task automatic axil_read(input [LiteAddrW-1:0] addr, output [31:0] data);
    begin
      @(posedge clk);
      l_araddr  <= addr; l_arvalid <= 1'b1; l_rready <= 1'b1;
      wait (l_arready); @(posedge clk); l_arvalid <= 1'b0;
      wait (l_rvalid);  data = l_rdata; @(posedge clk); l_rready <= 1'b0;
    end
  endtask

  // 자극(stimulus) + 체크
  int unsigned errors = 0;
  logic [31:0] status;

  initial begin
    // DDR을 알려진 램프 패턴으로 초기화
    for (int i = 0; i < TotalPix; i++) begin
      ddr_in[i]  = (i * 7 + 3) & 8'hFF;   // 그냥 만든 패턴
      ddr_out[i] = 8'hAA;                 // 오염값(poison)
    end

    // AXI-Lite 마스터 초기 idle 상태
    l_awaddr = '0; l_awvalid = 0; l_wdata = '0; l_wstrb = 0; l_wvalid = 0;
    l_bready = 0;  l_araddr = '0; l_arvalid = 0; l_rready = 0;

    rst_ni = 1'b0;
    repeat (4) @(posedge clk);
    rst_ni = 1'b1;
    repeat (2) @(posedge clk);

    // AXI-Lite로 베이스 주소를 설정하고 시작
    axil_write(REG_IN,  InBase);
    axil_write(REG_OUT, OutBase);
    axil_read (REG_IN,  status);
    if (status !== InBase) begin errors++; $display("REG_IN readback %08x", status); end
    axil_write(REG_CTRL, 32'h0000_0001);   // START

    // STATUS.DONE(bit1)을 타임아웃 걸고 폴링
    fork
      begin
        status = 32'd0;
        while (status[1] !== 1'b1) axil_read(REG_STATUS, status);
      end
      begin
        repeat (20000) @(posedge clk);
        $fatal(1, "TIMEOUT: STATUS.DONE never set");
      end
    join_any
    disable fork;

    if (irq !== 1'b1) begin errors++; $display("FAIL: irq not asserted with DONE"); end

    repeat (4) @(posedge clk);

    // 루프백 검증: ddr_out == ddr_in이어야 함
    for (int i = 0; i < TotalPix; i++) begin
      if (ddr_out[i] !== ddr_in[i]) begin
        errors++;
        if (errors <= 16)
          $display("MISMATCH px %0d: in=%02x out=%02x", i, ddr_in[i], ddr_out[i]);
      end
    end

    if (errors == 0) $display("PASS: %0d pixels looped back correctly", TotalPix);
    else             $display("FAIL: %0d mismatches", errors);

    $finish;
  end

  // 프로토콜 검증용: 이 설계에서는 쓰기가 항상 풀 strobe여야 함
  always_ff @(posedge clk) begin
    if (rst_ni && wvalid && wready && (wstrb !== 8'hFF))
      $display("WARN: non-0xFF wstrb=%02x (expected full beats)", wstrb);
  end

endmodule

`default_nettype wire

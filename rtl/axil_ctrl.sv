// axil_ctrl - Canny DDR 경로용 AXI4-Lite 제어/상태 레지스터
//
// PS가 레지스터 써서 ddr_interface 제어 + status 폴링
//
// 0x00 CTRL    (W) : bit0 START, 쓰면 자동클리어, DONE도 같이 클리어
// 0x04 STATUS  (R) : bit0 BUSY(레벨), bit1 DONE(sticky)
// 0x08 IN_BASE (RW): DDR 소스 주소, 8바이트 정렬
// 0x0C OUT_BASE(RW): DDR 목적지 주소, 8바이트 정렬
//
// irq_o = DONE 레벨

`default_nettype none

module axil_ctrl #(
  parameter int unsigned AddrW = 4,    // 16바이트 -> 레지스터 4개, [3:2]로 디코딩
  parameter int unsigned DataW = 32
) (
  input  wire               clk,
  input  wire               rst_ni,

  // AXI4-Lite 슬레이브 포트
  input  wire [AddrW-1:0]   s_axil_awaddr,
  input  wire [2:0]         s_axil_awprot,
  input  wire               s_axil_awvalid,
  output wire               s_axil_awready,
  input  wire [DataW-1:0]   s_axil_wdata,
  input  wire [DataW/8-1:0] s_axil_wstrb,
  input  wire               s_axil_wvalid,
  output wire               s_axil_wready,
  output wire [1:0]         s_axil_bresp,
  output wire               s_axil_bvalid,
  input  wire               s_axil_bready,
  input  wire [AddrW-1:0]   s_axil_araddr,
  input  wire [2:0]         s_axil_arprot,
  input  wire               s_axil_arvalid,
  output wire               s_axil_arready,
  output wire [DataW-1:0]   s_axil_rdata,
  output wire [1:0]         s_axil_rresp,
  output wire               s_axil_rvalid,
  input  wire               s_axil_rready,

  // ddr_interface로 나가는 신호
  output wire               start_o,        // 1클럭 펄스
  output wire [31:0]        in_base_o,
  output wire [31:0]        out_base_o,
  input  wire               busy_i,
  input  wire               done_i,         // 1클럭 펄스

  // 인터럽트
  output wire               irq_o
);

  // 레지스터 파일
  logic [31:0] in_base_q;
  logic [31:0] out_base_q;
  logic        done_q;
  logic        start_q;     // 1클럭 start 펄스

  // Write 채널 (AW와 W를 같이 받은 다음 B로 응답)
  logic               awready_q;
  logic               wready_q;
  logic               aw_en_q;       // 새 write 트랜잭션을 받을 수 있는 상태인지
  logic [AddrW-1:0]   awaddr_q;
  logic               bvalid_q;

  assign s_axil_awready = awready_q;
  assign s_axil_wready  = wready_q;
  assign s_axil_bvalid  = bvalid_q;
  assign s_axil_bresp   = 2'b00;     // OKAY

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      awready_q <= 1'b0;
      aw_en_q   <= 1'b1;
      awaddr_q  <= '0;
    end else begin
      if (~awready_q && s_axil_awvalid && s_axil_wvalid && aw_en_q) begin
        awready_q <= 1'b1;
        awaddr_q  <= s_axil_awaddr;
        aw_en_q   <= 1'b0;
      end else if (s_axil_bready && bvalid_q) begin
        aw_en_q   <= 1'b1;
        awready_q <= 1'b0;
      end else begin
        awready_q <= 1'b0;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_ni)
      wready_q <= 1'b0;
    else if (~wready_q && s_axil_wvalid && s_axil_awvalid && aw_en_q)
      wready_q <= 1'b1;
    else
      wready_q <= 1'b0;
  end

  wire wr_en_w = awready_q && s_axil_awvalid && wready_q && s_axil_wvalid;

  // 베이스 주소 레지스터에 바이트 단위로 write strobe를 적용해주는 함수
  function automatic [31:0] apply_wstrb(input [31:0] cur,
                                        input [31:0] data,
                                        input [3:0]  strb);
    apply_wstrb = cur;
    for (int b = 0; b < 4; b++)
      if (strb[b]) apply_wstrb[b*8 +: 8] = data[b*8 +: 8];
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      in_base_q  <= '0;
      out_base_q <= '0;
      done_q     <= 1'b0;
      start_q    <= 1'b0;
    end else begin
      start_q <= 1'b0;                       // 기본값은 펄스 로우

      if (wr_en_w) begin
        unique case (awaddr_q[3:2])
          2'd0: if (s_axil_wstrb[0] && s_axil_wdata[0]) begin
                  start_q <= 1'b1;           // CTRL.START
                  done_q  <= 1'b0;           // 새로 시작하니 DONE도 지운다
                end
          2'd2: in_base_q  <= apply_wstrb(in_base_q,  s_axil_wdata, s_axil_wstrb);
          2'd3: out_base_q <= apply_wstrb(out_base_q, s_axil_wdata, s_axil_wstrb);
          default: ; // CTRL 읽기값과 STATUS는 PS 쪽에서 읽기 전용
        endcase
      end

      // DONE은 sticky 비트다. done_i 펄스가 오면 세팅되고 위에서 START를 쓰면 지워진다.
      if (done_i) done_q <= 1'b1;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_ni)
      bvalid_q <= 1'b0;
    else if (wr_en_w && ~bvalid_q)
      bvalid_q <= 1'b1;
    else if (s_axil_bready && bvalid_q)
      bvalid_q <= 1'b0;
  end

  // Read 채널
  logic             arready_q;
  logic [AddrW-1:0] araddr_q;
  logic             rvalid_q;
  logic [31:0]      rdata_q;

  assign s_axil_arready = arready_q;
  assign s_axil_rvalid  = rvalid_q;
  assign s_axil_rresp   = 2'b00;
  assign s_axil_rdata   = rdata_q;

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      arready_q <= 1'b0;
      araddr_q  <= '0;
    end else if (~arready_q && s_axil_arvalid) begin
      arready_q <= 1'b1;
      araddr_q  <= s_axil_araddr;
    end else begin
      arready_q <= 1'b0;
    end
  end

  wire rd_en_w = arready_q && s_axil_arvalid && ~rvalid_q;

  always_ff @(posedge clk) begin
    if (!rst_ni) begin
      rvalid_q <= 1'b0;
      rdata_q  <= '0;
    end else if (rd_en_w) begin
      rvalid_q <= 1'b1;
      unique case (araddr_q[3:2])
        2'd0:    rdata_q <= 32'd0;                               // CTRL (START는 자동 클리어)
        2'd1:    rdata_q <= {30'd0, done_q, busy_i};             // STATUS
        2'd2:    rdata_q <= in_base_q;
        2'd3:    rdata_q <= out_base_q;
        default: rdata_q <= 32'd0;
      endcase
    end else if (rvalid_q && s_axil_rready) begin
      rvalid_q <= 1'b0;
    end
  end

  // 출력
  assign start_o    = start_q;
  assign in_base_o  = in_base_q;
  assign out_base_o = out_base_q;
  assign irq_o      = done_q;

  // 안 쓰는 AXI-Lite prot 필드들
  wire _unused = &{1'b0, s_axil_awprot, s_axil_arprot};

endmodule

`default_nettype wire

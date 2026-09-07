`default_nettype none

// Reproducible Compact bandwidth load at 100 MHz:
// - 256 scan words per active 512-pixel line, 288 lines/frame.
// - Requests for eight-word GSP bursts (the Compact pixel-section width)
//   every 256 cycles. Back-pressure deliberately reveals the accepted rate.
module compact_vram_traffic (
  input  logic        clk_i,
  input  logic        rst_i,
  output logic        scan_req_o,
  output logic [17:0] scan_addr_o,
  input  logic        scan_ack_i,
  output logic        gsp_req_o,
  output logic        gsp_we_o,
  output logic [17:0] gsp_addr_o,
  output logic [15:0] gsp_wdata_o,
  input  logic        gsp_ack_i
);
  logic [12:0] line_cycle;
  logic [8:0]  line;
  logic [7:0]  burst_timer;
  logic [3:0]  burst_left;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      line_cycle  <= 13'd0;
      line        <= 9'd0;
      scan_req_o  <= 1'b0;
      scan_addr_o <= 18'd0;
      burst_timer <= 8'd0;
      burst_left  <= 4'd0;
      gsp_req_o   <= 1'b0;
      gsp_we_o    <= 1'b0;
      gsp_addr_o  <= 18'h20000;
      gsp_wdata_o <= 16'h1ace;
    end else begin
      scan_req_o <= 1'b0;
      if (line_cycle == 13'd5119) begin
        line_cycle <= 13'd0;
        line <= (line == 9'd325) ? 9'd0 : line + 9'd1;
      end else begin
        line_cycle <= line_cycle + 13'd1;
      end

      if ((line < 9'd288) && (line_cycle < 13'd4096) &&
          (line_cycle[3:0] == 4'd0)) begin
        scan_req_o  <= 1'b1;
        scan_addr_o <= (line * 18'd256) + line_cycle[11:4];
      end

      burst_timer <= burst_timer + 8'd1;
      if ((burst_timer == 8'hff) && (burst_left == 0) && !gsp_req_o)
        burst_left <= 4'd8;

      if ((burst_left != 0) && !gsp_req_o) begin
        gsp_req_o   <= 1'b1;
        gsp_we_o    <= gsp_addr_o[3];
        gsp_wdata_o <= gsp_wdata_o + 16'h1021;
      end else if (gsp_req_o && gsp_ack_i) begin
        gsp_req_o  <= 1'b0;
        gsp_addr_o <= gsp_addr_o + 18'd1;
        burst_left <= burst_left - 4'd1;
      end
    end
  end

  logic unused_scan_ack;
  assign unused_scan_ack = scan_ack_i;
endmodule

`default_nettype wire

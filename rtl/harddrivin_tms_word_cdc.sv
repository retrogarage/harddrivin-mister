`default_nettype none

// Completed 16-bit memory cycle bridge. The source request remains asserted
// by the TMS34010 fabric until the response mailbox returns exactly one ack.
module harddrivin_tms_word_cdc #(parameter int ADDR_W = 18, parameter int RDATA_W = 16, parameter int WDATA_W = 16, parameter int BE_W = 2) (
  input  logic        src_clk_i,
  input  logic        src_rst_i,
  input  logic        src_req_i,
  input  logic        src_we_i,
  input  logic [BE_W-1:0] src_be_i,
  input  logic [ADDR_W-1:0] src_addr_i,
  input  logic [WDATA_W-1:0] src_wdata_i,
  output logic [RDATA_W-1:0] src_rdata_o,
  output logic        src_ack_o,

  input  logic        dst_clk_i,
  input  logic        dst_rst_i,
  output logic        dst_req_o,
  output logic        dst_we_o,
  output logic [BE_W-1:0] dst_be_o,
  output logic [ADDR_W-1:0] dst_addr_o,
  output logic [WDATA_W-1:0] dst_wdata_o,
  input  logic [RDATA_W-1:0] dst_rdata_i,
  input  logic        dst_ack_i
);
  localparam int CMD_WIDTH = 1 + BE_W + ADDR_W + WDATA_W;

  logic [CMD_WIDTH-1:0] cmd_src_data;
  logic                 cmd_src_accept;
  logic [CMD_WIDTH-1:0] cmd_dst_data;
  logic                 cmd_dst_valid;
  logic                 src_seen_q;
  logic                 src_inflight_q;

  logic [RDATA_W-1:0] response_src_data;
  logic        response_src_valid;
  logic        response_src_accept;
  logic [RDATA_W-1:0] response_dst_data;
  logic        response_dst_valid;
  logic        response_pending_q;

  assign cmd_src_data = {src_we_i, src_be_i, src_addr_i, src_wdata_i};

  tms34010_cdc_mailbox #(.WIDTH(CMD_WIDTH)) u_command_mailbox (
    .src_clk_i(src_clk_i), .src_rst_i(src_rst_i),
    .src_valid_i(src_req_i && !src_seen_q && !src_inflight_q),
    .src_data_i(cmd_src_data), .src_ready_o(),
    .src_accept_o(cmd_src_accept),
    .dst_clk_i(dst_clk_i), .dst_rst_i(dst_rst_i),
    .dst_data_o(cmd_dst_data), .dst_valid_o(cmd_dst_valid)
  );

  tms34010_cdc_mailbox #(.WIDTH(RDATA_W)) u_response_mailbox (
    .src_clk_i(dst_clk_i), .src_rst_i(dst_rst_i),
    .src_valid_i(response_src_valid), .src_data_i(response_src_data),
    .src_ready_o(), .src_accept_o(response_src_accept),
    .dst_clk_i(src_clk_i), .dst_rst_i(src_rst_i),
    .dst_data_o(response_dst_data), .dst_valid_o(response_dst_valid)
  );

  always_ff @(posedge src_clk_i) begin
    if (src_rst_i) begin
      src_seen_q     <= 1'b0;
      src_inflight_q <= 1'b0;
      src_rdata_o    <= '0;
      src_ack_o      <= 1'b0;
    end else begin
      src_ack_o <= 1'b0;
      if (!src_req_i) src_seen_q <= 1'b0;
      if (cmd_src_accept) begin
        src_seen_q     <= 1'b1;
        src_inflight_q <= 1'b1;
      end
      if (response_dst_valid) begin
        src_rdata_o    <= response_dst_data;
        src_ack_o      <= 1'b1;
        src_inflight_q <= 1'b0;
      end
    end
  end

  assign response_src_valid = response_pending_q;

  always_ff @(posedge dst_clk_i) begin
    if (dst_rst_i) begin
      dst_req_o          <= 1'b0;
      dst_we_o           <= 1'b0;
      dst_be_o           <= '1;
      dst_addr_o         <= 18'd0;
      dst_wdata_o        <= '0;
      response_src_data  <= '0;
      response_pending_q <= 1'b0;
    end else begin
      if (cmd_dst_valid && !dst_req_o) begin
        dst_req_o   <= 1'b1;
        dst_we_o    <= cmd_dst_data[CMD_WIDTH-1];
        dst_be_o    <= cmd_dst_data[CMD_WIDTH-2 -: BE_W];
        dst_addr_o  <= cmd_dst_data[WDATA_W+ADDR_W-1 -: ADDR_W];
        dst_wdata_o <= cmd_dst_data[WDATA_W-1:0];
      end

      if (dst_req_o && dst_ack_i) begin
        dst_req_o          <= 1'b0;
        response_src_data  <= dst_rdata_i;
        response_pending_q <= 1'b1;
      end

      if (response_src_accept)
        response_pending_q <= 1'b0;
    end
  end
endmodule

`default_nettype wire

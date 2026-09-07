// -----------------------------------------------------------------------------
// tms34010_cdc_mailbox.sv
//
// One-entry, source-ready multi-cycle-path (MCP) mailbox for an occasional
// packed word crossing between asynchronous clock domains.
//
// The source captures the complete payload and holds it stable while a
// request toggle crosses through a dedicated 2FF synchronizer. The destination
// captures that deliberately unsynchronized but multi-cycle-stable payload,
// emits one valid pulse, and returns the observed toggle through a second 2FF
// synchronizer. No payload bit is synchronized independently.
//
// The source may launch only while src_ready_o is high. Both synchronous
// resets must be asserted together so the request/acknowledge phases return to
// zero. The project SDC must declare the clocks asynchronous, preserve the
// synchronizer chains, and cut/waive only the protocol-protected payload path.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_cdc_mailbox #(
  parameter int WIDTH = 1
)(
  input  logic                 src_clk_i,
  input  logic                 src_rst_i,
  input  logic                 src_valid_i,
  input  logic [WIDTH-1:0]     src_data_i,
  output logic                 src_ready_o,
  output logic                 src_accept_o,

  input  logic                 dst_clk_i,
  input  logic                 dst_rst_i,
  output logic [WIDTH-1:0]     dst_data_o,
  output logic                 dst_valid_o
);

  logic [WIDTH-1:0] src_payload_q;
  logic             src_request_toggle_q;
  logic             src_ack_toggle_sync;
  logic             dst_request_toggle_sync;
  logic             dst_ack_toggle_q;

  tms34010_sync_bit u_request_sync (
    .clk     (dst_clk_i),
    .rst     (dst_rst_i),
    .async_i (src_request_toggle_q),
    .sync_o  (dst_request_toggle_sync)
  );

  tms34010_sync_bit u_ack_sync (
    .clk     (src_clk_i),
    .rst     (src_rst_i),
    .async_i (dst_ack_toggle_q),
    .sync_o  (src_ack_toggle_sync)
  );

  assign src_ready_o  = (src_request_toggle_q == src_ack_toggle_sync);
  assign src_accept_o = src_valid_i && src_ready_o;

  always_ff @(posedge src_clk_i) begin
    if (src_rst_i) begin
      src_payload_q        <= '0;
      src_request_toggle_q <= 1'b0;
    end else if (src_accept_o) begin
      src_payload_q        <= src_data_i;
      src_request_toggle_q <= ~src_request_toggle_q;
    end
  end

  always_ff @(posedge dst_clk_i) begin
    if (dst_rst_i) begin
      dst_data_o      <= '0;
      dst_valid_o     <= 1'b0;
      dst_ack_toggle_q <= 1'b0;
    end else begin
      dst_valid_o <= 1'b0;
      if (dst_request_toggle_sync != dst_ack_toggle_q) begin
        dst_data_o       <= src_payload_q;
        dst_valid_o      <= 1'b1;
        dst_ack_toggle_q <= dst_request_toggle_sync;
      end
    end
  end

endmodule : tms34010_cdc_mailbox

`default_nettype wire

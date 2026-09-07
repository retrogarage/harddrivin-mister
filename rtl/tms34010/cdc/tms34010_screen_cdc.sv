// -----------------------------------------------------------------------------
// tms34010_screen_cdc.sv
//
// Held screen-refresh transaction crossing from VCLK into the core/memory
// clock domain. This is a two-phase MCP request/complete handshake:
//
//   * VCLK captures SRFADR/DPYTAP/ORG and holds that packed payload while a
//     request toggle crosses through a dedicated 2FF synchronizer.
//   * The core captures the stable payload and holds core_req_o through
//     arbitrary memory-fabric waits.
//   * Only core_ack_i returns the observed toggle. VCLK then emits exactly one
//     source-clock completion pulse to retire the display scheduler's request.
//
// Payload bits never pass through parallel synchronizers. Both synchronous
// resets must assert together. The final SDC must declare the clocks
// asynchronous and cut/waive the stable MCP payload path only.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_screen_cdc
  import tms34010_pkg::*;
(
  input  logic        video_clk_i,
  input  logic        video_rst_i,
  input  logic        video_req_i,
  input  logic [13:0] video_srfaddr_i,
  input  logic [15:0] video_dpytap_i,
  input  logic        video_org_i,
  output logic        video_ack_o,

  input  logic        core_clk_i,
  input  logic        core_rst_i,
  output logic        core_req_o,
  output logic [13:0] core_srfaddr_o,
  output logic [15:0] core_dpytap_o,
  output logic        core_org_o,
  input  logic        core_ack_i
);

  typedef struct packed {
    logic [13:0] srfaddr;
    logic [15:0] dpytap;
    logic        org;
  } screen_payload_t;

  screen_payload_t video_payload_q;

  logic video_request_toggle_q;
  logic video_request_armed_q;
  logic video_outstanding_q;
  logic video_ack_toggle_sync;

  logic core_request_toggle_sync;
  logic core_request_seen_q;
  logic core_ack_toggle_q;

  tms34010_sync_bit u_request_sync (
    .clk     (core_clk_i),
    .rst     (core_rst_i),
    .async_i (video_request_toggle_q),
    .sync_o  (core_request_toggle_sync)
  );

  tms34010_sync_bit u_ack_sync (
    .clk     (video_clk_i),
    .rst     (video_rst_i),
    .async_i (core_ack_toggle_q),
    .sync_o  (video_ack_toggle_sync)
  );

  always_ff @(posedge video_clk_i) begin
    if (video_rst_i) begin
      video_payload_q        <= '0;
      video_request_toggle_q <= 1'b0;
      video_request_armed_q  <= 1'b1;
      video_outstanding_q    <= 1'b0;
      video_ack_o            <= 1'b0;
    end else begin
      video_ack_o <= 1'b0;

      if (!video_req_i)
        video_request_armed_q <= 1'b1;

      if (video_outstanding_q) begin
        if (video_ack_toggle_sync == video_request_toggle_q) begin
          video_ack_o         <= 1'b1;
          video_outstanding_q <= 1'b0;
        end
      end else if (video_req_i && video_request_armed_q) begin
        video_payload_q.srfaddr <= video_srfaddr_i;
        video_payload_q.dpytap  <= video_dpytap_i;
        video_payload_q.org     <= video_org_i;
        video_request_toggle_q  <= ~video_request_toggle_q;
        video_request_armed_q   <= 1'b0;
        video_outstanding_q     <= 1'b1;
      end
    end
  end

  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      core_req_o            <= 1'b0;
      core_srfaddr_o        <= '0;
      core_dpytap_o         <= '0;
      core_org_o            <= 1'b0;
      core_request_seen_q   <= 1'b0;
      core_ack_toggle_q     <= 1'b0;
    end else if (core_req_o) begin
      if (core_ack_i) begin
        core_req_o        <= 1'b0;
        core_ack_toggle_q <= core_request_seen_q;
      end
    end else if (core_request_toggle_sync != core_request_seen_q) begin
      core_srfaddr_o      <= video_payload_q.srfaddr;
      core_dpytap_o       <= video_payload_q.dpytap;
      core_org_o          <= video_payload_q.org;
      core_request_seen_q <= core_request_toggle_sync;
      core_req_o          <= 1'b1;
    end
  end

endmodule : tms34010_screen_cdc

`default_nettype wire

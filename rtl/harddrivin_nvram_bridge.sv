`default_nettype none

// MiSTer byte-stream bridge for the Compact board's two 2 KiB battery-backed
// RAM lanes.  The HPS side must keep running while the emulated board is held
// in reset, so this bridge deliberately has no board-reset input.  A bundled
// toggle handshake holds address/data stable across the 50 MHz -> 48 MHz CDC.
module harddrivin_nvram_bridge #(
  parameter integer SAVE_DELAY_CLOCKS = 150_000_000
) (
  input  logic        hps_clk_i,
  input  logic        ioctl_download_i,
  input  logic        ioctl_upload_i,
  input  logic        ioctl_wr_i,
  input  tri0         ioctl_rd_i,
  input  logic [15:0] ioctl_index_i,
  input  logic [11:0] ioctl_addr_i,
  input  logic [7:0]  ioctl_dout_i,
  output logic        ioctl_wait_o,
  output logic [7:0]  ioctl_din_o,
  output logic        ioctl_upload_req_o,

  input  logic        core_clk_i,
  output logic        core_req_o,
  output logic        core_we_o,
  output logic [11:0] core_addr_o,
  output logic [7:0]  core_wdata_o,
  input  logic        core_ack_i,
  input  logic [7:0]  core_rdata_i,
  input  logic        core_dirty_i
);
  localparam integer SAVE_TIMER_WIDTH = $clog2(SAVE_DELAY_CLOCKS + 1);

  logic req_toggle_q = 1'b0;
  logic ack_toggle_q = 1'b0;
  logic dirty_toggle_q = 1'b0;
  logic ack_sync1_q = 1'b0, ack_sync2_q = 1'b0, ack_sync3_q = 1'b0;
  logic dirty_sync1_q = 1'b0, dirty_sync2_q = 1'b0, dirty_sync3_q = 1'b0;
  logic busy_q = 1'b0;
  logic we_q = 1'b0;
  logic [11:0] addr_q = 12'd0;
  logic [7:0] wdata_q = 8'd0;
  logic [7:0] rdata_q = 8'd0;
  logic [11:0] fetched_addr_q = 12'hfff;
  logic fetched_valid_q = 1'b0;
  logic [SAVE_TIMER_WIDTH-1:0] save_timer_q = '0;

  logic save_pending_q = 1'b0;
  logic upload_prev_q = 1'b0, upload_complete_q = 1'b0;
  logic dirty_during_upload_q = 1'b0;
  wire nv_upload = ioctl_upload_i && ioctl_index_i == 16'd2;
  wire dirty_event = dirty_sync3_q != dirty_sync2_q;

  logic req_sync1_q = 1'b0, req_sync2_q = 1'b0, req_sync3_q = 1'b0;
  logic upload_needs_fetch;

  // hps_io advances ioctl_addr on the same clk_sys edge on which it samples
  // ioctl_din.  `busy_q` cannot see that new address until the following
  // edge, leaving a one-cycle low-wait window in which the HPS can consume
  // the preceding byte twice.  Assert wait combinationally as soon as the
  // visible upload address differs from the byte we prefetched.
  assign upload_needs_fetch = ioctl_upload_i && (ioctl_index_i == 16'd2) &&
                              (!fetched_valid_q ||
                               (fetched_addr_q != ioctl_addr_i));
  assign ioctl_wait_o = busy_q || upload_needs_fetch;
  assign ioctl_din_o = rdata_q;
  assign core_we_o = we_q;
  assign core_addr_o = addr_q;
  assign core_wdata_o = wdata_q;

  always_ff @(posedge hps_clk_i) begin
    {ack_sync3_q, ack_sync2_q, ack_sync1_q} <=
        {ack_sync2_q, ack_sync1_q, ack_toggle_q};
    {dirty_sync3_q, dirty_sync2_q, dirty_sync1_q} <=
        {dirty_sync2_q, dirty_sync1_q, dirty_toggle_q};
    ioctl_upload_req_o <= 1'b0;

    if (busy_q) begin
      if (ack_sync3_q != ack_sync2_q) begin
        busy_q <= 1'b0;
        if (!we_q) begin
          rdata_q <= core_rdata_i;
          fetched_addr_q <= addr_q;
          fetched_valid_q <= 1'b1;
        end
      end
    end else if (ioctl_download_i && (ioctl_index_i == 16'd2) &&
                     ioctl_wr_i) begin
      we_q <= 1'b1;
      addr_q <= ioctl_addr_i;
      wdata_q <= ioctl_dout_i;
      req_toggle_q <= !req_toggle_q;
      busy_q <= 1'b1;
      fetched_valid_q <= 1'b0;
    end else if (upload_needs_fetch) begin
      we_q <= 1'b0;
      addr_q <= ioctl_addr_i;
      req_toggle_q <= !req_toggle_q;
      busy_q <= 1'b1;
    end

    if (!ioctl_upload_i) fetched_valid_q <= 1'b0;

    upload_prev_q <= nv_upload;
    if (!upload_prev_q && nv_upload) begin
      upload_complete_q <= 1'b0;
      dirty_during_upload_q <= 1'b0;
    end
    // Clear the pending save only after HPS has actually consumed the final
    // byte and ended the upload. ioctl_rd accompanies the next address. An interrupted transfer remains retryable.
    if (nv_upload && ioctl_rd_i && fetched_valid_q &&
        fetched_addr_q == 12'hfff && ioctl_addr_i == 12'h000)
      upload_complete_q <= 1'b1;
    if (upload_prev_q && !nv_upload && upload_complete_q &&
        !dirty_during_upload_q) begin
      save_pending_q <= 1'b0;
      save_timer_q <= '0;
    end else if (save_pending_q) begin
      if (save_timer_q > SAVE_TIMER_WIDTH'(1))
        save_timer_q <= save_timer_q - 1'b1;
      else if (!ioctl_download_i && !ioctl_upload_i && !busy_q) begin
        ioctl_upload_req_o <= 1'b1;
        save_timer_q <= SAVE_TIMER_WIDTH'(SAVE_DELAY_CLOCKS);
      end
    end
    // New writes win over completion, including the final upload clock.
    if (dirty_event) begin
      save_pending_q <= 1'b1;
      save_timer_q <= SAVE_TIMER_WIDTH'(SAVE_DELAY_CLOCKS);
      if (nv_upload || upload_prev_q) dirty_during_upload_q <= 1'b1;
    end

  end

  always_ff @(posedge core_clk_i) begin
    {req_sync3_q, req_sync2_q, req_sync1_q} <=
        {req_sync2_q, req_sync1_q, req_toggle_q};
    if (req_sync3_q != req_sync2_q) core_req_o <= 1'b1;
    if (core_ack_i) begin
      core_req_o <= 1'b0;
      ack_toggle_q <= !ack_toggle_q;
    end
    if (core_dirty_i) dirty_toggle_q <= !dirty_toggle_q;
  end

  initial begin
    core_req_o = 1'b0;
    ioctl_upload_req_o = 1'b0;
  end
endmodule

`default_nettype wire

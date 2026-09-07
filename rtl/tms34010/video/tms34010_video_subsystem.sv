// -----------------------------------------------------------------------------
// tms34010_video_subsystem.sv
//
// Dedicated VCLK-domain composition for TMS34010 timing and automatic screen
// refresh, with every core/VCLK crossing made explicit.
//
// Clock-domain ownership:
//   * core_clk_i owns the memory-mapped I/O storage and memory-fabric client;
//   * video_clk_i owns HCOUNT, VCOUNT, interlaced field phase, all timing
//     compares, DPYADR, and the screen-refresh scheduler.
//
// CDC patterns:
//   * complete timing/display configuration crosses as one atomic MCP mailbox
//     snapshot; writes while busy coalesce into a later latest-value snapshot;
//   * HCOUNT/VCOUNT/DPYADR writes cross as one coalescing command mailbox;
//   * the three live values return as one continuously refreshed coherent MCP
//     snapshot (the core view is bounded-stale, never bit-torn);
//   * display-interrupt events use a held one-entry toggle mailbox;
//   * screen requests use tms34010_screen_cdc, whose bundled payload remains
//     stable until the completed core-domain memory transaction returns.
//
// The clocks may have any phase or frequency relationship. Both synchronous
// resets must assert together. The active edge of video_clk_i is the FPGA
// domain representation of the original device's falling-VCLK state-update
// edge; final clock/pin mapping belongs in the Cyclone V top and SDC.
//
// Spec source:
//   1988 TI TMS34010 User's Guide §2.4, §§9.2-9.6, and §9.10.1.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_video_subsystem
  import tms34010_pkg::*;
(
  input  logic        core_clk_i,
  input  logic        core_rst_i,
  input  logic        video_clk_i,
  input  logic        video_rst_i,
  input  logic        hsync_n_i,
  input  logic        vsync_n_i,

  input  logic [15:0] hesync_i,
  input  logic [15:0] heblnk_i,
  input  logic [15:0] hsblnk_i,
  input  logic [15:0] htotal_i,
  input  logic [15:0] vesync_i,
  input  logic [15:0] veblnk_i,
  input  logic [15:0] vsblnk_i,
  input  logic [15:0] vtotal_i,
  input  logic [15:0] dpyint_i,
  input  logic [15:0] dpystart_i,
  input  logic [15:0] dpyctl_i,
  input  logic [15:0] dpytap_i,
  input  logic        config_write_i,

  input  logic        hcount_write_i,
  input  logic [15:0] hcount_wdata_i,
  input  logic        vcount_write_i,
  input  logic [15:0] vcount_wdata_i,
  input  logic        dpyadr_write_i,
  input  logic [15:0] dpyadr_wdata_i,

  output logic [15:0] hcount_o,
  output logic [15:0] vcount_o,
  output logic [15:0] dpyadr_o,
  output logic        dpyint_pulse_o,

  output logic        hsync_o,
  output logic        vsync_o,
  output logic        hblank_o,
  output logic        vblank_o,
  output logic        blank_o,
  output logic        hsync_oe_o,
  output logic        vsync_oe_o,

  output logic        screen_req_o,
  input  logic        screen_ack_i,
  output logic [13:0] screen_srfaddr_o,
  output logic [15:0] screen_dpytap_o,
  output logic        screen_org_o
);

  typedef struct packed {
    logic [15:0] hesync;
    logic [15:0] heblnk;
    logic [15:0] hsblnk;
    logic [15:0] htotal;
    logic [15:0] vesync;
    logic [15:0] veblnk;
    logic [15:0] vsblnk;
    logic [15:0] vtotal;
    logic [15:0] dpyint;
    logic [15:0] dpystart;
    logic [15:0] dpyctl;
    logic [15:0] dpytap;
  } video_config_t;

  typedef struct packed {
    logic        hcount_load;
    logic        vcount_load;
    logic        dpyadr_load;
    logic [15:0] hcount_wdata;
    logic [15:0] vcount_wdata;
    logic [15:0] dpyadr_wdata;
  } video_command_t;

  typedef struct packed {
    logic [15:0] hcount;
    logic [15:0] vcount;
    logic [15:0] dpyadr;
  } video_status_t;

  video_config_t  config_source;
  video_config_t  config_received;
  video_config_t  config_video_q;
  video_command_t command_source;
  video_command_t command_received;
  video_status_t  status_source;
  video_status_t  status_received;

  logic        config_dirty_q;
  logic        config_accept;
  logic [2:0]  command_pending_q;
  logic [15:0] hcount_pending_data_q;
  logic [15:0] vcount_pending_data_q;
  logic [15:0] dpyadr_pending_data_q;
  logic        command_accept;
  logic        config_valid_video;
  logic        command_valid_video;

  logic [15:0] hcount_video;
  logic [15:0] vcount_video;
  logic [15:0] dpyadr_video;
  logic        hblank_start_video;
  logic        dpyint_video;
  logic        odd_field_video;

  logic        dip_pending_q;
  logic        dip_accept;

  logic        screen_req_video;
  logic        screen_ack_video;
  logic [13:0] screen_srfaddr_video;
  logic [15:0] screen_dpytap_video;
  logic        screen_org_video;

  always_comb begin
    config_source          = '0;
    config_source.hesync   = hesync_i;
    config_source.heblnk   = heblnk_i;
    config_source.hsblnk   = hsblnk_i;
    config_source.htotal   = htotal_i;
    config_source.vesync   = vesync_i;
    config_source.veblnk   = veblnk_i;
    config_source.vsblnk   = vsblnk_i;
    config_source.vtotal   = vtotal_i;
    config_source.dpyint   = dpyint_i;
    config_source.dpystart = dpystart_i;
    config_source.dpyctl   = dpyctl_i;
    config_source.dpytap   = dpytap_i;

    command_source                = '0;
    command_source.hcount_load    = command_pending_q[0];
    command_source.vcount_load    = command_pending_q[1];
    command_source.dpyadr_load    = command_pending_q[2];
    command_source.hcount_wdata   = hcount_pending_data_q;
    command_source.vcount_wdata   = vcount_pending_data_q;
    command_source.dpyadr_wdata   = dpyadr_pending_data_q;

    status_source         = '0;
    status_source.hcount  = hcount_video;
    status_source.vcount  = vcount_video;
    status_source.dpyadr  = dpyadr_video;
  end

  // Source-side coalescing lets the completion-qualified I/O write port remain
  // nonblocking. A write coincident with an older mailbox acceptance wins and
  // therefore causes one later latest-value transfer.
  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      config_dirty_q         <= 1'b0;
      command_pending_q      <= 3'b000;
      hcount_pending_data_q  <= 16'h0000;
      vcount_pending_data_q  <= 16'h0000;
      dpyadr_pending_data_q  <= 16'h0000;
    end else begin
      if (config_accept)
        config_dirty_q <= 1'b0;
      if (config_write_i)
        config_dirty_q <= 1'b1;

      if (command_accept)
        command_pending_q <= 3'b000;

      if (hcount_write_i) begin
        command_pending_q[0]     <= 1'b1;
        hcount_pending_data_q    <= hcount_wdata_i;
      end
      if (vcount_write_i) begin
        command_pending_q[1]     <= 1'b1;
        vcount_pending_data_q    <= vcount_wdata_i;
      end
      if (dpyadr_write_i) begin
        command_pending_q[2]     <= 1'b1;
        dpyadr_pending_data_q    <= dpyadr_wdata_i;
      end
    end
  end

  tms34010_cdc_mailbox #(
    .WIDTH($bits(video_config_t))
  ) u_config_mailbox (
    .src_clk_i   (core_clk_i),
    .src_rst_i   (core_rst_i),
    .src_valid_i (config_dirty_q),
    .src_data_i  (config_source),
    .src_ready_o (),
    .src_accept_o(config_accept),
    .dst_clk_i   (video_clk_i),
    .dst_rst_i   (video_rst_i),
    .dst_data_o  (config_received),
    .dst_valid_o (config_valid_video)
  );

  always_ff @(posedge video_clk_i) begin
    if (video_rst_i) begin
      config_video_q <= '0;
    end else if (config_valid_video) begin
      config_video_q <= config_received;
    end
  end

  tms34010_cdc_mailbox #(
    .WIDTH($bits(video_command_t))
  ) u_command_mailbox (
    .src_clk_i   (core_clk_i),
    .src_rst_i   (core_rst_i),
    .src_valid_i (|command_pending_q),
    .src_data_i  (command_source),
    .src_ready_o (),
    .src_accept_o(command_accept),
    .dst_clk_i   (video_clk_i),
    .dst_rst_i   (video_rst_i),
    .dst_data_o  (command_received),
    .dst_valid_o (command_valid_video)
  );

  tms34010_video u_video (
    .clk           (video_clk_i),
    .rst           (video_rst_i),
    .hesync        (config_video_q.hesync),
    .heblnk        (config_video_q.heblnk),
    .hsblnk        (config_video_q.hsblnk),
    .htotal        (config_video_q.htotal),
    .vesync        (config_video_q.vesync),
    .veblnk        (config_video_q.veblnk),
    .vsblnk        (config_video_q.vsblnk),
    .vtotal        (config_video_q.vtotal),
    .dpyint        (config_video_q.dpyint),
    .display_enable(config_video_q.dpyctl[DPYCTL_ENV_BIT]),
    .noninterlaced (config_video_q.dpyctl[DPYCTL_NIL_BIT]),
    .disable_external_video(config_video_q.dpyctl[DPYCTL_DXV_BIT]),
    .hsync_direction(config_video_q.dpyctl[DPYCTL_HSD_BIT]),
    .hsync_n_i     (hsync_n_i),
    .vsync_n_i     (vsync_n_i),
    .hcount_load   (command_valid_video
                    && command_received.hcount_load),
    .hcount_wdata  (command_received.hcount_wdata),
    .vcount_load   (command_valid_video
                    && command_received.vcount_load),
    .vcount_wdata  (command_received.vcount_wdata),
    .hcount        (hcount_video),
    .vcount        (vcount_video),
    .hsync         (hsync_o),
    .vsync         (vsync_o),
    .hblank        (hblank_o),
    .vblank        (vblank_o),
    .blank         (blank_o),
    .hsync_oe      (hsync_oe_o),
    .vsync_oe      (vsync_oe_o),
    .hblank_start  (hblank_start_video),
    .dpyint_pulse  (dpyint_video),
    .odd_field     (odd_field_video)
  );

  tms34010_display_addr u_display_addr (
    .clk             (video_clk_i),
    .rst             (video_rst_i),
    .hblank_start    (hblank_start_video),
    .vcount          (vcount_video),
    .odd_field       (odd_field_video),
    .veblnk          (config_video_q.veblnk),
    .vsblnk          (config_video_q.vsblnk),
    .dpystart        (config_video_q.dpystart),
    .dpyctl          (config_video_q.dpyctl),
    .dpytap          (config_video_q.dpytap),
    .dpyadr_load     (command_valid_video
                      && command_received.dpyadr_load),
    .dpyadr_wdata    (command_received.dpyadr_wdata),
    .dpyadr          (dpyadr_video),
    .refresh_req     (screen_req_video),
    .refresh_ack     (screen_ack_video),
    .refresh_srfaddr (screen_srfaddr_video),
    .refresh_dpytap  (screen_dpytap_video),
    .refresh_org     (screen_org_video)
  );

  // Continuously ship a coherent triple back to the core. The mailbox's
  // round-trip bounds throughput, so the visible snapshot may lag VCLK by
  // several cycles but can never mix bits or epochs from separate values.
  tms34010_cdc_mailbox #(
    .WIDTH($bits(video_status_t))
  ) u_status_mailbox (
    .src_clk_i   (video_clk_i),
    .src_rst_i   (video_rst_i),
    .src_valid_i (1'b1),
    .src_data_i  (status_source),
    .src_ready_o (),
    .src_accept_o(),
    .dst_clk_i   (core_clk_i),
    .dst_rst_i   (core_rst_i),
    .dst_data_o  (status_received),
    .dst_valid_o ()
  );

  assign hcount_o = status_received.hcount;
  assign vcount_o = status_received.vcount;
  assign dpyadr_o = status_received.dpyadr;

  // A frame event is much slower than the round-trip handshake. The pending
  // bit nevertheless holds an event until the mailbox accepts it, so a
  // one-VCLK pulse cannot disappear between core-clock edges.
  always_ff @(posedge video_clk_i) begin
    if (video_rst_i) begin
      dip_pending_q <= 1'b0;
    end else begin
      if (dip_accept)
        dip_pending_q <= 1'b0;
      if (dpyint_video)
        dip_pending_q <= 1'b1;
    end
  end

  tms34010_cdc_mailbox #(
    .WIDTH(1)
  ) u_dip_mailbox (
    .src_clk_i   (video_clk_i),
    .src_rst_i   (video_rst_i),
    .src_valid_i (dip_pending_q),
    .src_data_i  (1'b1),
    .src_ready_o (),
    .src_accept_o(dip_accept),
    .dst_clk_i   (core_clk_i),
    .dst_rst_i   (core_rst_i),
    .dst_data_o  (),
    .dst_valid_o (dpyint_pulse_o)
  );

  tms34010_screen_cdc u_screen_cdc (
    .video_clk_i     (video_clk_i),
    .video_rst_i     (video_rst_i),
    .video_req_i     (screen_req_video),
    .video_srfaddr_i (screen_srfaddr_video),
    .video_dpytap_i  (screen_dpytap_video),
    .video_org_i     (screen_org_video),
    .video_ack_o     (screen_ack_video),
    .core_clk_i      (core_clk_i),
    .core_rst_i      (core_rst_i),
    .core_req_o      (screen_req_o),
    .core_srfaddr_o  (screen_srfaddr_o),
    .core_dpytap_o   (screen_dpytap_o),
    .core_org_o      (screen_org_o),
    .core_ack_i      (screen_ack_i)
  );

endmodule : tms34010_video_subsystem

`default_nettype wire

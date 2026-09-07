// -----------------------------------------------------------------------------
// tms34010_video.sv
//
// Video timing generator for the TMS34010 (1988 User's Guide §9 "Video
// Timing" and the corresponding Chapter 6 I/O-register pages). Horizontal
// and vertical counters driven by the video clock (VCLK) produce
// HSYNC/VSYNC and blanking, plus a display-interrupt strobe.
//
//   HCOUNT increments every clock; when HCOUNT == HTOTAL it wraps to 0 and the
//   horizontal sync interval restarts. In noninterlaced mode, VCOUNT advances
//   at each HCOUNT wrap and wraps after VTOTAL.
//
//   In interlaced mode, an even field starts on the normal full-line boundary.
//   The following odd field starts when VCOUNT == VTOTAL at
//   HCOUNT == floor(HTOTAL/2): VCOUNT resets but HCOUNT continues, producing
//   the required half-line offset. During that odd field, the
//   VCOUNT == VESYNC half-line event performs the guide's extra VCOUNT
//   increment and ends VSYNC. The next even field starts at the normal
//   HCOUNT == HTOTAL boundary.
//
//   HSYNC is asserted while HCOUNT <= HESYNC (the sync interval that begins at
//   the HTOTAL wrap). HBLANK is asserted while HCOUNT <= HEBLNK (the leading
//   blank, including the sync interval) or HCOUNT > HSBLNK (the trailing
//   blank); the visible region is HEBLNK+1..HSBLNK. These inclusive/exclusive
//   endpoints model the guide's one-VCLK delay after each equality compare.
//   The vertical signals use the analogous compares on VCOUNT. BLANK is
//   asserted when either axis is blanking or DPYCTL.ENV is clear.
//   DPYINT_PULSE is a one-clock strobe at the HSBLNK equality event on the
//   line selected by DPYINT.
//
//   In external-sync mode, active-low HSYNC/VSYNC inputs pass through
//   dedicated two-stage synchronizers. A saved synchronized level recognizes
//   each high-to-low transition on the third state-update edge, matching the
//   guide's 2.5-VCLK delay from its rising-edge sample to the falling-edge
//   counter clear. HTOTAL/VTOTAL remain fallback limits. HSD optionally keeps
//   horizontal timing internally generated while VSYNC remains external.
//
// Task 0155 places this module wholly in the dedicated VCLK domain behind
// coherent configuration, command, status, interrupt, and screen-transaction
// crossings. The active edge of the FPGA video clock represents the original
// device's falling-VCLK update edge; final phase/pin mapping belongs in the
// FPGA top.
//
// Spec source:
//   third_party/TMS34010_Info/docs/ti-official/1988_TI_TMS34010_Users_Guide.pdf
//   pages 6-18..6-25, 6-31, 6-47..6-52, §§9.6.1.1/9.7, and §9.9.
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_video
  import tms34010_pkg::*;
(
  input  logic        clk,        // VCLK (video / pixel clock)
  input  logic        rst,

  // Horizontal timing registers.
  input  logic [15:0] hesync,     // end of horizontal sync
  input  logic [15:0] heblnk,     // end of horizontal blank (display starts)
  input  logic [15:0] hsblnk,     // start of horizontal blank
  input  logic [15:0] htotal,     // total - 1 line length (HCOUNT wrap)
  // Vertical timing registers.
  input  logic [15:0] vesync,     // end of vertical sync
  input  logic [15:0] veblnk,     // end of vertical blank
  input  logic [15:0] vsblnk,     // start of vertical blank
  input  logic [15:0] vtotal,     // total - frame length (VCOUNT wrap)
  // Display-interrupt scan line.
  input  logic [15:0] dpyint,
  input  logic        display_enable, // DPYCTL.ENV
  input  logic        noninterlaced,  // DPYCTL.NIL
  input  logic        disable_external_video, // DPYCTL.DXV
  input  logic        hsync_direction, // DPYCTL.HSD: 1=output

  // Original active-low synchronization pins. They may be asynchronous to
  // VCLK and are consumed only when the DPYCTL direction selects an input.
  input  logic        hsync_n_i,
  input  logic        vsync_n_i,

  // Processor counter writes arrive as destination-domain pulses from the
  // Task 0155 coherent command mailbox.
  input  logic        hcount_load,
  input  logic [15:0] hcount_wdata,
  input  logic        vcount_load,
  input  logic [15:0] vcount_wdata,

  output logic [15:0] hcount,
  output logic [15:0] vcount,
  output logic        hsync,      // 1 = within the horizontal sync interval
  output logic        vsync,      // 1 = within the vertical sync interval
  output logic        hblank,     // 1 = horizontal blanking
  output logic        vblank,     // 1 = vertical blanking
  output logic        blank,      // 1 = blanked (either axis)
  output logic        hsync_oe,   // drive HSYNC when 1
  output logic        vsync_oe,   // drive VSYNC when 1
  output logic        hblank_start,// one clock at HCOUNT == HSBLNK
  output logic        dpyint_pulse,// one-clock strobe at the DPYINT scan line start
  output logic        odd_field   // 0=even/noninterlaced, 1=odd interlaced field
);

  logic [15:0] half_line_count;
  logic        hwrap;
  logic        line_start;
  logic        even_to_odd;
  logic        odd_sync_end;
  logic        hsync_n_sync;
  logic        vsync_n_sync;
  logic        hsync_n_prev_q;
  logic        vsync_n_prev_q;
  logic        external_hsync_start;
  logic        external_vsync_start;
  logic        external_odd_field;

  assign half_line_count = {1'b0, htotal[15:1]};
  assign hwrap = (hcount == htotal);
  assign external_hsync_start =
      !disable_external_video && !hsync_direction
      && hsync_n_prev_q && !hsync_n_sync;
  assign external_vsync_start =
      !disable_external_video
      && vsync_n_prev_q && !vsync_n_sync;
  assign line_start =
      (!disable_external_video && !hsync_direction)
      ? (external_hsync_start || hwrap)
      : hwrap;
  assign even_to_odd =
      disable_external_video && !noninterlaced && !odd_field
      && (vcount == vtotal) && (hcount == half_line_count);
  assign odd_sync_end =
      disable_external_video && !noninterlaced && odd_field
      && (vcount == vesync) && (hcount == half_line_count);
  assign external_odd_field =
      (hcount > heblnk) && (hcount <= hsblnk);

  // The synchronizer output changes two update edges after an asynchronous
  // pin transition. Comparing it with one saved level recognizes the falling
  // edge on the third update edge: the guide's 2.5-VCLK delay measured from
  // the intervening rising-edge sample to the original falling-edge clear.
  tms34010_sync_bit #(.RESET_VALUE(1'b1)) u_hsync_input_sync (
    .clk     (clk),
    .rst     (rst),
    .async_i (hsync_n_i),
    .sync_o  (hsync_n_sync)
  );

  tms34010_sync_bit #(.RESET_VALUE(1'b1)) u_vsync_input_sync (
    .clk     (clk),
    .rst     (rst),
    .async_i (vsync_n_i),
    .sync_o  (vsync_n_sync)
  );

  // (c) Edge history is destination-domain state that converts each
  // synchronized active-low level transition into one recognition event.
  always_ff @(posedge clk) begin
    if (rst) begin
      hsync_n_prev_q <= 1'b1;
      vsync_n_prev_q <= 1'b1;
    end else begin
      hsync_n_prev_q <= hsync_n_sync;
      vsync_n_prev_q <= vsync_n_sync;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      hcount    <= 16'd0;
      vcount    <= 16'd0;
      odd_field <= 1'b0;
    end else begin
      // Returning to noninterlaced timing deterministically selects the even
      // phase. Interlaced operation always restarts its two-field sequence
      // from that phase.
      if (noninterlaced)
        odd_field <= 1'b0;

      if (hcount_load) begin
        hcount <= hcount_wdata;
      end else if (line_start) begin
        hcount <= 16'd0;
      end else begin
        hcount <= hcount + 16'd1;
      end

      if (vcount_load) begin
        vcount <= vcount_wdata;
      end else if (!disable_external_video && external_vsync_start) begin
        // VCOUNT clears independently of horizontal phase. In interlaced
        // mode, the pre-clear HCOUNT value classifies the field exactly where
        // the guide samples it: after input recognition and before the clear.
        vcount <= 16'd0;
        if (!noninterlaced)
          odd_field <= external_odd_field;
      end else if (disable_external_video
                   && !hcount_load && even_to_odd) begin
        // Start the odd field halfway through the current horizontal line.
        // HCOUNT deliberately continues rather than resetting here.
        vcount    <= 16'd0;
        odd_field <= 1'b1;
      end else if (disable_external_video
                   && !hcount_load && odd_sync_end) begin
        // The odd-field VESYNC compare increments VCOUNT at midline. This is
        // architecturally visible and prevents DPYINT=VESYNC from firing in
        // the odd field when HSBLNK is programmed to the same half-line point.
        vcount <= vcount + 16'd1;
      end else if (!hcount_load && line_start) begin
        if (vcount == vtotal) begin
          vcount <= 16'd0;
          // An external-mode limit fallback has no field phase to
          // discriminate, so it recovers deterministically to even.
          odd_field <= 1'b0;
        end else begin
          vcount <= vcount + 16'd1;
        end
      end
    end
  end

  // Sync/blank intervals in count space. The User's Guide specifies that the
  // output transition occurs one VCLK after the corresponding equality, so
  // each end value remains active and each start value remains inactive.
  assign hsync  = (hcount <= hesync);
  assign vsync  = (vcount <= vesync);
  assign hblank = (hcount <= heblnk) || (hcount > hsblnk);
  assign vblank = (vcount <= veblnk) || (vcount > vsblnk);
  assign blank  = !display_enable || hblank || vblank;
  assign hsync_oe = disable_external_video || hsync_direction;
  assign vsync_oe = disable_external_video;
  assign hblank_start = (hcount == hsblnk);

  // Display interrupt: DPYINT matches VCOUNT at the start of horizontal
  // blanking (HCOUNT == HSBLNK), at the end of the selected scan line.
  // DPYCTL.ENV=0 inhibits new display-interrupt requests.
  assign dpyint_pulse =
      display_enable && hblank_start && (vcount == dpyint)
      && !odd_sync_end;

endmodule : tms34010_video

`default_nettype wire

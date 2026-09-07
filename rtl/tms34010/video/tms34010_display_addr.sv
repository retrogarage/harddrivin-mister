// -----------------------------------------------------------------------------
// tms34010_display_addr.sv
//
// Live DPYADR state and automatic screen-refresh scheduler.
// A request is generated at an eligible start-of-horizontal-blanking event
// and held, with a stable SRFADR/DPYTAP payload, until the future memory
// controller acknowledges completion. DPYADR advances only on that completion.
//
// DPYADR contains:
//   bits  1:0  LNCNT  remaining scan lines before another refresh
//   bits 15:2  SRFADR screen-refresh row/column address source
//
// At the beginning of vertical blanking, SRFADR reloads from DPYSTRT.SRSTRT.
// At the final horizontal blank before active display, LNCNT reloads from
// DPYSTRT.LCSTRT and the first enabled screen refresh is requested. Subsequent
// refreshes occur every LCSTRT+1 eligible scan lines. On completion, the raw
// SRFADR counter always decrements by DPYCTL.DUDATE and LNCNT reloads. ORG
// selects whether this raw value is inverted at the local-bus pins: with
// ORG=0, decrementing the stored one's-complement value produces an effective
// increasing screen address; with ORG=1, the direct address decreases.
//
// In interlaced mode, the vertical blanking interval in the odd field
// precedes the next even field. That reload subtracts DUDATE/2 from the raw
// DPYSTRT value so the effective address starts on the alternate
// display-memory line.
// The blanking interval in the even field precedes the odd field and reloads
// DPYSTRT without the half-line displacement.
//
// Task 0148 carries captured ORG through the physical-bus bridge, and Task
// 0155 wraps this held request in a coherent VCLK-to-core transaction.
//
// Spec source:
//   1988 TI TMS34010 User's Guide pages 6-17..6-24 and §9.10.1.
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_display_addr
  import tms34010_pkg::*;
(
  input  logic        clk,
  input  logic        rst,

  input  logic        hblank_start,
  input  logic [15:0] vcount,
  input  logic        odd_field,
  input  logic [15:0] veblnk,
  input  logic [15:0] vsblnk,

  input  logic [15:0] dpystart,
  input  logic [15:0] dpyctl,
  input  logic [15:0] dpytap,

  input  logic        dpyadr_load,
  input  logic [15:0] dpyadr_wdata,

  output logic [15:0] dpyadr,
  output logic        refresh_req,
  input  logic        refresh_ack,
  output logic [13:0] refresh_srfaddr,
  output logic [15:0] refresh_dpytap,
  output logic        refresh_org
);

  logic        sre_prev_q;
  logic        first_after_enable_q;
  logic        eligible_hblank;
  logic        frame_blank_start;
  logic        first_active_hblank;
  logic        start_request;
  logic [7:0]  dudate;
  logic [13:0] dudate_step;
  logic [13:0] half_dudate_step;
  logic [13:0] next_srfaddr;
  logic [13:0] interlaced_even_srfaddr;
  logic [13:0] frame_srfaddr;

  assign eligible_hblank =
      hblank_start && (vcount >= veblnk) && (vcount < vsblnk);
  assign frame_blank_start =
      hblank_start && (vcount == vsblnk);
  assign first_active_hblank =
      hblank_start && (vcount == veblnk);

  assign dudate = dpyctl[DPYCTL_DUDATE_HI:DPYCTL_DUDATE_LO];
  assign dudate_step = {6'h00, dudate};
  assign half_dudate_step = {1'b0, dudate_step[13:1]};
  // User's Guide Figure 9-14 is explicit that the stored DPYADR field is
  // decremented for both ORG values. ORG changes only the pin representation.
  assign next_srfaddr =
      dpyadr[DPY_SRFADR_HI:DPY_SRFADR_LO] - dudate_step;
  assign interlaced_even_srfaddr =
      dpystart[DPY_SRFADR_HI:DPY_SRFADR_LO] - half_dudate_step;
  assign frame_srfaddr =
      (!dpyctl[DPYCTL_NIL_BIT] && odd_field)
      ? interlaced_even_srfaddr
      : dpystart[DPY_SRFADR_HI:DPY_SRFADR_LO];

  // SRE rising during the active region forces the next eligible HBLANK to
  // request a refresh regardless of the old LNCNT state. The first active
  // HBLANK of every frame is independently forced by the specification.
  assign start_request =
      eligible_hblank
      && dpyctl[DPYCTL_SRE_BIT]
      && !refresh_req
      && (first_active_hblank
          || first_after_enable_q
          || !sre_prev_q
          || (dpyadr[DPY_LNCNT_HI:DPY_LNCNT_LO] == 2'b00));

  always_ff @(posedge clk) begin
    if (rst) begin
      dpyadr              <= 16'h0000;
      refresh_req         <= 1'b0;
      refresh_srfaddr     <= 14'h0000;
      refresh_dpytap      <= 16'h0000;
      refresh_org         <= 1'b0;
      sre_prev_q          <= 1'b0;
      first_after_enable_q <= 1'b0;
    end else begin
      sre_prev_q <= dpyctl[DPYCTL_SRE_BIT];

      if (!dpyctl[DPYCTL_SRE_BIT]) begin
        first_after_enable_q <= 1'b0;
      end else if (start_request) begin
        first_after_enable_q <= 1'b0;
      end else if (!sre_prev_q) begin
        first_after_enable_q <= 1'b1;
      end

      // Acknowledge retires the held request. A new eligible request wins if
      // both conditions ever coincide, preserving continuous service.
      if (refresh_ack && refresh_req)
        refresh_req <= 1'b0;
      if (start_request) begin
        refresh_req     <= 1'b1;
        refresh_srfaddr <= dpyadr[DPY_SRFADR_HI:DPY_SRFADR_LO];
        refresh_dpytap  <= dpytap & DPYTAP_MASK;
        refresh_org     <= dpyctl[DPYCTL_ORG_BIT];
      end

      // A processor write has deterministic priority over automatic state
      // changes. Otherwise the two DPYADR fields follow their independent
      // architectural load/update events.
      if (dpyadr_load) begin
        dpyadr <= dpyadr_wdata;
      end else begin
        if (frame_blank_start) begin
          dpyadr[DPY_SRFADR_HI:DPY_SRFADR_LO]
              <= frame_srfaddr;
        end else if (refresh_ack && refresh_req) begin
          dpyadr[DPY_SRFADR_HI:DPY_SRFADR_LO] <= next_srfaddr;
        end

        if (first_active_hblank) begin
          dpyadr[DPY_LNCNT_HI:DPY_LNCNT_LO]
              <= dpystart[DPY_LNCNT_HI:DPY_LNCNT_LO];
        end else if (refresh_ack && refresh_req) begin
          dpyadr[DPY_LNCNT_HI:DPY_LNCNT_LO]
              <= dpystart[DPY_LNCNT_HI:DPY_LNCNT_LO];
        end else if (eligible_hblank
                     && dpyctl[DPYCTL_SRE_BIT]
                     && !refresh_req
                     && !start_request) begin
          dpyadr[DPY_LNCNT_HI:DPY_LNCNT_LO]
              <= dpyadr[DPY_LNCNT_HI:DPY_LNCNT_LO] - 2'b01;
        end
      end
    end
  end

endmodule : tms34010_display_addr

`default_nettype wire

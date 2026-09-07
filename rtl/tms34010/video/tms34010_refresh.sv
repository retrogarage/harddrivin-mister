// -----------------------------------------------------------------------------
// tms34010_refresh.sv
//
// REFCNT counter and DRAM-refresh request generator.
//
// The 1988 User's Guide pages 6-45/6-46 define REFCNT bits 2-15 as one
// continuous 14-bit down-counter. Bits 2-7 are RINTVL and bits 8-15 are
// ROWADR. CONTROL.RR=00 subtracts two from RINTVL per local clock, RR=01
// subtracts one, and a borrow from RINTVL decrements ROWADR and requests a
// refresh cycle. Consequently the row sequence descends from 255 to 0.
//
// REFCNT is also a processor-writable I/O register. A synchronous explicit
// load has priority over automatic counting. The guide recommends disabling
// refresh before writing; the priority here gives the FPGA implementation a
// deterministic result even if software violates that recommendation.
//
// `refresh_req` is registered for one clock at each RINTVL underflow. During
// that pulse, `refresh_row` is the newly decremented ROWADR value that the
// future memory controller must drive for the refresh cycle.
//
// Clock domain: core/local clock only. Reset is synchronous active-high per
// project convention A0003.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_refresh (
  input  logic        clk,
  input  logic        rst,
  input  logic [1:0]  rr,
  input  logic        refcnt_load,
  input  logic [15:0] refcnt_wdata,

  output logic [15:0] refcnt,
  output logic [7:0]  refresh_row,
  output logic        refresh_req
);

  logic        count_enable;
  logic [13:0] count_step;
  logic        interval_underflow;
  logic [15:0] refcnt_d;

  // Justification (a): architected REFCNT state persists across local clocks
  // and is both processor-visible and consumed by the refresh requester.
  logic [15:0] refcnt_q;

  always_comb begin
    count_enable = 1'b0;
    count_step   = 14'd0;

    unique case (rr)
      2'b00: begin
        count_enable = 1'b1;
        count_step   = 14'd2;
      end
      2'b01: begin
        count_enable = 1'b1;
        count_step   = 14'd1;
      end
      2'b10,
      2'b11: begin
        count_enable = 1'b0;
        count_step   = 14'd0;
      end
      default: begin
        count_enable = 1'b0;
        count_step   = 14'd0;
      end
    endcase
  end

  // Borrow out of the six-bit RINTVL field is the refresh-request event.
  assign interval_underflow =
      count_enable && ({8'd0, refcnt_q[7:2]} < count_step);

  always_comb begin
    refcnt_d = refcnt_q;
    if (count_enable)
      refcnt_d[15:2] = refcnt_q[15:2] - count_step;
  end

  always_ff @(posedge clk) begin
    if (rst)
      refcnt_q <= 16'h0000;
    else if (refcnt_load)
      refcnt_q <= refcnt_wdata;
    else
      refcnt_q <= refcnt_d;
  end

  // Justification (a): this one-cycle event records the underflow decision
  // across the clock edge so the simultaneously updated row is stable.
  always_ff @(posedge clk) begin
    if (rst)
      refresh_req <= 1'b0;
    else if (refcnt_load)
      refresh_req <= 1'b0;
    else
      refresh_req <= interval_underflow;
  end

  assign refcnt      = refcnt_q;
  assign refresh_row = refcnt_q[15:8];

endmodule : tms34010_refresh

`default_nettype wire

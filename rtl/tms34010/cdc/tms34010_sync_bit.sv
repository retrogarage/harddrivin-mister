// -----------------------------------------------------------------------------
// tms34010_sync_bit.sv
//
// Two-flop synchronizer for one asynchronous level entering a destination
// domain. The dedicated module and Quartus attributes keep the two registers
// recognizable by the metastability analyzer and prevent retiming/merging.
//
// LINT1/LINT2 are asynchronous level-sensitive pins (1988 User's Guide
// pages 6-41 and 8-3); HOLD, RUN/EMU, HSYNC, and VSYNC are likewise
// asynchronous physical inputs. Each pin receives its own instance; parallel
// bits are never treated as a coherent multi-bit value.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_sync_bit #(
  parameter logic RESET_VALUE = 1'b0
)(
  input  logic clk,
  input  logic rst,
  input  logic async_i,
  output logic sync_o
);

  // Justification (c): the first stage absorbs metastability from the
  // asynchronous pin; the second is the only stage consumed by destination
  // logic.
  (* preserve, useioff = 0,
     altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION \"FORCED IF ASYNCHRONOUS\"" *)
  logic [1:0] sync_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      sync_q <= {2{RESET_VALUE}};
    end else begin
      sync_q[0] <= async_i;
      sync_q[1] <= sync_q[0];
    end
  end

  assign sync_o = sync_q[1];

endmodule : tms34010_sync_bit

`default_nettype wire

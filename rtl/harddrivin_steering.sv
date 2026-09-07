`default_nettype none

// MiSTer controller steering to the Compact board's absolute 12-bit ADC.
// MiSTer supplies an 8-bit signed axis (-128..127, zero at rest); XORing the
// sign bit maps it monotonically to 0x00..0xff.  There is deliberately no
// temporal filter here: the cabinet exposes position, not a velocity control.
module harddrivin_steering #(
  // Real controllers commonly rest a few signed counts away from zero.  A
  // captured gameplay frame showed +4, exactly on the old exclusive
  // boundary, which the game received as 0x840 and treated as right input.
  // This is a positional deadzone only; values outside it still propagate
  // combinationally and full physical travel remains an immediate end stop.
  parameter logic [7:0] DEADZONE = 8'd8
) (
  input  logic [7:0]  axis_i,
  input  logic        right_i,
  input  logic        left_i,
  output logic [11:0] position_o
);
  logic [7:0]  unsigned_axis;
  logic [7:0]  centered_axis;
  logic [11:0] analog_position;
  logic        analog_centered;

  always_comb begin
    unsigned_axis = axis_i ^ 8'h80;
    centered_axis = ((unsigned_axis >= (8'h80 - DEADZONE)) &&
                     (unsigned_axis <= (8'h80 + DEADZONE)))
                  ? 8'h80 : unsigned_axis;
    analog_position = {centered_axis, 4'h0};
    analog_centered = (centered_axis == 8'h80);

    // MiSTer synthesizes digital direction bits from analog axes after roughly
    // 25% travel. Letting those bits win would turn a proportional stick into
    // an on/off switch. Use Left/Right only when no analog displacement is
    // present, retaining keyboard/D-pad steering as instant end stops.
    if (analog_centered && right_i && !left_i)
      position_o = 12'hff0;
    else if (analog_centered && left_i && !right_i)
      position_o = 12'h010;
    else if (analog_position < 12'h010)
      position_o = 12'h010;
    else if (analog_position > 12'hff0)
      position_o = 12'hff0;
    else
      position_o = analog_position;
  end
endmodule

`default_nettype wire

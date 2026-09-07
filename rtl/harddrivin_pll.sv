`default_nettype none

module harddrivin_pll #(parameter bit COCKPIT = 0) (
  input  logic refclk_i,
  input  logic rst_i,
  output logic clk_gsp_48_o,
  output logic clk_msp_50_o,
  output logic clk_100_o,
  output logic clk_video_6_o,
  output logic locked_o
);
`ifdef TMS34010_QUARTUS
  logic [3:0] pll_clks;
  altera_pll #(
    .fractional_vco_multiplier("false"),
    .reference_clock_frequency("50.0 MHz"),
    .operation_mode("direct"),
    .number_of_clocks(4),
    .output_clock_frequency0("48.0 MHz"),
    .phase_shift0("0 ps"),
    .duty_cycle0(50),
    .output_clock_frequency1("50.0 MHz"),
    .phase_shift1("0 ps"),
    .duty_cycle1(50),
    .output_clock_frequency2("100.0 MHz"),
    .phase_shift2("0 ps"),
    .duty_cycle2(50),
    .output_clock_frequency3(COCKPIT ? "4.0 MHz" : "6.0 MHz"),
    .phase_shift3("0 ps"),
    .duty_cycle3(50),
    .pll_type("General"),
    .pll_subtype("General")
  ) u_pll (
    .refclk(refclk_i), .refclk1(1'b0), .fbclk(1'b0), .rst(rst_i),
    .phase_en(1'b0), .updn(1'b0), .num_phase_shifts(3'b000),
    .scanclk(1'b0), .cntsel(5'b00000), .reconfig_to_pll(64'b0),
    .extswitch(1'b0), .adjpllin(1'b0), .cclk(1'b0),
    .outclk(pll_clks), .fboutclk(), .locked(locked_o), .phase_done(),
    .reconfig_from_pll(), .activeclk(), .clkbad(), .phout(),
    .lvds_clk(), .loaden(), .extclk_out(), .cascade_out(), .zdbfbclk()
  );
  assign clk_gsp_48_o = pll_clks[0];
  assign clk_msp_50_o = pll_clks[1];
  assign clk_100_o    = pll_clks[2];
  assign clk_video_6_o = pll_clks[3];
`else
  assign clk_gsp_48_o = refclk_i;
  assign clk_msp_50_o = refclk_i;
  assign clk_100_o    = refclk_i;
  assign clk_video_6_o = refclk_i;
  assign locked_o     = !rst_i;
`endif
endmodule

`default_nettype wire

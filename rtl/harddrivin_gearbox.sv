`default_nettype none
// Sequential buttons latch a gear; H-pattern switches describe its current
// position. Conflicting contacts are neutral, never an arbitrary priority gear.
module harddrivin_gearbox (
  input logic clk_i, rst_i, h_pattern_i,
  input logic up_i, down_i,
  input logic [3:0] positions_i,
  output logic [3:0] gear_o
);
  logic [1:0] previous_q;
  logic h_pattern_q;
  always_ff @(posedge clk_i) begin
    previous_q <= {up_i,down_i};
    h_pattern_q <= h_pattern_i;
    if (rst_i) begin
      previous_q <= 2'b11; // a held button at reset is not a new shift
      h_pattern_q <= h_pattern_i;
      gear_o <= 0;
    end else if (h_pattern_i) begin
      case (positions_i)
        4'b0001,4'b0010,4'b0100,4'b1000: gear_o <= positions_i;
        default: gear_o <= 0;
      endcase
    end else if (h_pattern_q) begin
      gear_o <= 0;
    end else if (up_i && !down_i && !previous_q[1]) begin
      case (gear_o)
        0: gear_o <= 1;
        1: gear_o <= 2;
        2: gear_o <= 4;
        default: gear_o <= 8;
      endcase
    end else if (down_i && !up_i && !previous_q[0]) begin
      case (gear_o)
        8: gear_o <= 4;
        4: gear_o <= 2;
        2: gear_o <= 1;
        default: gear_o <= 0;
      endcase
    end
  end
endmodule
`default_nettype wire

`default_nettype none

// Simple dual-port word RAM.  Port A owns writes and the GSP local-bus read;
// port B is the independent palette/scanout lookup port.
module harddrivin_dual_port_word_ram #(
  parameter integer ADDR_WIDTH = 10
) (
  input  logic                  clk_a_i,
  input  logic                  en_a_i,
  input  logic                  we_a_i,
  input  logic [ADDR_WIDTH-1:0] addr_a_i,
  input  logic [15:0]           wdata_a_i,
  output logic [15:0]           rdata_a_o,
  input  logic                  clk_b_i,
  input  logic                  en_b_i,
  input  logic [ADDR_WIDTH-1:0] addr_b_i,
  output logic [15:0]           rdata_b_o
);
  localparam integer WORDS = 1 << ADDR_WIDTH;
  (* ramstyle = "M10K, no_rw_check" *) logic [15:0] mem [0:WORDS-1];

  always_ff @(posedge clk_a_i) begin
    if (en_a_i) begin
      if (we_a_i) mem[addr_a_i] <= wdata_a_i;
      rdata_a_o <= mem[addr_a_i];
    end
  end

  always_ff @(posedge clk_b_i) begin
    if (en_b_i) rdata_b_o <= mem[addr_b_i];
  end
endmodule

`default_nettype wire

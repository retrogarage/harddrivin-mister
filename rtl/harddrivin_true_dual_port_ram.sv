`default_nettype none

// True dual-port word RAM: both ports read and write on one clock.  Used for
// the ADSP board's 8Kx16 data memory, which the 68010 and the ADSP-2100 both
// write (the physical board serialises them with bus request; the M10K true
// dual-port mode removes that need).  Reads are registered (one cycle).
module harddrivin_true_dual_port_ram #(
  parameter integer ADDR_WIDTH = 13
) (
  input  logic                  clk_i,
  input  logic                  en_a_i,
  input  logic                  we_a_i,
  input  logic [ADDR_WIDTH-1:0] addr_a_i,
  input  logic [15:0]           wdata_a_i,
  output logic [15:0]           rdata_a_o,
  input  logic                  en_b_i,
  input  logic                  we_b_i,
  input  logic [ADDR_WIDTH-1:0] addr_b_i,
  input  logic [15:0]           wdata_b_i,
  output logic [15:0]           rdata_b_o
);
  localparam integer WORDS = 1 << ADDR_WIDTH;
  (* ramstyle = "M10K, no_rw_check" *) logic [15:0] mem [0:WORDS-1];

  always_ff @(posedge clk_i) begin
    if (en_a_i) begin
      if (we_a_i) mem[addr_a_i] <= wdata_a_i;
      rdata_a_o <= mem[addr_a_i];
    end
  end

  always_ff @(posedge clk_i) begin
    if (en_b_i) begin
      if (we_b_i) mem[addr_b_i] <= wdata_b_i;
      rdata_b_o <= mem[addr_b_i];
    end
  end
endmodule

`default_nettype wire

`default_nettype none

// Canonical synchronous single-port RAM used for the Compact MSP DRAM bank.
// Keeping the storage primitive separate from bus completion logic prevents
// diagnostic muxes from turning the array into an asynchronous read in Quartus.
module harddrivin_word_ram #(
  parameter integer ADDR_WIDTH = 16
) (
  input  logic                  clk_i,
  input  logic                  en_i,
  input  logic                  we_i,
  input  logic [ADDR_WIDTH-1:0] addr_i,
  input  logic [15:0]           wdata_i,
  output logic [15:0]           rdata_o
);
  localparam integer WORDS = 1 << ADDR_WIDTH;
  (* ramstyle = "M10K, no_rw_check" *) logic [15:0] mem [0:WORDS-1];

  always_ff @(posedge clk_i) begin
    if (en_i) begin
      if (we_i) mem[addr_i] <= wdata_i;
      rdata_o <= mem[addr_i];
    end
  end
endmodule

`default_nettype wire

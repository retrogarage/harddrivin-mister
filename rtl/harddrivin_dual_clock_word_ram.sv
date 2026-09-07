`default_nettype none

// One-write/one-read dual-clock RAM used by the SDRAM row prefetcher and the
// independent 48 MHz pixel serializer.
module harddrivin_dual_clock_word_ram #(
  parameter integer ADDR_WIDTH = 11
) (
  input  logic                  wr_clk_i,
  input  logic                  wr_en_i,
  input  logic [ADDR_WIDTH-1:0] wr_addr_i,
  input  logic [15:0]           wr_data_i,
  input  logic                  rd_clk_i,
  input  logic [ADDR_WIDTH-1:0] rd_addr_i,
  output logic [15:0]           rd_data_o
);
  localparam integer WORDS = 1 << ADDR_WIDTH;
  (* ramstyle = "M10K, no_rw_check" *) logic [15:0] mem [0:WORDS-1];

  always_ff @(posedge wr_clk_i) begin
    if (wr_en_i) mem[wr_addr_i] <= wr_data_i;
  end

  always_ff @(posedge rd_clk_i) begin
    rd_data_o <= mem[rd_addr_i];
  end
endmodule

`default_nettype wire

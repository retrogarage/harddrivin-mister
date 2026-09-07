`default_nettype none

// 16-bit single-port RAM with independent 68010 upper/lower byte write lanes.
// Each lane owns its registered read output so Quartus can map both arrays to
// M10Ks even when their combined word later shares the main CPU data mux.
module harddrivin_lane_ram #(
  parameter integer ADDR_WIDTH = 14,
  // Optional power-up contents (hex, one byte per line) for each lane.
  parameter         INIT_HI_FILE = "",
  parameter         INIT_LO_FILE = ""
) (
  input  logic                  clk_i,
  input  logic                  en_i,
  input  logic [ADDR_WIDTH-1:0] addr_i,
  input  logic                  we_hi_i,
  input  logic                  we_lo_i,
  input  logic [15:0]           wdata_i,
  output logic [15:0]           rdata_o
);
  localparam integer WORDS = 1 << ADDR_WIDTH;
  (* ramstyle = "M10K, no_rw_check" *) logic [7:0] mem_hi [0:WORDS-1];
  (* ramstyle = "M10K, no_rw_check" *) logic [7:0] mem_lo [0:WORDS-1];
  initial begin
    if (INIT_HI_FILE != "") $readmemh(INIT_HI_FILE, mem_hi);
    if (INIT_LO_FILE != "") $readmemh(INIT_LO_FILE, mem_lo);
  end

  always_ff @(posedge clk_i) begin
    if (en_i) begin
      if (we_hi_i) mem_hi[addr_i] <= wdata_i[15:8];
      rdata_o[15:8] <= mem_hi[addr_i];
    end
  end

  always_ff @(posedge clk_i) begin
    if (en_i) begin
      if (we_lo_i) mem_lo[addr_i] <= wdata_i[7:0];
      rdata_o[7:0] <= mem_lo[addr_i];
    end
  end
endmodule

`default_nettype wire

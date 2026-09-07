`default_nettype none

// Atari's ADSP board implements 8K x 24 of writable program memory.  Port A
// is shared by instruction fetch and the halted 68010 upload path; port B is
// the ADSP program-data access used by PM(I,M) instructions.
module adsp2100_program_ram (
  input  logic        clk_i,

  input  logic [12:0] a_addr_i,
  input  logic        a_we_i,
  input  logic [23:0] a_wdata_i,
  output logic [23:0] a_rdata_o,

  input  logic [12:0] b_addr_i,
  input  logic        b_we_i,
  input  logic [23:0] b_wdata_i,
  output logic [23:0] b_rdata_o
);
  (* ramstyle = "M10K, no_rw_check" *) logic [23:0] mem [0:8191];

  always_ff @(posedge clk_i) begin
    if (a_we_i) mem[a_addr_i] <= a_wdata_i;
    a_rdata_o <= mem[a_addr_i];
  end

  always_ff @(posedge clk_i) begin
    if (b_we_i) mem[b_addr_i] <= b_wdata_i;
    b_rdata_o <= mem[b_addr_i];
  end
endmodule

`default_nettype wire

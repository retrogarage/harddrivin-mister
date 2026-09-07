// -----------------------------------------------------------------------------
// tms34010_regfile.sv
//
// General-purpose register file for the TMS34010 core.
//
// Architectural contract (per bibliography/hdl-reimplementation/03-registers.md):
//   - File A: A0..A14 (15 entries, 32 bits each).
//   - File B: B0..B14 (15 entries, 32 bits each).
//   - SP    : 1 entry, 32 bits, accessible from BOTH files at index 4'hF
//             (i.e., A15 and B15 are aliases for the same SP).
//
// Port shape:
//   - Two combinational read ports. Async read is FPGA-friendly for this
//     size (~1 Kb total) — fits in distributed/LUT RAM on Cyclone V or
//     plain flip-flops without inflating the LUT count.
//   - One synchronous write port. Write-after-read on the SAME port within
//     a cycle: a read on `rs1`/`rs2` issued in the same cycle as a write
//     to the matching index returns the OLD value (the new value lands at
//     the next clock edge). This matches the multi-cycle execute model.
//
// Reset:
//   Synchronous active-high `rst`. All entries clear to '0. The loop in
//   the reset arm is bounded and fully unrollable (15 iterations × 2 files
//   + 1 SP = 31 sequential element updates), so synthesis treats it as
//   plain parallel resets.
//
// Notes:
//   - SP aliasing is implemented in one place (the read/write decode
//     functions). Code that touches the regfile never has to know about
//     the alias.
//   - No `/`, no `%`, no `initial` (rst handles all init), no unbounded
//     loops, no latches (all combinational outputs assigned on every
//     path through `always_comb`).
//
// Spec source:
//   third_party/TMS34010_Info/bibliography/hdl-reimplementation/03-registers.md
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_regfile
  import tms34010_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst,

  // Read port 1.
  input  reg_file_t             rs1_file,
  input  reg_idx_t              rs1_idx,
  output logic [DATA_WIDTH-1:0] rs1_data,

  // Read port 2.
  input  reg_file_t             rs2_file,
  input  reg_idx_t              rs2_idx,
  output logic [DATA_WIDTH-1:0] rs2_data,

  // Read port 3. Added for instructions that need a third simultaneous
  // source — currently CPW (reads the point plus WSTART and WEND). Async
  // read like the others; a third read mux on this flop-based file is
  // cheap (no extra memory blocks).
  input  reg_file_t             rs3_file,
  input  reg_idx_t              rs3_idx,
  output logic [DATA_WIDTH-1:0] rs3_data,

  // Write port.
  input  logic                  wr_en,
  input  reg_file_t             wr_file,
  input  reg_idx_t              wr_idx,
  input  logic [DATA_WIDTH-1:0] wr_data,

  // Observability: SP value, used by testbenches and (eventually) host
  // interface debug paths.
  output logic [DATA_WIDTH-1:0] sp_o
);

  // ---------------------------------------------------------------------------
  // Storage
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] a_regs [0:14];
  logic [DATA_WIDTH-1:0] b_regs [0:14];
  logic [DATA_WIDTH-1:0] sp_q;

  // ---------------------------------------------------------------------------
  // Read multiplex (combinational / async read)
  //
  // Index 4'hF on either file returns SP. Other indices return the matching
  // file entry. Implemented inline rather than via a function to keep the
  // synthesizable form trivial for Quartus.
  // ---------------------------------------------------------------------------
  always_comb begin
    if (rs1_idx == REG_SP_IDX) begin
      rs1_data = sp_q;
    end else if (rs1_file == REG_FILE_B) begin
      rs1_data = b_regs[rs1_idx];
    end else begin
      rs1_data = a_regs[rs1_idx];
    end
  end

  always_comb begin
    if (rs2_idx == REG_SP_IDX) begin
      rs2_data = sp_q;
    end else if (rs2_file == REG_FILE_B) begin
      rs2_data = b_regs[rs2_idx];
    end else begin
      rs2_data = a_regs[rs2_idx];
    end
  end

  always_comb begin
    if (rs3_idx == REG_SP_IDX) begin
      rs3_data = sp_q;
    end else if (rs3_file == REG_FILE_B) begin
      rs3_data = b_regs[rs3_idx];
    end else begin
      rs3_data = a_regs[rs3_idx];
    end
  end

  // ---------------------------------------------------------------------------
  // Write
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      // Bounded, fully unrollable resets.
      for (int unsigned i = 0; i < 15; i++) begin
        a_regs[i] <= '0;
      end
      for (int unsigned i = 0; i < 15; i++) begin
        b_regs[i] <= '0;
      end
      sp_q <= '0;
    end else if (wr_en) begin
      if (wr_idx == REG_SP_IDX) begin
        sp_q <= wr_data;
      end else if (wr_file == REG_FILE_B) begin
        b_regs[wr_idx] <= wr_data;
      end else begin
        a_regs[wr_idx] <= wr_data;
      end
    end
  end

  assign sp_o = sp_q;

endmodule : tms34010_regfile
`default_nettype wire

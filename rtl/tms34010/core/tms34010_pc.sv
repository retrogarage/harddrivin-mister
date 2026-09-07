// -----------------------------------------------------------------------------
// tms34010_pc.sv
//
// Bit-addressed program counter for the TMS34010 core.
//
// What this module IS:
//   - A 32-bit register that holds the PC as a bit-address.
//   - Two write paths:
//       * `load_en` / `load_value` — absolute jump (overrides advance).
//       * `advance_en` / `advance_amount` — forward advance by N bits.
//         The amount is parameterized in bits to keep the spec-native
//         representation (PC is bit-addressed) without hidden conversions.
//   - Synchronous active-high reset to `RESET_VALUE`.
//
// What it is NOT, yet:
//   - There is no decrement / backward step. Branches use `load_en` with the
//     full target address.
//   - There is no "PC + INSTR_WORD" expressed in instruction-units; the user
//     of this module supplies the bit count via `advance_amount`. The core
//     drives `INSTR_WORD_BITS` (=16) from `tms34010_pkg`.
//   - There is no protection against `load_en` and `advance_en` asserted in
//     the same cycle — `load_en` simply wins (documented behavior).
//
// Synthesis notes:
//   - One sequential `always_ff` for the state register.
//   - One `always_comb` for next-state with a safe default of "hold".
//   - The adder is 32-bit wide; the advance amount is zero-extended
//     explicitly so synthesis never has to guess the width.
//   - No `/`, no `%`, no runtime loops, no `initial`.
//
// Spec sources:
//   third_party/TMS34010_Info/bibliography/hdl-reimplementation/
//     01-architecture.md, §"Datapath summary"
//     11-interrupts-reset.md, §"Reset"
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_pc
  import tms34010_pkg::*;
#(
  parameter logic [ADDR_WIDTH-1:0] RESET_VALUE = RESET_PC
)(
  input  logic                            clk,
  input  logic                            rst,

  // Absolute jump / load.
  input  logic                            load_en,
  input  logic [ADDR_WIDTH-1:0]           load_value,

  // Forward advance by amount (bits).
  input  logic                            advance_en,
  input  logic [PC_ADVANCE_WIDTH-1:0]     advance_amount,

  output logic [ADDR_WIDTH-1:0]           pc_o
);

  logic [ADDR_WIDTH-1:0] pc_q;
  logic [ADDR_WIDTH-1:0] pc_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      pc_q <= RESET_VALUE;
    end else begin
      pc_q <= pc_d;
    end
  end

  // Zero-extend advance_amount to full address width so the adder has
  // an explicit, lint-clean operand. The TMS34010 PC is unsigned bit-
  // addressed; wrap-around at 2^32 is the architectural behavior.
  logic [ADDR_WIDTH-1:0] advance_ext;
  assign advance_ext = {{(ADDR_WIDTH - PC_ADVANCE_WIDTH){1'b0}}, advance_amount};

  always_comb begin
    // Default: hold current value (no latch — written to a register that
    // already has this default unconditionally).
    pc_d = pc_q;
    if (load_en) begin
      pc_d = load_value;
    end else if (advance_en) begin
      pc_d = pc_q + advance_ext;
    end
  end

  assign pc_o = pc_q;

endmodule : tms34010_pc
`default_nettype wire

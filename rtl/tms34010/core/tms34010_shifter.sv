// -----------------------------------------------------------------------------
// tms34010_shifter.sv
//
// 32-bit purely combinational barrel shifter.
//
// Operations:
//   SLL : shift left, fill LSB with 0       (logical)
//   SLA : shift left, arithmetic            (same result as SLL; V records
//                                            sign/shifted-out-bit overflow)
//   SRL : shift right, fill MSB with 0      (logical)
//   SRA : shift right, sign-extend MSB      (arithmetic)
//   RL  : rotate left
//   RR  : rotate right
//
// Output flags (alu_flags_t):
//   N : result[31]
//   Z : result == 0
//   C : last bit shifted/rotated out:
//       SLL/SLA/RL: a[32 - amount]   (the MSB-side bit that just departed)
//       SRL/SRA/RR: a[amount - 1]    (the LSB-side bit that just departed)
//       amount == 0: C = 0 (nothing shifted).
//   V : SLA sets it when the new sign or any shifted-out bit differs from
//       the original sign; all other operations drive 0 for masking upstream.
//
// Synthesis notes:
//   - One `always_comb` block with safe defaults at the top.
//   - Five shift expressions are pre-computed in `always_comb`-free assigns
//     so Quartus shares the barrel-shifter network.
//   - amount == 0 is special-cased to avoid `>> 32` undefined-by-design
//     behavior in rotates.
//   - No `/`, no `%`, no `initial`, no runtime loops.
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_shifter
  import tms34010_pkg::*;
(
  input  shift_op_t                       op,
  input  logic [DATA_WIDTH-1:0]           a,
  input  logic [SHIFT_AMOUNT_WIDTH-1:0]   amount,

  output logic [DATA_WIDTH-1:0]           result,
  output alu_flags_t                      flags
);

  // Widened amount and its complement (32 - amount). Using 6 bits avoids
  // the wrap that happens when subtracting 32 in a 5-bit field.
  logic [SHIFT_AMOUNT_WIDTH:0] amount_w6;
  logic [SHIFT_AMOUNT_WIDTH:0] complement_w6;
  localparam logic [SHIFT_AMOUNT_WIDTH:0] TOP_BIT_INDEX =
    (SHIFT_AMOUNT_WIDTH + 1)'(DATA_WIDTH - 1);
  assign amount_w6     = {1'b0, amount};
  assign complement_w6 = 6'd32 - amount_w6;
  // 5-bit slices for indexing into `a`. For amount in [1..31] both are
  // valid bit indices; the amount==0 case is handled separately so the
  // bit index does not need to be 32.
  logic [SHIFT_AMOUNT_WIDTH-1:0] left_carry_idx;
  logic [SHIFT_AMOUNT_WIDTH-1:0] right_carry_idx;
  assign left_carry_idx  = complement_w6[SHIFT_AMOUNT_WIDTH-1:0];
  assign right_carry_idx = amount - 5'd1;

  // SLA overflows if any bit from the original sign through the new sign
  // differs from the original sign. XOR against a replicated sign marks all
  // differing bits; shifting the relevant top (amount+1) bits to the LSB
  // makes a reduction OR implement the rule without a variable-width slice.
  logic [DATA_WIDTH-1:0] sla_sign_diff;
  logic [DATA_WIDTH-1:0] sla_overflow_bits;
  assign sla_sign_diff     = a ^ {DATA_WIDTH{a[DATA_WIDTH-1]}};
  assign sla_overflow_bits = sla_sign_diff
                           >> (TOP_BIT_INDEX - amount_w6);

  // Pre-computed shift networks. SystemVerilog `<<`/`>>` accept any
  // unsigned amount; for amount == 0 these are all identity (no-op).
  logic [DATA_WIDTH-1:0]        left;        // a << amount
  logic [DATA_WIDTH-1:0]        right_l;     // a >> amount        (logical)
  logic signed [DATA_WIDTH-1:0] right_a;     // a >>> amount       (arithmetic)
  logic [DATA_WIDTH-1:0]        right_comp;  // a >> (32 - amount) — for RL
  logic [DATA_WIDTH-1:0]        left_comp;   // a << (32 - amount) — for RR

  assign left       = a << amount;
  assign right_l    = a >> amount;
  assign right_a    = $signed(a) >>> amount;
  assign right_comp = a >> complement_w6;
  assign left_comp  = a << complement_w6;

  always_comb begin
    // Defaults: identity passthrough.
    result  = a;
    flags.n = a[DATA_WIDTH-1];
    flags.c = 1'b0;
    flags.z = (a == '0);
    flags.v = 1'b0;

    if (amount != '0) begin
      unique case (op)
        SHIFT_OP_SLL: begin
          result  = left;
          flags.c = a[left_carry_idx];
        end
        SHIFT_OP_SLA: begin
          result  = left;
          flags.c = a[left_carry_idx];
          flags.v = |sla_overflow_bits;
        end
        SHIFT_OP_SRL: begin
          result  = right_l;
          flags.c = a[right_carry_idx];
        end
        SHIFT_OP_SRA: begin
          result  = right_a;
          flags.c = a[right_carry_idx];
        end
        SHIFT_OP_RL: begin
          result  = left | right_comp;
          flags.c = a[left_carry_idx];
        end
        SHIFT_OP_RR: begin
          result  = right_l | left_comp;
          flags.c = a[right_carry_idx];
        end
        default: ;  // unique case covers every enum
      endcase

      flags.n = result[DATA_WIDTH-1];
      flags.z = (result == '0);
    end
  end

endmodule : tms34010_shifter
`default_nettype wire

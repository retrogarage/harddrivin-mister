// -----------------------------------------------------------------------------
// tms34010_divider.sv
//
// Multi-cycle unsigned restoring divider: 64-bit dividend / 32-bit divisor
// -> 32-bit quotient + 32-bit remainder, with an overflow flag. Used by the
// DIVU / MODU instructions (and, with operand pre/post-conditioning in the
// core, DIVS / MODS later).
//
// Latency: 1 (start/latch) + 32 (iterate) + 1 (done) cycles for a normal
// divide; on overflow it short-circuits to done in 2 cycles. The result
// registers hold their values until the next `start`, so the consumer may
// read them in the cycle after `done` and beyond.
//
// Algorithm (restoring long division, MSB-first):
//   rem starts at dividend[63:32]; for each of the 32 low dividend bits,
//   shift rem left and bring the bit in, subtract the divisor if it fits,
//   and shift the quotient bit in. Overflow (true quotient >= 2^32) is
//   exactly dividend[63:32] >= divisor; divide-by-zero is divisor == 0.
//   On overflow the iterations are skipped (the consumer ignores the
//   result and only acts on the overflow flag).
//
// Single clock, synchronous active-high reset (project convention A0003).
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_divider
  import tms34010_pkg::*;
(
  input  logic                      clk,
  input  logic                      rst,
  input  logic                      start,      // 1-cycle pulse to begin
  input  logic [2*DATA_WIDTH-1:0]   dividend,   // 64-bit
  input  logic [DATA_WIDTH-1:0]     divisor,    // 32-bit
  output logic                      busy,        // high while computing
  output logic                      done,        // high in the result cycle
  output logic [DATA_WIDTH-1:0]     quotient,
  output logic [DATA_WIDTH-1:0]     remainder,
  output logic                      overflow
);

  typedef enum logic [1:0] {
    DIV_IDLE = 2'd0,
    DIV_RUN  = 2'd1,
    DIV_DONE = 2'd2
  } div_state_t;
  div_state_t            st_q;

  logic [DATA_WIDTH:0]   rem_q;       // 33-bit working remainder
  logic [DATA_WIDTH-1:0] quot_q;
  logic [DATA_WIDTH-1:0] dvd_lo_q;    // low 32 bits of the dividend, shifted out
  logic [DATA_WIDTH-1:0] divisor_q;
  logic                  ovf_q;
  logic [SHIFT_AMOUNT_WIDTH:0] iter_q;  // 6-bit, counts 0..31

  // One restoring-division step: (rem << 1) | next dividend bit, vs divisor.
  logic [DATA_WIDTH:0] shifted, divisor_ext;
  assign divisor_ext = {1'b0, divisor_q};
  assign shifted     = {rem_q[DATA_WIDTH-1:0], dvd_lo_q[DATA_WIDTH-1]};

  // Start-time overflow test (computed from the live inputs).
  logic start_overflow;
  assign start_overflow = (divisor == '0)
                       || (dividend[2*DATA_WIDTH-1:DATA_WIDTH] >= divisor);

  always_ff @(posedge clk) begin
    if (rst) begin
      st_q      <= DIV_IDLE;
      rem_q     <= '0;
      quot_q    <= '0;
      dvd_lo_q  <= '0;
      divisor_q <= '0;
      ovf_q     <= 1'b0;
      iter_q    <= '0;
    end else begin
      unique case (st_q)
        DIV_IDLE: begin
          if (start) begin
            divisor_q <= divisor;
            ovf_q     <= start_overflow;
            rem_q     <= {1'b0, dividend[2*DATA_WIDTH-1:DATA_WIDTH]};
            dvd_lo_q  <= dividend[DATA_WIDTH-1:0];
            quot_q    <= '0;
            iter_q    <= '0;
            st_q      <= start_overflow ? DIV_DONE : DIV_RUN;
          end
        end
        DIV_RUN: begin
          if (shifted >= divisor_ext) begin
            rem_q  <= shifted - divisor_ext;
            quot_q <= {quot_q[DATA_WIDTH-2:0], 1'b1};
          end else begin
            rem_q  <= shifted;
            quot_q <= {quot_q[DATA_WIDTH-2:0], 1'b0};
          end
          dvd_lo_q <= {dvd_lo_q[DATA_WIDTH-2:0], 1'b0};
          iter_q   <= iter_q + 1'b1;
          // After DATA_WIDTH iterations (indices 0..DATA_WIDTH-1) the
          // quotient is complete.
          if (iter_q == ($bits(iter_q))'(DATA_WIDTH - 1)) st_q <= DIV_DONE;
        end
        DIV_DONE: begin
          st_q <= DIV_IDLE;          // result regs persist until next start
        end
        default: st_q <= DIV_IDLE;
      endcase
    end
  end

  assign busy      = (st_q != DIV_IDLE);
  assign done      = (st_q == DIV_DONE);
  assign quotient  = quot_q;
  assign remainder = rem_q[DATA_WIDTH-1:0];
  assign overflow  = ovf_q;

endmodule : tms34010_divider
`default_nettype wire

// -----------------------------------------------------------------------------
// tms34010_int_ctrl.sv
//
// Maskable-interrupt priority encoder for the TMS34010 (1988 User's Guide §8.3
// External Interrupts / §8.4 Internal Interrupts, Tables 8-2 and 8-3).
//
// Combinational: given the INTPEND (pending) and INTENB (enable) I/O registers
// and the global interrupt-enable bit IE (ST.IE), it decides whether a maskable
// interrupt should be taken and supplies the trap-vector address of the
// highest-priority pending-AND-enabled interrupt.
//
//   active = INTPEND & INTENB. An interrupt is taken when IE = 1 and any active
//   bit is set. Priority (high → low), per the spec ("NMI > HI > DI > WV;
//   internal serviced before external; INT1 before INT2"):
//     HI (bit 9) > DI (bit 10) > WV (bit 11) > INT1 (bit 1) > INT2 (bit 2).
//   (NMI is requested through the HSTCTL register, not INTPEND, and is handled
//   separately by the core; it is not part of this encoder.)
//
// The actual interrupt RECOGNITION (sampling int_req at an instruction
// boundary) and ENTRY sequence (push ST/PC, clear IE, load PC from int_vector)
// live in the core FSM; this module only prioritises and maps to a vector.
//
// Spec source:
//   third_party/TMS34010_Info/docs/ti-official/1988_TI_TMS34010_Users_Guide.pdf
//   §8.3/§8.4, Table 8-2 (External Interrupt Vectors) / Table 8-3 (Internal).
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_int_ctrl
  import tms34010_pkg::*;
(
  input  logic [15:0]            intpend,    // INTPEND register
  input  logic [15:0]            intenb,     // INTENB register
  input  logic                   ie,         // ST.IE (global interrupt enable)

  output logic                   int_req,    // 1 = take a maskable interrupt
  output logic [ADDR_WIDTH-1:0]  int_vector  // trap-vector address of the winner
);

  // Pending and enabled.
  logic [15:0] active;
  assign active = intpend & intenb;

  // Any maskable source active, gated by the global enable.
  assign int_req = ie && (active[INT_HI_BIT] || active[INT_DI_BIT] ||
                          active[INT_WV_BIT] || active[INT_X1_BIT] ||
                          active[INT_X2_BIT]);

  // Highest-priority winner → its vector. (Value is don't-care when int_req=0.)
  always_comb begin
    if      (active[INT_HI_BIT]) int_vector = INT_VEC_HI;
    else if (active[INT_DI_BIT]) int_vector = INT_VEC_DI;
    else if (active[INT_WV_BIT]) int_vector = INT_VEC_WV;
    else if (active[INT_X1_BIT]) int_vector = INT_VEC_X1;
    else if (active[INT_X2_BIT]) int_vector = INT_VEC_X2;
    else                         int_vector = INT_VEC_HI;  // unused
  end

endmodule : tms34010_int_ctrl

`default_nettype wire

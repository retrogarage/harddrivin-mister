`default_nettype none

// MAME hdgsp_vram_1bpp_w: each source bit enables one of sixteen bytes.
// Reuse the existing four-word write-combining path in two consecutive halves.
// Shift-register transfers retain their native address and remain
// one transaction (the memory engine supplies the cockpit's 2048-word block).
module harddrivin_cockpit_expander # (parameter bit COCKPIT = 0) (
  input logic clk_i, rst_i, req_i,
  input tms34010_pkg::local_cycle_kind_t kind_i,
  input logic [31:0] addr_i,
  input logic [15:0] wdata_i,
  output logic [15:0] rdata_o,
  output logic ack_o, req_o,
  output tms34010_pkg::local_cycle_kind_t kind_o,
  output logic [31:0] addr_o,
  output logic [15:0] wdata_o,
  input logic [15:0] rdata_i,
  input logic ack_i
);
  import tms34010_pkg::*;
  typedef enum logic [2:0] {FIRST, GAP, SECOND, DONE} state_t;
  generate
  if (!COCKPIT) begin : g_compact
    assign req_o = req_i;
    assign kind_o = kind_i;
    assign addr_o = addr_i;
    assign wdata_o = wdata_i;
    assign ack_o = ack_i;
    assign rdata_o = rdata_i;
  end else begin : g_cockpit
    state_t state_q;
    wire aperture = addr_i[31:19] == (32'h02000000 >> 19);
    wire srt = kind_i == LOCAL_CYCLE_PIXEL_MTR || kind_i == LOCAL_CYCLE_PIXEL_RTM;
    wire split = aperture && kind_i == LOCAL_CYCLE_WORD_WRITE;
    assign req_o = req_i && (state_q == FIRST || state_q == SECOND);
    assign kind_o = kind_i;
    // Unmapped upper half must not alias the Compact expander aperture.
    assign addr_o = srt ? addr_i : aperture
      ? (32'h02000000 | {12'd0, addr_i[18:4], 5'd0} |
         (state_q == SECOND ? 32'h10 : 32'd0))
      : (addr_i[31:20] == 12'h020 ? 32'h02100000 : addr_i);
    always_comb begin
      wdata_o = wdata_i;
      if (split) begin
        wdata_o = 16'd0;
        for (int b = 0; b < 8; b++)
          wdata_o[2*b] = wdata_i[b + (state_q == SECOND ? 8 : 0)];
      end
    end
    assign ack_o = ack_i && req_o && (!split || state_q == SECOND);
    assign rdata_o = rdata_i;
    always_ff @(posedge clk_i) begin
      if (rst_i) state_q <= FIRST;
      else case (state_q)
        FIRST: if (req_i && ack_i) begin
          if (split) state_q <= GAP; else state_q <= DONE;
        end
        GAP: state_q <= SECOND;
        SECOND: if (ack_i) state_q <= DONE;
        DONE: if (!req_i) state_q <= FIRST;
        default: state_q <= FIRST;
      endcase
    end
  end
  endgenerate
endmodule
`default_nettype wire

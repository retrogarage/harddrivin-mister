`default_nettype none

// Two-client multiplexer for the single MiSTer DDR service port.  Each
// client is a harddrivin_ddr_rom instance that issues one read at a time and
// reports busy_o from its request until its last returned beat.  The grant
// is held for the whole transaction so DOUT_READY beats reach the owner;
// the other client sees DDRAM_BUSY high and simply waits.
module harddrivin_ddr_arbiter #(
  // 0: strict round robin (equal clients).  N > 0: side A wins while it has
  // had fewer than N consecutive grants, so a bandwidth-hungry B (the sound
  // board's EPROM/sample stream) cannot displace the main board's ROM and
  // ADSP traffic, while B still gets every N+1th slot.
  parameter int unsigned A_WEIGHT = 0
) (
  input  logic        clk_i,
  input  logic        rst_i,

  // Client A (68010 program ROM, priority).
  input  logic        a_rd_i,
  input  logic [7:0]  a_burstcnt_i,
  input  logic [28:0] a_addr_i,
  input  logic        a_busy_i,
  output logic        a_ddr_busy_o,
  output logic        a_dout_ready_o,

  // Client B (ADSP sequential EPROMs).
  input  logic        b_rd_i,
  input  logic [7:0]  b_burstcnt_i,
  input  logic [28:0] b_addr_i,
  input  logic        b_busy_i,
  output logic        b_ddr_busy_o,
  output logic        b_dout_ready_o,

  // MiSTer DDR service.
  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  input  logic        DDRAM_DOUT_READY,
  output logic        DDRAM_RD,
  // The arbiter itself as a client of an upstream arbiter: it is busy while
  // a granted transaction is outstanding or a read is being presented.
  output logic        busy_o
);
  logic grant_valid_q;
  logic grant_b_q;
  logic release_q;
  logic last_b_q;   // side served by the last grant (round robin)
  logic [7:0] a_run_q;   // consecutive grants to A (weighted mode)
  // The owner is whichever client currently holds a transaction.  A new
  // grant is decided from the request lines while nothing is outstanding.
  logic select_b;
  // Round robin when both sides request (the side that did not go last
  // wins), otherwise whichever requests: with four clients behind a tree of
  // these arbiters a continuously requesting side must not starve the other
  // (the sound 68000 waits on its EPROM reads with DTACK).
  assign select_b = grant_valid_q ? grant_b_q
                  : (a_rd_i && b_rd_i)
                      ? ((A_WEIGHT == 0) ? !last_b_q : (a_run_q >= A_WEIGHT))
                  : b_rd_i;

  // A one-cycle release bubble after each completed transaction is required
  // when this arbiter is itself a child of another arbiter. Without it,
  // back-to-back child requests keep busy_o continuously high and the parent
  // treats an arbitrarily long run as one grant, defeating its fairness/
  // weighting policy and starving the other parent-side client.
  assign DDRAM_RD       = !release_q && (select_b ? b_rd_i : a_rd_i);
  assign DDRAM_BURSTCNT = select_b ? b_burstcnt_i : a_burstcnt_i;
  assign DDRAM_ADDR     = select_b ? b_addr_i : a_addr_i;
  // Idle: a side is busy only while the OTHER side is requesting and wins
  // the selection (a side must never be held off by its own absence of a
  // request: harddrivin_ddr_rom only raises its read after seeing BUSY low).
  assign a_ddr_busy_o   = release_q || DDRAM_BUSY ||
                          (grant_valid_q && grant_b_q) ||
                          (!grant_valid_q && b_rd_i && select_b);
  assign b_ddr_busy_o   = release_q || DDRAM_BUSY ||
                          (grant_valid_q && !grant_b_q) ||
                          (!grant_valid_q && a_rd_i && !select_b);
  assign a_dout_ready_o = DDRAM_DOUT_READY && grant_valid_q && !grant_b_q;
  assign b_dout_ready_o = DDRAM_DOUT_READY && grant_valid_q && grant_b_q;
  assign busy_o         = grant_valid_q || DDRAM_RD;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      grant_valid_q <= 1'b0;
      grant_b_q     <= 1'b0;
      release_q     <= 1'b0;
      last_b_q      <= 1'b1;
      a_run_q       <= 8'd0;
    end else if (release_q) begin
      release_q <= 1'b0;
    end else if (!grant_valid_q) begin
      if (DDRAM_RD && !DDRAM_BUSY) begin
        grant_valid_q <= 1'b1;
        grant_b_q     <= select_b;
        last_b_q      <= select_b;
        a_run_q       <= select_b ? 8'd0 : (a_run_q + 8'd1);
      end
    end else if (!(grant_b_q ? b_busy_i : a_busy_i)) begin
      grant_valid_q <= 1'b0;
      release_q     <= 1'b1;
    end
  end
endmodule

`default_nettype wire

`default_nettype none

// Read-only 16-bit view of the MiSTer DDR service at byte address 0x30000000.
//
// The request/ack toggle interface and two-line cache are adapted from the
// official Genesis MiSTer DDR adapter:
//   Copyright (c) 2017, 2019 Sorgelig
//   https://github.com/MiSTer-devel/Genesis_MiSTer/blob/master/rtl/ddram.sv
//
// This modified version is distributed under GPL-3.0-or-later.  It removes
// write and second-reader support, adds deterministic reset state, and swaps
// each physical little-endian DDR halfword into the 68010's big-endian view.
module harddrivin_ddr_rom #(
  // 1: swap each physical little-endian halfword into the 68010's big-endian
  //    view.  0: return the halfword as stored (ADSP sequential EPROM image).
  parameter integer SWAP_BYTES = 1
) (
  input  logic        clk_i,
  input  logic        rst_i,

  // 16-bit word address relative to DDR byte address 0x30000000 (2 MiB).
  input  logic [20:1] rom_addr_i,
  input  logic        rom_req_i,
  output logic        rom_ack_o,
  output logic [15:0] rom_data_o,
  output logic        busy_o,

  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  input  logic [63:0] DDRAM_DOUT,
  input  logic        DDRAM_DOUT_READY,
  output logic        DDRAM_RD,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  output logic        DDRAM_WE
);
  logic [27:1] word_addr;
  logic [27:3] line_addr;
  logic [27:3] ddr_line_addr_q;
  logic [27:3] cache_addr_q;
  logic [63:0] cache_data_q;
  logic [63:0] next_data_q;
  logic        cache_valid_q;
  logic        next_valid_q;
  logic [15:0] physical_word;
  logic [1:0]  state_q;

  assign word_addr = {7'd0, rom_addr_i};
  assign line_addr = word_addr[27:3];
  assign physical_word =
    cache_data_q[{word_addr[2:1], 4'b0000} +: 16];
  assign rom_data_o = (SWAP_BYTES != 0) ? {physical_word[7:0], physical_word[15:8]}
                                       : physical_word;
  assign busy_o = (state_q != 2'd0) || DDRAM_RD;

  // DDRAM_ADDR is expressed in 64-bit words.  Prefix 0011 places line zero
  // at byte address 0x30000000, matching the MRA's direct-to-DDR address.
  assign DDRAM_ADDR = {4'b0011, ddr_line_addr_q};
  assign DDRAM_BURSTCNT = (state_q == 2'd1) ? 8'd2 : 8'd1;
  assign DDRAM_DIN = 64'd0;
  assign DDRAM_BE = {8{DDRAM_RD}};
  assign DDRAM_WE = 1'b0;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      rom_ack_o    <= 1'b0;
      cache_addr_q <= {25{1'b1}};
      ddr_line_addr_q <= 25'd0;
      cache_data_q <= 64'd0;
      next_data_q  <= 64'd0;
      cache_valid_q <= 1'b0;
      next_valid_q  <= 1'b0;
      state_q      <= 2'd0;
      DDRAM_RD     <= 1'b0;
    end else begin
      // A read request is held until the service samples it with BUSY low
      // (the Genesis adapter's contract); a one-cycle pulse is lost when
      // BUSY rises in the same cycle, which the shared-port arbiter makes
      // common.
      if (!DDRAM_BUSY) DDRAM_RD <= 1'b0;

      case (state_q)
        2'd0: begin
          if (rom_req_i != rom_ack_o) begin
            if (cache_valid_q && (cache_addr_q == line_addr)) begin
              rom_ack_o <= rom_req_i;
            end else if (next_valid_q &&
                         ((cache_addr_q + 25'd1) == line_addr)) begin
              cache_data_q <= next_data_q;
              cache_addr_q <= line_addr;
              cache_valid_q <= 1'b1;
              next_valid_q  <= 1'b0;
              rom_ack_o    <= rom_req_i;
              if (!DDRAM_BUSY) begin
                ddr_line_addr_q <= line_addr + 25'd1;
                DDRAM_RD <= 1'b1;
                state_q  <= 2'd3;
              end else begin
                ddr_line_addr_q <= line_addr + 25'd1;
                state_q <= 2'd2;
              end
            end else if (!DDRAM_BUSY) begin
              cache_addr_q <= line_addr;
              cache_valid_q <= 1'b0;
              next_valid_q  <= 1'b0;
              ddr_line_addr_q <= line_addr;
              DDRAM_RD     <= 1'b1;
              state_q      <= 2'd1;
            end
          end
        end

        // A two-word miss read has been accepted.  The first returned line
        // satisfies the CPU; the second is retained as a sequential prefetch.
        2'd1: begin
          if (DDRAM_DOUT_READY) begin
            cache_data_q <= DDRAM_DOUT;
            cache_valid_q <= 1'b1;
            rom_ack_o    <= rom_req_i;
            state_q      <= 2'd3;
          end
        end

        // Wait to issue the one-line prefetch after promoting next_data_q.
        2'd2: begin
          if (!DDRAM_BUSY) begin
            DDRAM_RD <= 1'b1;
            state_q  <= 2'd3;
          end
        end

        // Consume either the second beat of a miss or the promoted-line
        // prefetch.  The MiSTer service asserts DOUT_READY once per beat.
        default: begin
          if (DDRAM_DOUT_READY) begin
            next_data_q <= DDRAM_DOUT;
            next_valid_q <= 1'b1;
            state_q     <= 2'd0;
          end
        end
      endcase
    end
  end
endmodule

`default_nettype wire

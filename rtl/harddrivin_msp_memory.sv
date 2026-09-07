`default_nettype none

// Compact MSP storage: four 64Kx4 DRAMs form one 64Kx16 memory. The board
// aliases the same physical words at 00000000, 00700000 and fff00000 in the
// TMS34010 bit-address space.
module harddrivin_msp_memory
  import tms34010_pkg::*;
#(
  // Extra clocks before a RAM read/write is acknowledged, to approximate the
  // original board's DRAM cycle time (the reused core's local bus completes
  // in one clock against this glue).  0 = as before.
  parameter integer WAIT_STATES = 0
) (
  input  logic              clk_i,
  input  logic              rst_i,
  input  logic              cycle_req_i,
  input  local_cycle_kind_t cycle_kind_i,
  input  logic [31:0]       cycle_addr_i,
  input  logic [15:0]       cycle_wdata_i,
  input  logic [15:0]       cycle_io_rdata_i,
  input  logic              cycle_iaq_i,
  output logic [15:0]       cycle_rdata_o,
  output logic              cycle_ack_o,
  output logic [31:0]       write_count_o,
  output logic              instruction_fetch_seen_o,
  output logic              roundtrip_seen_o
);
  logic request_seen_q;
  logic [3:0] wait_q;
  logic ram_read_response_q;
  logic last_write_valid_q;
  logic [15:0] last_write_addr_q;
  logic [15:0] response_rdata_q;
  logic [15:0] ram_rdata;

  logic mapped_ram;
  logic word_read;
  logic word_write;
  logic [15:0] word_index;
  logic ram_access;

  assign mapped_ram = (cycle_addr_i[31:20] == 12'h000) ||
                      (cycle_addr_i[31:20] == 12'h007) ||
                      (cycle_addr_i[31:20] == 12'hfff);
  assign word_read = (cycle_kind_i == LOCAL_CYCLE_WORD_READ) ||
                     (cycle_kind_i == LOCAL_CYCLE_PIXEL_MTR);
  assign word_write = (cycle_kind_i == LOCAL_CYCLE_WORD_WRITE) ||
                      (cycle_kind_i == LOCAL_CYCLE_PIXEL_RTM);
  assign word_index = cycle_addr_i[19:4];
  assign ram_access = cycle_req_i && !request_seen_q && (wait_q == WAIT_STATES[3:0]) &&
                      mapped_ram && (word_read || word_write);
  assign cycle_rdata_o = ram_read_response_q ? ram_rdata : response_rdata_q;

  harddrivin_word_ram #(.ADDR_WIDTH(16)) u_msp_ram (
    .clk_i(clk_i), .en_i(ram_access), .we_i(word_write),
    .addr_i(word_index), .wdata_i(cycle_wdata_i), .rdata_o(ram_rdata)
  );

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      request_seen_q          <= 1'b0;
      wait_q                  <= 4'd0;
      ram_read_response_q     <= 1'b0;
      last_write_valid_q      <= 1'b0;
      last_write_addr_q       <= 16'd0;
      response_rdata_q        <= 16'hffff;
      cycle_ack_o             <= 1'b0;
      write_count_o           <= 32'd0;
      instruction_fetch_seen_o <= 1'b0;
      roundtrip_seen_o        <= 1'b0;
    end else begin
      cycle_ack_o <= 1'b0;
      ram_read_response_q <= 1'b0;
      if (!cycle_req_i) begin request_seen_q <= 1'b0; wait_q <= 4'd0; end
      else if (!request_seen_q && (wait_q != WAIT_STATES[3:0])) wait_q <= wait_q + 4'd1;

      if (cycle_req_i && !request_seen_q && (wait_q == WAIT_STATES[3:0])) begin
        wait_q         <= 4'd0;
        request_seen_q <= 1'b1;
        cycle_ack_o    <= 1'b1;
        response_rdata_q <= 16'hffff;

        if (cycle_kind_i == LOCAL_CYCLE_IO_READ)
          response_rdata_q <= cycle_io_rdata_i;
        else if (word_read && mapped_ram) begin
          ram_read_response_q <= 1'b1;
          if (cycle_iaq_i) instruction_fetch_seen_o <= 1'b1;
          if (last_write_valid_q && word_index == last_write_addr_q)
            roundtrip_seen_o <= 1'b1;
        end else if (word_write && mapped_ram) begin
          last_write_valid_q    <= 1'b1;
          last_write_addr_q     <= word_index;
          write_count_o         <= write_count_o + 32'd1;
        end
      end
    end
  end
endmodule

`default_nettype wire

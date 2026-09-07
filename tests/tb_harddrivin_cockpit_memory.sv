`timescale 1ns/1ps
`default_nettype none

module tb_harddrivin_cockpit_memory;
  import tms34010_pkg::*;

  logic clk = 1'b0;
  logic mem_clk = 1'b0;
  always #10 clk = !clk;
  always #7 mem_clk = !mem_clk;

  logic rst = 1'b1;
  logic mem_rst = 1'b1;
  logic cycle_req, cycle_ack, cycle_iaq;
  local_cycle_kind_t cycle_kind;
  logic [31:0] cycle_addr;
  logic [15:0] cycle_wdata, cycle_rdata, cycle_io_rdata;
  logic vram_req, vram_we, vram_ack;
  logic [15:0] srt_copy_count;
  logic srt_pending;
  integer srt_wait;
  task automatic wait_copies; begin srt_wait = 0; while (srt_pending && srt_wait < 40000) begin @(posedge clk); srt_wait = srt_wait + 1; end if (srt_pending) begin $error("SRT copies did not drain"); $fatal(1); end end endtask
  logic [7:0] vram_be;
  logic [18:0] vram_addr;
  logic [63:0] vram_wdata;
  logic [15:0] vram_rdata;
  logic [63:0] vram_rdata64;
  logic [31:0] peripheral_write_count, vram_write_count;
  logic vram_write_nonzero_seen;
  logic [15:0] control_write_count;
  logic [15:0] palette_lo_write_count, palette_hi_write_count;
  logic palette_write_nonzero_seen, palette_scan_nonzero_seen;
  logic [9:0] last_palette_addr;
  logic [15:0] last_palette_data;
  logic [15:0] palette_bank2_ff_lo, palette_bank2_ff_hi;
  logic [15:0] palette_bank3_ff_lo, palette_bank3_ff_hi;
  logic [63:0] control_write_trace;
  logic [7:0] control_hi_write_count;
  logic [31:0] control_read_trace;
  logic [7:0] control_hi_read_count;
  logic [15:0] expander_write_count;
  logic instruction_fetch_seen, peripheral_roundtrip_seen;
  logic vram_roundtrip_seen;
  logic [9:0] palette_scan_addr;
  logic [15:0] palette_scan_lo, palette_scan_hi;
  logic [3:0] fine_scroll;
  logic [1:0] palette_bank;
  logic shiftreg_enable;

  logic [15:0] memory [0:524287];
  logic memory_seen;
  integer memory_accesses;

  logic mem_req, mem_ack;
  local_cycle_kind_t mem_kind;
  logic [31:0] mem_addr;
  logic [15:0] mem_data, mem_rdata;
  harddrivin_cockpit_expander #(.COCKPIT(1)) adapter (
    .clk_i(clk), .rst_i(rst), .req_i(cycle_req), .kind_i(cycle_kind),
    .addr_i(cycle_addr), .wdata_i(cycle_wdata), .ack_o(cycle_ack), .rdata_o(cycle_rdata),
    .req_o(mem_req), .kind_o(mem_kind), .addr_o(mem_addr), .wdata_o(mem_data),
    .ack_i(mem_ack), .rdata_i(mem_rdata)
  );
  harddrivin_gsp_memory #(.COCKPIT(1), .VRAM_AW(19), .SRT_BIG_BITS(11)) dut (
    .clk_i(clk), .rst_i(rst), .cycle_req_i(mem_req),
    .cycle_kind_i(mem_kind), .cycle_addr_i(mem_addr),
    .cycle_wdata_i(mem_data), .cycle_io_rdata_i(cycle_io_rdata),
    .cycle_iaq_i(cycle_iaq), .cycle_rdata_o(mem_rdata),
    .cycle_ack_o(mem_ack), .mem_clk_i(mem_clk), .mem_rst_i(mem_rst),
    .vram_req_o(vram_req), .vram_we_o(vram_we), .vram_be_o(vram_be),
    .vram_addr_o(vram_addr),
    .vram_wdata_o(vram_wdata), .vram_rdata_i(vram_rdata), .vram_rdata64_i(vram_rdata64),
    .vram_ack_i(vram_ack), .srt_copy_count_o(srt_copy_count), .srt_pending_o(srt_pending), .peripheral_write_count_o(peripheral_write_count),
    .control_write_count_o(control_write_count),
    .palette_lo_write_count_o(palette_lo_write_count),
    .palette_hi_write_count_o(palette_hi_write_count),
    .palette_write_nonzero_seen_o(palette_write_nonzero_seen),
    .palette_scan_nonzero_seen_o(palette_scan_nonzero_seen),
    .last_palette_addr_o(last_palette_addr),
    .last_palette_data_o(last_palette_data),
    .palette_bank2_ff_lo_o(palette_bank2_ff_lo),
    .palette_bank2_ff_hi_o(palette_bank2_ff_hi),
    .palette_bank3_ff_lo_o(palette_bank3_ff_lo),
    .palette_bank3_ff_hi_o(palette_bank3_ff_hi),
    .control_write_trace_o(control_write_trace),
    .control_hi_write_count_o(control_hi_write_count),
    .control_read_trace_o(control_read_trace),
    .control_hi_read_count_o(control_hi_read_count),
    .palette_scan_addr_i(palette_scan_addr),
    .palette_scan_lo_o(palette_scan_lo),
    .palette_scan_hi_o(palette_scan_hi),
    .fine_scroll_o(fine_scroll), .palette_bank_o(palette_bank),
    .shiftreg_enable_o(shiftreg_enable),
    .vram_write_count_o(vram_write_count),
    .vram_write_nonzero_seen_o(vram_write_nonzero_seen),
    .expander_write_count_o(expander_write_count),
    .instruction_fetch_seen_o(instruction_fetch_seen),
    .peripheral_roundtrip_seen_o(peripheral_roundtrip_seen),
    .vram_roundtrip_seen_o(vram_roundtrip_seen)
  );

  always_ff @(posedge mem_clk) begin
    if (mem_rst) begin
      vram_ack       <= 1'b0;
      vram_rdata     <= 16'hffff;
      memory_seen    <= 1'b0;
      memory_accesses <= 0;
    end else begin
      vram_ack <= 1'b0;
      if (!vram_req) memory_seen <= 1'b0;
      if (vram_req && !memory_seen) begin
        memory_seen     <= 1'b1;
        memory_accesses <= memory_accesses + 1;
        if (vram_we) begin
          for (int k = 0; k < 4; k++) begin   // a write is the whole 4-word line
            if (vram_be[2*k])   memory[{vram_addr[18:2], 2'(k)}][7:0]  <= vram_wdata[16*k +: 8];
            if (vram_be[2*k+1]) memory[{vram_addr[18:2], 2'(k)}][15:8] <= vram_wdata[16*k+8 +: 8];
          end
        end
        else begin
          vram_rdata   <= memory[vram_addr[18:0]];
          vram_rdata64 <= {memory[{vram_addr[18:2], 2'b11}], memory[{vram_addr[18:2], 2'b10}],
                           memory[{vram_addr[18:2], 2'b01}], memory[{vram_addr[18:2], 2'b00}]};
        end
        vram_ack <= 1'b1;
      end
    end
  end

  task automatic access(input local_cycle_kind_t kind,
                        input logic [31:0] address,
                        input logic [15:0] write_data,
                        input logic iaq,
                        input logic [15:0] expected_read);
    integer timeout;
    begin
      @(negedge clk);
      cycle_req   = 1'b1;
      cycle_kind  = kind;
      cycle_addr  = address;
      cycle_wdata = write_data;
      cycle_iaq   = iaq;
      timeout = 0;
      while (!cycle_ack && timeout < 20000) begin
        @(posedge clk);
        #1;
        timeout = timeout + 1;
      end
      if (!cycle_ack) begin
        $error("GSP memory access timed out at %08x", address);
        $fatal(1);
      end
      if ((kind == LOCAL_CYCLE_WORD_READ || kind == LOCAL_CYCLE_PIXEL_MTR ||
           kind == LOCAL_CYCLE_IO_READ) && cycle_rdata !== expected_read) begin
        $error("GSP read %08x returned=%04x expected=%04x",
               address, cycle_rdata, expected_read);
        $fatal(1);
      end
      @(negedge clk);
      cycle_req = 1'b0;
      cycle_iaq = 1'b0;
      repeat (3) @(posedge clk);
    end
  endtask

  logic [15:0] mask, want;
  initial begin
    cycle_req=0; cycle_kind=LOCAL_CYCLE_WORD_READ; cycle_addr=0;
    cycle_wdata=0; cycle_io_rdata=16'h3cc3; cycle_iaq=0; palette_scan_addr=0;
    for(int k=0;k<524288;k++) memory[k]=16'h3c5a;
    repeat(5) @(negedge clk); rst=0; mem_rst=0;
    // Cockpit low-byte colour is replicated, not Compact's two-byte pair.
    access(LOCAL_CYCLE_WORD_WRITE,32'hf4000000,16'hab72,0,0);
    for(int t=0;t<80;t++) begin
      mask=t<16 ? (16'b1<<t) : 16'($random);
      access(LOCAL_CYCLE_WORD_WRITE,32'h02000000+32'(t*16),mask,0,0);
      for(int w=0;w<8;w++) begin
        want={mask[2*w+1]?8'h72:8'h3c,mask[2*w]?8'h72:8'h5a};
        access(LOCAL_CYCLE_WORD_READ,32'hff800000+32'((t*8+w)*16),0,0,want);
      end
    end
    access(LOCAL_CYCLE_WORD_READ,32'h02000000,0,0,0);
    access(LOCAL_CYCLE_WORD_READ,32'h02080000,0,0,16'hffff);
    // The upper 512 KiB is independent, including the reset vector region.
    access(LOCAL_CYCLE_WORD_WRITE,32'hff801000,16'h1357,0,0);
    access(LOCAL_CYCLE_WORD_WRITE,32'hffc01000,16'h2468,0,0);
    access(LOCAL_CYCLE_WORD_READ,32'hff801000,0,0,16'h1357);
    access(LOCAL_CYCLE_WORD_READ,32'hffc01000,0,0,16'h2468);
    for(int f=0;f<16;f++) begin
      access(LOCAL_CYCLE_WORD_WRITE,32'hf4800090,16'(f),0,0);
      if(fine_scroll!==4'(f)) $fatal(1,"cockpit fine scroll needs all four bits on every write");
    end
    for(int k=0;k<2048;k++) memory[8192+k]=16'(k)^16'hb519;
    access(LOCAL_CYCLE_WORD_WRITE,32'hf4800080,0,0,0);
    access(LOCAL_CYCLE_PIXEL_MTR,32'h02004000,0,0,0);
    access(LOCAL_CYCLE_PIXEL_RTM,32'h02080000,0,0,0);
    repeat(40) @(posedge clk); wait_copies();
    for(int k=0;k<2048;k++)
      if(memory[262144+k] !== (16'(k)^16'hb519)) $fatal(1,"cockpit 2048-word SRT copy at %0d got %h",k,memory[262144+k]);
    if(memory[264192]!==16'h3c5a) $fatal(1,"SRT overrun");
    $display("PASS cockpit MAME expander masks/colour, full VRAM, four-bit scroll, 2048-word SRT across upper half");
    $finish;
  end
endmodule

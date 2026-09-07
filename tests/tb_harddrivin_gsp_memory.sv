`timescale 1ns/1ps
`default_nettype none

module tb_harddrivin_gsp_memory;
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
  logic [17:0] vram_addr;
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

  logic [15:0] memory [0:4095];
  logic memory_seen;
  integer memory_accesses;

  harddrivin_gsp_memory dut (
    .clk_i(clk), .rst_i(rst), .cycle_req_i(cycle_req),
    .cycle_kind_i(cycle_kind), .cycle_addr_i(cycle_addr),
    .cycle_wdata_i(cycle_wdata), .cycle_io_rdata_i(cycle_io_rdata),
    .cycle_iaq_i(cycle_iaq), .cycle_rdata_o(cycle_rdata),
    .cycle_ack_o(cycle_ack), .mem_clk_i(mem_clk), .mem_rst_i(mem_rst),
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
            if (vram_be[2*k])   memory[{vram_addr[11:2], 2'(k)}][7:0]  <= vram_wdata[16*k +: 8];
            if (vram_be[2*k+1]) memory[{vram_addr[11:2], 2'(k)}][15:8] <= vram_wdata[16*k+8 +: 8];
          end
        end
        else begin
          vram_rdata   <= memory[vram_addr[11:0]];
          vram_rdata64 <= {memory[{vram_addr[11:2], 2'b11}], memory[{vram_addr[11:2], 2'b10}],
                           memory[{vram_addr[11:2], 2'b01}], memory[{vram_addr[11:2], 2'b00}]};
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

  initial begin
    cycle_req = 1'b0;
    cycle_kind = LOCAL_CYCLE_WORD_READ;
    cycle_addr = 32'd0;
    cycle_wdata = 16'd0;
    cycle_io_rdata = 16'h3cc3;
    cycle_iaq = 1'b0;
    palette_scan_addr = 10'd0;
    vram_ack = 1'b0;
    vram_rdata = 16'hffff;
    memory[10'h048] = 16'h1111;
    memory[10'h049] = 16'h2222;
    memory[10'h04a] = 16'h3333;
    memory[10'h04b] = 16'h4444;
    repeat (5) @(posedge clk);
    rst = 1'b0;
    mem_rst = 1'b0;

    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4000030, 16'h1234, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_READ,  32'hf4000030, 16'h0000, 1'b0, 16'h1234);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf5000450, 16'h5678, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_READ,  32'hf5000450, 16'h0000, 1'b0, 16'h5678);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf5800450, 16'h9abc, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_READ,  32'hf5800450, 16'h0000, 1'b0, 16'h9abc);
    palette_scan_addr = 10'h045;
    repeat (3) @(posedge clk);

    access(LOCAL_CYCLE_WORD_WRITE, 32'hff800120, 16'ha55a, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_READ,  32'hffc00120, 16'h0000, 1'b1, 16'ha55a);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4000000, 16'ha55a, 1'b0, 16'd0);
    // LA7 drives the LS259 SCROLL latch; its rising edge clocks LAD3:0 into
    // the board's fine-scroll register.  LAD is the multiplexed address/data
    // bus and the latch changes during the data phase, so the register
    // captures the write DATA (MAME: finescroll = data & 7); the address
    // nibble is irrelevant.  The game writes 000f (top ISR) / 00ff (bottom).
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4800010, 16'hffff, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf480009a, 16'h0002, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4800093, 16'hffff, 1'b0, 16'd0);
    if (fine_scroll !== 4'h2) begin
      $error("Compact fine scroll failed edge hold: %x", fine_scroll);
      $fatal(1);
    end
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4800010, 16'h0000, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4800095, 16'h000d, 1'b0, 16'd0);
    if (fine_scroll !== 4'h5) begin
      $error("Compact fine scroll failed second edge: %x", fine_scroll);
      $fatal(1);
    end
    // Select palette bank 3 through the two LS259 addressable latches and
    // exercise the entries used by the hardware palette probe.
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf48000a0, 16'h0000, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf48000b0, 16'h0000, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf5000010, 16'h0101, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf5800010, 16'h0101, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf5000ff0, 16'hffff, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf5800ff0, 16'hffff, 1'b0, 16'd0);
    palette_scan_addr = 10'h3ff;
    repeat (3) @(posedge clk);
    access(LOCAL_CYCLE_WORD_WRITE, 32'h02000120, 16'h5145, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_READ,  32'h02000120, 16'h0000, 1'b0, 16'h0000);
    access(LOCAL_CYCLE_IO_READ,    32'hc0000080, 16'h0000, 1'b0, 16'h3cc3);
    access(LOCAL_CYCLE_WORD_READ,  32'h12300120, 16'h0000, 1'b0, 16'hffff);
    // the expander line buffer flushes after 255 idle clocks and drains
    repeat (400) @(posedge clk);

    if (memory[10'h048] !== 16'ha55a || memory[10'h049] !== 16'ha522 ||
        memory[10'h04a] !== 16'h335a || memory[10'h04b] !== 16'ha55a) begin
      $error("Compact 2-bpp expansion mismatch %04x %04x %04x %04x",
             memory[10'h048], memory[10'h049], memory[10'h04a], memory[10'h04b]);
      $fatal(1);
    end

    if (peripheral_write_count !== 32'd15 ||
        control_write_count !== 16'd9 ||
        palette_lo_write_count !== 16'd3 ||
        palette_hi_write_count !== 16'd3 ||
        !palette_write_nonzero_seen || !palette_scan_nonzero_seen ||
        last_palette_addr !== 10'h3ff || last_palette_data !== 16'hffff ||
        palette_scan_lo !== 16'hffff || palette_scan_hi !== 16'hffff ||
        palette_bank2_ff_lo !== 16'h0000 ||
        palette_bank2_ff_hi !== 16'h0000 ||
        palette_bank3_ff_lo !== 16'hffff ||
        palette_bank3_ff_hi !== 16'hffff ||
        control_write_trace !== 64'h0000_0000_0199_19ab ||
        control_hi_write_count !== 8'd7 ||
        vram_write_count !== 32'd2 || !vram_write_nonzero_seen ||
        expander_write_count !== 16'd1 ||
        memory_accesses != 3 || !instruction_fetch_seen ||   // the expander word is one line write
        !peripheral_roundtrip_seen || !vram_roundtrip_seen) begin
      $error("GSP diagnostics peripheral=%0d control=%0d palette=%0d/%0d nz=%0b/%0b last=%03x:%04x vram=%0d/%0b expanded=%0d mem=%0d fetch=%0b peripheral_rt=%0b vram_rt=%0b",
             peripheral_write_count, control_write_count,
             palette_lo_write_count, palette_hi_write_count,
             palette_write_nonzero_seen, palette_scan_nonzero_seen,
             last_palette_addr, last_palette_data,
             vram_write_count, vram_write_nonzero_seen,
             expander_write_count,
             memory_accesses,
             instruction_fetch_seen, peripheral_roundtrip_seen,
             vram_roundtrip_seen);
      $fatal(1);
    end
    // ---- caches and posted writes: instruction/data misses reach VRAM,
    // hits do not, any write invalidates the line, posted writes drain ----
    begin : cache_test
      integer acc0;
      access(LOCAL_CYCLE_WORD_WRITE, 32'hfff40000, 16'h1234, 1'b0, 16'h0000);   // posted
      access(LOCAL_CYCLE_WORD_WRITE, 32'hfff40010, 16'h5678, 1'b0, 16'h0000);   // posted
      repeat (120) @(posedge clk);                                                // drain
      acc0 = memory_accesses;
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b1, 16'h1234);    // I-miss -> VRAM
      if (memory_accesses != acc0 + 1) begin $error("icache: miss did not reach VRAM (%0d)", memory_accesses - acc0); $fatal(1); end
      acc0 = memory_accesses;
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b1, 16'h1234);    // I-hit
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b1, 16'h1234);    // I-hit
      if (memory_accesses != acc0) begin $error("icache: hit reached VRAM (%0d)", memory_accesses - acc0); $fatal(1); end
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b0, 16'h1234);    // D-miss (separate cache)
      if (memory_accesses != acc0 + 1) begin $error("dcache: first data read must reach VRAM"); $fatal(1); end
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b0, 16'h1234);    // D-hit
      if (memory_accesses != acc0 + 1) begin $error("dcache: data hit reached VRAM"); $fatal(1); end
      access(LOCAL_CYCLE_WORD_WRITE, 32'hfff40000, 16'habcd, 1'b0, 16'h0000);   // posted, invalidates both lines
      repeat (120) @(posedge clk);
      acc0 = memory_accesses;
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b1, 16'habcd);    // I-miss, new data
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40000, 16'h0000, 1'b0, 16'habcd);    // D-miss, new data
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40010, 16'h0000, 1'b1, 16'h5678);    // same line: refilled by the I-miss's 4-word line fill -> hit
      if (memory_accesses != acc0 + 2) begin $error("caches: write did not invalidate / line fill missing (%0d accesses)", memory_accesses - acc0); $fatal(1); end
      acc0 = memory_accesses;
      access(LOCAL_CYCLE_WORD_READ, 32'hfff40010, 16'h0000, 1'b1, 16'h5678);    // I-hit again
      if (memory_accesses != acc0) begin $error("icache: refill did not cache"); $fatal(1); end
      // a line fill must not cache a word whose posted write has not drained
      access(LOCAL_CYCLE_WORD_WRITE, 32'hfff40020, 16'h9abc, 1'b0, 16'h0000);   // posted (word 2 of the line)
      access(LOCAL_CYCLE_WORD_READ,  32'hfff40030, 16'h0000, 1'b0, memory[12'h003]);   // D-miss on word 3: fills the line except word 2
      access(LOCAL_CYCLE_WORD_READ,  32'hfff40020, 16'h0000, 1'b0, 16'h9abc);   // must see the posted write (miss after the drain)
      access(LOCAL_CYCLE_WORD_READ,  32'hfff40000, 16'h0000, 1'b0, 16'habcd);   // word 0 came with the fill: hit
      // a read right after a posted write must see the new data (drain first)
      access(LOCAL_CYCLE_WORD_WRITE, 32'hfff40020, 16'h4444, 1'b0, 16'h0000);
      access(LOCAL_CYCLE_WORD_READ,  32'hfff40020, 16'h0000, 1'b0, 16'h4444);
      $display("cache test: I/D hits served from cache, misses, invalidations and posted-write ordering correct");
    end

    // ---- expander write combining ----
    // Two expander writes to the same line with different colour pairs merge
    // into one flush; a linear read of the line right after (forces the
    // flush, waits for the queued write to the same word) sees both.
    memory[10'h050] = 16'h0000; memory[10'h051] = 16'h0000;
    memory[10'h052] = 16'h0000; memory[10'h053] = 16'h0000;
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4000000, 16'h1122, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'h02000140, 16'h0005, 1'b0, 16'd0);   // word 0: both bytes
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4000000, 16'h3344, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'h02000140, 16'h0410, 1'b0, 16'd0);   // word 1 low byte, word 2 high byte
    access(LOCAL_CYCLE_WORD_READ,  32'hff800500, 16'h0000, 1'b0, 16'h1122);
    access(LOCAL_CYCLE_WORD_READ,  32'hff800510, 16'h0000, 1'b0, 16'h0044);
    access(LOCAL_CYCLE_WORD_READ,  32'hff800520, 16'h0000, 1'b0, 16'h3300);
    access(LOCAL_CYCLE_WORD_READ,  32'hff800530, 16'h0000, 1'b0, 16'h0000);
    // a different line flushes the first; both reach memory in order
    access(LOCAL_CYCLE_WORD_WRITE, 32'h02000150, 16'h0001, 1'b0, 16'd0);   // line 0x54, word 0 low byte
    access(LOCAL_CYCLE_WORD_WRITE, 32'h02000140, 16'h4000, 1'b0, 16'd0);   // back to line 0x50: word 3 high byte
    repeat (400) @(posedge clk);
    if (memory[10'h054][7:0] !== 8'h44 || memory[10'h053] !== 16'h3300 || memory[10'h050] !== 16'h1122) begin
      $error("expander write combining: mem[54]=%h mem[53]=%h mem[50]=%h", memory[10'h054], memory[10'h053], memory[10'h050]);
      $fatal(1);
    end
    // a posted linear write followed by a read of another word does not wait
    // for the drain, a read of the same word sees the write
    access(LOCAL_CYCLE_WORD_WRITE, 32'hff800600, 16'hbeef, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_READ,  32'hff800610, 16'h0000, 1'b0, memory[10'h061]);
    access(LOCAL_CYCLE_WORD_READ,  32'hff800600, 16'h0000, 1'b0, 16'hbeef);
    $display("expander write combining OK");

    // ---- VRAM shift-register block copies (DPYCTL.SRT pixel cycles) ----
    // Normal VRAM: MTR at ff800000 latches source block 0 (words 0..255);
    // RTM at ff802000 (a[21:12] = 2 -> row 2) copies it to words 512..767
    // when the SHIFTREG latch (f4800080) is set.
    for (int k = 0; k < 4096; k++) memory[k] = 16'h0000;
    for (int k = 0; k < 256; k++) memory[k] = 16'hc000 + 16'(k);
    for (int k = 512; k < 768; k++) memory[k] = 16'hdead;
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4800000, 16'h0000, 1'b0, 16'd0);   // latch off
    access(LOCAL_CYCLE_PIXEL_MTR,  32'hff800000, 16'h0000, 1'b0, 16'h0000);
    access(LOCAL_CYCLE_PIXEL_RTM,  32'hff802000, 16'h1234, 1'b0, 16'd0);   // latch off: no copy
    repeat (40) @(posedge clk); wait_copies();
    if (memory[512] !== 16'hdead || srt_copy_count !== 16'd0) begin
      $error("SRT: copy happened with the latch off (mem[512]=%h count=%0d)", memory[512], srt_copy_count);
      $fatal(1);
    end
    access(LOCAL_CYCLE_WORD_WRITE, 32'hf4800080, 16'h0000, 1'b0, 16'd0);   // latch on
    access(LOCAL_CYCLE_PIXEL_RTM,  32'hff802000, 16'h1234, 1'b0, 16'd0);   // copies 256 words (queued)
    // the RTM is acknowledged at once; a read inside the pending range waits
    // for the copy and returns the copied word, a read outside proceeds now
    srt_wait = 0;
    memory[3000] = 16'h3000;
    access(LOCAL_CYCLE_WORD_READ,  32'hff80bb80, 16'h0000, 1'b0, 16'h3000);   // word 3000: outside the pending range, served during the copy
    if (!srt_pending) begin $error("SRT: copy already finished before the outside read (no overlap)"); end
    access(LOCAL_CYCLE_WORD_READ,  32'hff802010, 16'h0000, 1'b0, 16'hc001);   // word 513 = copied data (waits for the drain)
    wait_copies();
    for (int k = 0; k < 256; k++) begin
      if (memory[512 + k] !== 16'hc000 + 16'(k)) begin
        $error("SRT normal copy: mem[%0d]=%h expected %h", 512 + k, memory[512 + k], 16'hc000 + 16'(k));
        $fatal(1);
      end
    end
    if (memory[768] !== 16'h0000 || srt_copy_count !== 16'd1) begin
      $error("SRT normal copy overran or count wrong (mem[768]=%h count=%0d)", memory[768], srt_copy_count);
      $fatal(1);
    end
    // Same (source, destination) pair again with nothing written in between:
    // the destination already holds the copy, the engine skips it.
    access(LOCAL_CYCLE_PIXEL_RTM,  32'hff802010, 16'h5678, 1'b0, 16'd0);
    repeat (40) @(posedge clk); wait_copies();
    if (srt_copy_count !== 16'd1) begin
      $error("SRT: repeated block pair was not skipped (count=%0d)", srt_copy_count);
      $fatal(1);
    end
    // An ordinary write in between forces the copy to run again.
    access(LOCAL_CYCLE_WORD_WRITE, 32'hff802000, 16'h0bad, 1'b0, 16'd0);
    access(LOCAL_CYCLE_PIXEL_RTM,  32'hff802000, 16'h5678, 1'b0, 16'd0);
    repeat (40) @(posedge clk); wait_copies();
    if (srt_copy_count !== 16'd2 || memory[512] !== 16'hc000) begin
      $error("SRT: copy after an ordinary write did not run (count=%0d mem[512]=%h)", srt_copy_count, memory[512]);
      $fatal(1);
    end
    // 2-bpp window: MTR at 02001000 (a[19:12] = 1) -> source block words
    // 1024..2047; RTM at 02002000 (a[19:12] = 2) -> destination words
    // 2048..3071 (1024 words = 4 rows).
    for (int k = 1024; k < 2048; k++) memory[k] = 16'ha000 + 16'(k);
    for (int k = 2048; k < 3072; k++) memory[k] = 16'hbeef;
    access(LOCAL_CYCLE_PIXEL_MTR,  32'h02001000, 16'h0000, 1'b0, 16'h0000);
    access(LOCAL_CYCLE_PIXEL_RTM,  32'h02002000, 16'h0000, 1'b0, 16'd0);
    // an access outside the pending range (word 3500) completes while the copy runs
    memory[3500] = 16'h3500;
    srt_wait = 0;
    access(LOCAL_CYCLE_WORD_READ,  32'hff80dac0, 16'h0000, 1'b0, 16'h3500);   // ff800000 + 3500*16 = ff80dac0
    if (!srt_pending) begin $error("SRT: 1024-word copy finished before an outside read could be served (no overlap)"); end
    wait_copies();
    for (int k = 0; k < 1024; k++) begin
      if (memory[2048 + k] !== 16'ha000 + 16'(1024 + k)) begin
        $error("SRT 2bpp copy: mem[%0d]=%h expected %h", 2048 + k, memory[2048 + k], 16'ha000 + 16'(1024 + k));
        $fatal(1);
      end
    end
    if (memory[3072] !== 16'h0000 || srt_copy_count !== 16'd3) begin
      $error("SRT 2bpp copy overran or count wrong (mem[3072]=%h count=%0d)", memory[3072], srt_copy_count);
      $fatal(1);
    end

    // ---- copy on demand: source block 3 (words 3072..4095), two queued
    // copies A (dst words 0..1023) then B (dst words 2048..3071); a read into
    // B's block must run B's copy first and return the copied word while A
    // is still pending.  (New block pairs: the repeat-skip would treat a
    // pair already copied above as done.)
    wait_copies();
    for (int k = 3072; k < 4096; k++) memory[k] = 16'h5000 + 16'(k);
    for (int k = 0; k < 1024; k++) memory[k] = 16'hbbbb;
    for (int k = 2048; k < 3072; k++) memory[k] = 16'hbbbb;
    access(LOCAL_CYCLE_PIXEL_MTR,  32'h02003000, 16'h0000, 1'b0, 16'h0000);
    access(LOCAL_CYCLE_PIXEL_RTM,  32'h02000000, 16'h0000, 1'b0, 16'd0);   // A: words 0..1023
    access(LOCAL_CYCLE_PIXEL_RTM,  32'h02002000, 16'h0000, 1'b0, 16'd0);   // B: words 2048..3071
    access(LOCAL_CYCLE_WORD_READ,  32'hff800000 + 32'(2076 * 16), 16'h0000, 1'b0, 16'h5000 + 16'(3072 + 28));   // word 2076 = B word 28 -> B copied first
    if (memory[0] === 16'h5000 + 16'(3072) && !srt_pending) begin
      $display("note: A had already completed when B's read returned");
    end
    wait_copies();
    for (int k = 0; k < 1024; k++) begin
      if (memory[k] !== 16'h5000 + 16'(3072 + k) || memory[2048 + k] !== 16'h5000 + 16'(3072 + k)) begin
        $error("SRT on-demand: block A/B mismatch at %0d", k); $fatal(1);
      end
    end

    // ---- stress: the game's clear = 47 queued 4-row copies back to back,
    // then an ordinary host write/read to a row outside the clear (row 976)
    // must complete while the copies drain; finally all copies land.
    wait_copies();
    for (int k = 0; k < 4096; k++) memory[k] = 16'h0000;
    for (int k = 0; k < 1024; k++) memory[k] = 16'h7000 + 16'(k);              // source block 0
    access(LOCAL_CYCLE_PIXEL_MTR,  32'h02000000, 16'h0000, 1'b0, 16'h0000);
    for (int n = 1; n <= 3; n++)                                                // 3 fit in the 4K model (blocks 1..3)
      access(LOCAL_CYCLE_PIXEL_RTM, 32'h02000000 + 32'(n) * 32'h1000, 16'h0000, 1'b0, 16'd0);
    access(LOCAL_CYCLE_WORD_WRITE, 32'hff800000 + 32'(3900 * 16), 16'h0976, 1'b0, 16'd0);   // word 3900: outside blocks 0..3? no: block 3 = words 3072..4095 -> use the source-free row? use word 4000? still in block 3.
    access(LOCAL_CYCLE_WORD_READ,  32'hff800000 + 32'(3900 * 16), 16'h0000, 1'b0, 16'h0976);   // inside block 3: the write waited for block 3's copy and landed after it (MAME order: RTM then write)
    wait_copies();

    // ---- template snapshot semantics: after RTM A (queued) a write into the
    // SOURCE rows must not wait and A must still copy the OLD template; an RTM B
    // without a new MTR re-snapshots (waits for A) and copies the NEW template.
    wait_copies();
    for (int k = 0; k < 4096; k++) memory[k] = 16'h0000;
    for (int k = 1024; k < 2048; k++) memory[k] = 16'h6000 + 16'(k);        // source block 1 (words 1024..2047)
    access(LOCAL_CYCLE_PIXEL_MTR,  32'h02001000, 16'h0000, 1'b0, 16'h0000);
    access(LOCAL_CYCLE_PIXEL_RTM,  32'h02002000, 16'h0000, 1'b0, 16'd0);   // A: -> words 2048..3071 (snapshot + copy)
    srt_wait = 0;
    access(LOCAL_CYCLE_WORD_WRITE, 32'hff800000 + 32'(1030 * 16), 16'h0bad, 1'b0, 16'd0);   // write into the source rows: must not wait for A
    access(LOCAL_CYCLE_WORD_READ,  32'hff800000 + 32'(1030 * 16), 16'h0000, 1'b0, 16'h0bad);
    wait_copies();
    if (memory[2048 + 6] !== 16'h6000 + 16'(1030)) begin
      $error("SRT snapshot: queued copy used the modified source (%h)", memory[2048 + 6]); $fatal(1);
    end
    access(LOCAL_CYCLE_PIXEL_RTM,  32'h02003000, 16'h0000, 1'b0, 16'd0);   // B: -> words 3072..4095, must re-snapshot (source changed)
    wait_copies();
    if (memory[3072 + 6] !== 16'h0bad || memory[3072] !== 16'h6000 + 16'(1024)) begin
      $error("SRT snapshot: copy after a source write did not see the new template (%h %h)", memory[3072 + 6], memory[3072]); $fatal(1);
    end
    $display("SRT template snapshot OK");


    $display("SRT copy-on-demand OK");

    $display("SRT block copies OK (normal 256 words, skip, re-copy after write, 2-bpp 1024 words)");

    $display("PASS harddrivin GSP peripherals, VRAM, and Compact 2-bpp expander");
    $finish;
  end
endmodule

`default_nettype wire

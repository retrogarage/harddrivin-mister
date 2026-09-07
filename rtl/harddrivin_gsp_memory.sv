`default_nettype none

// Compact MultiSync GSP local-memory adapter. Small decoded control/palette
// banks remain on chip. The 256Kx16 VRAM aperture and its 2-bpp alias cross to
// the existing 100 MHz burst-SDRAM path as completed 16-bit word cycles.
module harddrivin_gsp_memory
  import tms34010_pkg::*;
#(
  // Instruction cache for the VRAM-resident GSP program (the real 34010
  // fetches hot code from its on-chip cache; without one every instruction
  // word costs a full VRAM round trip and the GSP runs 1.5-2.3x slower
  // than the real chip, which breaks the 68010's bounded frame waits).
  // Per-word valid bits on 4-word tag lines, filled by instruction-fetch
  // misses, invalidated by any VRAM write to the line.  Tags and data live
  // in block RAM read every cycle at the request index, so a hit is known
  // one clock after the request and answers ICACHE_HIT_LATENCY (>= 2)
  // clocks later: 3 clocks in all, which paces a DSJS loop like the real
  // chip's 16 CLKIN.
  parameter integer ICACHE_LINES = 64,
  parameter integer ICACHE_HIT_LATENCY = 2,   // + the tag-RAM read cycle = 3 clocks request to acknowledge
  // Data read cache (same structure, for non-instruction linear word reads)
  // and posted word writes (acknowledged at once, drained through the CDC in
  // order; a read miss waits only for a queued write to the same word).
  // 2-bpp expander writes merge into a one-line write-combining buffer (the
  // four 4-aligned words of one 2-bpp word) that is flushed into the queue
  // when another line or access needs it, or after 255 idle clocks (5 us).
  parameter integer DCACHE_LINES = 64,
  parameter integer WQ_DEPTH = 4,
  // 1: a read miss fills the whole 4-word line from vram_rdata64_i (the
  // burst arbiter's burst); 0: only the requested word (legacy arbiter)
  parameter bit     LINE_FILL = 1'b1,
  parameter int VRAM_AW = 18,
  parameter int SRT_BIG_BITS = 10,
  parameter bit COCKPIT = 0
)
(
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

  input  logic              mem_clk_i,
  input  logic              mem_rst_i,
  output logic              vram_req_o,
  output logic              vram_we_o,
  output logic [7:0]        vram_be_o,        // byte enables of the four words of a line write
  output logic [VRAM_AW-1:0]       vram_addr_o,
  output logic [63:0]       vram_wdata_o,     // a write is the whole 4-word line at vram_addr_o[VRAM_AW-1:2]
  input  logic [15:0]       vram_rdata_i,
  input  logic [63:0]       vram_rdata64_i,   // the 4-word burst holding vram_rdata_i (cache line fills)
  input  logic              vram_ack_i,

  input  logic [9:0]        palette_scan_addr_i,
  output logic [15:0]       palette_scan_lo_o,
  output logic [15:0]       palette_scan_hi_o,
  output logic [3:0]        fine_scroll_o,
  output logic [1:0]        palette_bank_o,
  output logic              shiftreg_enable_o,

  output logic [31:0]       peripheral_write_count_o,
  output logic [15:0]       control_write_count_o,
  output logic [15:0]       palette_lo_write_count_o,
  output logic [15:0]       palette_hi_write_count_o,
  output logic              palette_write_nonzero_seen_o,
  output logic              palette_scan_nonzero_seen_o,
  output logic [9:0]        last_palette_addr_o,
  output logic [15:0]       last_palette_data_o,
  output logic [15:0]       palette_bank2_ff_lo_o,
  output logic [15:0]       palette_bank2_ff_hi_o,
  output logic [15:0]       palette_bank3_ff_lo_o,
  output logic [15:0]       palette_bank3_ff_hi_o,
  output logic [63:0]       control_write_trace_o,
  output logic [7:0]        control_hi_write_count_o,
  output logic [31:0]       control_read_trace_o,
  output logic [7:0]        control_hi_read_count_o,
  output logic [31:0]       vram_write_count_o,
  output logic              vram_write_nonzero_seen_o,
  output logic [15:0]       expander_write_count_o,
  output logic              instruction_fetch_seen_o,
  output logic              peripheral_roundtrip_seen_o,
  output logic              vram_roundtrip_seen_o,
  output logic [15:0]       srt_copy_count_o,       // VRAM shift-register block copies queued
  output logic              srt_pending_o           // copies queued or in progress
);
  // ICACHE_HIT_LATENCY is specified as at least two clocks.  Clamp the
  // derived terminal count so parameter checking/lint never has to evaluate
  // an unsigned negative expression for an accidental smaller override.
  localparam integer ICACHE_HIT_TERMINAL =
    (ICACHE_HIT_LATENCY >= 2) ? (ICACHE_HIT_LATENCY - 2) : 0;
  logic control_lo_select;
  logic control_hi_select;
  logic palette_lo_select;
  logic palette_hi_select;
  logic peripheral_select;
  logic vram_linear_select;
  logic vram_2bpp_select;
  logic srt_read, srt_write;
  logic [VRAM_AW-1:0] srt_block_addr;
  logic [SRT_BIG_BITS:0] srt_block_len;
  // shift-register copy engine (clk_i side)
  logic [VRAM_AW-1:0] srt_src_q, srt_dst_q, srt_last_src_q, srt_last_dst_q, srt_eng_src_q;
  logic [SRT_BIG_BITS:0] srt_len_q;
  logic srt_src_valid_q, srt_last_valid_q, srt_last_big_q;
  logic srt_start_tgl_q, srt_done_seen_q, srt_busy_q;
  logic [1:0] srt_done_sync_q;
  // Copy queue: an RTM is acknowledged at once and the block copy runs in
  // the background (the real VRAM does a row transfer in one memory cycle;
  // our SDRAM copy takes ~200 us, which must not stall the GSP and its
  // display interrupts).  Ordinary VRAM accesses that fall inside the
  // range covered by queued copies wait until the queue has drained, so
  // the memory image is always the one MAME's immediate copies produce.
  localparam int SRT_Q_AW = 6;   // 64 entries: a whole 47-block clear queues without holding an RTM (src/dst live in block RAM)
  logic [2*VRAM_AW-1:0] srt_q_ram [0:(1<<SRT_Q_AW)-1];   // {src, dst} of each entry (block RAM, reset-free process)
  logic [2*VRAM_AW-1:0] srt_q_ram_rd_q;                 // entry read for the hand-off (address = srt_pick)
  logic        srt_q_we_q, srt_q_we_d1_q;          // registered RAM write (two settle cycles before a hand-off)
  logic [2*VRAM_AW-1:0] srt_q_wdata_q;
  logic [SRT_Q_AW-1:0] srt_q_waddr_q, srt_pick;
  logic        srt_handoff_q;                  // second hand-off cycle: the entry's src/dst arrive from the RAM
  logic        srt_q_big [0:(1<<SRT_Q_AW)-1];
  logic [SRT_Q_AW:0] srt_q_wr_q, srt_q_rd_q;
  logic srt_q_empty, srt_q_full;
  // Per-entry interlock: an ordinary access that hits a queued block runs
  // that block's copy first (earliest matching entry, out of order) and
  // proceeds; the wait is one block copy, never the whole queue.
  logic [(1<<SRT_Q_AW)-1:0] srt_q_valid_q;      // entry pending (queued, not yet copied)
  // block-granular copies of the entry addresses for the cheap interlock
  // compares: destination row (a[VRAM_AW-1:8]) or 4-row block (a[VRAM_AW-1:SRT_BIG_BITS]); the source
  // span of a 4-row copy from a 256-word-aligned source covers two 4-row
  // blocks (src_blk, src_blk + 1)
  logic [VRAM_AW-9:0] srt_q_dst_row [0:(1<<SRT_Q_AW)-1];
  logic [(1<<SRT_Q_AW)-1:0] srt_match;          // pending entries covering the current access
  logic [SRT_Q_AW-1:0] srt_eng_idx_q;           // entry the engine is working on
  logic srt_hit;                                // the current access hits a pending entry
  logic srt_next_valid;                         // a pending entry exists
  logic [SRT_Q_AW-1:0] srt_next_idx_q;          // earliest pending entry (registered pick)
  logic srt_next_valid_q;
  logic [(1<<SRT_Q_AW)-1:0] srt_wait_mask_q;    // entries a held access waits for (registered)
  logic [SRT_Q_AW-1:0] srt_wait_idx_q;          // earliest of them (registered pick)
  logic srt_wait_valid_q;
  logic [VRAM_AW-9:0] srt_eng_dst_row_q;
  logic srt_eng_big_q;
  // Template snapshot: the copies read a 1024-word on-chip image of the
  // source block taken when the first copy after an MTR (or after a write
  // into the source rows) is queued.  Copies therefore never read the SDRAM
  // source and a later write into the template does not have to wait — the
  // real VRAM's row transfer moved the data at RTM time.
  // Two template buffers: a new snapshot is taken into the other buffer as an
  // ordered queue job while copies from the old one still drain, so an RTM
  // never waits for the queue.
  logic [VRAM_AW-1:0] srt_snap_src_q [0:1];   // source block held in each buffer
  logic [1:0] srt_snap_valid_q;        // buffer holds its source and no write hit it since
  logic [1:0] srt_snap_ready_q;        // the snapshot job for the buffer has completed
  logic srt_buf_cur_q;                 // buffer of the latest snapshot
  logic [(1<<SRT_Q_AW)-1:0] srt_q_kind_snap;   // entry is a snapshot job
  logic [(1<<SRT_Q_AW)-1:0] srt_q_buf;         // entry's template buffer
  logic [1:0] srt_buf_busy;            // a queued or running entry references the buffer
  logic srt_snap_job_q;                // hand-off: the engine's current job is a snapshot
  logic srt_eng_buf_q;                 // buffer of the engine's current job
  logic srt_job_snap_m_q, srt_job_buf_m_q;   // (mem_clk) current job kind / buffer
  logic srt_pick_ok;                   // the picked entry may run (snapshot, or its buffer is ready)
  // first set bit of a mask rotated so that the queue head is bit 0
  // lowest set bit of a mask (the order among several matching entries only
  // affects latency; the head pointer itself retires finished entries, so the
  // next entry to run is simply the head once it is valid)
  function automatic logic [SRT_Q_AW-1:0] srt_first_set(input logic [(1<<SRT_Q_AW)-1:0] mask);
    logic [SRT_Q_AW-1:0] pos;
    pos = '0;
    for (int k = (1<<SRT_Q_AW)-1; k >= 0; k--) if (mask[k]) pos = SRT_Q_AW'(k);
    return pos;
  endfunction
  // shift-register copy engine (mem_clk_i side)
  typedef enum logic [2:0] {SRT_IDLE, SRT_RD, SRT_RD_STORE, SRT_RD_GAP, SRT_LOAD, SRT_WR, SRT_WR_GAP} srt_state_t;
  srt_state_t srt_state_q;
  logic [1:0] srt_start_sync_q;
  logic srt_start_seen_q, srt_done_tgl_q;
  logic [VRAM_AW-1:0] srt_m_src_q, srt_m_dst_q, srt_m_addr_q;   // running source/destination pointers, current address
  logic [15:0] srt_tpl [0:(2<<SRT_BIG_BITS)-1];                        // template snapshots {buf, word} (mem_clk domain, block RAM)
  logic        srt_tpl_we;
  logic [SRT_BIG_BITS:0] srt_tpl_addr;
  logic [15:0] srt_tpl_rd_q;
  logic [SRT_BIG_BITS:0] srt_m_len_q, srt_m_idx_q;
  logic [63:0] srt_m_data64_q, srt_m_next64_q;
  logic        srt_next_ready_q;   // the next line was read from the template RAM during the write
  logic [2:0]  srt_ph_q;        // template-RAM phase within a 4-word step
  logic srt_m_req_q, srt_m_we_q, srt_active, srt_release_q;   // srt_release_q: one request-low cycle after an engine transaction (arbiter arming)
  logic [1:0] srt_snap_sync_q;
  logic cdc_req, cdc_we;
  logic [7:0] cdc_be;
  logic [VRAM_AW-1:0] cdc_addr;
  logic [63:0] cdc_wdata;
  logic vram_select;
  logic word_read;
  logic word_write;
  logic local_request_seen_q;
  logic vram_request_seen_q;
  logic local_ack_q;
  logic [15:0] local_rdata_q;
  logic peripheral_read_response_q;
  logic peripheral_roundtrip_check_q;
  logic [15:0] peripheral_expected_q;
  logic peripheral_access;
  logic control_access;
  logic palette_lo_access;
  logic palette_hi_access;
  logic [4:0] control_addr;
  logic [9:0] palette_addr;
  logic [15:0] control_rdata;
  logic [15:0] palette_lo_rdata;
  logic [15:0] palette_hi_rdata;
  logic [1:0] peripheral_read_source_q;
  logic [15:0] peripheral_selected_rdata;
  logic vram_cdc_req_q;
  logic vram_cdc_we_q;
  logic [7:0] vram_cdc_be_q;
  logic [VRAM_AW-1:0] vram_cdc_addr_q;
  logic [63:0] vram_cdc_wdata_q;
  logic vram_src_ack;
  logic [15:0] vram_src_rdata;
  logic [63:0] vram_src_rdata64;
  logic [3:0]  fill_word_ok;   // burst words with no newer posted/buffered write
  logic [VRAM_AW-1:0] vram_logical_addr;
  logic vram_cycle_ack_q;
  logic [15:0] vram_cycle_rdata_q;
  logic scroll_latch_q;

  typedef enum logic [1:0] {
    VRAM_IDLE,
    VRAM_WAIT_LINEAR,
    VRAM_DRAIN
  } vram_state_t;
  vram_state_t vram_state_q;
  logic [15:0] color_pair_q;

  logic peripheral_last_valid_q;
  logic [31:4] peripheral_last_addr_q;
  logic [15:0] peripheral_last_data_q;
  logic vram_last_valid_q;
  logic [VRAM_AW-1:0] vram_last_addr_q;
  logic [15:0] vram_last_data_q;
  logic vram_pending_write_q;
  logic vram_pending_iaq_q;
  logic vram_pending_linear_q;
  logic [VRAM_AW-1:0] vram_pending_addr_q;
  logic [15:0] vram_pending_wdata_q;

  // ---- instruction cache ----
  localparam int ICACHE_IDX_W = $clog2(ICACHE_LINES);
  localparam int ICACHE_TAG_W = VRAM_AW - ICACHE_IDX_W - 2;
  logic [63:0]             icache_ram [0:ICACHE_LINES-1];   // line data (block RAM, reset-free process)
  logic [63:0]             icache_rd64_q;
  logic [1:0]              icache_rd_off_q;
  logic                    icache_fill_we;
  logic [ICACHE_TAG_W-1:0] icache_tag_ram [0:ICACHE_LINES-1];   // tags (block RAM, reset-free process)
  logic [ICACHE_TAG_W-1:0] icache_tag_rd_q;
  logic [ICACHE_IDX_W-1:0] icache_tag_rd_idx_q;
  logic                    icache_fill_we_d1_q, icache_tag_ready;
  logic [3:0]              icache_valid [0:ICACHE_LINES-1];
  logic [ICACHE_IDX_W-1:0] icache_req_idx, icache_fill_idx, icache_inv_idx;
  logic [ICACHE_TAG_W-1:0] icache_req_tag, icache_fill_tag;
  logic [1:0]              icache_req_off, icache_fill_off;
  logic                    icache_lookup, icache_hit, icache_hit_busy_q;
  logic [7:0]              icache_hit_cnt_q;
  logic [ICACHE_IDX_W+1:0] icache_hit_addr_q;
  logic [15:0]             icache_rd_q;
  logic [31:0]             icache_hits_q, icache_misses_q;
  // A posted write invalidates both caches when accepted, before it drains.
  // During that drain the independent BRAM ports can still answer valid hits.
  // Misses retain the existing CDC ownership and pending-write interlocks.
  wire cache_lookup_available = (vram_state_q == VRAM_IDLE) ||
                                (COCKPIT && vram_state_q == VRAM_DRAIN);
  assign icache_req_idx = vram_logical_addr[ICACHE_IDX_W+1:2];
  assign icache_req_tag = vram_logical_addr[VRAM_AW-1:ICACHE_IDX_W+2];
  assign icache_req_off = vram_logical_addr[1:0];
  assign icache_fill_idx = vram_pending_addr_q[ICACHE_IDX_W+1:2];
  assign icache_fill_tag = vram_pending_addr_q[VRAM_AW-1:ICACHE_IDX_W+2];
  assign icache_fill_off = vram_pending_addr_q[1:0];
  assign icache_inv_idx  = vram_logical_addr[ICACHE_IDX_W+1:2];
  assign icache_lookup = cycle_req_i && vram_linear_select && word_read && cycle_iaq_i &&
                         !vram_request_seen_q && cache_lookup_available && !icache_hit_busy_q;
  // the tag RAM is read every cycle at the request's index; a hit or miss
  // is decided once that read matches the index and no fill landed last cycle
  assign icache_tag_ready = (icache_tag_rd_idx_q == icache_req_idx) && !icache_fill_we_d1_q;
  assign icache_hit = icache_lookup && icache_tag_ready && (icache_tag_rd_q == icache_req_tag) &&
                      icache_valid[icache_req_idx][icache_req_off];
  // BRAM read port for hits (registered).
  // a fill writes the whole line (words without fill_word_ok stay invalid)
  assign icache_fill_we = (vram_state_q == VRAM_WAIT_LINEAR) && vram_src_ack && vram_pending_linear_q && vram_pending_iaq_q;
  always_ff @(posedge clk_i) begin
    if (icache_fill_we) icache_ram[icache_fill_idx] <= vram_src_rdata64;
    icache_rd64_q <= icache_ram[icache_req_idx];
    icache_rd_off_q <= icache_req_off;
    if (icache_fill_we) icache_tag_ram[icache_fill_idx] <= icache_fill_tag;
    icache_tag_rd_q     <= icache_tag_ram[icache_req_idx];
    icache_tag_rd_idx_q <= icache_req_idx;
    icache_fill_we_d1_q <= icache_fill_we;
  end
  assign icache_rd_q = icache_rd64_q[16 * icache_rd_off_q +: 16];

  // ---- data read cache ----
  localparam int DCACHE_IDX_W = $clog2(DCACHE_LINES);
  localparam int DCACHE_TAG_W = VRAM_AW - DCACHE_IDX_W - 2;
  logic [63:0]             dcache_ram [0:DCACHE_LINES-1];   // line data (block RAM, reset-free process)
  logic [63:0]             dcache_rd64_q;
  logic [1:0]              dcache_rd_off_q;
  logic                    dcache_fill_we;
  logic [DCACHE_TAG_W-1:0] dcache_tag_ram [0:DCACHE_LINES-1];   // tags (block RAM, reset-free process)
  logic [DCACHE_TAG_W-1:0] dcache_tag_rd_q;
  logic [DCACHE_IDX_W-1:0] dcache_tag_rd_idx_q;
  logic                    dcache_fill_we_d1_q, dcache_tag_ready;
  logic [3:0]              dcache_valid [0:DCACHE_LINES-1];
  logic [DCACHE_IDX_W-1:0] dcache_req_idx, dcache_fill_idx, dcache_inv_idx;
  logic [DCACHE_TAG_W-1:0] dcache_req_tag, dcache_fill_tag;
  logic [1:0]              dcache_req_off, dcache_fill_off;
  logic                    dcache_lookup, dcache_hit;
  logic [DCACHE_IDX_W+1:0] dcache_hit_addr_q;
  logic [15:0]             dcache_rd_q;
  logic                    hit_is_dcache_q;
  logic [31:0]             dcache_hits_q, dcache_misses_q;
  assign dcache_req_idx = vram_logical_addr[DCACHE_IDX_W+1:2];
  assign dcache_req_tag = vram_logical_addr[VRAM_AW-1:DCACHE_IDX_W+2];
  assign dcache_req_off = vram_logical_addr[1:0];
  assign dcache_fill_idx = vram_pending_addr_q[DCACHE_IDX_W+1:2];
  assign dcache_fill_tag = vram_pending_addr_q[VRAM_AW-1:DCACHE_IDX_W+2];
  assign dcache_fill_off = vram_pending_addr_q[1:0];
  assign dcache_inv_idx  = vram_logical_addr[DCACHE_IDX_W+1:2];
  assign dcache_lookup = cycle_req_i && vram_linear_select && word_read && !cycle_iaq_i &&
                         !vram_request_seen_q && cache_lookup_available && !icache_hit_busy_q;
  assign dcache_tag_ready = (dcache_tag_rd_idx_q == dcache_req_idx) && !dcache_fill_we_d1_q;
  assign dcache_hit = dcache_lookup && dcache_tag_ready && (dcache_tag_rd_q == dcache_req_tag) &&
                      dcache_valid[dcache_req_idx][dcache_req_off];
  assign dcache_fill_we = (vram_state_q == VRAM_WAIT_LINEAR) && vram_src_ack && vram_pending_linear_q && !vram_pending_iaq_q;
  always_ff @(posedge clk_i) begin
    if (dcache_fill_we) dcache_ram[dcache_fill_idx] <= vram_src_rdata64;
    dcache_rd64_q <= dcache_ram[dcache_req_idx];
    dcache_rd_off_q <= dcache_req_off;
    if (dcache_fill_we) dcache_tag_ram[dcache_fill_idx] <= dcache_fill_tag;
    dcache_tag_rd_q     <= dcache_tag_ram[dcache_req_idx];
    dcache_tag_rd_idx_q <= dcache_req_idx;
    dcache_fill_we_d1_q <= dcache_fill_we;
  end
  assign dcache_rd_q = dcache_rd64_q[16 * dcache_rd_off_q +: 16];

  // ---- posted write queue (linear word writes) ----
  localparam int WQ_AW = $clog2(WQ_DEPTH);
  logic [VRAM_AW-3:0] wq_line [0:WQ_DEPTH-1];   // 4-word line address (word address >> 2)
  logic [63:0] wq_data [0:WQ_DEPTH-1];
  logic [WQ_AW:0] wq_wr_q, wq_rd_q;          // one extra bit for full/empty
  logic wq_empty, wq_full;
  logic wq_drain_busy_q;
  logic [7:0]  wq_be [0:WQ_DEPTH-1];     // per-word byte enables of the line
  logic        wq_merge_ok;               // the newest entry may take another word of its line
  logic [WQ_AW-1:0] wq_newest;
  logic [WQ_DEPTH-1:0] wq_valid_q;
  logic wq_match;
  // ---- 2-bpp expander write-combining line buffer ----
  logic        lb_valid_q, lb_flush_needed, lb_same_line, lb_pending_other, lb_any_be;
  logic [VRAM_AW-1:0] lb_addr_q, lb_new_addr;
  logic [15:0] lb_data_q [0:3];
  logic [1:0]  lb_be_q [0:3];
  logic [1:0]  lb_new_be [0:3];
  logic [7:0]  lb_age_q;
  logic        vram_accept;
  assign wq_empty = (wq_wr_q == wq_rd_q);
  assign wq_full  = (wq_wr_q[WQ_AW-1:0] == wq_rd_q[WQ_AW-1:0]) && (wq_wr_q[WQ_AW] != wq_rd_q[WQ_AW]);

  assign control_lo_select = cycle_addr_i[31:8] == 24'hf40000;
  assign control_hi_select = cycle_addr_i[31:8] == 24'hf48000;
  assign palette_lo_select = cycle_addr_i[31:12] == 20'hf5000;
  assign palette_hi_select = cycle_addr_i[31:12] == 20'hf5800;
  assign peripheral_select = control_lo_select || control_hi_select ||
                             palette_lo_select || palette_hi_select;
  assign vram_linear_select = cycle_addr_i[31:23] == 9'h1ff;
  assign vram_2bpp_select = cycle_addr_i[31:20] == 12'h020;
  assign vram_select = vram_linear_select || vram_2bpp_select;
  assign vram_logical_addr = vram_linear_select
                           ? cycle_addr_i[VRAM_AW+3:4]
                           : {cycle_addr_i[19:4], 2'b00};
  // DPYCTL.SRT pixel accesses are VRAM shift-register transfers (PIXEL_MTR =
  // memory -> shift register, PIXEL_RTM = shift register -> memory).  The
  // Compact board (MAME harddriv_v.cpp hdgsp_write_to_shiftreg /
  // hdgsp_read_from_shiftreg) uses them as whole-block copies: an MTR
  // latches the source block, an RTM copies the block to the destination
  // when the SHIFTREG latch (f4800000/f4800080) is set.  Blocks: 2-bpp window
  // 0x020xxxxx -> word ((a - 0x02000000) >> 2) & ~1023 = {a[19:12], 10'b0},
  // 2048 bytes = 1024 words (4 rows); normal VRAM 0xff8xxxxx -> word
  // ((a - 0xff800000) >> 4) & ~255 in 16-bit words, 512 bytes (256 words at
  // {a[21:12], 8'b0} = one 512-byte row).
  // SRT cycles that do not target VRAM fall back to plain word accesses.
  assign srt_read  = (cycle_kind_i == LOCAL_CYCLE_PIXEL_MTR) && vram_select;
  assign srt_write = (cycle_kind_i == LOCAL_CYCLE_PIXEL_RTM) && vram_select;
  assign word_read = (cycle_kind_i == LOCAL_CYCLE_WORD_READ) ||
                     ((cycle_kind_i == LOCAL_CYCLE_PIXEL_MTR) && !vram_select);
  assign word_write = (cycle_kind_i == LOCAL_CYCLE_WORD_WRITE) ||
                      ((cycle_kind_i == LOCAL_CYCLE_PIXEL_RTM) && !vram_select);
  // MAME's m_gsp_vram is a 16-bit array (vram_mask = words - 1): normal VRAM
  // block = word ((a - ff800000) >> 4) & ~255 = {a[21:12], 8'b0}, 256 words.
  // 2-bpp window: word ((a - 0x02000000) >> 2) & ~1023 = {a[19:12], 10'b0}
  // (1024-word = 4-row aligned blocks), 1024 words copied.
  assign srt_block_addr = vram_2bpp_select ? VRAM_AW'((cycle_addr_i[19:0] >> (COCKPIT ? 1 : 2)) & ~((1<<SRT_BIG_BITS)-1))
                                           : {cycle_addr_i[VRAM_AW+3:12], 8'b0};
  assign srt_block_len  = vram_2bpp_select ? (SRT_BIG_BITS+1)'(1<<SRT_BIG_BITS) : 11'd256;
  assign srt_q_empty = (srt_q_wr_q == srt_q_rd_q);
  assign srt_q_full  = srt_q_valid_q[srt_q_wr_q[SRT_Q_AW-1:0]] ||
                       ((srt_q_wr_q[SRT_Q_AW-1:0] == srt_q_rd_q[SRT_Q_AW-1:0]) && (srt_q_wr_q[SRT_Q_AW] != srt_q_rd_q[SRT_Q_AW]));
  assign srt_pending_o = (srt_q_valid_q != '0) || srt_busy_q;
  // Match the current access against every pending entry with small block
  // equalities (parallel, shallow); the entry picks are registered.
  always_comb begin
    for (int k = 0; k < (1 << SRT_Q_AW); k++) begin
      srt_match[k] = srt_q_valid_q[k] &&
        (srt_q_big[k] ? (vram_logical_addr[VRAM_AW-1:SRT_BIG_BITS] == srt_q_dst_row[k][VRAM_AW-9:SRT_BIG_BITS-8]) : (vram_logical_addr[VRAM_AW-1:8] == srt_q_dst_row[k]));
    end
  end
  assign srt_hit = |srt_match;
  assign srt_next_valid = |srt_q_valid_q;
  // the engine's current copy also blocks accesses into its destination block;
  // while a snapshot is being taken, writes into the snapshot rows wait
  logic srt_eng_hit;
  assign srt_eng_hit = srt_busy_q &&
      (srt_snap_job_q ? (word_write && (vram_logical_addr[VRAM_AW-1:SRT_BIG_BITS] == srt_snap_src_q[srt_eng_buf_q][VRAM_AW-1:SRT_BIG_BITS]))
                      : (srt_eng_big_q ? (vram_logical_addr[VRAM_AW-1:SRT_BIG_BITS] == srt_eng_dst_row_q[VRAM_AW-9:SRT_BIG_BITS-8])
                                       : (vram_logical_addr[VRAM_AW-1:8] == srt_eng_dst_row_q)));
  assign srt_pick = (srt_wait_valid_q && srt_q_valid_q[srt_wait_idx_q] && srt_snap_ready_q[srt_q_buf[srt_wait_idx_q]])
                    ? srt_wait_idx_q : srt_next_idx_q;
  always_ff @(posedge clk_i) begin
    if (srt_q_we_q) srt_q_ram[srt_q_waddr_q] <= srt_q_wdata_q;
    srt_q_ram_rd_q <= srt_q_ram[srt_pick];
  end
  // a queued snapshot job that has not run yet also protects its source rows
  logic srt_snapq_hit;
  always_comb begin
    srt_snapq_hit = 1'b0;
    for (int b = 0; b < 2; b++)
      if (srt_snap_valid_q[b] && !srt_snap_ready_q[b] && word_write && (vram_logical_addr[VRAM_AW-1:SRT_BIG_BITS] == srt_snap_src_q[b][VRAM_AW-1:SRT_BIG_BITS]))
        srt_snapq_hit = 1'b1;
  end
  always_comb begin
    srt_buf_busy = 2'b00;
    for (int k = 0; k < (1 << SRT_Q_AW); k++)
      if (srt_q_valid_q[k]) srt_buf_busy[srt_q_buf[k]] = 1'b1;
    if (srt_busy_q) srt_buf_busy[srt_eng_buf_q] = 1'b1;
  end
  assign peripheral_access = cycle_req_i && !local_request_seen_q &&
                             peripheral_select && (word_read || word_write);
  assign control_access = peripheral_access &&
                          (control_lo_select || control_hi_select);
  assign palette_lo_access = peripheral_access && palette_lo_select;
  assign palette_hi_access = peripheral_access && palette_hi_select;
  assign control_addr = {control_hi_select, cycle_addr_i[7:4]};
  assign palette_addr = {palette_bank_o, cycle_addr_i[11:4]};

  // The Compact pixel expander consumes two bits per output pixel but only
  // uses the even bit of each two-bit field as a byte write strobe.  Four
  // adjacent VRAM words are selected; control-low word zero supplies the low
  // and high byte colors.  The physical VRAMs implement this with write-mask
  // pins, which map directly to SDRAM byte enables here.  Consecutive
  // expander writes to the same line (a per-pixel PIXBLT/FILL rewrites one
  // 2-bpp word eight times) merge in the line buffer before one flush.
  assign lb_new_addr  = {cycle_addr_i[19:4], 2'b00};
  assign lb_new_be[0] = {cycle_wdata_i[2],  cycle_wdata_i[0]};
  assign lb_new_be[1] = {cycle_wdata_i[6],  cycle_wdata_i[4]};
  assign lb_new_be[2] = {cycle_wdata_i[10], cycle_wdata_i[8]};
  assign lb_new_be[3] = {cycle_wdata_i[14], cycle_wdata_i[12]};
  assign lb_same_line = lb_valid_q && (lb_addr_q == lb_new_addr);
  // a VRAM access that is not a same-line expander write, an expander read
  // or a cache hit is waiting on the buffered line
  assign lb_pending_other = cycle_req_i && vram_select && !vram_request_seen_q &&
                            !icache_hit && !dcache_hit &&
                            !(vram_2bpp_select && word_write && lb_same_line) &&
                            !(vram_2bpp_select && word_read);
  assign lb_flush_needed = lb_valid_q &&
                           (lb_pending_other || lb_age_q == 8'd255 || (srt_next_valid_q && !srt_busy_q));
  assign lb_any_be = (lb_be_q[0] | lb_be_q[1] | lb_be_q[2] | lb_be_q[3]) != 2'b00;
  // a read miss must not pass a queued write to the same word
  always_comb begin
    wq_match = 1'b0;
    for (int q = 0; q < WQ_DEPTH; q++)
      if (wq_valid_q[q] && wq_line[q] == vram_logical_addr[VRAM_AW-1:2] &&
          wq_be[q][2 * vram_logical_addr[1:0] +: 2] != 2'b00) wq_match = 1'b1;
  end
  // a 4-word line with one word replaced
  function automatic logic [63:0] line_merge(input logic [63:0] line, input logic [1:0] slot, input logic [15:0] word);
    line_merge = line;
    unique case (slot)
      2'd0: line_merge[15:0]  = word;
      2'd1: line_merge[31:16] = word;
      2'd2: line_merge[47:32] = word;
      default: line_merge[63:48] = word;
    endcase
  endfunction
  // posted word writes to one line merge into the newest entry while it is
  // not the head (the head may be copied into the CDC this cycle)
  assign wq_newest   = wq_wr_q[WQ_AW-1:0] - 1'b1;
  assign wq_merge_ok = ((wq_wr_q - wq_rd_q) >= (WQ_AW+1)'(2)) && wq_valid_q[wq_newest] &&
                       (wq_line[wq_newest] == vram_logical_addr[VRAM_AW-1:2]);
  // The draining head has already been captured by the CDC. New writes can
  // enter the queue/expander buffer without changing that transaction. The
  // existing capacity, merge and line-buffer interlocks still apply.
  wire posted_write_during_drain = COCKPIT && word_write &&
                                  (vram_state_q == VRAM_DRAIN);
  assign vram_accept = cycle_req_i && vram_select && (word_read || word_write) &&
          !vram_request_seen_q && (vram_state_q == VRAM_IDLE || posted_write_during_drain) && !icache_hit && !dcache_hit && !icache_hit_busy_q &&
          (!wq_drain_busy_q || posted_write_during_drain) && !srt_hit && !srt_eng_hit && !srt_snapq_hit &&
          !lb_flush_needed &&
          // a linear read decides hit/miss only once its tag read is in
          (!(word_read && vram_linear_select) || (cycle_iaq_i ? icache_tag_ready : dcache_tag_ready)) &&
          // a read waits only for a posted write to the same word
          (!(word_read && vram_linear_select) || !wq_match) &&
          // an expander write merges into an empty or same-line buffer
          (!(word_write && vram_2bpp_select) || !lb_valid_q || lb_same_line) &&
          // a posted write needs queue space
          (!(word_write && vram_linear_select) || !wq_full || wq_merge_ok);

  // Compact has four 256-entry palette banks.  CPU accesses use the bank
  // selected by the control-high address latches, while the serializer has a
  // simultaneous read port for the live pixel index.
  harddrivin_word_ram #(.ADDR_WIDTH(5)) u_control_ram (
    .clk_i(clk_i), .en_i(control_access), .we_i(word_write),
    .addr_i(control_addr), .wdata_i(cycle_wdata_i),
    .rdata_o(control_rdata)
  );

  harddrivin_dual_port_word_ram #(.ADDR_WIDTH(10)) u_palette_lo_ram (
    .clk_a_i(clk_i), .en_a_i(palette_lo_access), .we_a_i(word_write),
    .addr_a_i(palette_addr), .wdata_a_i(cycle_wdata_i),
    .rdata_a_o(palette_lo_rdata),
    .clk_b_i(clk_i), .en_b_i(1'b1), .addr_b_i(palette_scan_addr_i),
    .rdata_b_o(palette_scan_lo_o)
  );

  harddrivin_dual_port_word_ram #(.ADDR_WIDTH(10)) u_palette_hi_ram (
    .clk_a_i(clk_i), .en_a_i(palette_hi_access), .we_a_i(word_write),
    .addr_a_i(palette_addr), .wdata_a_i(cycle_wdata_i),
    .rdata_a_o(palette_hi_rdata),
    .clk_b_i(clk_i), .en_b_i(1'b1), .addr_b_i(palette_scan_addr_i),
    .rdata_b_o(palette_scan_hi_o)
  );

  harddrivin_tms_word_cdc #(.ADDR_W(VRAM_AW), .RDATA_W(64), .WDATA_W(64), .BE_W(8)) u_vram_cdc (
    .src_clk_i(clk_i), .src_rst_i(rst_i),
    .src_req_i(vram_cdc_req_q), .src_we_i(vram_cdc_we_q),
    .src_be_i(vram_cdc_be_q), .src_addr_i(vram_cdc_addr_q),
    .src_wdata_i(vram_cdc_wdata_q), .src_rdata_o(vram_src_rdata64),
    .src_ack_o(vram_src_ack),
    .dst_clk_i(mem_clk_i), .dst_rst_i(mem_rst_i),
    .dst_req_o(cdc_req), .dst_we_o(cdc_we), .dst_be_o(cdc_be),
    .dst_addr_o(cdc_addr), .dst_wdata_o(cdc_wdata),
    .dst_rdata_i(vram_rdata64_i), .dst_ack_i(vram_ack_i && !srt_active)
  );
  assign vram_src_rdata = vram_src_rdata64[16 * vram_pending_addr_q[1:0] +: 16];
  // A read miss fills the whole 4-word line, except words with a write still
  // in the posted queue or the expander line buffer (the burst predates it).
  always_comb begin
    for (int k = 0; k < 4; k++) begin
      fill_word_ok[k] = LINE_FILL;
      for (int q = 0; q < WQ_DEPTH; q++)
        if (wq_valid_q[q] && wq_line[q] == vram_pending_addr_q[VRAM_AW-1:2] && wq_be[q][2 * k +: 2] != 2'b00) fill_word_ok[k] = 1'b0;
      if (lb_valid_q && lb_addr_q == {vram_pending_addr_q[VRAM_AW-1:2], 2'b00} && lb_be_q[k] != 2'b00) fill_word_ok[k] = 1'b0;
    end
    fill_word_ok[vram_pending_addr_q[1:0]] = 1'b1;   // the requested word is never stale
  end

  // The copy engine owns the VRAM port while a block copy runs; the CDC bridge
  // is idle then (the GSP cycle that started the copy is still pending).
  assign srt_active   = (srt_state_q == SRT_RD) || (srt_state_q == SRT_WR);   // owns the port only with a request in flight
  assign vram_req_o   = srt_active ? srt_m_req_q : (srt_release_q ? 1'b0 : cdc_req);
  assign vram_we_o    = srt_active ? srt_m_we_q  : cdc_we;
  assign vram_be_o    = srt_active ? 8'hff       : cdc_be;
  assign vram_addr_o  = srt_active ? srt_m_addr_q : cdc_addr;   // registered: no adder in front of the arbiter
  assign vram_wdata_o = srt_active ? srt_m_data64_q : cdc_wdata;

  always_ff @(posedge mem_clk_i) begin
    if (mem_rst_i) begin
      srt_state_q      <= SRT_IDLE;
      srt_start_sync_q <= 2'b00;
      srt_start_seen_q <= 1'b0;
      srt_done_tgl_q   <= 1'b0;
      srt_m_req_q      <= 1'b0;
      srt_m_we_q       <= 1'b0;
      srt_m_src_q      <= 18'd0;
      srt_m_dst_q      <= 18'd0;
      srt_m_addr_q     <= 18'd0;
      srt_m_len_q      <= 11'd0;
      srt_m_idx_q      <= 11'd0;
      srt_m_data64_q   <= 64'd0;
      srt_m_next64_q   <= 64'd0;
      srt_next_ready_q <= 1'b0;
      srt_ph_q         <= 3'd0;
      srt_release_q    <= 1'b0;
      srt_job_snap_m_q <= 1'b0;
      srt_job_buf_m_q  <= 1'b0;
    end else begin
      srt_start_sync_q <= {srt_start_sync_q[0], srt_start_tgl_q};
      srt_release_q    <= srt_active && vram_ack_i;   // the cycle after an engine ack keeps the port request low
      unique case (srt_state_q)
        SRT_IDLE: if ((srt_start_sync_q[1] != srt_start_seen_q) && !cdc_req) begin
          // the parameters were written at least two clk_i cycles before the
          // toggle crossed, so they are stable here; a pending GSP request
          // (cdc_req) is served first
          srt_start_seen_q <= srt_start_sync_q[1];
          srt_job_snap_m_q <= srt_snap_job_q;
          srt_job_buf_m_q  <= srt_eng_buf_q;
          srt_m_src_q  <= srt_eng_src_q + 18'd4;   // one 4-word line per port transaction
          srt_ph_q     <= 3'd0;
          srt_next_ready_q <= 1'b0;
          srt_m_dst_q  <= srt_dst_q;
          srt_m_len_q  <= srt_len_q;
          srt_m_idx_q  <= 11'd0;
          srt_m_we_q   <= 1'b0;
          if (srt_snap_job_q) begin            // snapshot: read source words into the template buffer
            srt_m_addr_q <= srt_eng_src_q;
            srt_m_req_q  <= 1'b1;
            srt_state_q  <= SRT_RD;
          end else begin                       // copy: template buffer -> destination
            srt_state_q  <= SRT_LOAD;
          end
        end
        SRT_RD: if (vram_ack_i) begin          // a snapshot line (4 words) arrived
          srt_m_req_q    <= 1'b0;
          srt_m_data64_q <= vram_rdata64_i;
          srt_ph_q       <= 3'd0;
          srt_state_q    <= SRT_RD_STORE;
        end
        SRT_RD_STORE: begin                    // four template-RAM writes (srt_tpl process)
          srt_ph_q <= srt_ph_q + 3'd1;
          if (srt_ph_q == 3'd3) begin
            if (srt_m_idx_q == (SRT_BIG_BITS+1)'((1<<SRT_BIG_BITS)-4)) begin
              srt_done_tgl_q <= ~srt_done_tgl_q;
              srt_state_q    <= SRT_IDLE;
            end else begin
              srt_m_idx_q  <= srt_m_idx_q + 11'd4;
              srt_m_addr_q <= srt_m_src_q;
              srt_m_src_q  <= srt_m_src_q + 18'd4;
              srt_state_q  <= SRT_RD_GAP;
            end
          end
        end
        SRT_RD_GAP: if (!cdc_req) begin        // let a GSP access through first
          srt_m_req_q <= 1'b1;
          srt_state_q <= SRT_RD;
        end
        SRT_LOAD: begin                        // four registered template-RAM reads: words 0..3
          unique case (srt_ph_q)
            3'd1: srt_m_data64_q[15:0]  <= srt_tpl_rd_q;
            3'd2: srt_m_data64_q[31:16] <= srt_tpl_rd_q;
            3'd3: srt_m_data64_q[47:32] <= srt_tpl_rd_q;
            3'd4: srt_m_data64_q[63:48] <= srt_tpl_rd_q;
            default: ;
          endcase
          if (srt_ph_q == 3'd4) begin
            srt_ph_q     <= 3'd0;
            srt_m_addr_q <= srt_m_dst_q;
            srt_m_dst_q  <= srt_m_dst_q + 18'd4;
            srt_state_q  <= SRT_WR_GAP;
          end else srt_ph_q <= srt_ph_q + 3'd1;
        end
        SRT_WR_GAP: begin
          if (!cdc_req) begin
            srt_m_we_q  <= 1'b1;
            srt_m_req_q <= 1'b1;
            srt_state_q <= SRT_WR;
          end
        end
        SRT_WR: begin
          // while the line write is in flight, read the next line from the
          // template RAM (addressed at idx + 4 by srt_tpl_addr)
          if (!srt_next_ready_q) begin
            unique case (srt_ph_q)
              3'd1: srt_m_next64_q[15:0]  <= srt_tpl_rd_q;
              3'd2: srt_m_next64_q[31:16] <= srt_tpl_rd_q;
              3'd3: srt_m_next64_q[47:32] <= srt_tpl_rd_q;
              3'd4: begin srt_m_next64_q[63:48] <= srt_tpl_rd_q; srt_next_ready_q <= 1'b1; end
              default: ;
            endcase
            if (srt_ph_q != 3'd4) srt_ph_q <= srt_ph_q + 3'd1;
          end
          if (vram_ack_i) begin
            srt_m_req_q <= 1'b0;
            srt_m_we_q  <= 1'b0;
            srt_ph_q    <= 3'd0;
            srt_next_ready_q <= 1'b0;
            if (srt_m_idx_q == srt_m_len_q - 11'd4) begin
              srt_done_tgl_q <= ~srt_done_tgl_q;
              srt_state_q    <= SRT_IDLE;
            end else begin
              srt_m_idx_q <= srt_m_idx_q + 11'd4;
              if (srt_next_ready_q) begin       // prefetched: straight to the next write
                srt_m_data64_q <= srt_m_next64_q;
                srt_m_addr_q   <= srt_m_dst_q;
                srt_m_dst_q    <= srt_m_dst_q + 18'd4;
                srt_state_q    <= SRT_WR_GAP;
              end else srt_state_q <= SRT_LOAD;
            end
          end
        end
        default: srt_state_q <= SRT_IDLE;
      endcase
    end
  end

  // Template snapshot storage: one simple dual-port block RAM (its own
  // reset-free process so Quartus infers M10K instead of 32K registers).
  assign srt_tpl_we   = (srt_state_q == SRT_RD_STORE);
  assign srt_tpl_addr = {srt_job_buf_m_q, srt_m_idx_q[SRT_BIG_BITS-1:0] + SRT_BIG_BITS'(srt_ph_q[1:0]) + ((srt_state_q == SRT_WR) ? SRT_BIG_BITS'(4) : SRT_BIG_BITS'(0))};
  always_ff @(posedge mem_clk_i) begin
    if (srt_tpl_we) srt_tpl[srt_tpl_addr] <= srt_m_data64_q[16 * srt_ph_q[1:0] +: 16];
    srt_tpl_rd_q <= srt_tpl[srt_tpl_addr];
  end

  always_comb begin
    if (peripheral_read_source_q == 2'd0)
      peripheral_selected_rdata = control_rdata;
    else if (peripheral_read_source_q == 2'd1)
      peripheral_selected_rdata = palette_lo_rdata;
    else
      peripheral_selected_rdata = palette_hi_rdata;
  end

  assign cycle_ack_o = local_ack_q || vram_cycle_ack_q;
  assign cycle_rdata_o = vram_cycle_ack_q ? vram_cycle_rdata_q :
                           peripheral_read_response_q
                             ? peripheral_selected_rdata :
                           local_rdata_q;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      local_request_seen_q       <= 1'b0;
      vram_request_seen_q        <= 1'b0;
      local_ack_q                <= 1'b0;
      local_rdata_q              <= 16'hffff;
      vram_cdc_req_q             <= 1'b0;
      vram_cdc_we_q              <= 1'b0;
      vram_cdc_be_q              <= 8'hff;
      vram_cdc_addr_q            <= 18'd0;
      vram_cdc_wdata_q           <= 64'd0;
      vram_cycle_ack_q           <= 1'b0;
      vram_cycle_rdata_q         <= 16'hffff;
      vram_state_q               <= VRAM_IDLE;
      color_pair_q               <= 16'd0;
      wq_valid_q                 <= '0;
      lb_valid_q                 <= 1'b0;
      lb_age_q                   <= 8'd0;
      peripheral_read_response_q   <= 1'b0;
      peripheral_roundtrip_check_q <= 1'b0;
      peripheral_expected_q        <= 16'd0;
      peripheral_read_source_q     <= 2'd0;
      fine_scroll_o                <= 4'd0;
      scroll_latch_q               <= 1'b0;
      palette_bank_o               <= 2'd0;
      shiftreg_enable_o            <= 1'b0;
      peripheral_write_count_o   <= 32'd0;
      control_write_count_o      <= 16'd0;
      palette_lo_write_count_o   <= 16'd0;
      palette_hi_write_count_o   <= 16'd0;
      palette_write_nonzero_seen_o <= 1'b0;
      palette_scan_nonzero_seen_o <= 1'b0;
      last_palette_addr_o        <= 10'd0;
      last_palette_data_o        <= 16'd0;
      palette_bank2_ff_lo_o      <= 16'd0;
      palette_bank2_ff_hi_o      <= 16'd0;
      palette_bank3_ff_lo_o      <= 16'd0;
      palette_bank3_ff_hi_o      <= 16'd0;
      control_write_trace_o      <= 64'd0;
      control_hi_write_count_o   <= 8'd0;
      control_read_trace_o       <= 32'd0;
      control_hi_read_count_o    <= 8'd0;
      vram_write_count_o         <= 32'd0;
      vram_write_nonzero_seen_o  <= 1'b0;
      expander_write_count_o     <= 16'd0;
      instruction_fetch_seen_o   <= 1'b0;
      peripheral_roundtrip_seen_o <= 1'b0;
      vram_roundtrip_seen_o      <= 1'b0;
      peripheral_last_valid_q    <= 1'b0;
      peripheral_last_addr_q     <= 28'd0;
      peripheral_last_data_q     <= 16'd0;
      vram_last_valid_q          <= 1'b0;
      vram_last_addr_q           <= 18'd0;
      vram_last_data_q           <= 16'd0;
      vram_pending_write_q       <= 1'b0;
      vram_pending_iaq_q         <= 1'b0;
      vram_pending_linear_q      <= 1'b0;
      vram_pending_addr_q        <= 18'd0;
      vram_pending_wdata_q       <= 16'd0;
      icache_hit_busy_q          <= 1'b0;
      icache_hit_cnt_q           <= 8'd0;
      icache_hit_addr_q          <= '0;
      icache_hits_q              <= 32'd0;
      icache_misses_q            <= 32'd0;
      dcache_hits_q              <= 32'd0;
      dcache_misses_q            <= 32'd0;
      hit_is_dcache_q            <= 1'b0;
      dcache_hit_addr_q          <= '0;
      wq_wr_q                    <= '0;
      wq_rd_q                    <= '0;
      wq_drain_busy_q            <= 1'b0;
      srt_src_q                  <= 18'd0;
      srt_dst_q                  <= 18'd0;
      srt_eng_src_q              <= 18'd0;
      srt_last_src_q             <= 18'd0;
      srt_last_dst_q             <= 18'd0;
      srt_len_q                  <= 11'd0;
      srt_src_valid_q            <= 1'b0;
      srt_last_valid_q           <= 1'b0;
      srt_last_big_q             <= 1'b0;
      srt_start_tgl_q            <= 1'b0;
      srt_done_seen_q            <= 1'b0;
      srt_done_sync_q            <= 2'b00;
      srt_copy_count_o           <= 16'd0;
      srt_busy_q                 <= 1'b0;
      srt_q_wr_q                 <= '0;
      srt_q_rd_q                 <= '0;
      srt_q_valid_q              <= '0;
      srt_eng_idx_q              <= '0;
      srt_wait_mask_q            <= '0;
      srt_wait_idx_q             <= '0;
      srt_wait_valid_q           <= 1'b0;
      srt_next_idx_q             <= '0;
      srt_next_valid_q           <= 1'b0;
      srt_eng_dst_row_q          <= 10'd0;
      srt_eng_big_q              <= 1'b0;
      srt_snap_src_q[0]          <= 18'd0;
      srt_snap_src_q[1]          <= 18'd0;
      srt_snap_valid_q           <= 2'b00;
      srt_snap_ready_q           <= 2'b00;
      srt_buf_cur_q              <= 1'b0;
      srt_q_kind_snap            <= '0;
      srt_q_buf                  <= '0;
      srt_q_we_q                 <= 1'b0;
      srt_q_we_d1_q              <= 1'b0;
      srt_q_waddr_q              <= '0;
      srt_q_wdata_q              <= '0;
      srt_handoff_q              <= 1'b0;
      srt_snap_job_q             <= 1'b0;
      srt_eng_buf_q              <= 1'b0;
      for (int k = 0; k < ICACHE_LINES; k++) begin
        icache_valid[k] <= 4'b0000;
      end
      for (int k = 0; k < DCACHE_LINES; k++) begin
        dcache_valid[k] <= 4'b0000;
      end
    end else begin
      local_ack_q <= 1'b0;
      vram_cycle_ack_q <= 1'b0;
      peripheral_read_response_q <= 1'b0;
      if ((palette_scan_lo_o != 16'd0) || (palette_scan_hi_o != 16'd0))
        palette_scan_nonzero_seen_o <= 1'b1;
      if (peripheral_roundtrip_check_q) begin
        if (peripheral_selected_rdata == peripheral_expected_q)
          peripheral_roundtrip_seen_o <= 1'b1;
        peripheral_roundtrip_check_q <= 1'b0;
      end
      if (!cycle_req_i) begin
        local_request_seen_q <= 1'b0;
        vram_request_seen_q  <= 1'b0;
      end

      // Capture one TMS memory cycle. Linear accesses cross once. A Compact
      // 2-bpp write expands into as many as four byte-masked physical writes,
      // and the source cycle is acknowledged only after the final write.
      // Cache hit (instruction or data): answer after the paced latency.
      if (icache_hit || dcache_hit) begin
        vram_request_seen_q <= 1'b1;
        icache_hit_busy_q   <= 1'b1;
        icache_hit_cnt_q    <= 8'd0;
        hit_is_dcache_q     <= dcache_hit;
        icache_hit_addr_q   <= {icache_req_idx, icache_req_off};
        dcache_hit_addr_q   <= {dcache_req_idx, dcache_req_off};
        if (icache_hit) icache_hits_q <= icache_hits_q + 32'd1;
        else dcache_hits_q <= dcache_hits_q + 32'd1;
      end
      if (icache_hit_busy_q) begin
        if (icache_hit_cnt_q == 8'(ICACHE_HIT_TERMINAL)) begin
          icache_hit_busy_q  <= 1'b0;
          vram_cycle_ack_q   <= 1'b1;
          vram_cycle_rdata_q <= hit_is_dcache_q ? dcache_rd_q : icache_rd_q;
        end else icache_hit_cnt_q <= icache_hit_cnt_q + 8'd1;
      end

      // Posted-write drain: one queued write at a time through the CDC while
      // no other VRAM transaction is in progress.
      if (vram_state_q == VRAM_IDLE && !wq_empty && !wq_drain_busy_q &&
          !(vram_accept && word_read && vram_linear_select)) begin
        wq_drain_busy_q  <= 1'b1;
        vram_cdc_req_q   <= 1'b1;
        vram_cdc_we_q    <= 1'b1;
        vram_cdc_be_q    <= wq_be[wq_rd_q[WQ_AW-1:0]];
        vram_cdc_addr_q  <= {wq_line[wq_rd_q[WQ_AW-1:0]], 2'b00};
        vram_cdc_wdata_q <= wq_data[wq_rd_q[WQ_AW-1:0]];
        vram_state_q     <= VRAM_DRAIN;
      end

      // line-buffer flush: the whole line becomes one posted-write entry
      if (lb_valid_q && lb_age_q != 8'd255) lb_age_q <= lb_age_q + 8'd1;
      if (lb_flush_needed) begin
        if (!lb_any_be) lb_valid_q <= 1'b0;
        else if (!wq_full) begin
          wq_line[wq_wr_q[WQ_AW-1:0]]    <= lb_addr_q[VRAM_AW-1:2];
          wq_data[wq_wr_q[WQ_AW-1:0]]    <= {lb_data_q[3], lb_data_q[2], lb_data_q[1], lb_data_q[0]};
          wq_be[wq_wr_q[WQ_AW-1:0]]      <= {lb_be_q[3], lb_be_q[2], lb_be_q[1], lb_be_q[0]};
          wq_valid_q[wq_wr_q[WQ_AW-1:0]] <= 1'b1;
          wq_wr_q                        <= wq_wr_q + 1'b1;
          lb_valid_q                     <= 1'b0;
        end
      end

      srt_q_we_q    <= 1'b0;
      srt_q_we_d1_q <= srt_q_we_q;
      srt_done_sync_q <= {srt_done_sync_q[0], srt_done_tgl_q};
      // Shift-register transfers (see srt_read / srt_write above).
      if (cycle_req_i && srt_read && !vram_request_seen_q && vram_state_q == VRAM_IDLE &&
          !icache_hit_busy_q && !wq_drain_busy_q) begin
        vram_request_seen_q <= 1'b1;
        srt_src_q           <= srt_block_addr;
        srt_src_valid_q     <= 1'b1;
        // a re-latch of the same block (a PIXBLT reading its source row word
        // by word) keeps the repeat-skip valid
        if (srt_block_addr != srt_src_q) srt_last_valid_q <= 1'b0;
        vram_cycle_rdata_q  <= 16'h0000;
        vram_cycle_ack_q    <= 1'b1;
      end
      if (cycle_req_i && srt_write && !vram_request_seen_q && vram_state_q == VRAM_IDLE &&
          !icache_hit_busy_q && !wq_drain_busy_q && wq_empty && !lb_valid_q) begin
        // a block copy rewrites its destination rows behind the caches
        for (int q = 0; q < ICACHE_LINES; q++) icache_valid[q] <= 4'b0000;
        for (int q = 0; q < DCACHE_LINES; q++) dcache_valid[q] <= 4'b0000;
        if (!shiftreg_enable_o || !srt_src_valid_q || srt_src_q == srt_block_addr ||
            (srt_last_valid_q && srt_last_src_q == srt_src_q && srt_last_dst_q == srt_block_addr &&
             srt_last_big_q == vram_2bpp_select)) begin
          // latch off: nothing happens; source == destination (a read-modify-
          // write cycle latched the block itself): the copy is a no-op; same
          // block pair as the last copy with no intervening write: the
          // destination already holds the copy
          vram_request_seen_q <= 1'b1;
          vram_cycle_rdata_q  <= 16'hffff;
          vram_cycle_ack_q    <= 1'b1;
        end else if (!(srt_snap_valid_q[srt_buf_cur_q] && srt_snap_src_q[srt_buf_cur_q] == srt_src_q)) begin
          // no buffer holds this source: queue a snapshot job into the other
          // buffer when it is free (no queued/running copy reads it); the RTM
          // is captured as a copy on the next cycle
          if (!srt_q_full && !srt_buf_busy[!srt_buf_cur_q]) begin
            srt_q_kind_snap[srt_q_wr_q[SRT_Q_AW-1:0]] <= 1'b1;
            srt_q_buf[srt_q_wr_q[SRT_Q_AW-1:0]]       <= !srt_buf_cur_q;
            srt_q_we_q    <= 1'b1;
            srt_q_waddr_q <= srt_q_wr_q[SRT_Q_AW-1:0];
            srt_q_wdata_q <= {srt_src_q, {VRAM_AW{1'b0}}};
            srt_q_valid_q[srt_q_wr_q[SRT_Q_AW-1:0]]   <= 1'b1;
            srt_q_wr_q                    <= srt_q_wr_q + 1'b1;
            srt_snap_src_q[!srt_buf_cur_q]   <= srt_src_q;
            srt_snap_valid_q[!srt_buf_cur_q] <= 1'b1;
            srt_snap_ready_q[!srt_buf_cur_q] <= 1'b0;
            srt_buf_cur_q                    <= !srt_buf_cur_q;
          end
        end else if (!srt_q_full) begin
          vram_request_seen_q <= 1'b1;
          srt_q_kind_snap[srt_q_wr_q[SRT_Q_AW-1:0]] <= 1'b0;
          srt_q_buf[srt_q_wr_q[SRT_Q_AW-1:0]]       <= srt_buf_cur_q;
          srt_q_we_q    <= 1'b1;
          srt_q_waddr_q <= srt_q_wr_q[SRT_Q_AW-1:0];
          srt_q_wdata_q <= {srt_src_q, srt_block_addr};
          srt_q_big[srt_q_wr_q[SRT_Q_AW-1:0]] <= vram_2bpp_select;
          srt_q_dst_row[srt_q_wr_q[SRT_Q_AW-1:0]]  <= srt_block_addr[VRAM_AW-1:8];
          srt_q_valid_q[srt_q_wr_q[SRT_Q_AW-1:0]] <= 1'b1;
          srt_q_wr_q       <= srt_q_wr_q + 1'b1;
          srt_last_valid_q <= 1'b1;
          srt_last_big_q   <= vram_2bpp_select;
          srt_last_src_q   <= srt_src_q;
          srt_last_dst_q   <= srt_block_addr;
          srt_copy_count_o <= srt_copy_count_o + 16'd1;
          for (int k = 0; k < ICACHE_LINES; k++) icache_valid[k] <= 4'b0000;
          for (int k = 0; k < DCACHE_LINES; k++) dcache_valid[k] <= 4'b0000;
          vram_cycle_rdata_q <= 16'hffff;
          vram_cycle_ack_q   <= 1'b1;
        end
        // queue full: the cycle waits
      end
      // Hand the next queued copy to the engine; retire the range when the
      // queue has drained.
      // (posted writes captured before the RTM drain first so the copy lands
      // after them, as MAME's immediate copy did).  A held access's entry
      // (srt_wait_idx_q) goes first; otherwise the earliest pending entry.
      // registered picks (one cycle stale, checked against the valid bits)
      srt_next_valid_q <= srt_next_valid;
      srt_next_idx_q   <= srt_q_rd_q[SRT_Q_AW-1:0];
      srt_wait_idx_q   <= srt_first_set(srt_wait_mask_q);
      if (srt_handoff_q) begin
        // second cycle: src/dst of the picked entry arrive from the block RAM
        srt_handoff_q  <= 1'b0;
        srt_dst_q      <= srt_q_ram_rd_q[VRAM_AW-1:0];
        srt_eng_src_q  <= srt_q_ram_rd_q[2*VRAM_AW-1:VRAM_AW];
        srt_start_tgl_q <= ~srt_start_tgl_q;
      end else if (!srt_busy_q && srt_next_valid_q && srt_q_valid_q[srt_next_idx_q] && wq_empty && !wq_drain_busy_q && !lb_valid_q &&
                   !srt_q_we_q && !srt_q_we_d1_q) begin
        // first cycle: pick (a held access's entry first if its buffer is
        // ready, else the head; snapshot jobs run in queue order; a copy runs
        // only once its buffer's snapshot has completed)
        if (srt_q_kind_snap[srt_pick] || srt_snap_ready_q[srt_q_buf[srt_pick]]) begin
          srt_eng_idx_q      <= srt_pick;
          srt_len_q          <= srt_q_kind_snap[srt_pick] ? (SRT_BIG_BITS+1)'(1<<SRT_BIG_BITS) : (srt_q_big[srt_pick] ? (SRT_BIG_BITS+1)'(1<<SRT_BIG_BITS) : 11'd256);
          srt_eng_dst_row_q  <= srt_q_dst_row[srt_pick];
          srt_eng_big_q      <= srt_q_big[srt_pick];
          srt_snap_job_q     <= srt_q_kind_snap[srt_pick];
          srt_eng_buf_q      <= srt_q_buf[srt_pick];
          srt_q_valid_q[srt_pick] <= 1'b0;
          srt_busy_q         <= 1'b1;
          srt_handoff_q      <= 1'b1;
        end
      end else if (srt_busy_q && (srt_done_sync_q[1] != srt_done_seen_q)) begin
        srt_done_seen_q <= srt_done_sync_q[1];
        srt_busy_q      <= 1'b0;
        if (srt_snap_job_q) begin
          srt_snap_ready_q[srt_eng_buf_q] <= 1'b1;
          srt_snap_job_q <= 1'b0;
        end
      end
      // an ordinary write into a buffer's source rows invalidates that
      // snapshot for FUTURE copies (queued ones keep the data the row
      // transfer moved at RTM time)
      if (cycle_req_i && vram_select && !vram_request_seen_q && word_write) begin
        for (int b = 0; b < 2; b++)
          if (srt_snap_valid_q[b] && srt_snap_ready_q[b] && (vram_logical_addr[VRAM_AW-1:SRT_BIG_BITS] == srt_snap_src_q[b][VRAM_AW-1:SRT_BIG_BITS]))
            srt_snap_valid_q[b] <= 1'b0;
      end
      // retire done entries at the head so the FIFO order stays meaningful
      if (!srt_q_empty && !srt_q_valid_q[srt_q_rd_q[SRT_Q_AW-1:0]] &&
          !(srt_busy_q && srt_eng_idx_q == srt_q_rd_q[SRT_Q_AW-1:0]))
        srt_q_rd_q <= srt_q_rd_q + 1'b1;
      // remember which entry a held ordinary access is waiting for
      if (cycle_req_i && vram_select && (word_read || word_write) && !vram_request_seen_q && srt_hit) begin
        srt_wait_valid_q <= 1'b1;
        srt_wait_mask_q  <= srt_match;
      end else if (!cycle_req_i) begin
        srt_wait_valid_q <= 1'b0;
        srt_wait_mask_q  <= '0;
      end
      if (vram_accept) begin
        vram_request_seen_q  <= 1'b1;
        if (word_write) srt_last_valid_q <= 1'b0;
        vram_pending_write_q <= word_write;
        vram_pending_iaq_q   <= cycle_iaq_i;
        vram_pending_linear_q <= vram_linear_select;
        vram_pending_addr_q  <= vram_logical_addr;
        vram_pending_wdata_q <= cycle_wdata_i;
        // Any VRAM write invalidates the cache lines it may touch (the 2-bpp
        // expander writes four 4-aligned words = one line).
        if (word_write) begin
          icache_valid[icache_inv_idx] <= 4'b0000;
          dcache_valid[dcache_inv_idx] <= 4'b0000;
        end
        if (word_read && vram_linear_select) begin
          if (cycle_iaq_i) icache_misses_q <= icache_misses_q + 32'd1;
          else dcache_misses_q <= dcache_misses_q + 32'd1;
        end
        if (vram_2bpp_select && word_read) begin
          // The expander aperture is write-only on the MultiSync board; reads
          // return zero without issuing a physical VRAM transaction.
          vram_cycle_rdata_q <= 16'h0000;
          vram_cycle_ack_q   <= 1'b1;
        end else if (vram_2bpp_select && word_write) begin
          // expander write: merge the enabled bytes into the line buffer
          lb_valid_q <= 1'b1;
          lb_addr_q  <= lb_new_addr;
          lb_age_q   <= 8'd0;
          for (int q = 0; q < 4; q++) begin
            if (lb_new_be[q][0]) lb_data_q[q][7:0]  <= color_pair_q[7:0];
            if (lb_new_be[q][1]) lb_data_q[q][15:8] <= COCKPIT ? color_pair_q[7:0] : color_pair_q[15:8];
            lb_be_q[q] <= (lb_same_line ? lb_be_q[q] : 2'b00) | lb_new_be[q];
          end
          if (((lb_new_be[0][0] | lb_new_be[1][0] | lb_new_be[2][0] | lb_new_be[3][0]) && color_pair_q[7:0] != 8'd0) ||
              ((lb_new_be[0][1] | lb_new_be[1][1] | lb_new_be[2][1] | lb_new_be[3][1]) && color_pair_q[15:8] != 8'd0))
            vram_write_nonzero_seen_o <= 1'b1;
          vram_cycle_rdata_q     <= 16'hffff;
          vram_cycle_ack_q       <= 1'b1;
          vram_write_count_o     <= vram_write_count_o + 32'd1;
          expander_write_count_o <= expander_write_count_o + 16'd1;
        end else if (word_write) begin
          // posted linear word write
          if (wq_merge_ok) begin
            wq_data[wq_newest] <= line_merge(wq_data[wq_newest], vram_logical_addr[1:0], cycle_wdata_i);
            wq_be[wq_newest]   <= wq_be[wq_newest] | (8'b11 << (2 * vram_logical_addr[1:0]));
          end else begin
            wq_line[wq_wr_q[WQ_AW-1:0]]    <= vram_logical_addr[VRAM_AW-1:2];
            wq_data[wq_wr_q[WQ_AW-1:0]]    <= {4{cycle_wdata_i}};
            wq_be[wq_wr_q[WQ_AW-1:0]]      <= 8'b11 << (2 * vram_logical_addr[1:0]);
            wq_valid_q[wq_wr_q[WQ_AW-1:0]] <= 1'b1;
            wq_wr_q            <= wq_wr_q + 1'b1;
          end
          vram_cycle_rdata_q <= 16'hffff;
          vram_cycle_ack_q   <= 1'b1;
          vram_write_count_o <= vram_write_count_o + 32'd1;
          if (cycle_wdata_i != 16'd0) vram_write_nonzero_seen_o <= 1'b1;
          vram_last_valid_q  <= 1'b1;
          vram_last_addr_q   <= vram_logical_addr;
          vram_last_data_q   <= cycle_wdata_i;
        end else begin
          vram_cdc_req_q   <= 1'b1;
          vram_cdc_we_q    <= 1'b0;
          vram_cdc_be_q    <= 8'hff;
          vram_cdc_addr_q  <= vram_logical_addr;
          vram_cdc_wdata_q <= {4{cycle_wdata_i}};
          vram_state_q     <= VRAM_WAIT_LINEAR;
        end
      end

      unique case (vram_state_q)
        VRAM_WAIT_LINEAR: if (vram_src_ack) begin
          vram_cdc_req_q     <= 1'b0;
          vram_cycle_rdata_q <= vram_src_rdata;
          vram_cycle_ack_q   <= 1'b1;
          vram_state_q       <= VRAM_IDLE;
          if (vram_pending_iaq_q) instruction_fetch_seen_o <= 1'b1;
          if (vram_pending_linear_q) begin
            if (vram_pending_iaq_q) begin
              // (line data written by the icache_ram process)
              if (icache_tag_rd_q != icache_fill_tag) icache_valid[icache_fill_idx] <= fill_word_ok;
              else icache_valid[icache_fill_idx] <= icache_valid[icache_fill_idx] | fill_word_ok;
            end else begin
              // (line data written by the dcache_ram process)
              if (dcache_tag_rd_q != dcache_fill_tag) dcache_valid[dcache_fill_idx] <= fill_word_ok;
              else dcache_valid[dcache_fill_idx] <= dcache_valid[dcache_fill_idx] | fill_word_ok;
            end
          end
          if (vram_last_valid_q &&
              vram_pending_addr_q == vram_last_addr_q &&
              vram_src_rdata == vram_last_data_q)
            vram_roundtrip_seen_o <= 1'b1;
        end

        VRAM_DRAIN: if (vram_src_ack) begin
          vram_cdc_req_q  <= 1'b0;
          wq_valid_q[wq_rd_q[WQ_AW-1:0]] <= 1'b0;
          wq_rd_q         <= wq_rd_q + 1'b1;
          wq_drain_busy_q <= 1'b0;
          vram_state_q    <= VRAM_IDLE;
        end


        VRAM_IDLE: ;
        default: vram_state_q <= VRAM_IDLE;
      endcase

      if (cycle_req_i && !vram_select && !local_request_seen_q) begin
        local_request_seen_q <= 1'b1;
        local_ack_q          <= 1'b1;
        local_rdata_q        <= 16'hffff;

        if (cycle_kind_i == LOCAL_CYCLE_IO_READ) begin
          local_rdata_q <= cycle_io_rdata_i;
        end else if (word_read && peripheral_select) begin
          peripheral_read_response_q <= 1'b1;
          if (control_lo_select || control_hi_select)
            peripheral_read_source_q <= 2'd0;
          else if (palette_lo_select)
            peripheral_read_source_q <= 2'd1;
          else
            peripheral_read_source_q <= 2'd2;

          if (control_hi_select) begin
            control_read_trace_o <= {
              control_read_trace_o[27:0], cycle_addr_i[7:4]
            };
            control_hi_read_count_o <= control_hi_read_count_o + 8'd1;
          end

          if (cycle_iaq_i) instruction_fetch_seen_o <= 1'b1;
          if (peripheral_last_valid_q &&
              cycle_addr_i[31:4] == peripheral_last_addr_q) begin
            peripheral_expected_q        <= peripheral_last_data_q;
            peripheral_roundtrip_check_q <= 1'b1;
          end
        end else if (word_write && peripheral_select) begin
          peripheral_write_count_o <= peripheral_write_count_o + 32'd1;
          peripheral_last_valid_q  <= 1'b1;
          peripheral_last_addr_q   <= cycle_addr_i[31:4];
          peripheral_last_data_q   <= cycle_wdata_i;

          if (control_lo_select || control_hi_select) begin
            control_write_count_o <= control_write_count_o + 16'd1;
            if (control_hi_select) begin
              control_write_trace_o <= {
                control_write_trace_o[59:0], cycle_addr_i[7:4]
              };
              control_hi_write_count_o <= control_hi_write_count_o + 8'd1;
            end
          end
          if (palette_lo_select) begin
            palette_lo_write_count_o <= palette_lo_write_count_o + 16'd1;
            last_palette_addr_o <= palette_addr;
            last_palette_data_o <= cycle_wdata_i;
            if (palette_addr == 10'h2ff)
              palette_bank2_ff_lo_o <= cycle_wdata_i;
            if (palette_addr == 10'h3ff)
              palette_bank3_ff_lo_o <= cycle_wdata_i;
            if (cycle_wdata_i != 16'd0)
              palette_write_nonzero_seen_o <= 1'b1;
          end
          if (palette_hi_select) begin
            palette_hi_write_count_o <= palette_hi_write_count_o + 16'd1;
            last_palette_addr_o <= palette_addr;
            last_palette_data_o <= cycle_wdata_i;
            if (palette_addr == 10'h2ff)
              palette_bank2_ff_hi_o <= cycle_wdata_i;
            if (palette_addr == 10'h3ff)
              palette_bank3_ff_hi_o <= cycle_wdata_i;
            if (cycle_wdata_i != 16'd0)
              palette_write_nonzero_seen_o <= 1'b1;
          end

          if (control_lo_select && cycle_addr_i[7:4] == 4'd0)
            color_pair_q <= cycle_wdata_i;

          if (control_hi_select) begin
            unique case (cycle_addr_i[6:4])
              3'd0: begin shiftreg_enable_o <= cycle_addr_i[7]; srt_last_valid_q <= 1'b0; end
              // Compact MultiSync sheets 10/12: the LS259 Q1 output is the
              // SCROLL clock, with LA7 as its data input. The downstream
              // ALS574 captures LAD3:0 only on a low-to-high SCROLL edge.
              3'd1: begin
                // The ALS574 captures the DATA bus LAD3:0 on the SCROLL rising
                // edge (MAME: finescroll = data & 7 on the MultiSync board).
                if (COCKPIT || (!scroll_latch_q && cycle_addr_i[7]))
                  fine_scroll_o <= COCKPIT ? cycle_wdata_i[3:0] : {1'b0, cycle_wdata_i[2:0]};
                scroll_latch_q <= cycle_addr_i[7];
              end
              3'd2: palette_bank_o[0] <= cycle_addr_i[7];
              3'd3: palette_bank_o[1] <= cycle_addr_i[7];
              default: ;
            endcase
          end
        end
      end
    end
  end
endmodule

`default_nettype wire

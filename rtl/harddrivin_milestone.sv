`default_nettype none

module tms34010_probe #(
  parameter logic [15:0] DATA_SEED = 16'h0000,
  parameter bit          PIXEL_OPS = 1'b1,
  parameter bit          VIDEO = 1'b1,
  parameter bit          FAST_BLIT = 1'b0,
  parameter bit          DRAM_REFRESH = 1'b1
) (
  input  logic        clk_i,
  input  logic        vclk_i,
  input  logic        rst_i,
  input  logic        vclk_rst_i,
  input  logic        hsync_i,
  input  logic        vsync_i,
  input  logic        host_req_i,
  input  logic        host_we_i,
  input  logic [1:0]  host_reg_i,
  input  logic [1:0]  host_be_i,
  input  logic [15:0] host_wdata_i,
  output logic [15:0] host_rdata_o,
  output logic        host_ack_o,
  output logic        host_busy_o,
  output logic        hint_n_o,
  output logic [31:0] pc_o,
  output logic [5:0]  state_o,
  output logic [31:0] cycles_o,
  output logic        illegal_o,
  output logic [15:0] instr_word_o,
  output logic [15:0] hstctll_o,
  output logic        video_hsync_o,
  output logic        video_vsync_o,
  output logic        video_hblank_o,
  output logic        video_vblank_o,
  output logic        video_blank_o,
  output logic        cycle_req_o,
  output tms34010_pkg::local_cycle_kind_t cycle_kind_o,
  output logic [31:0] cycle_addr_o,
  output logic [15:0] cycle_wdata_o,
  output logic [15:0] cycle_io_rdata_o,
  output logic        cycle_iaq_o,
  output logic [13:0] cycle_srfaddr_o,
  output logic [15:0] cycle_dpytap_o,
  output logic        cycle_screen_org_o,
  output logic [7:0]  cycle_dram_row_o,
  input  logic [15:0] cycle_rdata_i,
  input  logic        cycle_ack_i
);
  import tms34010_pkg::*;

  core_state_t core_state;
  host_reg_sel_t host_reg;

  assign host_reg = host_reg_sel_t'(host_reg_i);

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      cycles_o <= 32'd0;
    end else if (cycle_req_o && cycle_ack_i)
      cycles_o <= cycles_o + 32'd1;
  end

  assign state_o = core_state;

  (* keep_hierarchy = "yes" *) tms34010_system #(.PIXEL_OPS(PIXEL_OPS), .VIDEO(VIDEO), .FAST_BLIT(FAST_BLIT), .DRAM_REFRESH(DRAM_REFRESH)) u_tms34010 (
    .clk(clk_i),
    .vclk_i(vclk_i),
    .rst(rst_i),
    .vclk_rst_i(vclk_rst_i),
    .video_hsync_n_i(!hsync_i),
    .video_vsync_n_i(!vsync_i),
    .run_emu_n_i(1'b1),
    .emua_n_o(),
    // HCS is inactive during reset on the board.  The 34010 samples that
    // level into HSTCTL.HLT and the 68010 then releases it through HSTCTL.
    .hcs_n_i(1'b1),
    .host_req_i(host_req_i),
    .host_we_i(host_we_i),
    .host_reg_i(host_reg),
    .host_be_i(host_be_i),
    .host_wdata_i(host_wdata_i),
    .host_rdata_o(host_rdata_o),
    .host_ack_o(host_ack_o),
    .host_busy_o(host_busy_o),
    .hint_n_o(hint_n_o),
    .lint1_n_i(1'b1),
    .lint2_n_i(1'b1),
    .dpyint_set_i(1'b0),
    .hold_req_i(1'b0),
    .hold_ack_o(),
    .video_hsync_o(video_hsync_o),
    .video_vsync_o(video_vsync_o),
    .video_hblank_o(video_hblank_o),
    .video_vblank_o(video_vblank_o),
    .video_blank_o(video_blank_o),
    .video_hsync_oe_o(),
    .video_vsync_oe_o(),
    .cycle_req_o(cycle_req_o),
    .cycle_kind_o(cycle_kind_o),
    .cycle_addr_o(cycle_addr_o),
    .cycle_wdata_o(cycle_wdata_o),
    .cycle_io_rdata_o(cycle_io_rdata_o),
    .cycle_iaq_o(cycle_iaq_o),
    .cycle_srfaddr_o(cycle_srfaddr_o),
    .cycle_dpytap_o(cycle_dpytap_o),
    .cycle_screen_org_o(cycle_screen_org_o),
    .cycle_dram_row_o(cycle_dram_row_o),
    .cycle_rdata_i(cycle_rdata_i),
    .cycle_ack_i(cycle_ack_i),
    .state_o(core_state),
    .pc_o(pc_o),
    .instr_word_o(instr_word_o),
    .illegal_opcode_o(illegal_o),
    .hstctll_diag_o(hstctll_o)
  );

  logic [15:0] unused_seed;
  assign unused_seed = DATA_SEED;
endmodule

// The reserve revision's sound budget macro now builds the real board.
`ifdef HD_SOUND_BUDGET
`define HD_SOUND_BOARD
`endif

module harddrivin_milestone #(
  parameter bit COCKPIT = 0,
  parameter        ZERO_RAM_INIT_HI_FILE = "",
  parameter        ZERO_RAM_INIT_LO_FILE = "",
  // 1: every diagnostic page is captured; 0: only pages 50-54 (GSP boot),
  // 72 (runtime) and 76 (display-interrupt latency) - the other 32 pages'
  // 4096 snapshot flops and their probes drop out of the fit
  parameter bit    DIAG_FULL = 1'b1
) (
  input  logic        clk_50_i,
  input  logic        rst_i,
  output logic        clk_core_o,
  output logic        ce_pix_o,
  output logic        hblank_o,
  output logic        vblank_o,
  output logic        hsync_o,
  output logic        vsync_o,
  output logic [23:0] rgb_o,
  output logic [31:0] debug_o,
  input  logic        diagnostic_enable_i,
  input  logic        controls_overlay_i,   // OSD: draw the controls debug overlay
  input  logic [31:0] analog_axes_i,        // {RY, RX, LY, LX}, unsigned, 0x80 centre
  input  logic [3:0]  axis_moved_i,         // sticky: which axes have ever moved
  // Sound board 68000 EPROM download (MiSTer ROM index 1, ioctl clock).
  input  logic        snd_rom_wr_clk_i,
  input  logic        snd_rom_wr_en_i,
  input  logic [14:0] snd_rom_wr_addr_i,
  input  logic [15:0] snd_rom_wr_data_i,
  // NVRAM (.nvm) side port into the 68010 zero RAM, clk_gsp domain.
  input  logic        nvram_req_i,
  input  logic        nvram_we_i,
  input  logic [11:0] nvram_addr_i,
  input  logic [7:0]  nvram_wdata_i,
  output logic        nvram_ack_o,
  output logic [7:0]  nvram_rdata_o,
  output logic        nvram_dirty_o,
  output logic        pot_prompt_active_o,
  output logic        brake_calibration_active_o,
  output logic        shifter_calibration_active_o,
  output logic        seat_calibration_active_o,
  output logic        calibration_active_o,

  input  logic        coin1_i,
  input  logic        coin2_i,
  input  logic        service_i,
  input  logic        diag_jumper_i,   // port 0 bit 0 (DIAGN), 1 = off
  input  logic [7:0]  sw1_i,           // port 0 high byte: SW1 DIP switches, 1 = off
  input  logic [15:0] compact_port1_i,
  input  logic [7:0]  accelerator_i,
  input  logic [7:0]  clutch_i,
  input  logic [7:0]  seat_i,
  input  logic [7:0]  shifter_x_i,
  input  logic [7:0]  shifter_y_i,
  input  logic [7:0]  brake_i,
  input  logic [11:0] steering_i,
  input  logic [11:0] brake12_i,
  output logic signed [15:0] audio_o,   // driver sound board DAC (HD_SOUND_BOARD)

  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  input  logic [63:0] DDRAM_DOUT,
  input  logic        DDRAM_DOUT_READY,
  output logic        DDRAM_RD,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  output logic        DDRAM_WE,

  inout  wire [15:0]  SDRAM_DQ,
  output logic [12:0] SDRAM_A,
  output logic [1:0]  SDRAM_BA,
  output logic        SDRAM_CLK,
  output logic        SDRAM_CKE,
  output logic        SDRAM_DQML,
  output logic        SDRAM_DQMH,
  output logic        SDRAM_nCS,
  output logic        SDRAM_nCAS,
  output logic        SDRAM_nRAS,
  output logic        SDRAM_nWE
);
  logic clk_gsp, clk_msp, clk_100, clk_video;
  localparam int VRAM_AW = COCKPIT ? 19 : 18;
  logic pll_locked;
  logic rst_gsp, rst_msp, rst_100, rst_video;
  logic [3:0] reset_pipe_gsp, reset_pipe_msp, reset_pipe_100;
  logic [3:0] reset_pipe_video;

  harddrivin_pll #(.COCKPIT(COCKPIT)) u_pll (
    .refclk_i(clk_50_i),
    // File downloads hold the emulated board in reset, but NVRAM transfer
    // acknowledgements use clk_gsp.  Resetting the PLL here stopped that
    // clock and deadlocked MiSTer's ioctl_wait on every saved .nvm load.
    .rst_i(1'b0),
    .clk_gsp_48_o(clk_gsp),
    .clk_msp_50_o(clk_msp),
    .clk_100_o(clk_100),
    .clk_video_6_o(clk_video),
    .locked_o(pll_locked)
  );
  assign clk_core_o = clk_gsp;

  always_ff @(posedge clk_100 or negedge pll_locked) begin
    if (!pll_locked) reset_pipe_100 <= 4'hf;
    else reset_pipe_100 <= {reset_pipe_100[2:0], rst_i};
  end
  always_ff @(posedge clk_gsp or negedge pll_locked) begin
    if (!pll_locked) reset_pipe_gsp <= 4'hf;
    else reset_pipe_gsp <= {reset_pipe_gsp[2:0], rst_i};
  end
  always_ff @(posedge clk_msp or negedge pll_locked) begin
    if (!pll_locked) reset_pipe_msp <= 4'hf;
    else reset_pipe_msp <= {reset_pipe_msp[2:0], rst_i};
  end
  always_ff @(posedge clk_video or negedge pll_locked) begin
    if (!pll_locked) reset_pipe_video <= 4'hf;
    else reset_pipe_video <= {reset_pipe_video[2:0], rst_i};
  end
  assign rst_100 = reset_pipe_100[3];
  assign rst_gsp = reset_pipe_gsp[3];
  assign rst_msp = reset_pipe_msp[3];
  assign rst_video = reset_pipe_video[3];

  logic frame_start;
  logic [7:0] main_diagnostic;
  logic [7:0] visible_diagnostic;
  logic [4991:0] diagnostic_snapshot_q;
  // GSP download checksum (page A7 58) at the first HSTCTL 7800.
  logic [15:0] gsp_dl_count_q, gsp_dl_sum_q, gsp_dl_xor_q;
  logic [15:0] gsp_dl_count_at_release_q, gsp_dl_sum_at_release_q;
  logic [15:0] gsp_dl_xor_at_release_q;
  logic        gsp_dl_release_seen_q;
  logic [15:0] adsp_upload_sum_lo, adsp_upload_xor_lo, adsp_upload_sum_hi, adsp_upload_xor_hi;
  // MSP download split by HSTADRH bucket (page A7 59): A = HSTADRH 0070
  // (program staged through main RAM, 0x00700000+), B = 0077 (ROM table at
  // 0x00778e80), C = everything else (I/O registers, vectors).
  logic [15:0] msp_hstadrh_q;
  logic [15:0] msp_bkt_a_count_q, msp_bkt_a_sum_q, msp_bkt_a_xor_q;
  logic [15:0] msp_bkt_b_sum_q, msp_bkt_b_xor_q, msp_bkt_c_sum_q, msp_bkt_c_xor_q;
  // Page A7 5A: bucket C sub-split: 0000 (RAM fill, sum only), 007b (ROM
  // table 2), 0079, ffff (vectors) sum/xor.
  logic [15:0] msp_bkt_c0_sum_q, msp_bkt_c7b_sum_q, msp_bkt_c7b_xor_q;
  logic [15:0] msp_bkt_c79_sum_q, msp_bkt_c79_xor_q, msp_bkt_cff_sum_q, msp_bkt_cff_xor_q;
  // Frozen-at-release copies (the 68010 keeps writing to the MSP after HLT
  // release, so the live accumulators are not comparable with the oracles).
  logic [15:0] msp_bkt_a_count_rel_q, msp_bkt_a_sum_rel_q, msp_bkt_a_xor_rel_q;
  logic [15:0] msp_bkt_b_sum_rel_q, msp_bkt_b_xor_rel_q, msp_bkt_c_sum_rel_q, msp_bkt_c_xor_rel_q;
  logic [15:0] msp_bkt_c0_sum_rel_q, msp_bkt_c7b_sum_rel_q, msp_bkt_c7b_xor_rel_q;
  logic [15:0] msp_bkt_c79_sum_rel_q, msp_bkt_c79_xor_rel_q, msp_bkt_cff_sum_rel_q, msp_bkt_cff_xor_rel_q;
  // Page A7 69: boot timeline in milliseconds since reset (first occurrence):
  // first L5 (periodic) handler entry, first frame-count write, MSP release
  // (first 7800 to the MSP), GSP release (first 7800 to the GSP), first MSP
  // HINT, first ADSP GINT (irq), error-display entry (fetch of 0x0188c4).
  logic [15:0] ms_counter_q;
  logic [15:0] ms_div_q;
  logic [15:0] t_first_l5_q, t_first_frame_q, t_msp_release_q, t_gsp_release_q;
  logic [15:0] t_first_msp_hint_q, t_first_adsp_irq_q, t_error_display_q;
  logic        seen_l5_q, seen_frame_q, seen_msp_rel_q, seen_gsp_rel_q, seen_msp_hint_q, seen_adsp_irq_q, seen_err_q;
  // Page A7 68: last values the 68010 WRITES to the frame counter ff9134
  // (low word ff9136), the periodic tick counter ffff8014 (low word ffff8016),
  // the ADC channel index ff968c, plus write counts of the frame counter and
  // the tick counter.
  logic [15:0] wr_ff9136_last_q, wr_ffff8016_last_q, wr_ff968c_last_q;
  logic [15:0] wr_ff9136_count_q, wr_ffff8016_count_q, wr_ff968c_count_q;
  logic [15:0] wr_60c000_count_q;   // periodic IRQ clears
  // Page A7 67: 68010 interrupt handler entries (fetches of the runtime
  // vector table's handler addresses, VBR 0x15000) and periodic IRQ
  // assertions.  MAME: L5 ~244/s from ~1.3 s, L1/L4 ~80/s from 2.7 s, L2 0.
  logic [15:0] irq_l1_entries_q, irq_l2_entries_q, irq_l3_entries_q, irq_l4_entries_q;
  logic [15:0] irq_l5_entries_q, irq_l6_entries_q;
  logic [23:0] irq_fetch_d1_q;
  // Page A7 65: the first seven HSTCTL values the 68010 READS from the MSP
  // after the HLT release (MAME: 3800 3890 3810 3880 ...).  Page A7 66: the
  // first seven HSTCTLL values the MSP itself WRITES after release (MAME:
  // 0090 0080 0000 ...).
  logic [15:0] msp_hstctl_reads_q [0:6];
  logic [2:0]  msp_hstctl_reads_n_q;
  logic [15:0] msp_hstctll_writes_q [0:6];
  logic [2:0]  msp_hstctll_writes_n_q;
  logic [15:0] msp_hstctll, msp_hstctll_d1_q;
  // Page A7 63: the MSP's last instruction fetch addresses before its first
  // illegal opcode (3 previous IAQ addresses + the faulting one).
  logic [31:0] msp_iaq_hist_q [0:3];
  logic [31:0] msp_iaq_at_illegal_q [0:3];
  // Page A7 64: 68010 -> MSP host traffic after the HLT release.
  logic [15:0] msp_post_data_count_q, msp_post_adr_count_q;
  logic [15:0] msp_last_hstadrh_q, msp_last_hstadrl_q;
  // Pages A7 5C..62: 68010 PC sampled every 100 ms after reset (32 samples).
  logic [22:0] pc_sample_timer_q;
  logic [4:0]  pc_sample_index_q;
  logic        pc_sample_full_q;
  logic [23:0] pc_samples_q [0:31];
  // Pages A7 6A-71: GSP PC samples every 100 ms from 1.0 s after reset;
  // page A7 72: GSP runtime host-port statistics after the GSP release.
  logic [26:0] gsp_pc_sample_timer_q;
  logic [4:0]  gsp_pc_sample_index_q;
  logic        gsp_pc_sample_full_q, gsp_pc_sample_armed_q;
  logic [25:0] gsp_pc_sample_delay_q;
  logic [23:0] gsp_pc_samples_q [0:31];
  logic [15:0] gsp_hstctll;
  logic [15:0] gsp_rt_hstctl_writes_q, gsp_rt_hstctl_reads_q, gsp_rt_hstdata_writes_q, gsp_rt_last_hstctl_write_q;
  logic [15:0] main_timeout_entries_q;   // fetches of 0x18918 (runtime error display)
  logic [15:0] gsp_pblt_done_q, gsp_ckpt_q;   // PIXBLT/FILL completions and array checkpoints (mod 65536)
  logic [15:0] gsp_di_vec_q, gsp_dpyadr_wr_q; // DI vector fetches (0xfffffea0) and processor DPYADR writes (mod 65536)
  logic [15:0] gsp_hint_gap_max_q, gsp_last_hint_ms_q;   // longest interval between GSP HINTs after release (ms) and the time of the last HINT
  // First gameplay GSP timeout snapshot.  The ROM halts/reinitialises the
  // GSP immediately after entering its error handler, so live PC/state at a
  // later screenshot are useless.  Capture them on the handler's first
  // instruction fetch, while the overdue graphics job is still intact.
  logic        gsp_timeout_snapshot_valid_q;
  logic [23:0] gsp_timeout_pc_q;
  logic [5:0]  gsp_timeout_state_q;
  logic [15:0] gsp_timeout_instr_q;
  logic [15:0] gsp_timeout_hint_age_q;
  logic        gsp_timeout_illegal_q;
  // Page A7 75: palette lookups seen by the scan-out at VRAM row 400 (raster 208) and row 250 (raster 58), x 128.
  logic [8:0]  scan_line_dbg; logic [9:0] scan_x_dbg;
  logic [15:0] pal_r400_addr_q, pal_r400_lo_q, pal_r400_hi_q, pal_r250_addr_q, pal_r250_lo_q, pal_r250_hi_q, pal_bank_r400_q;
  // Page A7 76: raster line of each DI vector fetch (the bottom ISR should
  // start near raster 191 = vcount 0xd0; a late fetch = the ISR was delayed),
  // max/last line seen for fetches in the lower half, count of fetches later
  // than raster 200, and the copy engine's counters.
  logic [15:0] di_bot_max_line_q, di_bot_last_line_q, di_bot_late_q, di_top_max_line_q;
  logic [15:0] gsp_srt_copy_count, gsp_srt_pending_cycles_q;
  logic gsp_srt_pending;
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      di_bot_max_line_q <= 16'd0; di_bot_last_line_q <= 16'd0; di_bot_late_q <= 16'd0; di_top_max_line_q <= 16'd0; gsp_srt_pending_cycles_q <= 16'd0;
    end else begin
    if (gsp_cycle_req && gsp_cycle_ack && !gsp_cycle_iaq && gsp_cycle_addr == 32'hfffffea0) begin
      if (scan_line_dbg >= 9'd150 && scan_line_dbg < 9'd288) begin
        di_bot_last_line_q <= {7'd0, scan_line_dbg};
        if ({7'd0, scan_line_dbg} > di_bot_max_line_q) di_bot_max_line_q <= {7'd0, scan_line_dbg};
        if (scan_line_dbg > 9'd200) di_bot_late_q <= di_bot_late_q + 16'd1;
      end else begin
        if ({7'd0, scan_line_dbg} > di_top_max_line_q && scan_line_dbg < 9'd150) di_top_max_line_q <= {7'd0, scan_line_dbg};
      end
    end
    if (gsp_srt_pending) gsp_srt_pending_cycles_q <= gsp_srt_pending_cycles_q + 16'd1;
    end
  end
  always_ff @(posedge clk_gsp) begin
    // raster line = VRAM row - 192 (visible lines 0..287): row 400 -> 208, row 250 -> 58
    if (scan_line_dbg == 9'd208 && scan_x_dbg == 10'd128) begin
      pal_r400_addr_q <= {6'd0, gsp_palette_scan_addr}; pal_r400_lo_q <= gsp_palette_scan_lo; pal_r400_hi_q <= gsp_palette_scan_hi; pal_bank_r400_q <= {14'd0, gsp_palette_bank};
    end
    if (scan_line_dbg == 9'd58 && scan_x_dbg == 10'd128) begin
      pal_r250_addr_q <= {6'd0, gsp_palette_scan_addr}; pal_r250_lo_q <= gsp_palette_scan_lo; pal_r250_hi_q <= gsp_palette_scan_hi;
    end
  end
  // Pages A7 73/74: VRAM words seen by the scan-out at rows 400 and 250, columns 0x40..0x46 (mem_clk domain).
  logic [15:0] vram_row400_q [0:6];
  logic [15:0] vram_row250_q [0:6];
  always_ff @(posedge clk_100) begin
    if (scan_ack && scan_addr[17:8] == 10'd400 && scan_addr[7:0] >= 8'h40 && scan_addr[7:0] <= 8'h46) vram_row400_q[scan_addr[2:0]] <= scan_data;
    if (scan_ack && scan_addr[17:8] == 10'd250 && scan_addr[7:0] >= 8'h40 && scan_addr[7:0] <= 8'h46) vram_row250_q[scan_addr[2:0]] <= scan_data;
  end
  logic [5:0]  gsp_state_d1_q;
  // Page A7 5B: MSP host-port access statistics up to release + interrupt counts.
  logic [15:0] msp_host_read_count_q, msp_hstctl_write_count_q, msp_hstadr_write_count_q;
  logic [15:0] msp_first_hstctl_read_q;
  logic        msp_first_hstctl_read_seen_q;
  logic [15:0] msp_hint_count_q, gsp_hint_count_q, main_irq_ack_count_q;
  logic        msp_hint_n_d1_q, gsp_hint_n_d1_q;
  // MSP download/execution observability (page A7 57).
  logic [15:0] msp_hstdata_count_q, msp_hstdata_sum_q, msp_hstdata_xor_q;
  logic [15:0] msp_hstdata_count_at_release_q, msp_hstdata_sum_at_release_q;
  logic [15:0] msp_hstdata_xor_at_release_q;
  logic        msp_release_seen_q;
  logic [31:0] msp_first_illegal_pc_q;
  logic [15:0] msp_first_illegal_instr_q;
  logic        msp_illegal_seen_q;
  logic [15:0] msp_instr_word;
  // ADSP board observability (driven in the ADSP section below).
  logic [13:0] adsp_pc;
  logic        adsp_running, adsp_illegal, adsp_dirq, adsp_xflag, adsp_mpage;
  logic [7:0]  adsp_latch;
  logic [15:0] adsp_gint_count, adsp_upload_count, adsp_sim_reads, adsp_som_writes;
  logic        adsp_irq;
  logic [19:0] diagnostic_div_q;
  logic [5:0]  diagnostic_page_index_q;
  logic [4:0] diagnostic_page_hold_q;
  logic [127:0] diagnostic_page;
  logic [15:0] gsp_last_hstctl_write_q;
  logic [15:0] gsp_last_hstctl_read_q;
  logic [15:0] gsp_last_hstadrl_write_q;
  logic [15:0] gsp_last_hstadrh_write_q;
  logic [15:0] gsp_last_hstdata_write_q;
  logic [15:0] gsp_host_write_count_q;
  logic [15:0] gsp_hstdata_write_count_q;
  logic [15:0] gsp_hstdata_count_at_release_q;
  logic [31:0] gsp_host_addr_shadow_q;
  logic [31:0] gsp_host_control_trace_q;
  logic [7:0]  gsp_host_control_write_count_q;
  logic [63:0] gsp_hstctl_write_trace_q;
  logic [7:0]  gsp_hstctl_write_count_q;
  logic [7:0]  gsp_hstctl_seen_q;
  logic [1:0]  gsp_hstctl_last_be_q;
  logic        gsp_reset_halt_seen_q;
  logic        gsp_left_reset_halt_seen_q;
  logic [15:0] gsp_reset_vector_lo_q;
  logic [15:0] gsp_reset_vector_hi_q;
  logic [1:0]  gsp_reset_vector_seen_q;
  logic [31:0] gsp_first_iaq_addr_q;
  logic [15:0] gsp_first_iaq_data_q;
  logic        gsp_first_iaq_seen_q;
  logic [7:0]  main_boot_path_seen_q;
  logic [15:0] gsp_dpyctl_q;
  logic [15:0] gsp_dpystart_q;
  logic [15:0] gsp_heblnk_q;
  logic [15:0] gsp_control_q;
  logic [15:0] gsp_dpytap_q;
  logic        gsp_linear_cycle_seen_q;
  logic [7:0]  gsp_vram_path_level;
  logic [6:0]  gsp_vram_path_seen_100_q;

  logic [31:0] gsp_pc, msp_pc;
  logic [5:0] gsp_state, msp_state;
  logic [15:0] gsp_instr_word;
  logic [31:0] gsp_cycles, msp_cycles;
  logic gsp_illegal, msp_illegal;

  logic [7:0] main_switch1;
  logic [7:0] main_control_latch;
  logic main_watchdog_clear, main_irq_clear;
  logic gsp_reset_release, msp_reset_release;
  logic msp_reset_release_sync;
  logic tms_gsp_rst, tms_msp_rst;

  logic gsp_host_req, gsp_host_we;
  logic [1:0] gsp_host_reg, gsp_host_be;
  logic [15:0] gsp_host_wdata, gsp_host_rdata;
  logic gsp_host_ack, gsp_host_busy;
  logic gsp_hint_n, msp_hint_n;

  logic msp_host_src_req, msp_host_src_we;
  logic [1:0] msp_host_src_reg, msp_host_src_be;
  logic [15:0] msp_host_src_wdata, msp_host_src_rdata;
  logic msp_host_src_ack;
  logic msp_host_req, msp_host_we;
  logic [1:0] msp_host_reg, msp_host_be;
  logic [15:0] msp_host_wdata, msp_host_rdata;
  logic msp_host_ack, msp_host_busy;

  logic gsp_cycle_req, gsp_cycle_ack;
  tms34010_pkg::local_cycle_kind_t gsp_cycle_kind;
  logic [31:0] gsp_cycle_addr;
  logic [15:0] gsp_cycle_wdata, gsp_cycle_rdata, gsp_cycle_io_rdata;
  logic gsp_cycle_iaq;
  logic [13:0] gsp_cycle_srfaddr;
  logic [15:0] gsp_cycle_dpytap;
  logic gsp_cycle_screen_org;
  logic [7:0] gsp_cycle_dram_row;

  logic msp_cycle_req, msp_cycle_ack;
  tms34010_pkg::local_cycle_kind_t msp_cycle_kind;
  logic [31:0] msp_cycle_addr;
  logic [15:0] msp_cycle_wdata, msp_cycle_rdata, msp_cycle_io_rdata;
  logic msp_cycle_iaq;
  logic [13:0] msp_cycle_srfaddr;
  logic [15:0] msp_cycle_dpytap;
  logic msp_cycle_screen_org;
  logic [7:0] msp_cycle_dram_row;

  logic gsp_vram_req_100, gsp_vram_we_100, gsp_vram_ack_100;
  logic [63:0] gsp_vram_rdata64_100;
  logic [7:0] gsp_vram_be_100;          // line write: byte enables of four words
  logic [VRAM_AW-1:0] gsp_vram_addr_100;
  logic [63:0] gsp_vram_wdata_100;      // line write: four words
  logic [15:0] gsp_vram_rdata_100;
  logic burst_initialized;
  logic traffic_reset;
  logic [31:0] gsp_peripheral_write_count, gsp_vram_write_count;
  logic [15:0] gsp_control_write_count;
  logic [15:0] gsp_palette_lo_write_count, gsp_palette_hi_write_count;
  logic gsp_palette_write_nonzero_seen, gsp_palette_scan_nonzero_seen;
  logic [9:0] gsp_last_palette_addr;
  logic [15:0] gsp_last_palette_data;
  logic [15:0] gsp_palette_bank2_ff_lo, gsp_palette_bank2_ff_hi;
  logic [15:0] gsp_palette_bank3_ff_lo, gsp_palette_bank3_ff_hi;
  logic [63:0] gsp_control_write_trace;
  logic [7:0] gsp_control_hi_write_count;
  logic [31:0] gsp_control_read_trace;
  logic [7:0] gsp_control_hi_read_count;
  logic [15:0] gsp_expander_write_count;
  logic gsp_vram_write_nonzero_seen;
  logic [31:0] msp_memory_write_count;
  logic gsp_instruction_fetch_seen, msp_instruction_fetch_seen;
  logic gsp_peripheral_roundtrip_seen, gsp_vram_roundtrip_seen;
  logic msp_memory_roundtrip_seen;
  logic gsp_memory_cycle_req, gsp_memory_cycle_ack;
  logic [15:0] gsp_memory_cycle_rdata;
  logic gsp_screen_cycle;
  logic gsp_screen_ack;
  logic gsp_video_hsync, gsp_video_vsync;
  logic gsp_video_hblank, gsp_video_vblank, gsp_video_blank;
  logic msp_video_hsync, msp_video_vsync;
  logic msp_video_hblank, msp_video_vblank, msp_video_blank;
  logic [9:0] gsp_palette_scan_addr;
  logic [15:0] gsp_palette_scan_lo, gsp_palette_scan_hi;
  logic [3:0] gsp_fine_scroll;
  logic [1:0] gsp_palette_bank;
  logic gsp_shiftreg_enable;
  logic [31:0] screen_refresh_count, screen_row_count;
  logic [13:0] screen_loaded_srfaddr;
  logic screen_loaded_org;
  logic [7:0] screen_loaded_col, screen_loaded_row_select;
  logic [15:0] screen_loaded_row_or;
  logic [10:0] screen_loaded_row_nonzero;
  logic screen_row_valid;

  // SP-327 sheet 8 low switch byte. Both ADCs complete immediately in this
  // functional model; the two blanking inputs retain their board polarity.
  assign main_switch1 = {
    !coin1_i, !coin2_i, !service_i, 2'b11,
    !vblank_o, !hblank_o, diag_jumper_i
  };
  assign tms_gsp_rst = rst_gsp || !gsp_reset_release;

  tms34010_sync_bit #(.RESET_VALUE(1'b0)) u_msp_release_sync (
    .clk(clk_msp), .rst(rst_msp), .async_i(msp_reset_release),
    .sync_o(msp_reset_release_sync)
  );
  assign tms_msp_rst = rst_msp || !msp_reset_release_sync;
  assign gsp_screen_cycle = gsp_cycle_req &&
      (gsp_cycle_kind == tms34010_pkg::LOCAL_CYCLE_SCREEN_REFRESH);
  assign gsp_memory_cycle_req = gsp_cycle_req && !gsp_screen_cycle;
  assign gsp_cycle_ack = gsp_screen_cycle
                       ? gsp_screen_ack : gsp_memory_cycle_ack;
  assign gsp_cycle_rdata = gsp_screen_cycle
                         ? 16'hffff : gsp_memory_cycle_rdata;

  // The 68010 pulses the MSP core reset (msp_reset_release / NBUS latch bit 7)
  // during the program download.  The download host CDC must NOT be torn down
  // by that pulse: its two sides would leave reset a synchronizer-skew apart
  // (src on clk_gsp, dst on clk_msp through msp_reset_release_sync), and the
  // one-entry mailbox requires both resets to assert and deassert together.
  // The skew dropped ~10 of the ~100k downloaded words, holing the MSP program
  // (BAD POLY BUFF).  Reset the CDC only on the global domain resets, which
  // both track rst_i and are effectively simultaneous at power-on when no host
  // traffic is in flight.  A reset of the MSP core+host_if mid-download simply
  // stalls a crossing command (main bus holds its request) until the host_if
  // returns; nothing is lost.
  harddrivin_host_cdc u_msp_host_cdc (
    .src_clk_i(clk_gsp),
    .src_rst_i(rst_gsp),
    .src_req_i(msp_host_src_req), .src_we_i(msp_host_src_we),
    .src_reg_i(msp_host_src_reg), .src_be_i(msp_host_src_be),
    .src_wdata_i(msp_host_src_wdata), .src_rdata_o(msp_host_src_rdata),
    .src_ack_o(msp_host_src_ack),
    .dst_clk_i(clk_msp), .dst_rst_i(rst_msp),
    .dst_req_o(msp_host_req), .dst_we_o(msp_host_we),
    .dst_reg_o(msp_host_reg), .dst_be_o(msp_host_be),
    .dst_wdata_o(msp_host_wdata), .dst_rdata_i(msp_host_rdata),
    .dst_ack_i(msp_host_ack)
  );

`ifdef HD_ARCH_MILESTONE
  localparam bit HD_GSP_LINE_FILL = 1'b1;
`else
  localparam bit HD_GSP_LINE_FILL = 1'b0;
`endif
  logic gsp_adapt_req, gsp_adapt_ack;
  logic [31:0] gsp_adapt_addr;
  logic [15:0] gsp_adapt_wdata, gsp_adapt_rdata;
  tms34010_pkg::local_cycle_kind_t gsp_adapt_kind;
  harddrivin_cockpit_expander #(.COCKPIT(COCKPIT)) u_cockpit_expander (
    .clk_i(clk_gsp), .rst_i(tms_gsp_rst),
    .req_i(gsp_memory_cycle_req), .kind_i(gsp_cycle_kind),
    .addr_i(gsp_cycle_addr), .wdata_i(gsp_cycle_wdata),
    .rdata_o(gsp_memory_cycle_rdata), .ack_o(gsp_memory_cycle_ack),
    .req_o(gsp_adapt_req), .kind_o(gsp_adapt_kind),
    .addr_o(gsp_adapt_addr), .wdata_o(gsp_adapt_wdata),
    .rdata_i(gsp_adapt_rdata), .ack_i(gsp_adapt_ack)
  );
  harddrivin_gsp_memory #(.LINE_FILL(HD_GSP_LINE_FILL), .COCKPIT(COCKPIT),
    .VRAM_AW(VRAM_AW), .SRT_BIG_BITS(COCKPIT ? 11 : 10)) u_gsp_memory (
    .clk_i(clk_gsp), .rst_i(tms_gsp_rst),
    .cycle_req_i(gsp_adapt_req), .cycle_kind_i(gsp_adapt_kind),
    .cycle_addr_i(gsp_adapt_addr), .cycle_wdata_i(gsp_adapt_wdata),
    .cycle_io_rdata_i(gsp_cycle_io_rdata), .cycle_iaq_i(gsp_cycle_iaq),
    .cycle_rdata_o(gsp_adapt_rdata),
    .cycle_ack_o(gsp_adapt_ack),
    // Reset the destination half of the VRAM mailbox with its consumer.
    // The physical burst arbiter cannot accept a command until SDRAM has
    // completed initialization; releasing the mailbox earlier creates a
    // startup window in which the two halves disagree about readiness.
    .mem_clk_i(clk_100), .mem_rst_i(traffic_reset),
    .vram_req_o(gsp_vram_req_100), .vram_we_o(gsp_vram_we_100),
    .vram_be_o(gsp_vram_be_100),
    .vram_addr_o(gsp_vram_addr_100), .vram_wdata_o(gsp_vram_wdata_100),
    .vram_rdata_i(gsp_vram_rdata_100), .vram_rdata64_i(gsp_vram_rdata64_100), .vram_ack_i(gsp_vram_ack_100),
    .palette_scan_addr_i(gsp_palette_scan_addr),
    .palette_scan_lo_o(gsp_palette_scan_lo),
    .palette_scan_hi_o(gsp_palette_scan_hi),
    .fine_scroll_o(gsp_fine_scroll),
    .palette_bank_o(gsp_palette_bank),
    .shiftreg_enable_o(gsp_shiftreg_enable),
    .srt_copy_count_o(gsp_srt_copy_count), .srt_pending_o(gsp_srt_pending),
    .peripheral_write_count_o(gsp_peripheral_write_count),
    .control_write_count_o(gsp_control_write_count),
    .palette_lo_write_count_o(gsp_palette_lo_write_count),
    .palette_hi_write_count_o(gsp_palette_hi_write_count),
    .palette_write_nonzero_seen_o(gsp_palette_write_nonzero_seen),
    .palette_scan_nonzero_seen_o(gsp_palette_scan_nonzero_seen),
    .last_palette_addr_o(gsp_last_palette_addr),
    .last_palette_data_o(gsp_last_palette_data),
    .palette_bank2_ff_lo_o(gsp_palette_bank2_ff_lo),
    .palette_bank2_ff_hi_o(gsp_palette_bank2_ff_hi),
    .palette_bank3_ff_lo_o(gsp_palette_bank3_ff_lo),
    .palette_bank3_ff_hi_o(gsp_palette_bank3_ff_hi),
    .control_write_trace_o(gsp_control_write_trace),
    .control_hi_write_count_o(gsp_control_hi_write_count),
    .control_read_trace_o(gsp_control_read_trace),
    .control_hi_read_count_o(gsp_control_hi_read_count),
    .vram_write_count_o(gsp_vram_write_count),
    .vram_write_nonzero_seen_o(gsp_vram_write_nonzero_seen),
    .expander_write_count_o(gsp_expander_write_count),
    .instruction_fetch_seen_o(gsp_instruction_fetch_seen),
    .peripheral_roundtrip_seen_o(gsp_peripheral_roundtrip_seen),
    .vram_roundtrip_seen_o(gsp_vram_roundtrip_seen)
  );

  // GSP PC sampler: armed 1.0 s after reset, then every 100 ms, 32 samples.
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      gsp_pc_sample_timer_q <= 27'd0; gsp_pc_sample_index_q <= 5'd0; gsp_pc_sample_full_q <= 1'b0;
      gsp_pc_sample_armed_q <= 1'b0; gsp_pc_sample_delay_q <= 26'd0;
    end else if (!gsp_pc_sample_armed_q) begin
      if (gsp_pc_sample_delay_q == 26'd47_999_999) gsp_pc_sample_armed_q <= 1'b1;
      else gsp_pc_sample_delay_q <= gsp_pc_sample_delay_q + 26'd1;
    end else if (!gsp_pc_sample_full_q) begin
      if (gsp_pc_sample_timer_q == 27'd95_999_999) begin
        gsp_pc_sample_timer_q <= 27'd0;
        gsp_pc_samples_q[gsp_pc_sample_index_q] <= gsp_pc[23:0];
        if (gsp_pc_sample_index_q == 5'd31) gsp_pc_sample_full_q <= 1'b1;
        gsp_pc_sample_index_q <= gsp_pc_sample_index_q + 5'd1;
      end else gsp_pc_sample_timer_q <= gsp_pc_sample_timer_q + 27'd1;
    end
  end
  // GSP host-port statistics after the release (HSTCTL 0x7800).
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      gsp_rt_hstctl_writes_q <= 16'd0; gsp_rt_hstctl_reads_q <= 16'd0;
      gsp_rt_hstdata_writes_q <= 16'd0; gsp_rt_last_hstctl_write_q <= 16'd0;
      main_timeout_entries_q <= 16'd0;
      gsp_pblt_done_q <= 16'd0; gsp_ckpt_q <= 16'd0; gsp_state_d1_q <= 6'd0; gsp_di_vec_q <= 16'd0; gsp_dpyadr_wr_q <= 16'd0;
      gsp_hint_gap_max_q <= 16'd0; gsp_last_hint_ms_q <= 16'd0;
      gsp_timeout_snapshot_valid_q <= 1'b0;
      gsp_timeout_pc_q <= 24'd0;
      gsp_timeout_state_q <= 6'd0;
      gsp_timeout_instr_q <= 16'd0;
      gsp_timeout_hint_age_q <= 16'd0;
      gsp_timeout_illegal_q <= 1'b0;
    end else begin
      gsp_state_d1_q <= gsp_state;
      // CORE_PBLT_WB2 (15) / CORE_FILL_WB (11) entries = completed array ops; CORE_ARRAY_CKPT_B14 (53) entries = checkpoints
      if ((gsp_state == 6'd15 && gsp_state_d1_q != 6'd15) || (gsp_state == 6'd11 && gsp_state_d1_q != 6'd11)) gsp_pblt_done_q <= gsp_pblt_done_q + 16'd1;
      if (gsp_state == 6'd53 && gsp_state_d1_q != 6'd53) gsp_ckpt_q <= gsp_ckpt_q + 16'd1;
      if (gsp_cycle_req && gsp_cycle_ack && !gsp_cycle_iaq && gsp_cycle_addr == 32'hfffffea0) gsp_di_vec_q <= gsp_di_vec_q + 16'd1;
      if (seen_gsp_rel_q && gsp_hint_n_d1_q && !gsp_hint_n) begin
        if ((ms_counter_q - gsp_last_hint_ms_q) > gsp_hint_gap_max_q) gsp_hint_gap_max_q <= ms_counter_q - gsp_last_hint_ms_q;
        gsp_last_hint_ms_q <= ms_counter_q;
      end
      if (gsp_cycle_req && gsp_cycle_ack && gsp_cycle_kind == tms34010_pkg::LOCAL_CYCLE_IO_WRITE && gsp_cycle_addr == 32'hc00001e0) gsp_dpyadr_wr_q <= gsp_dpyadr_wr_q + 16'd1;
      if (main_cpu_ce && main_busstate_effective == 2'b00 && main_addr[23:1] == 23'h00c48c) begin
        main_timeout_entries_q <= main_timeout_entries_q + 16'd1;
        if (!gsp_timeout_snapshot_valid_q) begin
          gsp_timeout_snapshot_valid_q <= 1'b1;
          gsp_timeout_pc_q <= gsp_pc[23:0];
          gsp_timeout_state_q <= gsp_state;
          gsp_timeout_instr_q <= gsp_instr_word;
          gsp_timeout_hint_age_q <= ms_counter_q - gsp_last_hint_ms_q;
          gsp_timeout_illegal_q <= gsp_illegal;
        end
      end
      if (seen_gsp_rel_q && gsp_host_req && gsp_host_ack) begin
      if (gsp_host_we && gsp_host_reg == 2'b11) begin gsp_rt_hstctl_writes_q <= gsp_rt_hstctl_writes_q + 16'd1; gsp_rt_last_hstctl_write_q <= gsp_host_wdata; end
      if (!gsp_host_we && gsp_host_reg == 2'b11) gsp_rt_hstctl_reads_q <= gsp_rt_hstctl_reads_q + 16'd1;
      if (gsp_host_we && gsp_host_reg == 2'b10) gsp_rt_hstdata_writes_q <= gsp_rt_hstdata_writes_q + 16'd1;
      end
    end
  end

  // 68010 PC sampler (clk_gsp = 48 MHz; 4,800,000 cycles = 100 ms).
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      pc_sample_timer_q <= 23'd0; pc_sample_index_q <= 5'd0; pc_sample_full_q <= 1'b0;
    end else if (!pc_sample_full_q) begin
      if (pc_sample_timer_q == 23'd4_799_999) begin
        pc_sample_timer_q <= 23'd0;
        pc_samples_q[pc_sample_index_q] <= main_last_fetch_addr;
        if (pc_sample_index_q == 5'd31) pc_sample_full_q <= 1'b1;
        pc_sample_index_q <= pc_sample_index_q + 5'd1;
      end else pc_sample_timer_q <= pc_sample_timer_q + 23'd1;
    end
  end

  // Millisecond timeline (clk_gsp = 48 MHz -> 48000 cycles per ms).
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      ms_counter_q <= 16'd0; ms_div_q <= 16'd0;
      t_first_l5_q <= 16'd0; t_first_frame_q <= 16'd0; t_msp_release_q <= 16'd0; t_gsp_release_q <= 16'd0;
      t_first_msp_hint_q <= 16'd0; t_first_adsp_irq_q <= 16'd0; t_error_display_q <= 16'd0;
      seen_l5_q <= 1'b0; seen_frame_q <= 1'b0; seen_msp_rel_q <= 1'b0; seen_gsp_rel_q <= 1'b0;
      seen_msp_hint_q <= 1'b0; seen_adsp_irq_q <= 1'b0; seen_err_q <= 1'b0;
    end else begin
      if (ms_div_q == 16'd47999) begin ms_div_q <= 16'd0; ms_counter_q <= ms_counter_q + 16'd1; end
      else ms_div_q <= ms_div_q + 16'd1;
      if (!seen_l5_q && main_last_fetch_addr == 24'h03db0a) begin seen_l5_q <= 1'b1; t_first_l5_q <= ms_counter_q; end
      if (!seen_frame_q && main_cpu_ce && main_busstate_effective == 2'b11 && main_addr[23:1] == 23'h7fc89b) begin seen_frame_q <= 1'b1; t_first_frame_q <= ms_counter_q; end
      if (!seen_msp_rel_q && msp_host_src_req && msp_host_src_ack && msp_host_src_we && msp_host_src_reg == 2'b11 && msp_host_src_wdata == 16'h7800) begin seen_msp_rel_q <= 1'b1; t_msp_release_q <= ms_counter_q; end
      if (!seen_gsp_rel_q && gsp_host_req && gsp_host_ack && gsp_host_we && gsp_host_reg == 2'b11 && gsp_host_wdata == 16'h7800) begin seen_gsp_rel_q <= 1'b1; t_gsp_release_q <= ms_counter_q; end
      if (!seen_msp_hint_q && !msp_hint_n) begin seen_msp_hint_q <= 1'b1; t_first_msp_hint_q <= ms_counter_q; end
      if (!seen_adsp_irq_q && adsp_irq) begin seen_adsp_irq_q <= 1'b1; t_first_adsp_irq_q <= ms_counter_q; end
      if (!seen_err_q && main_last_fetch_addr == 24'h0188c4) begin seen_err_q <= 1'b1; t_error_display_q <= ms_counter_q; end
    end
  end

  // 68010 write snoop (clk_gsp): main bus write cycles at the RAM variables
  // of interest (word writes; the low word of a long write has A1=1).
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      wr_ff9136_last_q <= 16'd0; wr_ffff8016_last_q <= 16'd0; wr_ff968c_last_q <= 16'd0;
      wr_ff9136_count_q <= 16'd0; wr_ffff8016_count_q <= 16'd0; wr_ff968c_count_q <= 16'd0;
      wr_60c000_count_q <= 16'd0;
    end else if (main_cpu_ce && main_busstate_effective == 2'b11) begin
      case (main_addr[23:1])
        23'h7fc89b: begin wr_ff9136_last_q <= main_data_out; wr_ff9136_count_q <= wr_ff9136_count_q + 16'd1; end   // 0xff9136
        23'h7fc00b: begin wr_ffff8016_last_q <= main_data_out; wr_ffff8016_count_q <= wr_ffff8016_count_q + 16'd1; end // 0xff8016
        23'h7fcb46: begin wr_ff968c_last_q <= main_data_out; wr_ff968c_count_q <= wr_ff968c_count_q + 16'd1; end   // 0xff968c
        23'h306000: wr_60c000_count_q <= wr_60c000_count_q + 16'd1;   // 0x60c000
        default: ;
      endcase
    end
  end

  // Handler-entry counters (clk_gsp): count each new fetch address equal to a
  // runtime handler entry (main_last_fetch_addr changes on every fetch).
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      irq_l1_entries_q <= 16'd0; irq_l2_entries_q <= 16'd0; irq_l3_entries_q <= 16'd0;
      irq_l4_entries_q <= 16'd0; irq_l5_entries_q <= 16'd0; irq_l6_entries_q <= 16'd0;
      irq_fetch_d1_q <= 24'd0;
    end else begin
      irq_fetch_d1_q <= main_last_fetch_addr;
      if (main_last_fetch_addr != irq_fetch_d1_q) begin
        case (main_last_fetch_addr)
          24'h03cc9c: irq_l1_entries_q <= irq_l1_entries_q + 16'd1;
          24'h03cc4a: irq_l2_entries_q <= irq_l2_entries_q + 16'd1;
          24'h03cc84: irq_l3_entries_q <= irq_l3_entries_q + 16'd1;
          24'h03ccb4: irq_l4_entries_q <= irq_l4_entries_q + 16'd1;
          24'h03db0a: irq_l5_entries_q <= irq_l5_entries_q + 16'd1;
          24'h03f77e: irq_l6_entries_q <= irq_l6_entries_q + 16'd1;
          default: ;
        endcase
      end
    end
  end

  // Interrupt statistics: MSP HINT falling edges (clk_msp), GSP HINT falling
  // edges and 68010 IPL-active transitions (clk_gsp).
  always_ff @(posedge clk_msp) begin
    if (rst_msp) begin msp_hint_count_q <= 16'd0; msp_hint_n_d1_q <= 1'b1; end
    else begin
      msp_hint_n_d1_q <= msp_hint_n;
      if (msp_hint_n_d1_q && !msp_hint_n) msp_hint_count_q <= msp_hint_count_q + 16'd1;
    end
  end
  logic [2:0] main_ipl_n_d1_q;
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin gsp_hint_count_q <= 16'd0; gsp_hint_n_d1_q <= 1'b1; main_irq_ack_count_q <= 16'd0; main_ipl_n_d1_q <= 3'b111; end
    else begin
      gsp_hint_n_d1_q <= gsp_hint_n;
      if (gsp_hint_n_d1_q && !gsp_hint_n) gsp_hint_count_q <= gsp_hint_count_q + 16'd1;
      main_ipl_n_d1_q <= main_ipl_n;
      if (main_ipl_n_d1_q == 3'b111 && main_ipl_n != 3'b111) main_irq_ack_count_q <= main_irq_ack_count_q + 16'd1;
    end
  end

  // GSP host-port download probe (same clock as the 68010 bus).
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      gsp_dl_count_q <= 16'd0; gsp_dl_sum_q <= 16'd0; gsp_dl_xor_q <= 16'd0;
      gsp_dl_count_at_release_q <= 16'd0; gsp_dl_sum_at_release_q <= 16'd0;
      gsp_dl_xor_at_release_q <= 16'd0; gsp_dl_release_seen_q <= 1'b0;
    end else if (gsp_host_req && gsp_host_ack && gsp_host_we) begin
      if (gsp_host_reg == 2'b10) begin
        gsp_dl_count_q <= gsp_dl_count_q + 16'd1;
        gsp_dl_sum_q   <= gsp_dl_sum_q + gsp_host_wdata;
        gsp_dl_xor_q   <= gsp_dl_xor_q ^ gsp_host_wdata;
      end
      if (gsp_host_reg == 2'b11 && gsp_host_wdata == 16'h7800 && !gsp_dl_release_seen_q) begin
        gsp_dl_release_seen_q <= 1'b1;
        gsp_dl_count_at_release_q <= gsp_dl_count_q;
        gsp_dl_sum_at_release_q   <= gsp_dl_sum_q;
        gsp_dl_xor_at_release_q   <= gsp_dl_xor_q;
      end
    end
  end

  // MSP fetch history (clk_msp): shift in every acknowledged IAQ cycle;
  // freeze a copy on the first illegal opcode.
  always_ff @(posedge clk_msp) begin
    if (rst_msp) begin
      for (int k = 0; k < 4; k++) begin msp_iaq_hist_q[k] <= 32'd0; msp_iaq_at_illegal_q[k] <= 32'd0; end
    end else begin
      if (msp_cycle_req && msp_cycle_ack && msp_cycle_iaq) begin
        msp_iaq_hist_q[3] <= msp_iaq_hist_q[2];
        msp_iaq_hist_q[2] <= msp_iaq_hist_q[1];
        msp_iaq_hist_q[1] <= msp_iaq_hist_q[0];
        msp_iaq_hist_q[0] <= msp_cycle_addr;
      end
      if (msp_illegal && !msp_illegal_seen_q)
        for (int k = 0; k < 4; k++) msp_iaq_at_illegal_q[k] <= msp_iaq_hist_q[k];
    end
  end

  // MSP host-port download probe (50 MHz side, i.e. what the MSP receives).
  // Reset only with the domain (rst_msp): the 68010 pulses the MSP core reset
  // mid-download, which used to clear these counters (page 57 then showed
  // only the post-pulse tail: 34723 words, exactly MAME's tail count).
  always_ff @(posedge clk_msp) begin
    if (rst_msp) begin
      msp_hstadrh_q <= 16'd0;
      msp_bkt_a_count_q <= 16'd0; msp_bkt_a_sum_q <= 16'd0; msp_bkt_a_xor_q <= 16'd0;
      msp_bkt_b_sum_q <= 16'd0; msp_bkt_b_xor_q <= 16'd0;
      msp_bkt_c_sum_q <= 16'd0; msp_bkt_c_xor_q <= 16'd0;
      msp_bkt_c0_sum_q <= 16'd0; msp_bkt_c7b_sum_q <= 16'd0; msp_bkt_c7b_xor_q <= 16'd0;
      msp_bkt_c79_sum_q <= 16'd0; msp_bkt_c79_xor_q <= 16'd0;
      msp_bkt_cff_sum_q <= 16'd0; msp_bkt_cff_xor_q <= 16'd0;
      msp_bkt_a_count_rel_q <= 16'd0; msp_bkt_a_sum_rel_q <= 16'd0; msp_bkt_a_xor_rel_q <= 16'd0;
      msp_bkt_b_sum_rel_q <= 16'd0; msp_bkt_b_xor_rel_q <= 16'd0; msp_bkt_c_sum_rel_q <= 16'd0; msp_bkt_c_xor_rel_q <= 16'd0;
      msp_bkt_c0_sum_rel_q <= 16'd0; msp_bkt_c7b_sum_rel_q <= 16'd0; msp_bkt_c7b_xor_rel_q <= 16'd0;
      msp_bkt_c79_sum_rel_q <= 16'd0; msp_bkt_c79_xor_rel_q <= 16'd0; msp_bkt_cff_sum_rel_q <= 16'd0; msp_bkt_cff_xor_rel_q <= 16'd0;
      msp_hstctl_reads_n_q <= 3'd0; msp_hstctll_writes_n_q <= 3'd0; msp_hstctll_d1_q <= 16'd0;
      for (int k = 0; k < 7; k++) begin msp_hstctl_reads_q[k] <= 16'd0; msp_hstctll_writes_q[k] <= 16'd0; end
      msp_post_data_count_q <= 16'd0; msp_post_adr_count_q <= 16'd0;
      msp_last_hstadrh_q <= 16'd0; msp_last_hstadrl_q <= 16'd0;
      msp_host_read_count_q <= 16'd0; msp_hstctl_write_count_q <= 16'd0;
      msp_hstadr_write_count_q <= 16'd0; msp_first_hstctl_read_q <= 16'd0;
      msp_first_hstctl_read_seen_q <= 1'b0;
      msp_hstdata_count_q <= 16'd0;
      msp_hstdata_sum_q   <= 16'd0;
      msp_hstdata_xor_q   <= 16'd0;
      msp_hstdata_count_at_release_q <= 16'd0;
      msp_hstdata_sum_at_release_q   <= 16'd0;
      msp_hstdata_xor_at_release_q   <= 16'd0;
      msp_release_seen_q  <= 1'b0;
      msp_first_illegal_pc_q    <= 32'd0;
      msp_first_illegal_instr_q <= 16'd0;
      msp_illegal_seen_q  <= 1'b0;
    end else begin
      if (msp_host_req && msp_host_ack && !msp_host_we && msp_release_seen_q &&
          msp_host_reg == 2'b11 && msp_hstctl_reads_n_q != 3'd7) begin
        msp_hstctl_reads_q[msp_hstctl_reads_n_q] <= msp_host_rdata;
        msp_hstctl_reads_n_q <= msp_hstctl_reads_n_q + 3'd1;
      end
      // Page 66: the first seven CHANGES of the live HSTCTLL register value
      // after release (either side may write it; MAME sequence 0090 0080
      // 0000 0090 ... as seen at the register).
      msp_hstctll_d1_q <= msp_hstctll;
      if (msp_release_seen_q && msp_hstctll != msp_hstctll_d1_q && msp_hstctll_writes_n_q != 3'd7) begin
        msp_hstctll_writes_q[msp_hstctll_writes_n_q] <= msp_hstctll;
        msp_hstctll_writes_n_q <= msp_hstctll_writes_n_q + 3'd1;
      end
      if (msp_host_req && msp_host_ack && !msp_host_we && !msp_release_seen_q) begin
        msp_host_read_count_q <= msp_host_read_count_q + 16'd1;
        if (msp_host_reg == 2'b11 && !msp_first_hstctl_read_seen_q) begin
          msp_first_hstctl_read_seen_q <= 1'b1;
          msp_first_hstctl_read_q <= msp_host_rdata;
        end
      end
      if (msp_host_req && msp_host_ack && msp_host_we && !msp_release_seen_q) begin
        if (msp_host_reg == 2'b11) msp_hstctl_write_count_q <= msp_hstctl_write_count_q + 16'd1;
        if (msp_host_reg == 2'b00 || msp_host_reg == 2'b01) msp_hstadr_write_count_q <= msp_hstadr_write_count_q + 16'd1;
      end
      if (msp_host_req && msp_host_ack && msp_host_we && msp_release_seen_q) begin
        if (msp_host_reg == 2'b10) msp_post_data_count_q <= msp_post_data_count_q + 16'd1;
        if (msp_host_reg == 2'b00 || msp_host_reg == 2'b01) msp_post_adr_count_q <= msp_post_adr_count_q + 16'd1;
        if (msp_host_reg == 2'b01) msp_last_hstadrh_q <= msp_host_wdata;
        if (msp_host_reg == 2'b00) msp_last_hstadrl_q <= msp_host_wdata;
      end
      if (msp_host_req && msp_host_ack && msp_host_we) begin
        if (msp_host_reg == 2'b01) msp_hstadrh_q <= msp_host_wdata;   // HFS=01 is HSTADRH
        if (msp_host_reg == 2'b10) begin
          if (msp_hstadrh_q == 16'h0070) begin
            msp_bkt_a_count_q <= msp_bkt_a_count_q + 16'd1;
            msp_bkt_a_sum_q <= msp_bkt_a_sum_q + msp_host_wdata;
            msp_bkt_a_xor_q <= msp_bkt_a_xor_q ^ msp_host_wdata;
          end else if (msp_hstadrh_q == 16'h0077) begin
            msp_bkt_b_sum_q <= msp_bkt_b_sum_q + msp_host_wdata;
            msp_bkt_b_xor_q <= msp_bkt_b_xor_q ^ msp_host_wdata;
          end else begin
            msp_bkt_c_sum_q <= msp_bkt_c_sum_q + msp_host_wdata;
            msp_bkt_c_xor_q <= msp_bkt_c_xor_q ^ msp_host_wdata;
            case (msp_hstadrh_q)
              16'h0000: msp_bkt_c0_sum_q <= msp_bkt_c0_sum_q + msp_host_wdata;
              16'h007b: begin
                msp_bkt_c7b_sum_q <= msp_bkt_c7b_sum_q + msp_host_wdata;
                msp_bkt_c7b_xor_q <= msp_bkt_c7b_xor_q ^ msp_host_wdata;
              end
              16'h0079: begin
                msp_bkt_c79_sum_q <= msp_bkt_c79_sum_q + msp_host_wdata;
                msp_bkt_c79_xor_q <= msp_bkt_c79_xor_q ^ msp_host_wdata;
              end
              16'hffff: begin
                msp_bkt_cff_sum_q <= msp_bkt_cff_sum_q + msp_host_wdata;
                msp_bkt_cff_xor_q <= msp_bkt_cff_xor_q ^ msp_host_wdata;
              end
              default: ;
            endcase
          end
          msp_hstdata_count_q <= msp_hstdata_count_q + 16'd1;
          msp_hstdata_sum_q   <= msp_hstdata_sum_q + msp_host_wdata;
          msp_hstdata_xor_q   <= msp_hstdata_xor_q ^ msp_host_wdata;
        end
        if (msp_host_reg == 2'b11 && msp_host_wdata == 16'h7800 &&
            !msp_release_seen_q) begin
          msp_release_seen_q <= 1'b1;
          msp_bkt_a_count_rel_q <= msp_bkt_a_count_q; msp_bkt_a_sum_rel_q <= msp_bkt_a_sum_q; msp_bkt_a_xor_rel_q <= msp_bkt_a_xor_q;
          msp_bkt_b_sum_rel_q <= msp_bkt_b_sum_q; msp_bkt_b_xor_rel_q <= msp_bkt_b_xor_q;
          msp_bkt_c_sum_rel_q <= msp_bkt_c_sum_q; msp_bkt_c_xor_rel_q <= msp_bkt_c_xor_q;
          msp_bkt_c0_sum_rel_q <= msp_bkt_c0_sum_q; msp_bkt_c7b_sum_rel_q <= msp_bkt_c7b_sum_q; msp_bkt_c7b_xor_rel_q <= msp_bkt_c7b_xor_q;
          msp_bkt_c79_sum_rel_q <= msp_bkt_c79_sum_q; msp_bkt_c79_xor_rel_q <= msp_bkt_c79_xor_q;
          msp_bkt_cff_sum_rel_q <= msp_bkt_cff_sum_q; msp_bkt_cff_xor_rel_q <= msp_bkt_cff_xor_q;
          msp_hstdata_count_at_release_q <= msp_hstdata_count_q;
          msp_hstdata_sum_at_release_q   <= msp_hstdata_sum_q;
          msp_hstdata_xor_at_release_q   <= msp_hstdata_xor_q;
        end
      end
      if (msp_illegal && !msp_illegal_seen_q) begin
        msp_illegal_seen_q        <= 1'b1;
        msp_first_illegal_pc_q    <= msp_pc;
        msp_first_illegal_instr_q <= msp_instr_word;
      end
    end
  end

  // Three wait states per MSP RAM access: calibrated against MAME with the
  // full 68010 host stream (tests/tb_harddrivin_msp_cosim.sv): the MSP's
  // message hold time (HSTCTLL 0x0090 -> 0x0080) is 796 us with 3, 529 us
  // with 0; MAME/real board ~790 us.  The 68010 <-> MSP result-buffer
  // protocol depends on that pacing.
  harddrivin_msp_memory #(.WAIT_STATES(3)) u_msp_memory (
    .clk_i(clk_msp), .rst_i(tms_msp_rst),
    .cycle_req_i(msp_cycle_req), .cycle_kind_i(msp_cycle_kind),
    .cycle_addr_i(msp_cycle_addr), .cycle_wdata_i(msp_cycle_wdata),
    .cycle_io_rdata_i(msp_cycle_io_rdata), .cycle_iaq_i(msp_cycle_iaq),
    .cycle_rdata_o(msp_cycle_rdata), .cycle_ack_o(msp_cycle_ack),
    .write_count_o(msp_memory_write_count),
    .instruction_fetch_seen_o(msp_instruction_fetch_seen),
    .roundtrip_seen_o(msp_memory_roundtrip_seen)
  );

  assign visible_diagnostic = {
    msp_instruction_fetch_seen,
    gsp_instruction_fetch_seen,
    |gsp_vram_write_count[31:10],
    |gsp_peripheral_write_count[31:9],
    main_diagnostic[7:5],
    &main_diagnostic[4:0]
  };

  // Snapshot independently of the GSP display timing. During host bootstrap
  // the real board deliberately leaves the GSP halted, so VSYNC can remain at
  // its reset level and is not a valid trigger for bring-up observability.
  // Roughly 46 snapshots/second keep multi-domain values visually stable.
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      diagnostic_div_q          <= 20'd0;
      diagnostic_page_index_q   <= 6'd0;
      diagnostic_page_hold_q    <= 5'd0;
      diagnostic_snapshot_q     <= 4992'd0;
      gsp_last_hstctl_write_q    <= 16'd0;
      gsp_last_hstctl_read_q     <= 16'd0;
      gsp_last_hstadrl_write_q   <= 16'd0;
      gsp_last_hstadrh_write_q   <= 16'd0;
      gsp_last_hstdata_write_q   <= 16'd0;
      gsp_host_write_count_q     <= 16'd0;
      gsp_hstdata_write_count_q  <= 16'd0;
      gsp_hstdata_count_at_release_q <= 16'd0;
      gsp_host_addr_shadow_q     <= 32'd0;
      gsp_host_control_trace_q   <= 32'd0;
      gsp_host_control_write_count_q <= 8'd0;
      gsp_hstctl_write_trace_q   <= 64'd0;
      gsp_hstctl_write_count_q   <= 8'd0;
      gsp_hstctl_seen_q          <= 8'd0;
      gsp_hstctl_last_be_q       <= 2'd0;
      gsp_reset_halt_seen_q      <= 1'b0;
      gsp_left_reset_halt_seen_q <= 1'b0;
      gsp_reset_vector_lo_q      <= 16'd0;
      gsp_reset_vector_hi_q      <= 16'd0;
      gsp_reset_vector_seen_q    <= 2'd0;
      gsp_first_iaq_addr_q       <= 32'd0;
      gsp_first_iaq_data_q       <= 16'd0;
      gsp_first_iaq_seen_q       <= 1'b0;
      main_boot_path_seen_q      <= 8'd0;
      gsp_dpyctl_q               <= 16'd0;
      gsp_dpystart_q             <= 16'd0;
      gsp_heblnk_q               <= 16'd0;
      gsp_control_q              <= 16'd0;
      gsp_dpytap_q               <= 16'd0;
      gsp_linear_cycle_seen_q    <= 1'b0;
    end else begin
      diagnostic_div_q <= diagnostic_div_q + 20'd1;

      if (gsp_memory_cycle_req &&
          (gsp_cycle_addr[31:23] == 9'h1ff))
        gsp_linear_cycle_seen_q <= 1'b1;

      if (gsp_host_ack) begin
        if (gsp_host_we) begin
          gsp_host_write_count_q <= gsp_host_write_count_q + 16'd1;
          unique case (gsp_host_reg)
            2'b00: begin
              gsp_last_hstadrl_write_q <= gsp_host_wdata;
              gsp_host_addr_shadow_q[15:0] <= gsp_host_wdata;
            end
            2'b01: begin
              gsp_last_hstadrh_write_q <= gsp_host_wdata;
              gsp_host_addr_shadow_q[31:16] <= gsp_host_wdata;
            end
            2'b10: begin
              gsp_last_hstdata_write_q <= gsp_host_wdata;
              gsp_hstdata_write_count_q <= gsp_hstdata_write_count_q + 16'd1;
              if (gsp_host_addr_shadow_q[31:8] == 24'hf48000) begin
                gsp_host_control_trace_q <= {
                  gsp_host_control_trace_q[27:0],
                  gsp_host_addr_shadow_q[7:4]
                };
                gsp_host_control_write_count_q <=
                    gsp_host_control_write_count_q + 8'd1;
              end
            end
            2'b11: begin
              gsp_last_hstctl_write_q <= gsp_host_wdata;
              // Freeze the first four accepted HSTCTL writes. Later error
              // handling rewrites HSTCTL and would otherwise erase the
              // Compact bootstrap's B800, 7800, 3800 release sequence.
              if (gsp_hstctl_write_count_q < 8'd4)
                gsp_hstctl_write_trace_q <= {
                  gsp_hstctl_write_trace_q[47:0], gsp_host_wdata
                };
              gsp_hstctl_write_count_q <=
                  gsp_hstctl_write_count_q + 8'd1;
              gsp_hstctl_last_be_q <= gsp_host_be;
              if (gsp_host_wdata == 16'h7800)
                begin
                  gsp_hstctl_seen_q[0] <= 1'b1;
                  if (!gsp_hstctl_seen_q[0])
                    gsp_hstdata_count_at_release_q <=
                        gsp_hstdata_write_count_q;
                end
              if (gsp_host_wdata == 16'h3800)
                gsp_hstctl_seen_q[1] <= 1'b1;
              if (gsp_host_be[1] && !gsp_host_wdata[15])
                gsp_hstctl_seen_q[2] <= 1'b1;
              if (gsp_host_be[1] && gsp_host_wdata[15])
                gsp_hstctl_seen_q[3] <= 1'b1;
              if ((gsp_host_be == 2'b11) &&
                  (gsp_host_wdata == 16'h7800))
                gsp_hstctl_seen_q[4] <= 1'b1;
              if ((gsp_host_be == 2'b11) &&
                  (gsp_host_wdata == 16'h3800))
                gsp_hstctl_seen_q[5] <= 1'b1;
              if (gsp_host_be == 2'b01)
                gsp_hstctl_seen_q[6] <= 1'b1;
              if (gsp_host_be == 2'b10)
                gsp_hstctl_seen_q[7] <= 1'b1;
            end
            default: ;
          endcase
        end else if (gsp_host_reg == 2'b11) begin
          gsp_last_hstctl_read_q <= gsp_host_rdata;
        end
      end

      if (gsp_state == tms34010_pkg::CORE_RESET_HALT)
        gsp_reset_halt_seen_q <= 1'b1;
      else if (gsp_reset_halt_seen_q)
        gsp_left_reset_halt_seen_q <= 1'b1;

      // Preserve the two physical words used by the architectural level-0
      // reset-vector read and the first acknowledged instruction fetch.  The
      // Compact ROM/MAME oracle writes F720/FFF5 here, forming PC=FFF5F720.
      // Freeze the FIRST acknowledged read of each vector word after the
      // 7800 release, not the last. The seed-38 hardware capture showed that
      // later re-reads (host prefetch, retries) overwrite the values that the
      // core actually assembled into its first PC, which made the page-55
      // vector fields disagree with the frozen first-IAQ address.
      if (gsp_cycle_req && gsp_cycle_ack && gsp_hstctl_seen_q[0]) begin
        if (gsp_cycle_addr == 32'hffffffe0 && !gsp_reset_vector_seen_q[0]) begin
          gsp_reset_vector_lo_q   <= gsp_cycle_rdata;
          gsp_reset_vector_seen_q[0] <= 1'b1;
        end
        if (gsp_cycle_addr == 32'hfffffff0 && !gsp_reset_vector_seen_q[1]) begin
          gsp_reset_vector_hi_q   <= gsp_cycle_rdata;
          gsp_reset_vector_seen_q[1] <= 1'b1;
        end
        if (gsp_cycle_iaq && !gsp_first_iaq_seen_q) begin
          gsp_first_iaq_addr_q <= gsp_cycle_addr;
          gsp_first_iaq_data_q <= gsp_cycle_rdata;
          gsp_first_iaq_seen_q <= 1'b1;
        end
      end

      if (gsp_cycle_req && gsp_cycle_ack &&
          (gsp_cycle_kind == tms34010_pkg::LOCAL_CYCLE_IO_WRITE)) begin
        unique case (gsp_cycle_addr[8:4])
          5'h01: gsp_heblnk_q   <= gsp_cycle_wdata;
          5'h08: gsp_dpyctl_q   <= gsp_cycle_wdata;
          5'h09: gsp_dpystart_q <= gsp_cycle_wdata;
          5'h0b: gsp_control_q  <= gsp_cycle_wdata;
          5'h1b: gsp_dpytap_q   <= gsp_cycle_wdata;
          default: ;
        endcase
      end

      // Sticky ROM-path checkpoints around the GSP download and release.
      // They are sampled from the completed-cycle adapter's fetch address,
      // so a short-lived opcode fetch remains visible after the error screen.
      unique case (main_last_fetch_addr)
        24'h01b198: main_boot_path_seen_q[0] <= 1'b1; // download B800
        24'h0155a8: main_boot_path_seen_q[1] <= 1'b1; // write 7800
        24'h0155b0: main_boot_path_seen_q[2] <= 1'b1; // write 3800
        24'h015686: main_boot_path_seen_q[3] <= 1'b1; // enter game init
        24'h01a46a: main_boot_path_seen_q[4] <= 1'b1; // GSP handshake
        24'h01a4ca: main_boot_path_seen_q[5] <= 1'b1; // handshake error
        24'h0187f2: main_boot_path_seen_q[6] <= 1'b1; // error display
        24'h03cabe,
        24'h03cac0: main_boot_path_seen_q[7] <= 1'b1; // terminal STOP
        default: ;
      endcase

      if (&diagnostic_div_q) begin
        if (&diagnostic_page_hold_q) begin
          diagnostic_page_hold_q <= 5'd0;
          diagnostic_page_index_q <= (diagnostic_page_index_q == 6'd38)
                                   ? 6'd0
                                   : diagnostic_page_index_q + 6'd1;
        end else begin
          diagnostic_page_hold_q <= diagnostic_page_hold_q + 5'd1;
        end
        // Fixed little-endian 16-byte pages. Each carries its own magic
        // header because real MiSTer video capture preserves only 128
        // independent horizontal samples from this source raster.
        diagnostic_snapshot_q[7:0]     <= 8'ha7;
        diagnostic_snapshot_q[15:8]    <= 8'h50;
        diagnostic_snapshot_q[23:16]   <= {
          tms_msp_rst, tms_gsp_rst, msp_reset_release, gsp_reset_release,
          msp_illegal, gsp_illegal, gsp_last_palette_addr[9:8]
        };
        diagnostic_snapshot_q[31:24]   <= {
          gsp_palette_scan_nonzero_seen, gsp_palette_write_nonzero_seen,
          gsp_palette_bank, gsp_fine_scroll
        };
        diagnostic_snapshot_q[39:32]   <= gsp_dpyctl_q[7:0];
        diagnostic_snapshot_q[47:40]   <= gsp_dpyctl_q[15:8];
        diagnostic_snapshot_q[55:48]   <= gsp_dpystart_q[7:0];
        diagnostic_snapshot_q[63:56]   <= gsp_dpystart_q[15:8];
        diagnostic_snapshot_q[71:64]   <= gsp_control_q[7:0];
        diagnostic_snapshot_q[79:72]   <= gsp_control_q[15:8];
        diagnostic_snapshot_q[87:80]   <= gsp_dpytap_q[7:0];
        diagnostic_snapshot_q[95:88]   <= gsp_dpytap_q[15:8];
        diagnostic_snapshot_q[103:96]  <= screen_loaded_srfaddr[7:0];
        diagnostic_snapshot_q[111:104] <= {
          screen_loaded_org, 1'b0, screen_loaded_srfaddr[13:8]
        };
        diagnostic_snapshot_q[119:112] <= screen_loaded_col;
        diagnostic_snapshot_q[127:120] <= screen_loaded_row_select;
        diagnostic_snapshot_q[135:128] <= 8'ha7;
        diagnostic_snapshot_q[143:136] <= 8'h51;
        diagnostic_snapshot_q[151:144] <= screen_loaded_row_or[7:0];
        diagnostic_snapshot_q[159:152] <= screen_loaded_row_or[15:8];
        diagnostic_snapshot_q[167:160] <= screen_loaded_row_nonzero[7:0];
        diagnostic_snapshot_q[175:168] <= {
          4'd0, gsp_vram_write_nonzero_seen,
          screen_loaded_row_nonzero[10:8]
        };
        diagnostic_snapshot_q[183:176] <= screen_refresh_count[7:0];
        diagnostic_snapshot_q[191:184] <= screen_refresh_count[15:8];
        diagnostic_snapshot_q[199:192] <= screen_row_count[7:0];
        diagnostic_snapshot_q[207:200] <= screen_row_count[15:8];
        diagnostic_snapshot_q[215:208] <= gsp_vram_write_count[7:0];
        diagnostic_snapshot_q[223:216] <= gsp_vram_write_count[15:8];
        diagnostic_snapshot_q[231:224] <= gsp_control_write_count[7:0];
        diagnostic_snapshot_q[239:232] <= gsp_palette_lo_write_count[7:0];
        diagnostic_snapshot_q[247:240] <= gsp_palette_hi_write_count[7:0];
        diagnostic_snapshot_q[255:248] <= gsp_last_palette_addr[7:0];
        diagnostic_snapshot_q[263:256] <= 8'ha7;
        diagnostic_snapshot_q[271:264] <= 8'h52;
        diagnostic_snapshot_q[279:272] <= {
          gsp_instruction_fetch_seen, gsp_vram_roundtrip_seen,
          gsp_peripheral_roundtrip_seen, gsp_host_busy,
          gsp_reset_release, msp_reset_release, gsp_illegal, msp_illegal
        };
        diagnostic_snapshot_q[287:280] <= main_diagnostic;
        diagnostic_snapshot_q[303:288] <= gsp_last_hstadrl_write_q;
        diagnostic_snapshot_q[319:304] <= gsp_last_hstadrh_write_q;
        diagnostic_snapshot_q[335:320] <= gsp_last_hstdata_write_q;
        diagnostic_snapshot_q[351:336] <= gsp_last_hstctl_write_q;
        diagnostic_snapshot_q[359:352] <= gsp_host_write_count_q[7:0];
        diagnostic_snapshot_q[367:360] <= gsp_hstdata_write_count_q[7:0];
        // Live and sticky milestones across the complete host-prefetch path.
        // The sticky byte is ordered local-cycle, CDC request/ack, SDRAM
        // request/accept/done, and scan request/ack.
        diagnostic_snapshot_q[375:368] <= gsp_vram_path_level;
        diagnostic_snapshot_q[383:376] <= {
          gsp_linear_cycle_seen_q, gsp_vram_path_seen_100_q
        };
        diagnostic_snapshot_q[391:384] <= 8'ha7;
        diagnostic_snapshot_q[399:392] <= 8'h53;
        // Temporary GSP-boot page. The first four HSTCTL writes are frozen;
        // within that group the newest write is trace bits 15:0. Sticky flags
        // prove whether the release values arrived with the required lanes.
        diagnostic_snapshot_q[431:400] <= gsp_hstctl_write_trace_q[31:0];
        diagnostic_snapshot_q[463:432] <= gsp_hstctl_write_trace_q[63:32];
        diagnostic_snapshot_q[471:464] <= gsp_hstctl_write_count_q;
        diagnostic_snapshot_q[479:472] <= main_boot_path_seen_q;
        diagnostic_snapshot_q[487:480] <= gsp_hstctl_seen_q;
        diagnostic_snapshot_q[495:488] <= {
          5'd0, gsp_left_reset_halt_seen_q, gsp_hstctl_last_be_q
        };
        diagnostic_snapshot_q[503:496] <= {
          gsp_host_busy, gsp_last_hstctl_write_q[15], gsp_state
        };
        diagnostic_snapshot_q[511:504] <= gsp_pc[7:0];
        diagnostic_snapshot_q[519:512] <= 8'ha7;
        diagnostic_snapshot_q[527:520] <= 8'h54;
        diagnostic_snapshot_q[551:528] <= main_last_fetch_addr;
        diagnostic_snapshot_q[575:552] <= main_last_read_addr;
        diagnostic_snapshot_q[599:576] <= main_last_write_addr;
        diagnostic_snapshot_q[615:600] <= main_fetch_count[15:0];
        diagnostic_snapshot_q[623:616] <= main_read_count[7:0];
        diagnostic_snapshot_q[631:624] <= main_write_count[7:0];
        diagnostic_snapshot_q[639:632] <= {
          main_skip_fetch, main_bus_ready, main_busstate_effective,
          main_ipl_n, main_periodic_irq_q
        };
        diagnostic_snapshot_q[647:640] <= 8'ha7;
        diagnostic_snapshot_q[655:648] <= 8'h55;
        diagnostic_snapshot_q[671:656] <= gsp_reset_vector_lo_q;
        diagnostic_snapshot_q[687:672] <= gsp_reset_vector_hi_q;
        diagnostic_snapshot_q[695:688] <= {
          2'd0,
          (gsp_first_iaq_addr_q == 32'hfff5f720),
          (gsp_reset_vector_hi_q == 16'hfff5),
          (gsp_reset_vector_lo_q == 16'hf720),
          gsp_first_iaq_seen_q,
          gsp_reset_vector_seen_q
        };
        diagnostic_snapshot_q[703:696] <= {
          2'd0, gsp_state
        };
        diagnostic_snapshot_q[735:704] <= gsp_first_iaq_addr_q;
        diagnostic_snapshot_q[751:736] <= gsp_first_iaq_data_q;
        diagnostic_snapshot_q[767:752] <=
            gsp_hstdata_count_at_release_q;
        // Page A7 56: ADSP board state.
        diagnostic_snapshot_q[775:768] <= 8'ha7;
        diagnostic_snapshot_q[783:776] <= 8'h56;
        diagnostic_snapshot_q[799:784] <= {2'd0, adsp_pc};
        diagnostic_snapshot_q[807:800] <= adsp_latch;
        diagnostic_snapshot_q[815:808] <= {
          2'd0, adsp_irq, adsp_mpage, adsp_xflag, adsp_dirq, adsp_illegal,
          adsp_running
        };
        diagnostic_snapshot_q[831:816] <= adsp_upload_count;
        diagnostic_snapshot_q[847:832] <= adsp_gint_count;
        diagnostic_snapshot_q[863:848] <= adsp_sim_reads;
        diagnostic_snapshot_q[879:864] <= adsp_som_writes;
        diagnostic_snapshot_q[895:880] <= main_last_fetch_addr[23:8];
        // Page A7 57: MSP download checksum at release and first illegal.
        diagnostic_snapshot_q[903:896]  <= 8'ha7;
        diagnostic_snapshot_q[911:904]  <= 8'h57;
        diagnostic_snapshot_q[927:912]  <= msp_hstdata_count_at_release_q;
        diagnostic_snapshot_q[943:928]  <= msp_hstdata_sum_at_release_q;
        diagnostic_snapshot_q[959:944]  <= msp_hstdata_xor_at_release_q;
        diagnostic_snapshot_q[991:960]  <= msp_first_illegal_pc_q;
        diagnostic_snapshot_q[1007:992] <= msp_first_illegal_instr_q;
        diagnostic_snapshot_q[1015:1008] <= {
          6'd0, msp_illegal_seen_q, msp_release_seen_q
        };
        diagnostic_snapshot_q[1023:1016] <= {2'd0, msp_state};
        // Page A7 58: GSP download checksum at release + ADSP upload checksums.
        diagnostic_snapshot_q[1031:1024] <= 8'ha7;
        diagnostic_snapshot_q[1039:1032] <= 8'h58;
        diagnostic_snapshot_q[1055:1040] <= gsp_dl_count_at_release_q;
        diagnostic_snapshot_q[1071:1056] <= gsp_dl_sum_at_release_q;
        diagnostic_snapshot_q[1087:1072] <= gsp_dl_xor_at_release_q;
        diagnostic_snapshot_q[1103:1088] <= adsp_upload_sum_lo;
        diagnostic_snapshot_q[1119:1104] <= adsp_upload_xor_lo;
        diagnostic_snapshot_q[1135:1120] <= adsp_upload_sum_hi;
        diagnostic_snapshot_q[1151:1136] <= adsp_upload_xor_hi;
        // Page A7 59: MSP download checksums per HSTADRH bucket.
        diagnostic_snapshot_q[1159:1152] <= 8'ha7;
        diagnostic_snapshot_q[1167:1160] <= 8'h59;
        diagnostic_snapshot_q[1183:1168] <= msp_bkt_a_count_rel_q;
        diagnostic_snapshot_q[1199:1184] <= msp_bkt_a_sum_rel_q;
        diagnostic_snapshot_q[1215:1200] <= msp_bkt_a_xor_rel_q;
        diagnostic_snapshot_q[1231:1216] <= msp_bkt_b_sum_rel_q;
        diagnostic_snapshot_q[1247:1232] <= msp_bkt_b_xor_rel_q;
        diagnostic_snapshot_q[1263:1248] <= msp_bkt_c_sum_rel_q;
        diagnostic_snapshot_q[1279:1264] <= msp_bkt_c_xor_rel_q;
        // Page A7 5A: bucket C sub-split.
        diagnostic_snapshot_q[1287:1280] <= 8'ha7;
        diagnostic_snapshot_q[1295:1288] <= 8'h5a;
        diagnostic_snapshot_q[1311:1296] <= msp_bkt_c0_sum_rel_q;
        diagnostic_snapshot_q[1327:1312] <= msp_bkt_c7b_sum_rel_q;
        diagnostic_snapshot_q[1343:1328] <= msp_bkt_c7b_xor_rel_q;
        diagnostic_snapshot_q[1359:1344] <= msp_bkt_c79_sum_rel_q;
        diagnostic_snapshot_q[1375:1360] <= msp_bkt_c79_xor_rel_q;
        diagnostic_snapshot_q[1391:1376] <= msp_bkt_cff_sum_rel_q;
        diagnostic_snapshot_q[1407:1392] <= msp_bkt_cff_xor_rel_q;
        // Page A7 5B: MSP host access statistics and interrupt counts.
        diagnostic_snapshot_q[1415:1408] <= 8'ha7;
        diagnostic_snapshot_q[1423:1416] <= 8'h5b;
        diagnostic_snapshot_q[1439:1424] <= msp_host_read_count_q;
        diagnostic_snapshot_q[1455:1440] <= msp_hstctl_write_count_q;
        diagnostic_snapshot_q[1471:1456] <= msp_hstadr_write_count_q;
        diagnostic_snapshot_q[1487:1472] <= msp_first_hstctl_read_q;
        diagnostic_snapshot_q[1503:1488] <= msp_hint_count_q;
        diagnostic_snapshot_q[1519:1504] <= gsp_hint_count_q;
        diagnostic_snapshot_q[1535:1520] <= main_irq_ack_count_q;
        diagnostic_snapshot_q[1543:1536] <= 8'ha7;
        diagnostic_snapshot_q[1551:1544] <= 8'h5c;
        diagnostic_snapshot_q[1575:1552] <= pc_samples_q[0];
        diagnostic_snapshot_q[1599:1576] <= pc_samples_q[1];
        diagnostic_snapshot_q[1623:1600] <= pc_samples_q[2];
        diagnostic_snapshot_q[1647:1624] <= pc_samples_q[3];
        diagnostic_snapshot_q[1663:1648] <= {11'd0, pc_sample_index_q};
        diagnostic_snapshot_q[1671:1664] <= 8'ha7;
        diagnostic_snapshot_q[1679:1672] <= 8'h5d;
        diagnostic_snapshot_q[1703:1680] <= pc_samples_q[4];
        diagnostic_snapshot_q[1727:1704] <= pc_samples_q[5];
        diagnostic_snapshot_q[1751:1728] <= pc_samples_q[6];
        diagnostic_snapshot_q[1775:1752] <= pc_samples_q[7];
        diagnostic_snapshot_q[1791:1776] <= {11'd0, pc_sample_index_q};
        diagnostic_snapshot_q[1799:1792] <= 8'ha7;
        diagnostic_snapshot_q[1807:1800] <= 8'h5e;
        diagnostic_snapshot_q[1831:1808] <= pc_samples_q[8];
        diagnostic_snapshot_q[1855:1832] <= pc_samples_q[9];
        diagnostic_snapshot_q[1879:1856] <= pc_samples_q[10];
        diagnostic_snapshot_q[1903:1880] <= pc_samples_q[11];
        diagnostic_snapshot_q[1919:1904] <= {11'd0, pc_sample_index_q};
        diagnostic_snapshot_q[1927:1920] <= 8'ha7;
        diagnostic_snapshot_q[1935:1928] <= 8'h5f;
        diagnostic_snapshot_q[1959:1936] <= pc_samples_q[12];
        diagnostic_snapshot_q[1983:1960] <= pc_samples_q[13];
        diagnostic_snapshot_q[2007:1984] <= pc_samples_q[14];
        diagnostic_snapshot_q[2031:2008] <= pc_samples_q[15];
        diagnostic_snapshot_q[2047:2032] <= {11'd0, pc_sample_index_q};
        diagnostic_snapshot_q[2055:2048] <= 8'ha7;
        diagnostic_snapshot_q[2063:2056] <= 8'h60;
        diagnostic_snapshot_q[2087:2064] <= pc_samples_q[16];
        diagnostic_snapshot_q[2111:2088] <= pc_samples_q[17];
        diagnostic_snapshot_q[2135:2112] <= pc_samples_q[18];
        diagnostic_snapshot_q[2159:2136] <= pc_samples_q[19];
        diagnostic_snapshot_q[2175:2160] <= {11'd0, pc_sample_index_q};
        diagnostic_snapshot_q[2183:2176] <= 8'ha7;
        diagnostic_snapshot_q[2191:2184] <= 8'h61;
        diagnostic_snapshot_q[2215:2192] <= pc_samples_q[20];
        diagnostic_snapshot_q[2239:2216] <= pc_samples_q[21];
        diagnostic_snapshot_q[2263:2240] <= pc_samples_q[22];
        diagnostic_snapshot_q[2287:2264] <= pc_samples_q[23];
        diagnostic_snapshot_q[2303:2288] <= {11'd0, pc_sample_index_q};
        diagnostic_snapshot_q[2311:2304] <= 8'ha7;
        diagnostic_snapshot_q[2319:2312] <= 8'h62;
        diagnostic_snapshot_q[2343:2320] <= pc_samples_q[24];
        diagnostic_snapshot_q[2367:2344] <= pc_samples_q[25];
        diagnostic_snapshot_q[2391:2368] <= pc_samples_q[26];
        diagnostic_snapshot_q[2415:2392] <= pc_samples_q[27];
        diagnostic_snapshot_q[2431:2416] <= {11'd0, pc_sample_index_q};
        // Page A7 63: MSP fetch history at the first illegal opcode.
        diagnostic_snapshot_q[2439:2432] <= 8'ha7;
        diagnostic_snapshot_q[2447:2440] <= 8'h63;
        diagnostic_snapshot_q[2479:2448] <= msp_iaq_at_illegal_q[3];
        diagnostic_snapshot_q[2511:2480] <= msp_iaq_at_illegal_q[2];
        diagnostic_snapshot_q[2543:2512] <= msp_iaq_at_illegal_q[1];
        diagnostic_snapshot_q[2559:2544] <= msp_iaq_at_illegal_q[0][15:0];
        // Page A7 64: post-release 68010 -> MSP host traffic.
        diagnostic_snapshot_q[2567:2560] <= 8'ha7;
        diagnostic_snapshot_q[2575:2568] <= 8'h64;
        diagnostic_snapshot_q[2591:2576] <= msp_post_data_count_q;
        diagnostic_snapshot_q[2607:2592] <= msp_post_adr_count_q;
        diagnostic_snapshot_q[2623:2608] <= msp_last_hstadrh_q;
        diagnostic_snapshot_q[2639:2624] <= msp_last_hstadrl_q;
        diagnostic_snapshot_q[2655:2640] <= msp_iaq_at_illegal_q[0][31:16];
        diagnostic_snapshot_q[2687:2656] <= 32'd0;
        // Page A7 65: first seven host HSTCTL reads after release.
        diagnostic_snapshot_q[2695:2688] <= 8'ha7;
        diagnostic_snapshot_q[2703:2696] <= 8'h65;
        diagnostic_snapshot_q[2719:2704] <= msp_hstctl_reads_q[0];
        diagnostic_snapshot_q[2735:2720] <= msp_hstctl_reads_q[1];
        diagnostic_snapshot_q[2751:2736] <= msp_hstctl_reads_q[2];
        diagnostic_snapshot_q[2767:2752] <= msp_hstctl_reads_q[3];
        diagnostic_snapshot_q[2783:2768] <= msp_hstctl_reads_q[4];
        diagnostic_snapshot_q[2799:2784] <= msp_hstctl_reads_q[5];
        diagnostic_snapshot_q[2815:2800] <= msp_hstctl_reads_q[6];
        // Page A7 66: first seven MSP HSTCTLL writes after release.
        diagnostic_snapshot_q[2823:2816] <= 8'ha7;
        diagnostic_snapshot_q[2831:2824] <= 8'h66;
        diagnostic_snapshot_q[2847:2832] <= msp_hstctll_writes_q[0];
        diagnostic_snapshot_q[2863:2848] <= msp_hstctll_writes_q[1];
        diagnostic_snapshot_q[2879:2864] <= msp_hstctll_writes_q[2];
        diagnostic_snapshot_q[2895:2880] <= msp_hstctll_writes_q[3];
        diagnostic_snapshot_q[2911:2896] <= msp_hstctll_writes_q[4];
        diagnostic_snapshot_q[2927:2912] <= msp_hstctll_writes_q[5];
        diagnostic_snapshot_q[2943:2928] <= msp_hstctll_writes_q[6];
        // Page A7 67: handler entries per level + periodic IRQ assertions.
        diagnostic_snapshot_q[2951:2944] <= 8'ha7;
        diagnostic_snapshot_q[2959:2952] <= 8'h67;
        diagnostic_snapshot_q[2975:2960] <= irq_l1_entries_q;
        diagnostic_snapshot_q[2991:2976] <= irq_l2_entries_q;
        diagnostic_snapshot_q[3007:2992] <= irq_l3_entries_q;
        diagnostic_snapshot_q[3023:3008] <= irq_l4_entries_q;
        diagnostic_snapshot_q[3039:3024] <= irq_l5_entries_q;
        diagnostic_snapshot_q[3055:3040] <= irq_l6_entries_q;
        diagnostic_snapshot_q[3071:3056] <= main_irq_count_q[15:0];
        // Page A7 68: 68010 RAM variables (last written values, write counts).
        diagnostic_snapshot_q[3079:3072] <= 8'ha7;
        diagnostic_snapshot_q[3087:3080] <= 8'h68;
        diagnostic_snapshot_q[3103:3088] <= wr_ff9136_last_q;
        diagnostic_snapshot_q[3119:3104] <= wr_ff9136_count_q;
        diagnostic_snapshot_q[3135:3120] <= wr_ffff8016_last_q;
        diagnostic_snapshot_q[3151:3136] <= wr_ffff8016_count_q;
        diagnostic_snapshot_q[3167:3152] <= wr_ff968c_last_q;
        diagnostic_snapshot_q[3183:3168] <= wr_ff968c_count_q;
        diagnostic_snapshot_q[3199:3184] <= wr_60c000_count_q;
        // Page A7 69: boot timeline (ms since reset).
        diagnostic_snapshot_q[3207:3200] <= 8'ha7;
        diagnostic_snapshot_q[3215:3208] <= 8'h69;
        diagnostic_snapshot_q[3231:3216] <= t_first_l5_q;
        diagnostic_snapshot_q[3247:3232] <= t_first_frame_q;
        diagnostic_snapshot_q[3263:3248] <= t_msp_release_q;
        diagnostic_snapshot_q[3279:3264] <= t_gsp_release_q;
        diagnostic_snapshot_q[3295:3280] <= t_first_msp_hint_q;
        diagnostic_snapshot_q[3311:3296] <= t_first_adsp_irq_q;
        diagnostic_snapshot_q[3327:3312] <= t_error_display_q;
        // Page A7 6A: GSP PC samples 0..3.
        diagnostic_snapshot_q[3335:3328] <= 8'ha7;
        diagnostic_snapshot_q[3343:3336] <= 8'h6a;
        diagnostic_snapshot_q[3367:3344] <= gsp_pc_samples_q[0];
        diagnostic_snapshot_q[3391:3368] <= gsp_pc_samples_q[1];
        diagnostic_snapshot_q[3415:3392] <= gsp_pc_samples_q[2];
        diagnostic_snapshot_q[3439:3416] <= gsp_pc_samples_q[3];
        diagnostic_snapshot_q[3455:3440] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 6B: GSP PC samples 4..7.
        diagnostic_snapshot_q[3463:3456] <= 8'ha7;
        diagnostic_snapshot_q[3471:3464] <= 8'h6b;
        diagnostic_snapshot_q[3495:3472] <= gsp_pc_samples_q[4];
        diagnostic_snapshot_q[3519:3496] <= gsp_pc_samples_q[5];
        diagnostic_snapshot_q[3543:3520] <= gsp_pc_samples_q[6];
        diagnostic_snapshot_q[3567:3544] <= gsp_pc_samples_q[7];
        diagnostic_snapshot_q[3583:3568] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 6C: GSP PC samples 8..11.
        diagnostic_snapshot_q[3591:3584] <= 8'ha7;
        diagnostic_snapshot_q[3599:3592] <= 8'h6c;
        diagnostic_snapshot_q[3623:3600] <= gsp_pc_samples_q[8];
        diagnostic_snapshot_q[3647:3624] <= gsp_pc_samples_q[9];
        diagnostic_snapshot_q[3671:3648] <= gsp_pc_samples_q[10];
        diagnostic_snapshot_q[3695:3672] <= gsp_pc_samples_q[11];
        diagnostic_snapshot_q[3711:3696] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 6D: GSP PC samples 12..15.
        diagnostic_snapshot_q[3719:3712] <= 8'ha7;
        diagnostic_snapshot_q[3727:3720] <= 8'h6d;
        diagnostic_snapshot_q[3751:3728] <= gsp_pc_samples_q[12];
        diagnostic_snapshot_q[3775:3752] <= gsp_pc_samples_q[13];
        diagnostic_snapshot_q[3799:3776] <= gsp_pc_samples_q[14];
        diagnostic_snapshot_q[3823:3800] <= gsp_pc_samples_q[15];
        diagnostic_snapshot_q[3839:3824] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 6E: GSP PC samples 16..19.
        diagnostic_snapshot_q[3847:3840] <= 8'ha7;
        diagnostic_snapshot_q[3855:3848] <= 8'h6e;
        diagnostic_snapshot_q[3879:3856] <= gsp_pc_samples_q[16];
        diagnostic_snapshot_q[3903:3880] <= gsp_pc_samples_q[17];
        diagnostic_snapshot_q[3927:3904] <= gsp_pc_samples_q[18];
        diagnostic_snapshot_q[3951:3928] <= gsp_pc_samples_q[19];
        diagnostic_snapshot_q[3967:3952] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 6F: GSP PC samples 20..23.
        diagnostic_snapshot_q[3975:3968] <= 8'ha7;
        diagnostic_snapshot_q[3983:3976] <= 8'h6f;
        diagnostic_snapshot_q[4007:3984] <= gsp_pc_samples_q[20];
        diagnostic_snapshot_q[4031:4008] <= gsp_pc_samples_q[21];
        diagnostic_snapshot_q[4055:4032] <= gsp_pc_samples_q[22];
        diagnostic_snapshot_q[4079:4056] <= gsp_pc_samples_q[23];
        diagnostic_snapshot_q[4095:4080] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 70: GSP PC samples 24..27.
        diagnostic_snapshot_q[4103:4096] <= 8'ha7;
        diagnostic_snapshot_q[4111:4104] <= 8'h70;
        diagnostic_snapshot_q[4135:4112] <= gsp_pc_samples_q[24];
        diagnostic_snapshot_q[4159:4136] <= gsp_pc_samples_q[25];
        diagnostic_snapshot_q[4183:4160] <= gsp_pc_samples_q[26];
        diagnostic_snapshot_q[4207:4184] <= gsp_pc_samples_q[27];
        diagnostic_snapshot_q[4223:4208] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 71: GSP PC samples 28..31.
        diagnostic_snapshot_q[4231:4224] <= 8'ha7;
        diagnostic_snapshot_q[4239:4232] <= 8'h71;
        diagnostic_snapshot_q[4263:4240] <= gsp_pc_samples_q[28];
        diagnostic_snapshot_q[4287:4264] <= gsp_pc_samples_q[29];
        diagnostic_snapshot_q[4311:4288] <= gsp_pc_samples_q[30];
        diagnostic_snapshot_q[4335:4312] <= gsp_pc_samples_q[31];
        diagnostic_snapshot_q[4351:4336] <= {11'd0, gsp_pc_sample_index_q};
        // Page A7 72: GSP runtime host-port statistics.
        diagnostic_snapshot_q[4359:4352] <= 8'ha7;
        diagnostic_snapshot_q[4367:4360] <= 8'h72;
        diagnostic_snapshot_q[4383:4368] <= gsp_hint_gap_max_q;
        diagnostic_snapshot_q[4399:4384] <= gsp_di_vec_q;
        diagnostic_snapshot_q[4415:4400] <= gsp_dpyadr_wr_q;
        diagnostic_snapshot_q[4431:4416] <= gsp_pblt_done_q;
        diagnostic_snapshot_q[4447:4432] <= main_timeout_entries_q;
        diagnostic_snapshot_q[4463:4448] <= gsp_rt_hstdata_writes_q;
        diagnostic_snapshot_q[4479:4464] <= gsp_rt_last_hstctl_write_q;
        // Pages A7 73/74: VRAM row samples (scan-out) rows 400 / 250, columns 0x40..0x46.
        diagnostic_snapshot_q[4487:4480] <= 8'ha7;
        diagnostic_snapshot_q[4495:4488] <= 8'h73;
        diagnostic_snapshot_q[4511:4496] <= vram_row400_q[0];
        diagnostic_snapshot_q[4527:4512] <= vram_row400_q[1];
        diagnostic_snapshot_q[4543:4528] <= vram_row400_q[2];
        diagnostic_snapshot_q[4559:4544] <= vram_row400_q[3];
        diagnostic_snapshot_q[4575:4560] <= vram_row400_q[4];
        diagnostic_snapshot_q[4591:4576] <= vram_row400_q[5];
        diagnostic_snapshot_q[4607:4592] <= vram_row400_q[6];
        diagnostic_snapshot_q[4615:4608] <= 8'ha7;
        diagnostic_snapshot_q[4623:4616] <= 8'h74;
        diagnostic_snapshot_q[4639:4624] <= vram_row250_q[0];
        diagnostic_snapshot_q[4655:4640] <= vram_row250_q[1];
        diagnostic_snapshot_q[4671:4656] <= vram_row250_q[2];
        diagnostic_snapshot_q[4687:4672] <= vram_row250_q[3];
        diagnostic_snapshot_q[4703:4688] <= vram_row250_q[4];
        diagnostic_snapshot_q[4719:4704] <= vram_row250_q[5];
        diagnostic_snapshot_q[4735:4720] <= vram_row250_q[6];
        // Page A7 75: palette lookups at (row 400, x 128) and (row 250, x 128).
        diagnostic_snapshot_q[4743:4736] <= 8'ha7;
        diagnostic_snapshot_q[4751:4744] <= 8'h75;
        diagnostic_snapshot_q[4767:4752] <= pal_r400_addr_q;
        diagnostic_snapshot_q[4783:4768] <= pal_r400_lo_q;
        diagnostic_snapshot_q[4799:4784] <= pal_r400_hi_q;
        diagnostic_snapshot_q[4815:4800] <= pal_bank_r400_q;
        diagnostic_snapshot_q[4831:4816] <= pal_r250_addr_q;
        diagnostic_snapshot_q[4847:4832] <= pal_r250_lo_q;
        diagnostic_snapshot_q[4863:4848] <= pal_r250_hi_q;
        // Page A7 76: DI vector fetch raster lines + copy engine counters.
        diagnostic_snapshot_q[4871:4864] <= 8'ha7;
        diagnostic_snapshot_q[4879:4872] <= 8'h76;
        diagnostic_snapshot_q[4895:4880] <= di_bot_max_line_q;
        diagnostic_snapshot_q[4911:4896] <= di_bot_last_line_q;
        diagnostic_snapshot_q[4927:4912] <= di_bot_late_q;
        diagnostic_snapshot_q[4943:4928] <= di_top_max_line_q;
        diagnostic_snapshot_q[4959:4944] <= gsp_srt_copy_count;
        diagnostic_snapshot_q[4975:4960] <= gsp_srt_pending_cycles_q;
        diagnostic_snapshot_q[4991:4976] <= 16'd0;
        if (!DIAG_FULL) begin   // later assignments win: the dropped pages stay zero
          diagnostic_snapshot_q[767:640] <= 128'd0;
          diagnostic_snapshot_q[895:768] <= 128'd0;
          diagnostic_snapshot_q[1023:896] <= 128'd0;
          diagnostic_snapshot_q[1151:1024] <= 128'd0;
          diagnostic_snapshot_q[1279:1152] <= 128'd0;
          diagnostic_snapshot_q[1407:1280] <= 128'd0;
          diagnostic_snapshot_q[1535:1408] <= 128'd0;
          diagnostic_snapshot_q[1663:1536] <= 128'd0;
          diagnostic_snapshot_q[1791:1664] <= 128'd0;
          diagnostic_snapshot_q[1919:1792] <= 128'd0;
          diagnostic_snapshot_q[2047:1920] <= 128'd0;
          diagnostic_snapshot_q[2175:2048] <= 128'd0;
          diagnostic_snapshot_q[2303:2176] <= 128'd0;
          diagnostic_snapshot_q[2431:2304] <= 128'd0;
          diagnostic_snapshot_q[2559:2432] <= 128'd0;
          diagnostic_snapshot_q[2687:2560] <= 128'd0;
          diagnostic_snapshot_q[2815:2688] <= 128'd0;
          diagnostic_snapshot_q[2943:2816] <= 128'd0;
          diagnostic_snapshot_q[3071:2944] <= 128'd0;
          diagnostic_snapshot_q[3199:3072] <= 128'd0;
          diagnostic_snapshot_q[3327:3200] <= 128'd0;
          diagnostic_snapshot_q[3455:3328] <= 128'd0;
          diagnostic_snapshot_q[3583:3456] <= 128'd0;
          diagnostic_snapshot_q[3711:3584] <= 128'd0;
          diagnostic_snapshot_q[3839:3712] <= 128'd0;
          diagnostic_snapshot_q[3967:3840] <= 128'd0;
          diagnostic_snapshot_q[4095:3968] <= 128'd0;
          diagnostic_snapshot_q[4223:4096] <= 128'd0;
          diagnostic_snapshot_q[4351:4224] <= 128'd0;
          diagnostic_snapshot_q[4607:4480] <= 128'd0;
          diagnostic_snapshot_q[4735:4608] <= 128'd0;
          diagnostic_snapshot_q[4863:4736] <= 128'd0;
        end
      end
    end
  end

`ifdef HD_LEAN
  // Lean builds (sound revisions at 93 % fill): no developer bit pages, so
  // the 4992-bit snapshot, its 39:1 page mux and the barcode overlay are
  // pruned; the OSD "Diagnostic overlay" entry then does nothing.
  always_comb diagnostic_page = 128'd0;
`else
  always_comb begin
    unique case (diagnostic_page_index_q)
      6'd0: diagnostic_page = diagnostic_snapshot_q[127:0];
      6'd1: diagnostic_page = diagnostic_snapshot_q[255:128];
      6'd2: diagnostic_page = diagnostic_snapshot_q[383:256];
      6'd3: diagnostic_page = diagnostic_snapshot_q[511:384];
      6'd4: diagnostic_page = diagnostic_snapshot_q[639:512];
      6'd5: diagnostic_page = diagnostic_snapshot_q[767:640];
      6'd6: diagnostic_page = diagnostic_snapshot_q[895:768];
      6'd7: diagnostic_page = diagnostic_snapshot_q[1023:896];
      6'd8: diagnostic_page = diagnostic_snapshot_q[1151:1024];
      6'd9: diagnostic_page = diagnostic_snapshot_q[1279:1152];
      6'd10: diagnostic_page = diagnostic_snapshot_q[1407:1280];
      6'd11: diagnostic_page = diagnostic_snapshot_q[1535:1408];
      6'd12: diagnostic_page = diagnostic_snapshot_q[1663:1536];
      6'd13: diagnostic_page = diagnostic_snapshot_q[1791:1664];
      6'd14: diagnostic_page = diagnostic_snapshot_q[1919:1792];
      6'd15: diagnostic_page = diagnostic_snapshot_q[2047:1920];
      6'd16: diagnostic_page = diagnostic_snapshot_q[2175:2048];
      6'd17: diagnostic_page = diagnostic_snapshot_q[2303:2176];
      6'd18: diagnostic_page = diagnostic_snapshot_q[2431:2304];
      6'd19: diagnostic_page = diagnostic_snapshot_q[2559:2432];
      6'd20: diagnostic_page = diagnostic_snapshot_q[2687:2560];
      6'd21: diagnostic_page = diagnostic_snapshot_q[2815:2688];
      6'd22: diagnostic_page = diagnostic_snapshot_q[2943:2816];
      6'd23: diagnostic_page = diagnostic_snapshot_q[3071:2944];
      6'd24: diagnostic_page = diagnostic_snapshot_q[3199:3072];
      6'd25: diagnostic_page = diagnostic_snapshot_q[3327:3200];
      6'd26: diagnostic_page = diagnostic_snapshot_q[3455:3328];
      6'd27: diagnostic_page = diagnostic_snapshot_q[3583:3456];
      6'd28: diagnostic_page = diagnostic_snapshot_q[3711:3584];
      6'd29: diagnostic_page = diagnostic_snapshot_q[3839:3712];
      6'd30: diagnostic_page = diagnostic_snapshot_q[3967:3840];
      6'd31: diagnostic_page = diagnostic_snapshot_q[4095:3968];
      6'd32: diagnostic_page = diagnostic_snapshot_q[4223:4096];
      6'd33: diagnostic_page = diagnostic_snapshot_q[4351:4224];
      6'd34: diagnostic_page = diagnostic_snapshot_q[4479:4352];
      6'd35: diagnostic_page = diagnostic_snapshot_q[4607:4480];
      6'd36: diagnostic_page = diagnostic_snapshot_q[4735:4608];
      6'd37: diagnostic_page = diagnostic_snapshot_q[4863:4736];
      default: diagnostic_page = diagnostic_snapshot_q[4991:4864];
    endcase
  end
`endif

  // Controls debug overlay: the values the ROM reads, sound-board state,
  // and a sticky GSP timeout snapshot packed for the scan-out's panel.
  logic [149:0] controls_bus;
  logic        snd_dsp_run;
  logic [15:0] snd_last_cmd;
  logic        snd_cramen;
  logic        snd_comram_write;
  logic        snd_comram_act_q;
  logic [7:0]  snd_ser_byte;
  logic [7:0]  snd_dac_peak;
  logic [7:0]  snd_comram_count;
  logic        snd_dsp_irq_seen;
  logic [7:0]  snd_cpu_pc_page;
  logic [11:0] snd_dac;
  logic        snd_dac_write;
  logic        snd_dac_act_q;
  logic [21:0] snd_dac_act_div_q;
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      snd_dac_act_q     <= 1'b0;
      snd_dac_act_div_q <= 22'd0;
    end else if (snd_dac_write) begin
      // Blink while the DSP is writing samples (about 5 Hz at 20 kHz).
      if (snd_dac_act_div_q == 22'd2000) begin
        snd_dac_act_div_q <= 22'd0;
        snd_dac_act_q     <= !snd_dac_act_q;
      end else begin
        snd_dac_act_div_q <= snd_dac_act_div_q + 22'd1;
      end
    end
  end
  // Blink on every main-CPU access to the sound data latch, so a game that
  // never talks to the sound board is visible at a glance.
  logic snd_main_wr_act_q, snd_main_rd_act_q;
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      snd_main_wr_act_q <= 1'b0;
      snd_main_rd_act_q <= 1'b0;
    end else begin
      if (snd_wr) snd_main_wr_act_q <= !snd_main_wr_act_q;
      if (snd_rd) snd_main_rd_act_q <= !snd_main_rd_act_q;
    end
  end
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) snd_comram_act_q <= 1'b0;
    else if (snd_comram_write) snd_comram_act_q <= !snd_comram_act_q;
  end
  // Live controls/sound telemetry occupies bits 44..119 during normal play.
  // A fatal GSP timeout replaces only that region with the sticky snapshot;
  // the higher sticky sound evidence remains intact. Keeping the fields at
  // explicit bit positions avoids the old concat's accidental shifts.
  always_comb begin
    controls_bus = 150'd0;
    controls_bus[11:0] = steering_counter;
    controls_bus[19:12] = accelerator_i;
    controls_bus[27:20] = brake_i;
    controls_bus[35:28] = clutch_i;
    controls_bus[39:36] = compact_port1_i[11:8];
    controls_bus[40] = !compact_port1_i[1];
    controls_bus[41] = !compact_port1_i[0];
    controls_bus[42] = coin1_i;
    controls_bus[43] = service_i;
    controls_bus[75:44] = analog_axes_i;
    controls_bus[76] = snd_dsp_run;
    controls_bus[77] = snd_mainflag;
    controls_bus[78] = snd_soundflag;
    controls_bus[79] = snd_dac_act_q;
    controls_bus[91:80] = snd_dac;
    controls_bus[92] = snd_main_wr_act_q;
    controls_bus[93] = snd_main_rd_act_q;
    controls_bus[109:94] = snd_last_cmd;
    controls_bus[110] = snd_comram_act_q;
    controls_bus[111] = snd_cramen;
    controls_bus[119:112] = snd_ser_byte;
    controls_bus[127:120] = snd_dac_peak;
    controls_bus[135:128] = snd_comram_count;
    controls_bus[136] = snd_dsp_irq_seen;
    controls_bus[140:137] = axis_moved_i;
    controls_bus[148:141] = snd_cpu_pc_page;
    controls_bus[149] = gsp_timeout_snapshot_valid_q;
    if (gsp_timeout_snapshot_valid_q) begin
      controls_bus[67:44] = gsp_timeout_pc_q;
      controls_bus[73:68] = gsp_timeout_state_q;
      controls_bus[89:74] = gsp_timeout_instr_q;
      controls_bus[105:90] = gsp_timeout_hint_age_q;
      controls_bus[106] = gsp_timeout_illegal_q;
    end
  end

  harddrivin_compact_scanout #(.COCKPIT(COCKPIT), .VRAM_AW(VRAM_AW)) u_scanout (
    .core_clk_i(clk_gsp), .core_rst_i(tms_gsp_rst),
    .screen_req_i(gsp_screen_cycle),
    .screen_srfaddr_i(gsp_cycle_srfaddr),
    .screen_dpytap_i(gsp_cycle_dpytap),
    .screen_lcstrt_i(gsp_dpystart_q[1:0]),
    .screen_org_i(gsp_cycle_screen_org), .screen_ack_o(gsp_screen_ack),
    .mem_clk_i(clk_100), .mem_rst_i(traffic_reset),
    .scan_req_o(scan_req), .scan_addr_o(scan_addr),
    .scan_ack_i(scan_ack), .scan_data_i(scan_data),
    .scan_overflow_i(scan_overflow),
    .video_hsync_i(gsp_video_hsync), .video_vsync_i(gsp_video_vsync),
    .video_hblank_i(gsp_video_hblank), .video_vblank_i(gsp_video_vblank),
    .video_blank_i(gsp_video_blank), .video_enable_i(gsp_dpyctl_q[15]),
    .video_heblnk_i(gsp_heblnk_q),
    .fine_scroll_i(gsp_fine_scroll),
    .palette_bank_i(gsp_palette_bank),
    .diagnostic_enable_i(diagnostic_enable_i),
    .diagnostic_i(diagnostic_page),
`ifdef HD_VIDEO_DIAGNOSTICS
    .video_debug_i(video_debug_q),
`endif
    .controls_enable_i(controls_overlay_i), .controls_i(controls_bus),
    .palette_addr_o(gsp_palette_scan_addr),
    .palette_lo_i(gsp_palette_scan_lo), .palette_hi_i(gsp_palette_scan_hi),
    .ce_pix_o(ce_pix_o), .hsync_o(hsync_o), .vsync_o(vsync_o),
    .hblank_o(hblank_o), .vblank_o(vblank_o), .rgb_o(rgb_o),
    .frame_start_o(frame_start), .refresh_count_o(screen_refresh_count),
    .row_count_o(screen_row_count),
    .loaded_srfaddr_o(screen_loaded_srfaddr),
    .line_o(scan_line_dbg), .x_o(scan_x_dbg),
    .loaded_org_o(screen_loaded_org), .loaded_col_o(screen_loaded_col),
    .loaded_row_select_o(screen_loaded_row_select),
    .loaded_row_or_o(screen_loaded_row_or),
    .loaded_row_nonzero_o(screen_loaded_row_nonzero),
    .row_valid_o(screen_row_valid)
  );

  // the GSP's VRAM is SDRAM (self-refreshed by its controller): no DRAM
  // refresh cycles on its local bus
  tms34010_probe #(.DATA_SEED(16'h3401), .FAST_BLIT(1'b1), .DRAM_REFRESH(1'b0)) u_gsp (
    .clk_i(clk_gsp), .vclk_i(clk_video), .rst_i(tms_gsp_rst),
    .vclk_rst_i(rst_video || !gsp_reset_release),
    .hsync_i(1'b0), .vsync_i(1'b0),
    .host_req_i(gsp_host_req), .host_we_i(gsp_host_we),
    .host_reg_i(gsp_host_reg), .host_be_i(gsp_host_be),
    .host_wdata_i(gsp_host_wdata), .host_rdata_o(gsp_host_rdata),
    .host_ack_o(gsp_host_ack), .host_busy_o(gsp_host_busy),
    .hint_n_o(gsp_hint_n),
    .pc_o(gsp_pc), .state_o(gsp_state), .cycles_o(gsp_cycles),
    .instr_word_o(gsp_instr_word), .hstctll_o(gsp_hstctll),
    .illegal_o(gsp_illegal),
    .video_hsync_o(gsp_video_hsync), .video_vsync_o(gsp_video_vsync),
    .video_hblank_o(gsp_video_hblank), .video_vblank_o(gsp_video_vblank),
    .video_blank_o(gsp_video_blank),
    .cycle_req_o(gsp_cycle_req), .cycle_kind_o(gsp_cycle_kind),
    .cycle_addr_o(gsp_cycle_addr), .cycle_wdata_o(gsp_cycle_wdata),
    .cycle_io_rdata_o(gsp_cycle_io_rdata), .cycle_iaq_o(gsp_cycle_iaq),
    .cycle_srfaddr_o(gsp_cycle_srfaddr),
    .cycle_dpytap_o(gsp_cycle_dpytap),
    .cycle_screen_org_o(gsp_cycle_screen_org),
    .cycle_dram_row_o(gsp_cycle_dram_row),
    .cycle_rdata_i(gsp_cycle_rdata), .cycle_ack_i(gsp_cycle_ack)
  );
  // The MSP never executes FILL/PIXBLT/DRAV/LINE (tb_harddrivin_msp_run
  // histogram); drop that datapath from this 50 MHz instance (Task 0180).
  // Task 0188 note: the MSP keeps its display subsystem (VIDEO=1).  Building
  // it with VIDEO=0 saved ~150 ALMs but changed on-hardware behaviour (car
  // pulling, dashboard colours), so the MSP program is assumed to use the
  // display counters/DPYINT as a time base.
  tms34010_probe #(.DATA_SEED(16'h5a2c), .PIXEL_OPS(1'b0)) u_msp (
    .clk_i(clk_msp), .vclk_i(clk_msp), .rst_i(tms_msp_rst),
    .vclk_rst_i(tms_msp_rst), .hsync_i(1'b0), .vsync_i(1'b0),
    .host_req_i(msp_host_req), .host_we_i(msp_host_we),
    .host_reg_i(msp_host_reg), .host_be_i(msp_host_be),
    .host_wdata_i(msp_host_wdata), .host_rdata_o(msp_host_rdata),
    .host_ack_o(msp_host_ack), .host_busy_o(msp_host_busy),
    .hint_n_o(msp_hint_n),
    .pc_o(msp_pc), .state_o(msp_state), .cycles_o(msp_cycles),
    .instr_word_o(msp_instr_word), .hstctll_o(msp_hstctll),
    .illegal_o(msp_illegal),
    .video_hsync_o(msp_video_hsync), .video_vsync_o(msp_video_vsync),
    .video_hblank_o(msp_video_hblank), .video_vblank_o(msp_video_vblank),
    .video_blank_o(msp_video_blank),
    .cycle_req_o(msp_cycle_req), .cycle_kind_o(msp_cycle_kind),
    .cycle_addr_o(msp_cycle_addr), .cycle_wdata_o(msp_cycle_wdata),
    .cycle_io_rdata_o(msp_cycle_io_rdata), .cycle_iaq_o(msp_cycle_iaq),
    .cycle_srfaddr_o(msp_cycle_srfaddr),
    .cycle_dpytap_o(msp_cycle_dpytap),
    .cycle_screen_org_o(msp_cycle_screen_org),
    .cycle_dram_row_o(msp_cycle_dram_row),
    .cycle_rdata_i(msp_cycle_rdata), .cycle_ack_i(msp_cycle_ack)
  );

  // Main 68010 candidate. CPU=01 selects TG68K's 68010 behavior.
  logic [31:0] main_addr;
  logic [15:0] main_data_out;
  logic [15:0] main_data_in;
  logic [31:0] main_regin;
  logic [1:0] main_busstate;
  logic [1:0] main_busstate_effective;
  logic main_nwr, main_nuds, main_nlds;
  logic main_skip_fetch;
  logic main_cycle;
  logic main_bus_ready;
  logic [19:1] main_rom_addr;
  logic main_rom_req, main_rom_ack;
  logic [15:0] main_rom_data;
  logic [23:0] main_last_fetch_addr;
  logic [23:0] main_last_read_addr;
  logic [23:0] main_last_write_addr;
  logic [31:0] main_fetch_count;
  logic [31:0] main_read_count, main_write_count;
  logic [17:0] main_irq_div_q;
  logic        main_periodic_irq_q;
  logic [31:0] main_irq_count_q;
  logic [2:0]  main_ipl_n;
  logic [3:0] cpu_div;

  // Export the ROM-observed startup calibration windows separately.  This is
  // development instrumentation: it lets the MiSTer top exercise the same
  // pot, brake, shifter, seat, and steering sequence as the cabinet without
  // changing the board-facing bus implementation.
  assign pot_prompt_active_o = (main_last_fetch_addr[23:8] == 16'h01a3);
  assign brake_calibration_active_o =
    (main_last_fetch_addr[23:8] == 16'h0193);
  assign shifter_calibration_active_o =
    (main_last_fetch_addr[23:8] == 16'h0197);
  // Do not arm on the whole 0x019axx page: the preceding shifter routine
  // finishes at 0x019a00-0x019aa4 and deliberately waits before returning.
  // 0x019aa6 is the first instruction of the seat routine itself.
  assign seat_calibration_active_o =
    (main_last_fetch_addr == 24'h019aa6);
  assign calibration_active_o = (main_last_fetch_addr[23:8] == 16'h019d);

  // SP-327 sheet 3 divides the 32 MHz master by 16 four times and then
  // toggles a 74LS74, producing the periodic 68010 IRQ at 244.140625 Hz.
  // clk_gsp is exactly 48 MHz, hence one assertion every 196608 clocks.
  // /IRCLR is the asynchronous clear on the PCB; clear is dominant here too.
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) begin
      main_irq_div_q      <= 18'd0;
      main_periodic_irq_q <= 1'b0;
      main_irq_count_q    <= 32'd0;
    end else begin
      if (main_irq_div_q == 18'd196607) begin
        main_irq_div_q      <= 18'd0;
        main_periodic_irq_q <= 1'b1;
        main_irq_count_q    <= main_irq_count_q + 32'd1;
      end else begin
        main_irq_div_q <= main_irq_div_q + 18'd1;
      end
      if (main_irq_clear) main_periodic_irq_q <= 1'b0;
    end
  end

  // Populated Compact-board priorities from the 74S148 encoder: periodic
  // IRQ=5, GSP HINT=3, MSP HINT=1. IPL pins and the TG68 port are active-low
  // encoded, and these board interrupts are autovectored.
  always_comb begin
    if (main_periodic_irq_q) main_ipl_n = 3'b010;
    else if (!gsp_hint_n)    main_ipl_n = 3'b100;
    else if (adsp_irq)       main_ipl_n = 3'b101;
    else if (!msp_hint_n)    main_ipl_n = 3'b110;
    else                     main_ipl_n = 3'b111;
  end

  // 68010 pacing.  TG68K advances one internal/bus step per clkena.  With an
  // enable every 6 clocks (8 MHz) the boot timeline ran ~2.7x ahead of the
  // real machine (MAME: first periodic tick 1.3 s, MSP release 2.70 s, GSP
  // release 2.93 s; ours 0.60 / 1.00 / 1.06 s) and the game's bounded waits
  // shrank accordingly (the 0x80000-iteration GSP wait lasted ~1.2 s instead
  // of ~3.9 s).  An enable every 16 clocks (3 MHz) brings the timeline to
  // MAME's within ~5 %.  The periodic 244 Hz tick, MSP/GSP/ADSP are
  // independent of this divider.
  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) cpu_div <= 4'd0;
    else cpu_div <= (cpu_div == 4'd15) ? 4'd0 : cpu_div + 4'd1;
  end
  wire main_cpu_ce = (cpu_div == 4'd0) && main_bus_ready;

  // TG68K's kernel exposes skipFetch for cycles which exist internally but
  // must not reach the external bus.  Its standard asynchronous-bus wrapper
  // suppresses AS/UDS/LDS while this signal is asserted.  In particular, a
  // 68000 performs CLR as read-then-write, whereas the Compact board's 68010
  // performs only the write.  Present the suppressed phase as BUS_IDLE to
  // this completed-cycle adapter, preserving the kernel's internal progress
  // without issuing a side-effecting host-register read.
  assign main_busstate_effective = main_skip_fetch ? 2'b01 : main_busstate;

  always_ff @(posedge clk_gsp) begin
    if (rst_gsp) main_cycle <= 1'b0;
    else if (main_cpu_ce) main_cycle <= !main_cycle;
  end

  // Driver sound board latch traffic (the bus models the handshake itself
  // when the board is not built).
  logic        snd_wr, snd_rd, snd_reset;
  logic [15:0] snd_rdata;
  logic        snd_mainflag, snd_soundflag;
  logic [11:0] steering_counter;
`ifdef HD_SOUND_BOARD
  localparam bit MAIN_SOUND_STUB = 1'b0;
`else
  localparam bit MAIN_SOUND_STUB = 1'b1;
`endif

  harddrivin_main_bus #(
    .SOUND_STUB(MAIN_SOUND_STUB), .COCKPIT(COCKPIT),
    .ZERO_RAM_INIT_HI_FILE(ZERO_RAM_INIT_HI_FILE),
    .ZERO_RAM_INIT_LO_FILE(ZERO_RAM_INIT_LO_FILE)
  ) u_main_bus (
    .clk_i(clk_gsp), .rst_i(rst_gsp), .cycle_i(main_cycle),
    .addr_i(main_addr), .wdata_i(main_data_out),
    .busstate_i(main_busstate_effective),
    .nuds_i(main_nuds), .nlds_i(main_nlds),
    .rdata_o(main_data_in), .ready_o(main_bus_ready),
    .rom_addr_o(main_rom_addr), .rom_req_o(main_rom_req),
    .rom_ack_i(main_rom_ack), .rom_data_i(main_rom_data),
    .switch1_i(main_switch1), .sw1_i(sw1_i),
    .nvram_req_i(nvram_req_i), .nvram_we_i(nvram_we_i), .nvram_addr_i(nvram_addr_i),
    .nvram_wdata_i(nvram_wdata_i), .nvram_ack_o(nvram_ack_o),
    .nvram_rdata_o(nvram_rdata_o), .nvram_dirty_o(nvram_dirty_o),
    .snd_wr_o(snd_wr), .snd_rd_o(snd_rd), .snd_reset_o(snd_reset),
    .snd_rdata_i(snd_rdata), .snd_mainflag_i(snd_mainflag),
    .snd_soundflag_i(snd_soundflag),
    .compact_port1_i(compact_port1_i),
    .accelerator_i(accelerator_i), .clutch_i(clutch_i),
    .seat_i(seat_i), .shifter_x_i(shifter_x_i),
    .shifter_y_i(shifter_y_i),
    .brake_i(brake_i), .brake12_i(brake12_i), .steering_i(steering_i),
    .steering_counter_o(steering_counter),
    .watchdog_clear_o(main_watchdog_clear),
    .irq_clear_o(main_irq_clear),
    .control_latch_o(main_control_latch),
    .gsp_reset_release_o(gsp_reset_release),
    .msp_reset_release_o(msp_reset_release),
    .gsp_host_req_o(gsp_host_req), .gsp_host_we_o(gsp_host_we),
    .gsp_host_reg_o(gsp_host_reg), .gsp_host_be_o(gsp_host_be),
    .gsp_host_wdata_o(gsp_host_wdata), .gsp_host_rdata_i(gsp_host_rdata),
    .gsp_host_ack_i(gsp_host_ack),
    .adsp_req_o(adsp_host_req), .adsp_we_o(adsp_host_we),
    .adsp_addr_o(adsp_host_addr), .adsp_wdata_o(adsp_host_wdata),
    .adsp_rdata_i(adsp_host_rdata), .adsp_ack_i(adsp_host_ack),
    .msp_host_req_o(msp_host_src_req), .msp_host_we_o(msp_host_src_we),
    .msp_host_reg_o(msp_host_src_reg), .msp_host_be_o(msp_host_src_be),
    .msp_host_wdata_o(msp_host_src_wdata),
    .msp_host_rdata_i(msp_host_src_rdata),
    .msp_host_ack_i(msp_host_src_ack),
    .last_fetch_addr_o(main_last_fetch_addr),
    .last_read_addr_o(main_last_read_addr),
    .last_write_addr_o(main_last_write_addr),
    .fetch_count_o(main_fetch_count),
    .read_count_o(main_read_count), .write_count_o(main_write_count),
    .diagnostic_o(main_diagnostic)
  );

  // Two DDR read clients share the MiSTer port: the 68010 ROM (1 MiB at
  // 0x30000000) and the ADSP sequential EPROM image (256 KiB at
  // 0x30100000, word offset 0x80000).
  logic        ddr_a_rd, ddr_a_busy, ddr_a_ddr_busy, ddr_a_ready;
  logic [7:0]  ddr_a_burst;
  logic [28:0] ddr_a_addr;
  logic        ddr_b_rd, ddr_b_busy, ddr_b_ddr_busy, ddr_b_ready;
  logic [7:0]  ddr_b_burst;
  logic [28:0] ddr_b_addr;
  logic [63:0] ddr_unused_din_a, ddr_unused_din_b;
  logic [7:0]  ddr_unused_be_a, ddr_unused_be_b;
  logic        ddr_unused_we_a, ddr_unused_we_b;

  harddrivin_ddr_rom #(.SWAP_BYTES(1)) u_main_rom (
    .clk_i(clk_gsp), .rst_i(rst_gsp),
    .rom_addr_i({1'b0, main_rom_addr}), .rom_req_i(main_rom_req),
    .rom_ack_o(main_rom_ack), .rom_data_o(main_rom_data), .busy_o(ddr_a_busy),
    .DDRAM_BUSY(ddr_a_ddr_busy), .DDRAM_BURSTCNT(ddr_a_burst),
    .DDRAM_ADDR(ddr_a_addr), .DDRAM_DOUT(DDRAM_DOUT),
    .DDRAM_DOUT_READY(ddr_a_ready), .DDRAM_RD(ddr_a_rd),
    .DDRAM_DIN(ddr_unused_din_a), .DDRAM_BE(ddr_unused_be_a), .DDRAM_WE(ddr_unused_we_a)
  );

  // ADSP sequential-input EPROMs: MAME's user1 region is 16_BE with 10h at
  // even bytes, so the ADSP sees (10h << 8) | 10k (index 0x41 = 0x0714).  The
  // MRA interleaves 10h into byte 2i and 10k into byte 2i+1, exactly like the
  // main ROM pair, so the same byte swap applies (SWAP_BYTES=0 was wrong:
  // it fed the ADSP byte-swapped tables).
  harddrivin_ddr_rom #(.SWAP_BYTES(1)) u_adsp_rom (
    .clk_i(clk_gsp), .rst_i(rst_gsp),
    .rom_addr_i({3'b100, adsp_sim_addr}), .rom_req_i(adsp_sim_req),
    .rom_ack_o(adsp_sim_ack), .rom_data_o(adsp_sim_data), .busy_o(ddr_b_busy),
    .DDRAM_BUSY(ddr_b_ddr_busy), .DDRAM_BURSTCNT(ddr_b_burst),
    .DDRAM_ADDR(ddr_b_addr), .DDRAM_DOUT(DDRAM_DOUT),
    .DDRAM_DOUT_READY(ddr_b_ready), .DDRAM_RD(ddr_b_rd),
    .DDRAM_DIN(ddr_unused_din_b), .DDRAM_BE(ddr_unused_be_b), .DDRAM_WE(ddr_unused_we_b)
  );

  // Three DDR read clients: main ROM and ADSP EPROMs share a strict
  // round-robin arbiter, then that pair gets four of every five contended
  // grants against the sound sample stream. The sound 68000's program lives
  // in on-chip RAM, so only its sample stream needs the DDR port.
  logic        ddr_d_rd, ddr_d_busy, ddr_d_ddr_busy, ddr_d_ready;
  logic [7:0]  ddr_d_burst;
  logic [28:0] ddr_d_addr;
  logic        ddr_ab_rd, ddr_ab_busy, ddr_ab_ddr_busy, ddr_ab_ready;
  logic [7:0]  ddr_ab_burst;
  logic [28:0] ddr_ab_addr;

  // Neither processor may starve the other. In particular, an eight-to-one
  // preference for 68010 cache fills can delay the ADSP's sequential EPROM
  // stream long enough for the game to raise ADSP TIME OUT ERROR. Display
  // refresh does not use this DDR arbiter, so that preference cannot repair
  // a GSP display interrupt.
  harddrivin_ddr_arbiter #(.A_WEIGHT(0)) u_ddr_arbiter_ab (
    .clk_i(clk_gsp), .rst_i(rst_gsp),
    .a_rd_i(ddr_a_rd), .a_burstcnt_i(ddr_a_burst), .a_addr_i(ddr_a_addr),
    .a_busy_i(ddr_a_busy), .a_ddr_busy_o(ddr_a_ddr_busy), .a_dout_ready_o(ddr_a_ready),
    .b_rd_i(ddr_b_rd), .b_burstcnt_i(ddr_b_burst), .b_addr_i(ddr_b_addr),
    .b_busy_i(ddr_b_busy), .b_ddr_busy_o(ddr_b_ddr_busy), .b_dout_ready_o(ddr_b_ready),
    .DDRAM_BUSY(ddr_ab_ddr_busy), .DDRAM_BURSTCNT(ddr_ab_burst),
    .DDRAM_ADDR(ddr_ab_addr), .DDRAM_DOUT_READY(ddr_ab_ready),
    .DDRAM_RD(ddr_ab_rd), .busy_o(ddr_ab_busy)
  );
  harddrivin_ddr_arbiter #(.A_WEIGHT(4)) u_ddr_arbiter (
    .clk_i(clk_gsp), .rst_i(rst_gsp),
    .a_rd_i(ddr_ab_rd), .a_burstcnt_i(ddr_ab_burst), .a_addr_i(ddr_ab_addr),
    .a_busy_i(ddr_ab_busy), .a_ddr_busy_o(ddr_ab_ddr_busy), .a_dout_ready_o(ddr_ab_ready),
    .b_rd_i(ddr_d_rd), .b_burstcnt_i(ddr_d_burst), .b_addr_i(ddr_d_addr),
    .b_busy_i(ddr_d_busy), .b_ddr_busy_o(ddr_d_ddr_busy), .b_dout_ready_o(ddr_d_ready),
    .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
    .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
    .DDRAM_RD(DDRAM_RD), .busy_o()
  );
  assign DDRAM_DIN = 64'd0;
  assign DDRAM_BE  = 8'hff;
  assign DDRAM_WE  = 1'b0;
  logic unused_ddr;
  assign unused_ddr = ^{ddr_unused_din_a, ddr_unused_din_b, ddr_unused_be_a,
                        ddr_unused_be_b, ddr_unused_we_a, ddr_unused_we_b};

  // HD_TG68K_NO_BARREL: serial shifter (1 bit per cycle, as the real 68010)
  // saves a few hundred ALMs for fill-limited builds; the 68010 pacing was
  // calibrated with the barrel shifter, so check the boot timeline after
  // switching.
`ifdef HD_TG68K_NO_BARREL
  localparam integer MAIN_BARREL_SHIFTER = 0;
`else
  localparam integer MAIN_BARREL_SHIFTER = 1;
`endif
  (* keep_hierarchy = "yes" *) TG68KdotC_Kernel #(
    .SR_Read(0), .VBR_Stackframe(1), .extAddr_Mode(0),
    .MUL_Mode(0), .DIV_Mode(0), .BitField(0),
    .BarrelShifter(MAIN_BARREL_SHIFTER), .MUL_Hardware(1)
  ) u_main_68010 (
    .clk(clk_gsp), .nReset(!rst_gsp), .clkena_in(main_cpu_ce),
    .data_in(main_data_in), .IPL(main_ipl_n), .IPL_autovector(1'b1),
    .berr(1'b0), .CPU(2'b01), .addr_out(main_addr),
    .data_write(main_data_out), .nWr(main_nwr), .nUDS(main_nuds),
    .nLDS(main_nlds),
    .busstate(main_busstate), .longword(), .nResetOut(), .FC(),
    .clr_berr(), .skipFetch(main_skip_fetch),
    .regin_out(main_regin), .CACR_out(), .VBR_out()
  );

  logic [23:1] sound_addr;
  logic [15:0] sound_data_out;
  logic [11:0] dsp_addr;
  logic [15:0] dsp_data_out;

`ifdef HD_SOUND_BOARD
  // Driver sound board: the 68000 EPROM pair is on-chip; only the serial
  // sample EPROMs at DDR byte 0x150000 use a DDR ROM client.
  logic [20:1] snd_ser_addr;
  logic        snd_ser_req, snd_ser_ack;
  logic [15:0] snd_ser_data;
  logic        snd_dsp_running;
  logic [63:0] ddr_unused_din_d;
  logic [7:0]  ddr_unused_be_d;
  logic        ddr_unused_we_d;

  harddrivin_ddr_rom #(.SWAP_BYTES(1)) u_sound_serial_rom (
    .clk_i(clk_gsp), .rst_i(rst_gsp),
    .rom_addr_i(snd_ser_addr), .rom_req_i(snd_ser_req),
    .rom_ack_o(snd_ser_ack), .rom_data_o(snd_ser_data), .busy_o(ddr_d_busy),
    .DDRAM_BUSY(ddr_d_ddr_busy), .DDRAM_BURSTCNT(ddr_d_burst),
    .DDRAM_ADDR(ddr_d_addr), .DDRAM_DOUT(DDRAM_DOUT),
    .DDRAM_DOUT_READY(ddr_d_ready), .DDRAM_RD(ddr_d_rd),
    .DDRAM_DIN(ddr_unused_din_d), .DDRAM_BE(ddr_unused_be_d), .DDRAM_WE(ddr_unused_we_d)
  );

  (* keep_hierarchy = "yes" *) harddrivin_sound_board u_sound_board (
    .clk_i(clk_gsp), .rst_i(rst_gsp), .board_reset_i(snd_reset),
    .main_wr_i(snd_wr), .main_wdata_i(main_data_out), .main_rd_i(snd_rd),
    .main_rdata_o(snd_rdata), .mainflag_o(snd_mainflag),
    .soundflag_o(snd_soundflag),
    .rom_wr_clk_i(snd_rom_wr_clk_i), .rom_wr_en_i(snd_rom_wr_en_i),
    .rom_wr_addr_i(snd_rom_wr_addr_i), .rom_wr_data_i(snd_rom_wr_data_i),
    .ser_rom_addr_o(snd_ser_addr), .ser_rom_req_o(snd_ser_req),
    .ser_rom_ack_i(snd_ser_ack), .ser_rom_data_i(snd_ser_data),
    .audio_o(audio_o), .cpu_addr_o(sound_addr), .dsp_addr_o(dsp_addr),
    .dsp_running_o(snd_dsp_running), .dac_o(snd_dac), .dac_write_o(snd_dac_write),
    .last_cmd_o(snd_last_cmd), .cramen_o(snd_cramen),
    .comram_write_o(snd_comram_write), .ser_byte_o(snd_ser_byte),
    .dac_peak_o(snd_dac_peak), .comram_count_o(snd_comram_count),
    .dsp_irq_seen_o(snd_dsp_irq_seen), .cpu_pc_page_o(snd_cpu_pc_page)
  );
  assign sound_data_out = snd_rdata;
  assign dsp_data_out   = {15'd0, snd_dsp_running};
  assign snd_dsp_run    = snd_dsp_running;
  logic unused_ddr_d;
  assign unused_ddr_d = ^{ddr_unused_din_d, ddr_unused_be_d, ddr_unused_we_d};
`else
  assign sound_addr     = 23'd0;
  assign sound_data_out = 16'd0;
  assign dsp_addr       = 12'd0;
  assign dsp_data_out   = 16'd0;
  assign snd_rdata      = 16'h0000;
  assign snd_mainflag   = 1'b0;
  assign snd_soundflag  = 1'b0;
  assign snd_dsp_run    = 1'b0;
  assign snd_dac        = 12'd0;
  assign snd_dac_write  = 1'b0;
  assign snd_last_cmd   = 16'h0000;
  assign snd_cramen     = 1'b0;
  assign snd_comram_write = 1'b0;
  assign snd_ser_byte   = 8'h00;
  assign snd_dac_peak   = 8'h00;
  assign snd_comram_count = 8'h00;
  assign snd_dsp_irq_seen = 1'b0;
  assign snd_cpu_pc_page  = 8'h00;
  assign audio_o        = 16'sd0;
  assign ddr_d_rd = 1'b0; assign ddr_d_burst = 8'd0; assign ddr_d_addr = 29'd0;
  assign ddr_d_busy = 1'b0;
  logic unused_snd;
  assign unused_snd = ^{snd_wr, snd_rd, snd_reset, ddr_d_ddr_busy, ddr_d_ready,
                        snd_rom_wr_en_i, snd_rom_wr_addr_i, snd_rom_wr_data_i};
`endif

  logic [31:0] adsp_signature;
  logic        adsp_host_req, adsp_host_we, adsp_host_ack;
  logic [17:1] adsp_host_addr;
  logic        adsp_sim_req, adsp_sim_ack;
  logic [16:0] adsp_sim_addr;
  logic [15:0] adsp_sim_data;
  logic [15:0] adsp_host_wdata, adsp_host_rdata;

`ifdef HD_FUNCTIONAL_ADSP
  // Atari ADSP board: 68010 window 0x800000-0x83ffff, ADSP-2100 core, data
  // memory, sequential input EPROMs, double-buffered output memory, and the
  // GINT interrupt to the 68010 (level 2 on the 74S148).
  (* keep_hierarchy = "yes" *) harddrivin_adsp_board u_adsp_board (
    .clk_i(clk_gsp), .rst_i(rst_gsp),
    .host_req_i(adsp_host_req), .host_we_i(adsp_host_we),
    .host_addr_i(adsp_host_addr), .host_wdata_i(adsp_host_wdata),
    .host_rdata_o(adsp_host_rdata), .host_ack_o(adsp_host_ack),
    .irq_o(adsp_irq),
    .sim_req_o(adsp_sim_req), .sim_addr_o(adsp_sim_addr),
    .sim_ack_i(adsp_sim_ack), .sim_data_i(adsp_sim_data),
    .adsp_pc_o(adsp_pc), .adsp_running_o(adsp_running),
    .adsp_illegal_o(adsp_illegal), .latch_o(adsp_latch),
    .dirq_o(adsp_dirq), .xflag_o(adsp_xflag), .mpage_o(adsp_mpage),
    .gint_count_o(adsp_gint_count), .upload_count_o(adsp_upload_count),
    .upload_sum_lo_o(adsp_upload_sum_lo), .upload_xor_lo_o(adsp_upload_xor_lo),
    .upload_sum_hi_o(adsp_upload_sum_hi), .upload_xor_hi_o(adsp_upload_xor_hi),
    .sim_read_count_o(adsp_sim_reads), .som_write_count_o(adsp_som_writes)
  );

  always_comb begin
    adsp_signature = {18'd0, adsp_pc} ^ {24'd0, adsp_latch} ^
                     {16'd0, adsp_gint_count} ^ {8'd0, adsp_upload_count, 8'd0} ^
                     {adsp_sim_reads, adsp_som_writes} ^
                     {26'd0, adsp_running, adsp_illegal, adsp_dirq, adsp_xflag,
                      adsp_mpage, adsp_irq};
  end
`else
`ifdef HD_ADSP_RESERVE
  localparam integer ADSP_RESERVE_BITS = 16384;
`else
  localparam integer ADSP_RESERVE_BITS = 1;
`endif
  adsp2100_budget_model #(.RESERVE_BITS(ADSP_RESERVE_BITS)) u_adsp_budget (
    .clk_i(clk_gsp), .rst_i(rst_gsp), .signature_o(adsp_signature)
  );
  // No board in this revision: acknowledge immediately as an open bus.
  assign adsp_host_ack   = adsp_host_req;
  assign adsp_host_rdata = 16'hffff;
  assign adsp_irq        = 1'b0;
  assign adsp_pc = 14'd0; assign adsp_running = 1'b0; assign adsp_illegal = 1'b0;
  assign adsp_dirq = 1'b0; assign adsp_xflag = 1'b0; assign adsp_mpage = 1'b0;
  assign adsp_latch = 8'd0; assign adsp_gint_count = 16'd0;
  assign adsp_upload_count = 16'd0; assign adsp_sim_reads = 16'd0;
  assign adsp_upload_sum_lo = 16'd0; assign adsp_upload_xor_lo = 16'd0;
  assign adsp_upload_sum_hi = 16'd0; assign adsp_upload_xor_hi = 16'd0;
  assign adsp_som_writes = 16'd0;
  assign adsp_sim_req  = 1'b0;
  assign adsp_sim_addr = 17'd0;
  logic unused_adsp_rom;
  assign unused_adsp_rom = ^{adsp_sim_ack, adsp_sim_data,
                             adsp_host_we, adsp_host_addr, adsp_host_wdata};
`endif

  // Physical 100 MHz SDRAM controller under deterministic Compact traffic.
  logic scan_req, scan_ack, scan_overflow;
  logic [VRAM_AW-1:0] scan_addr;
  logic [15:0] scan_data;
  logic synthetic_gsp_req, synthetic_gsp_we;
  logic [17:0] synthetic_gsp_addr;
  logic [15:0] synthetic_gsp_wdata;
  logic [24:0] sdram_addr;
  logic [15:0] sdram_din, sdram_dout;
  logic [1:0] sdram_be;
  logic sdram_rd, sdram_we, sdram_ready;
  logic [31:0] scan_words, traffic_gsp_words;
  logic burst_mem_req, burst_mem_we, burst_mem_accept, burst_mem_done;
  logic [23:0] burst_mem_addr;
  logic [63:0] burst_mem_wdata;
  logic [7:0]  burst_mem_be;
  logic [63:0] burst_mem_rdata;
  logic [31:0] burst_reads, burst_writes;
  logic [7:0] vram_arbiter_debug;
`ifdef HD_ARCH_MILESTONE
  assign traffic_reset = rst_100 || !burst_initialized;
`else
  assign traffic_reset = rst_100;
`endif

  assign gsp_vram_path_level = vram_arbiter_debug;

  // Hardware-only failures can otherwise disappear between diagnostic-page
  // captures. Preserve every completed boundary until the core is reset.
  always_ff @(posedge clk_100) begin
    if (traffic_reset) begin
      gsp_vram_path_seen_100_q <= 7'd0;
    end else begin
      if (gsp_vram_req_100) gsp_vram_path_seen_100_q[6] <= 1'b1;
      if (gsp_vram_ack_100) gsp_vram_path_seen_100_q[5] <= 1'b1;
      if (burst_mem_req) gsp_vram_path_seen_100_q[4] <= 1'b1;
      if (burst_mem_req && burst_mem_accept)
        gsp_vram_path_seen_100_q[3] <= 1'b1;
      if (burst_mem_done) gsp_vram_path_seen_100_q[2] <= 1'b1;
      if (scan_req) gsp_vram_path_seen_100_q[1] <= 1'b1;
      if (scan_ack) gsp_vram_path_seen_100_q[0] <= 1'b1;
    end
  end

  // The real GSP refresh serializer is now the scan client.  Retain the old
  // synthetic-client observability wires at zero for reserve-build parity.
  assign synthetic_gsp_req   = 1'b0;
  assign synthetic_gsp_we    = 1'b0;
  assign synthetic_gsp_addr  = 18'd0;
  assign synthetic_gsp_wdata = 16'd0;

`ifdef HD_ARCH_MILESTONE
  compact_vram_burst_arbiter #(.ADDR_W(VRAM_AW)) u_vram_burst_arbiter (
    .clk_i(clk_100), .rst_i(traffic_reset),
    .scan_req_i(scan_req), .scan_addr_i(scan_addr), .scan_ack_o(scan_ack),
    .scan_data_o(scan_data), .scan_overflow_o(scan_overflow),
    .gsp_req_i(gsp_vram_req_100), .gsp_we_i(gsp_vram_we_100),
    .gsp_be_i(gsp_vram_be_100),
    .gsp_addr_i(gsp_vram_addr_100), .gsp_wdata_i(gsp_vram_wdata_100),
    .gsp_ack_o(gsp_vram_ack_100), .gsp_rdata_o(gsp_vram_rdata_100),
    .gsp_rdata64_o(gsp_vram_rdata64_100),
    .mem_req_o(burst_mem_req), .mem_we_o(burst_mem_we),
    .mem_addr_o(burst_mem_addr), .mem_wdata_o(burst_mem_wdata),
    .mem_be_o(burst_mem_be), .mem_accept_i(burst_mem_accept),
    .mem_done_i(burst_mem_done), .mem_rdata_i(burst_mem_rdata),
    .scan_words_o(scan_words), .gsp_words_o(traffic_gsp_words),
    .read_bursts_o(burst_reads), .write_words_o(burst_writes),
    .debug_o(vram_arbiter_debug)
  );

  compact_sdram_burst u_sdram_burst (
    .init_i(rst_100), .clk_i(clk_100),
    .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_DQML(SDRAM_DQML),
    .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_BA(SDRAM_BA), .SDRAM_nCS(SDRAM_nCS),
    .SDRAM_nWE(SDRAM_nWE), .SDRAM_nRAS(SDRAM_nRAS),
    .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_CKE(SDRAM_CKE), .SDRAM_CLK(SDRAM_CLK),
    .req_i(burst_mem_req), .we_i(burst_mem_we), .addr_i(burst_mem_addr),
    .wdata_i(burst_mem_wdata), .be_i(burst_mem_be),
    .accept_o(burst_mem_accept), .initialized_o(burst_initialized),
    .done_o(burst_mem_done),
    .rdata_o(burst_mem_rdata)
  );
`else
  assign gsp_vram_rdata64_100 = {4{gsp_vram_rdata_100}};   // legacy arbiter: no burst data (LINE_FILL=0)
  compact_vram_arbiter u_vram_arbiter (
    .clk_i(clk_100), .rst_i(rst_100),
    .scan_req_i(scan_req), .scan_addr_i(scan_addr), .scan_ack_o(scan_ack),
    .scan_data_o(scan_data), .scan_overflow_o(scan_overflow),
    .gsp_req_i(gsp_vram_req_100), .gsp_we_i(gsp_vram_we_100),
    // legacy word-write arbiter: only word 0 of a line write is honoured
    // (the arch build uses the burst arbiter above)
    .gsp_be_i(gsp_vram_be_100[1:0]),
    .gsp_addr_i(gsp_vram_addr_100), .gsp_wdata_i(gsp_vram_wdata_100[15:0]),
    .gsp_ack_o(gsp_vram_ack_100), .gsp_rdata_o(gsp_vram_rdata_100),
    .sdram_addr_o(sdram_addr), .sdram_din_o(sdram_din),
    .sdram_be_o(sdram_be),
    .sdram_rd_o(sdram_rd), .sdram_we_o(sdram_we),
    .sdram_dout_i(sdram_dout), .sdram_ready_i(sdram_ready),
    .scan_words_o(scan_words), .gsp_words_o(traffic_gsp_words)
  );

  sdram u_sdram (
    .init(rst_100), .clk(clk_100), .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A),
    .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH), .SDRAM_BA(SDRAM_BA),
    .SDRAM_nCS(SDRAM_nCS), .SDRAM_nWE(SDRAM_nWE), .SDRAM_nRAS(SDRAM_nRAS),
    .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_CKE(SDRAM_CKE), .SDRAM_CLK(SDRAM_CLK),
    .wtbt(sdram_be), .addr(sdram_addr), .dout(sdram_dout), .din(sdram_din),
    .we(sdram_we), .rd(sdram_rd), .ready(sdram_ready)
  );
  assign burst_reads = 32'd0;
  assign burst_writes = 32'd0;
  assign burst_initialized = 1'b0;
`endif

  // Every subsystem contributes to an externally visible signature so the
  // fitter cannot discard an idle candidate core from the resource result.
  always_comb begin
    debug_o = gsp_pc ^ {msp_pc[15:0], msp_pc[31:16]} ^ main_addr ^
              {8'd0, sound_addr} ^ {20'd0, dsp_addr} ^ adsp_signature ^
              scan_words ^ {traffic_gsp_words[15:0], traffic_gsp_words[31:16]} ^
              screen_refresh_count ^
              {screen_row_count[15:0], screen_row_count[31:16]} ^
              burst_reads ^ {burst_writes[15:0], burst_writes[31:16]} ^
              main_read_count ^ {main_write_count[15:0], main_write_count[31:16]} ^
              {24'd0, gsp_state[2:0], msp_state[2:0], gsp_illegal,
               msp_illegal} ^ main_regin ^ {16'd0, main_data_out} ^
              {16'd0, sound_data_out ^ dsp_data_out ^ scan_data ^ gsp_vram_rdata_100} ^
              {30'd0, scan_overflow ^ frame_start, gsp_shiftreg_enable};
  end

  logic [31:0] unused_cycle_counts;
  logic [1:0] unused_busstate;
  logic unused_main_nwr;
  logic [35:0] unused_synthetic_traffic;
  logic [4:0] unused_msp_video;
  assign unused_cycle_counts = gsp_cycles ^ msp_cycles;
  assign unused_busstate = main_busstate;
  assign unused_main_nwr = main_nwr;
  assign unused_synthetic_traffic = {
    synthetic_gsp_req, synthetic_gsp_we, synthetic_gsp_addr,
    synthetic_gsp_wdata
  };
  assign unused_msp_video = {
    msp_video_hsync, msp_video_vsync, msp_video_hblank,
    msp_video_vblank, msp_video_blank
  };
`ifdef HD_VIDEO_DIAGNOSTICS
`include "rtl/harddrivin_video_debug.svh"
`endif
// Passive simulation trace, absent from synthesized release builds.
`ifdef HD_VIDEO_TRACE
`include "tests/harddrivin_video_trace.svh"
`endif
endmodule

`default_nettype wire

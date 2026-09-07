`default_nettype none

// Compact MultiSync display path.  A TMS34010 screen-refresh transaction
// loads the 4461 serial banks. The MultiSync serial sequencer keeps emitting
// consecutive 512-byte slices between screen transfers (schematic sheets
// 11-13). DPYSTRT.LCSTRT selects 1..4 visible lines per transfer (TI 1988
// User's Guide 6-24 / 9.10.1.5). Service uses four; gameplay normally uses one.
module harddrivin_compact_scanout #(
  parameter bit COCKPIT = 0,
  parameter int VRAM_AW = COCKPIT ? 19 : 18
) (
  input  logic        core_clk_i,
  input  logic        core_rst_i,
  input  logic        screen_req_i,
  input  logic [13:0] screen_srfaddr_i,
  input  logic [15:0] screen_dpytap_i,
  input  logic        screen_org_i,
  input  logic [1:0]  screen_lcstrt_i,
  output logic        screen_ack_o,

  input  logic        mem_clk_i,
  input  logic        mem_rst_i,
  output logic        scan_req_o,
  output logic [VRAM_AW-1:0] scan_addr_o,
  input  logic        scan_ack_i,
  input  logic [15:0] scan_data_i,
  input  logic        scan_overflow_i,

  input  logic        video_hsync_i,
  input  logic        video_vsync_i,
  input  logic        video_hblank_i,
  input  logic        video_vblank_i,
  input  logic        video_blank_i,
  input  tri1         video_enable_i,
  input  logic [15:0] video_heblnk_i,
  input  logic [3:0]  fine_scroll_i,
  input  logic [1:0]  palette_bank_i,
  input  logic        diagnostic_enable_i,
  input  logic [127:0] diagnostic_i,
`ifdef HD_VIDEO_DIAGNOSTICS
  input  logic [255:0] video_debug_i,
`endif
  // Controls debug overlay (graphical panel at the top-left): [11:0] wheel
  // position, [19:12] gas, [27:20] force brake (f0 released .. 20 full),
  // [35:28] clutch, [39:36] gear switches (port 1 bits 11:8, active low),
  // [40] key, [41] abort, [42] coin, [43] service.  Normally [75:44] are the
  // four raw analog axes, [76] DSP-running, [77] main flag, [78] sound flag,
  // [79] DAC activity, [91:80] the last DAC value, [127:120] DAC peak,
  // [135:128] COM-RAM activity, [136] DSP IRQ seen, [140:137] moved axes,
  // and [148:141] sound-CPU PC page.  If [149] is set, [106:44] instead hold
  // the first GSP-timeout snapshot (PC, state, instruction, HINT age, illegal).
  input  logic        controls_enable_i,
  input  logic [149:0] controls_i,
  output logic [9:0]  palette_addr_o,
  input  logic [15:0] palette_lo_i,
  input  logic [15:0] palette_hi_i,

  output logic        ce_pix_o,
  output logic        hsync_o,
  output logic        vsync_o,
  output logic        hblank_o,
  output logic        vblank_o,
  output logic [23:0] rgb_o,
  output logic        frame_start_o,
  output logic [31:0] refresh_count_o,
  output logic [31:0] row_count_o,
  output logic [13:0] loaded_srfaddr_o,
  output logic        loaded_org_o,
  output logic [7:0]  loaded_col_o,
  output logic [7:0]  loaded_row_select_o,
  output logic [15:0] loaded_row_or_o,
  output logic [10:0] loaded_row_nonzero_o,
  output logic        row_valid_o,
  output logic [8:0]  line_o,      // diagnostics: raster line of the pixel in the palette stage
  output logic [9:0]  x_o          // diagnostics: raster x of that pixel
);
  localparam int GROUP_BITS = COCKPIT ? 11 : 10;
  localparam logic [9:0] H_VISIBLE = COCKPIT ? 508 : 512;
  localparam logic [9:0] H_LAST = COCKPIT ? 639 : 645;
  localparam logic [8:0] V_VISIBLE = COCKPIT ? 384 : 288;
  localparam logic [8:0] V_LAST = COCKPIT ? 416 : 307;
  // Bundled command mailbox. Acknowledge acceptance into private loader
  // state so the GSP can resume. Publish the first complete slice promptly;
  // later slices are filled ahead of their serial scanline deadlines.
  logic [13:0] command_srfaddr_q;
  logic [15:0] command_dpytap_q;
  logic        command_org_q;
  logic [1:0]  command_palette_bank_q;
  logic [3:0]  command_fine_scroll_q;
  logic [1:0]  command_lcstrt_q;
  logic [15:0] viewport_heblnk_q;
  logic viewport_vblank_prev_q;
  logic [15:0] sync_viewport_heblnk_q;
  logic [11:0] command_x_crop_q;
  logic [9:0] command_clip_left_q;
  wire [15:0] left_blank_clocks = video_heblnk_i - viewport_heblnk_q;

  // MAME renders each line at its programmed HEBLNK inside one fixed
  // frame viewport. Preserve that origin when a split moves HEBLNK:
  // earlier starts are cropped; later starts leave a black left margin.
  // One cockpit VCLK spans four pixels. Capture the viewport in VBLANK,
  // after the ROM restores the scene's timing for the next frame.
  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      viewport_heblnk_q <= COCKPIT ? 16'd26 : 16'd0;
      viewport_vblank_prev_q <= 1'b1;
      sync_viewport_heblnk_q <= 16'd26;
    end else begin
      viewport_vblank_prev_q <= video_vblank_i;
      if (COCKPIT && video_vblank_i) viewport_heblnk_q <= video_heblnk_i;
      if (COCKPIT && viewport_vblank_prev_q && !video_vblank_i)
        sync_viewport_heblnk_q <= viewport_heblnk_q;
    end
  end
  logic        command_toggle_q;
  logic        screen_seen_q;

  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      command_srfaddr_q <= 14'd0;
      command_dpytap_q  <= 16'd0;
      command_org_q     <= 1'b0;
      command_palette_bank_q <= 2'd0;
      command_fine_scroll_q  <= 4'd0;
      command_lcstrt_q <= 2'd0;
      command_x_crop_q <= 12'd0;
      command_clip_left_q <= 10'd0;
      command_toggle_q  <= 1'b0;
      screen_seen_q     <= 1'b0;
      refresh_count_o   <= 32'd0;
    end else begin
      if (!screen_req_i) screen_seen_q <= 1'b0;
      if (screen_req_i && !screen_seen_q) begin
        command_srfaddr_q <= screen_srfaddr_i;
        command_dpytap_q  <= screen_dpytap_i;
        command_org_q     <= screen_org_i;
        // These latches describe the same physical serializer row. The SDRAM
        // loader may publish its pixels later, so keep the controls bundled
        // with the descriptor instead of sampling unrelated live state.
        command_palette_bank_q <= palette_bank_i;
        command_fine_scroll_q  <= fine_scroll_i;
        command_x_crop_q <= {viewport_heblnk_q[9:0], 2'b00} -
                            {video_heblnk_i[9:0], 2'b00};
        command_clip_left_q <= video_heblnk_i > viewport_heblnk_q
          ? (left_blank_clocks >= 16'd127 ? 10'd508 : {left_blank_clocks[7:0], 2'b00})
          : 10'd0;
        command_lcstrt_q <= screen_lcstrt_i;
        command_toggle_q  <= !command_toggle_q;
        screen_seen_q     <= 1'b1;
        refresh_count_o   <= refresh_count_o + 32'd1;
      end
    end
  end

  logic command_sync1_q, command_sync2_q, command_seen_mem_q;
  // A row group remains owned by scanout until its descriptor is actually
  // promoted. Returning that toggle prevents reuse of the displayed group
  // and keeps a later descriptor from overwriting the pending mailbox.
  logic consumed_toggle_q;
  logic [1:0] consumed_sync_q;
  logic [13:0] effective_srfaddr;
  logic [VRAM_AW-1:0] row_base_q;
  logic [7:0]  row_col_q;
  logic [9:0]  load_index_q;
  logic [1:0]  load_lcstrt_q, published_lcstrt_q;
  logic [GROUP_BITS-1:0] serial_group_offset;
  logic        load_bank_q;
  logic        loader_busy_q;
  logic        loader_wait_q;
  logic        published_bank_q;
  logic [7:0]  published_col_q;
  logic [13:0] published_srfaddr_q;
  logic        published_org_q;
  logic [7:0]  published_row_select_q;
  logic [15:0] published_row_or_q;
  logic [10:0] published_row_nonzero_q;
  logic        published_toggle_q;
  logic        accepted_toggle_q;
  logic [13:0] load_srfaddr_q;
  logic        load_org_q;
  logic [1:0]  load_palette_bank_q;
  logic [3:0]  load_fine_scroll_q;
  logic [9:0] load_clip_left_q, published_clip_left_q;
  logic [1:0]  published_palette_bank_q;
  logic [3:0]  published_fine_scroll_q;
  logic [15:0] load_row_or_q;
  logic [10:0] load_row_nonzero_q;

  // A superseding cockpit transfer may cancel unneeded trailing slices.
  // Each bit becomes visible only after all 256 words of its slice arrived.
  logic [7:0] completed_slices_q;
  (* async_reg = "true" *) logic [7:0] completed_slices_sync1_q, completed_slices_sync2_q;
  wire superseding_transfer = COCKPIT &&
      command_sync2_q != command_seen_mem_q && load_index_q[9:8] != 2'd0;

  logic row_write_en;
  logic [10:0] row_write_addr;
  logic [10:0] row_read_addr;
  logic [15:0] row_word_q;

  assign row_write_en = loader_busy_q && loader_wait_q && scan_ack_i;
  assign row_write_addr = {load_bank_q, load_index_q};

  // Two groups of four slices permit filling the following transfer while
  // the serializer consumes the current group. Single-line transfers fill
  // only slice zero and retain their original bandwidth and publication time.
  harddrivin_dual_clock_word_ram #(.ADDR_WIDTH(11)) u_row_ram (
    .wr_clk_i(mem_clk_i), .wr_en_i(row_write_en),
    .wr_addr_i(row_write_addr), .wr_data_i(scan_data_i),
    .rd_clk_i(core_clk_i), .rd_addr_i(row_read_addr), .rd_data_o(row_word_q)
  );

  assign effective_srfaddr = command_org_q
                           ? command_srfaddr_q
                           : ~command_srfaddr_q;

  // The four CAS banks form a circular 2048-byte serial group.
  assign serial_group_offset = row_base_q[GROUP_BITS-1:0] + {{(GROUP_BITS-10){1'b0}}, load_index_q};

  // Exact MAME get_display_params + scanline_driver formula, in bytes:
  // row=(effective DPYADR >> 4), col=((DPYADR & 0x7c)<<4)|DPYTAP.
  // Its low eight column bits select sixteen-pixel groups. Preload the
  // visible window, wrapping within 4096 bytes; each later LCSTRT slice
  // advances 512 bytes. This keeps the existing 256-word line buffers.
  logic [7:0] cockpit_col;
  logic [11:0] cockpit_byte_start;
  assign cockpit_col = {effective_srfaddr[1:0], 6'd0} | command_dpytap_q[7:0];
  assign cockpit_byte_start = {cockpit_col, 4'd0} - 12'd15 +
                              {8'd0, command_fine_scroll_q} + command_x_crop_q;
  always_ff @(posedge mem_clk_i) begin
    if (mem_rst_i) begin
      consumed_sync_q <= 2'b00;
      completed_slices_q <= 8'd0;
      command_sync1_q    <= 1'b0;
      command_sync2_q    <= 1'b0;
      command_seen_mem_q <= 1'b0;
      row_base_q         <= 18'd0;
      row_col_q          <= 8'd0;
      load_index_q       <= 10'd0;
      load_lcstrt_q      <= 2'd0;
      published_lcstrt_q <= 2'd0;
      load_bank_q        <= 1'b0;
      loader_busy_q      <= 1'b0;
      loader_wait_q      <= 1'b0;
      scan_req_o         <= 1'b0;
      scan_addr_o        <= 18'd0;
      published_bank_q   <= 1'b0;
      published_col_q    <= 8'd0;
      published_srfaddr_q <= 14'd0;
      published_org_q     <= 1'b0;
      published_row_select_q <= 8'd0;
      published_row_or_q  <= 16'd0;
      published_row_nonzero_q <= 11'd0;
      published_toggle_q <= 1'b0;
      accepted_toggle_q  <= 1'b0;
      load_srfaddr_q     <= 14'd0;
      load_org_q         <= 1'b0;
      load_palette_bank_q <= 2'd0;
      load_fine_scroll_q  <= 4'd0;
      load_clip_left_q <= 10'd0;
      published_clip_left_q <= 10'd0;
      published_palette_bank_q <= 2'd0;
      published_fine_scroll_q  <= 4'd0;
      load_row_or_q      <= 16'd0;
      load_row_nonzero_q <= 11'd0;
      row_count_o        <= 32'd0;
    end else begin
      consumed_sync_q <= {consumed_sync_q[0], consumed_toggle_q};
      command_sync1_q <= command_toggle_q;
      command_sync2_q <= command_sync1_q;
      scan_req_o      <= 1'b0;

      if (!loader_busy_q && command_sync2_q != command_seen_mem_q &&
          consumed_sync_q[1] == published_toggle_q) begin
        completed_slices_q[{load_bank_q, 2'b00} +: 4] <= 4'd0;
        command_seen_mem_q <= command_sync2_q;
        // The descriptor and all of its payload are now private loader
        // registers. Acknowledge this acceptance independently from row
        // publication so the TMS34010 can resume while the 4461-equivalent
        // serial transfer completes. If loading ever takes longer than a
        // scanline, the next held request naturally waits here; no mailbox
        // payload can be overwritten.
        accepted_toggle_q <= command_sync2_q;
        // The external SDRAM view is linear in physical scanlines: the low
        // ten effective SRFADDR bits select one of 1024 rows and DPYTAP alone
        // selects the serial starting byte.  A previous implementation
        // shifted SRFADDR by two and then reused its low nibble as a column;
        // that double-applied the PCB bank decode, repeating every row four
        // times and rotating it 128 pixels on each advance.
        row_base_q    <= COCKPIT
          ? {effective_srfaddr[9:2], cockpit_byte_start[11:1]}
          : VRAM_AW'({effective_srfaddr[9:0], 8'b0});
        row_col_q     <= COCKPIT ? cockpit_col : command_dpytap_q[9:2];
        load_srfaddr_q <= command_srfaddr_q;
        load_org_q     <= command_org_q;
        load_palette_bank_q <= command_palette_bank_q;
        load_fine_scroll_q  <= command_fine_scroll_q;
        load_clip_left_q <= command_clip_left_q;
        load_lcstrt_q <= command_lcstrt_q;
        load_row_or_q  <= 16'd0;
        load_row_nonzero_q <= 11'd0;
        load_index_q  <= 10'd0;
        loader_busy_q <= 1'b1;
        loader_wait_q <= 1'b0;
      end else if (loader_busy_q && !loader_wait_q) begin
        scan_addr_o   <= {row_base_q[VRAM_AW-1:GROUP_BITS], serial_group_offset};
        scan_req_o    <= 1'b1;
        loader_wait_q <= 1'b1;
      end else if (loader_busy_q && loader_wait_q && scan_ack_i) begin
        loader_wait_q <= 1'b0;
        load_row_or_q <= load_row_or_q | scan_data_i;
        if (scan_data_i != 16'd0)
          load_row_nonzero_q <= load_row_nonzero_q + 11'd1;
        if (load_index_q[7:0] == 8'hff)
          completed_slices_q[{load_bank_q,load_index_q[9:8]}] <= 1'b1;
        if (load_index_q == 10'h0ff) begin
          published_bank_q   <= load_bank_q;
          published_col_q    <= row_col_q;
          published_srfaddr_q <= load_srfaddr_q;
          published_org_q     <= load_org_q;
          published_palette_bank_q <= load_palette_bank_q;
          published_fine_scroll_q  <= load_fine_scroll_q;
          published_clip_left_q <= load_clip_left_q;
          published_lcstrt_q <= load_lcstrt_q;
          published_row_select_q <= row_base_q[VRAM_AW-1:GROUP_BITS];
          // Include the final returning word; the accumulators above update
          // on the same clock edge as the completed-row publication.
          published_row_or_q <= load_row_or_q | scan_data_i;
          published_row_nonzero_q <= load_row_nonzero_q +
                                      (scan_data_i != 16'd0);
          published_toggle_q <= !published_toggle_q;
          row_count_o        <= row_count_o + 32'd1;
        end
        // At a two-line scene -> one-line dashboard split, finishing the
        // obsolete scene slice can make the first dashboard row miss its
        // boundary. Retire the in-flight word, then give the queued transfer
        // the loader. Per-slice completion bits prevent reading a truncated
        // slice if the replacement is late. Buffer ownership still gates
        // acceptance of the new command until the old publication is used.
        if (load_index_q == {load_lcstrt_q, 8'hff} || superseding_transfer) begin
          loader_busy_q <= 1'b0;
          load_bank_q <= !load_bank_q;
        end else begin
          load_index_q <= load_index_q + 10'd1;
        end
      end
    end
  end

  // Synchronize the completed-row notification into the 48 MHz
  // output/palette domain. The Compact ROM always programs the fixed
  // 6 MHz, 323-by-308 TMS timing shown by the board documentation. Emit the
  // corresponding two-pixels-per-VCLK MiSTer raster locally so host bootstrap
  // cannot expose partially programmed timing registers at the video pins.
  logic [1:0] publish_sync_q;
  logic publish_seen_q;
  logic [1:0] accepted_sync_q;
  logic accepted_seen_q;
  logic pending_bank_q, active_bank_q;
  logic [7:0] pending_col_q, active_col_q;
  logic [1:0] pending_palette_bank_q;
  logic [3:0] pending_fine_scroll_q;
  logic [9:0] pending_clip_left_q, display_clip_left_q;
  logic [1:0] pending_lcstrt_q, active_lcstrt_q, serial_line_q;
  logic pending_row_q, row_valid_q;
  logic [9:0] raster_h_q;
  logic [8:0] raster_v_q;
  logic [9:0] visible_x_q;
  logic [1:0] pixel_phase_q;
  logic [10:0] pixel_offset;
  logic [9:0] pixel_word_addr;
  logic pixel_byte_select_q;
  logic captured_blank_q, captured_hblank_q, captured_vblank_q;
  logic captured_hsync_q, captured_vsync_q;
  logic [9:0] captured_x_q;
  logic [8:0] captured_line_q;
  // ---- controls debug overlay -------------------------------------------
  // Panel x 8..199, lines 20..81: "ST" track, "GA"/"BR"/"CL" bars,
  // input cells, screen-split telemetry, and sound/axis telemetry.
  localparam logic [9:0] CTL_PX0 = 10'd8;
  localparam logic [9:0] CTL_PX1 = 10'd200;
  localparam logic [8:0] CTL_PY0 = 9'd20;
  localparam logic [8:0] CTL_PY1 = 9'd82;
  function automatic logic [4:0] ctl_glyph_row(input logic [5:0] code, input logic [2:0] row);
    logic [34:0] g;
    case (code)
      6'd0:  g = {5'b01110, 5'b10001, 5'b10011, 5'b10101, 5'b11001, 5'b10001, 5'b01110};
      6'd1:  g = {5'b00100, 5'b01100, 5'b00100, 5'b00100, 5'b00100, 5'b00100, 5'b01110};
      6'd2:  g = {5'b01110, 5'b10001, 5'b00001, 5'b00010, 5'b00100, 5'b01000, 5'b11111};
      6'd3:  g = {5'b11111, 5'b00010, 5'b00100, 5'b00010, 5'b00001, 5'b10001, 5'b01110};
      6'd4:  g = {5'b00010, 5'b00110, 5'b01010, 5'b10010, 5'b11111, 5'b00010, 5'b00010};
      6'd10: g = {5'b01110, 5'b10001, 5'b10001, 5'b11111, 5'b10001, 5'b10001, 5'b10001};  // A
      6'd11: g = {5'b11110, 5'b10001, 5'b10001, 5'b11110, 5'b10001, 5'b10001, 5'b11110};  // B
      6'd12: g = {5'b01110, 5'b10001, 5'b10000, 5'b10000, 5'b10000, 5'b10001, 5'b01110};  // C
      6'd13: g = {5'b11110, 5'b10001, 5'b10001, 5'b11110, 5'b10000, 5'b10000, 5'b10000};  // P
      6'd17: g = {5'b01110, 5'b10001, 5'b10000, 5'b10111, 5'b10001, 5'b10001, 5'b01111};  // G
      6'd18: g = {5'b10001, 5'b10010, 5'b10100, 5'b11000, 5'b10100, 5'b10010, 5'b10001};  // K
      6'd19: g = {5'b10000, 5'b10000, 5'b10000, 5'b10000, 5'b10000, 5'b10000, 5'b11111};  // L
      6'd20: g = {5'b10001, 5'b11001, 5'b10101, 5'b10011, 5'b10001, 5'b10001, 5'b10001};  // N
      6'd21: g = {5'b11110, 5'b10001, 5'b10001, 5'b11110, 5'b10100, 5'b10010, 5'b10001};  // R
      6'd22: g = {5'b01111, 5'b10000, 5'b10000, 5'b01110, 5'b00001, 5'b00001, 5'b11110};  // S
      6'd23: g = {5'b11111, 5'b00100, 5'b00100, 5'b00100, 5'b00100, 5'b00100, 5'b00100};  // T
      6'd28: g = {5'b10001, 5'b10001, 5'b10001, 5'b10001, 5'b10001, 5'b01010, 5'b00100};  // V
      default: g = 35'd0;
    endcase
    case (row)
      3'd0: ctl_glyph_row = g[34:30];
      3'd1: ctl_glyph_row = g[29:25];
      3'd2: ctl_glyph_row = g[24:20];
      3'd3: ctl_glyph_row = g[19:15];
      3'd4: ctl_glyph_row = g[14:10];
      3'd5: ctl_glyph_row = g[9:5];
      3'd6: ctl_glyph_row = g[4:0];
      default: ctl_glyph_row = 5'd0;
    endcase
  endfunction
  // Glyph pixel of code `code` for a cell whose left edge is `cell_x`.
  function automatic logic ctl_glyph_px(input logic [5:0] code, input logic [9:0] x,
                                        input logic [9:0] cell_x, input logic [2:0] row);
    logic [9:0] dx;
    logic [4:0] bits;
    dx = x - cell_x;
    bits = ctl_glyph_row(code, row);
    ctl_glyph_px = (dx < 10'd5) && bits[3'd4 - dx[2:0]];
  endfunction
  logic [11:0] ctl_wheel;
  logic [7:0]  ctl_axis_lx;
  logic [7:0]  ctl_gas, ctl_brake, ctl_clutch;
  logic [3:0]  ctl_gear_n;
  logic        ctl_key, ctl_abort, ctl_coin, ctl_service;
  logic        ctl_dsp_run, ctl_mainflag, ctl_soundflag, ctl_dac_act;
  logic [11:0] ctl_dac;
  assign ctl_dsp_run   = controls_i[76];
  assign ctl_mainflag  = controls_i[77];
  assign ctl_soundflag = controls_i[78];
  assign ctl_dac_act   = controls_i[79];
  assign ctl_dac       = controls_i[91:80];
  assign ctl_axis_lx   = controls_i[51:44];
  logic ctl_main_wr_act, ctl_main_rd_act;
  assign ctl_main_wr_act = controls_i[92];
  assign ctl_main_rd_act = controls_i[93];
  // Sound protocol row: [109:94] last command from the main CPU,
  // [110] COM RAM write activity, [111] CRAMEN, [119:112] last sample byte.
  // Sticky evidence: peak DAC excursion, COM RAM write count, DSP IRQ seen.
  logic [7:0] ctl_dac_peak, ctl_comram_count;
  logic       ctl_dsp_irq_seen;
  assign ctl_dac_peak     = controls_i[127:120];
  assign ctl_comram_count = controls_i[135:128];
  assign ctl_dsp_irq_seen = controls_i[136];
  logic [3:0] ctl_axis_moved;
  assign ctl_axis_moved = controls_i[140:137];
  logic [7:0] ctl_pc_page;
  assign ctl_pc_page = controls_i[148:141];
  // Sticky first-timeout snapshot from the GSP clock domain.  On a timeout
  // the ROM promptly halts the GSP; these preserved values identify the
  // overdue instruction and FSM state instead of that later halt state.
  logic        ctl_gsp_timeout_valid, ctl_gsp_timeout_illegal;
  logic [23:0] ctl_gsp_timeout_pc;
  logic [5:0]  ctl_gsp_timeout_state;
  logic [15:0] ctl_gsp_timeout_instr, ctl_gsp_timeout_hint_age;
  assign ctl_gsp_timeout_pc       = controls_i[67:44];
  assign ctl_gsp_timeout_state    = controls_i[73:68];
  assign ctl_gsp_timeout_instr    = controls_i[89:74];
  assign ctl_gsp_timeout_hint_age = controls_i[105:90];
  assign ctl_gsp_timeout_illegal  = controls_i[106];
  assign ctl_gsp_timeout_valid    = controls_i[149];

  // Screen-split detector: the game switches the palette bank (and the fine
  // scroll) part way down the frame for the dashboard. Once such a lower-
  // screen transition has been seen, a frame without one is a split miss.
  // Keep enough sticky/recent evidence to catch a one-frame glitch in a
  // later native screenshot.
  logic [1:0] bank_last_q;
  logic       split_miss_q;
  logic [7:0] split_miss_count_q;
  logic [8:0] frame_split_line_q;
  logic [8:0] split_line_last_q;
  logic [8:0] split_line_window_max_q;
  logic [8:0] split_line_recent_max_q;
  logic [5:0] split_rate_frames_q;
  logic [7:0] split_miss_window_q;
  logic [7:0] split_miss_rate_q;
  logic       split_monitor_armed_q;
  // Arm only after the game has produced a real lower-screen palette
  // transition once. This avoids treating boot/self-test frames as misses
  // and cannot be poisoned by an unusually busy startup frame.
  wire frame_split_missed = split_monitor_armed_q &&
                            (frame_split_line_q == 9'd0);
  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      bank_last_q        <= 2'd0;
      split_miss_q       <= 1'b0;
      split_miss_count_q <= 8'd0;
      frame_split_line_q <= 9'd0;
      split_line_last_q <= 9'd0;
      split_line_window_max_q <= 9'd0;
      split_line_recent_max_q <= 9'd0;
      split_rate_frames_q <= 6'd0;
      split_miss_window_q <= 8'd0;
      split_miss_rate_q <= 8'd0;
      split_monitor_armed_q <= 1'b0;
    end else begin
      bank_last_q <= palette_bank_i;
      if (palette_bank_i != bank_last_q) begin
        // Ignore the top-of-frame bank selection. The later active-display
        // change is the dashboard split whose nominal line is about 208.
        if (raster_v_q >= 9'd128 && raster_v_q < 9'd288) begin
          frame_split_line_q <= raster_v_q;
        end
      end
      if (frame_start_o) begin
        split_line_last_q <= frame_split_line_q;
        frame_split_line_q <= 9'd0;
        if (frame_split_line_q != 9'd0) split_monitor_armed_q <= 1'b1;
        if (frame_split_missed) begin
          split_miss_q <= 1'b1;
          if (!(&split_miss_count_q)) split_miss_count_q <= split_miss_count_q + 8'd1;
        end

        // The Compact raster is about 60 Hz, so 60 frames form a useful
        // one-second diagnostic window. Include the frame that closes it in
        // both the miss count and the worst observed split line.
        if (split_rate_frames_q == 6'd59) begin
          split_rate_frames_q <= 6'd0;
          split_miss_rate_q <= split_miss_window_q +
                               {{7{1'b0}}, frame_split_missed};
          split_miss_window_q <= 8'd0;
          split_line_recent_max_q <=
            (frame_split_line_q > split_line_window_max_q)
              ? frame_split_line_q : split_line_window_max_q;
          split_line_window_max_q <= 9'd0;
        end else begin
          split_rate_frames_q <= split_rate_frames_q + 6'd1;
          if (frame_split_missed && !(&split_miss_window_q))
            split_miss_window_q <= split_miss_window_q + 8'd1;
          if (frame_split_line_q > split_line_window_max_q)
            split_line_window_max_q <= frame_split_line_q;
        end
      end
    end
  end
  logic [7:0]  ctl_brake_depth, ctl_len_ga, ctl_len_br, ctl_len_cl;
  logic [8:0]  ctl_brake_scaled;
  assign ctl_wheel   = controls_i[11:0];
  assign ctl_gas     = controls_i[19:12];
  assign ctl_brake   = controls_i[27:20];
  assign ctl_clutch  = controls_i[35:28];
  assign ctl_gear_n  = controls_i[39:36];
  assign ctl_key     = controls_i[40];
  assign ctl_abort   = controls_i[41];
  assign ctl_coin    = controls_i[42];
  assign ctl_service = controls_i[43];
  assign ctl_brake_depth  = (ctl_brake < 8'hf0) ? (8'hf0 - ctl_brake) : 8'h00;   // 0..d0
  assign ctl_brake_scaled = {1'b0, ctl_brake_depth[7:1]} + {3'b000, ctl_brake_depth[7:3]};  // x 0.625
  assign ctl_len_ga = {1'b0, ctl_gas[7:1]};
  assign ctl_len_br = (ctl_brake_scaled > 9'd127) ? 8'd127 : ctl_brake_scaled[7:0];
  assign ctl_len_cl = {1'b0, ctl_clutch[7:1]};
  logic        ctl_hit;
  logic [23:0] ctl_rgb;
  logic        ctl_hit_q;
  logic [23:0] ctl_rgb_q;
  always_comb begin
    logic [9:0] cx;
    logic [8:0] cy;
    logic [2:0] row;
    logic       b_st, b_ga, b_br, b_cl, b_in, b_sn, b_pk;
    logic [9:0] dot_x, raw_dot_x;
    logic [7:0] bar_len;
    logic [23:0] bar_rgb;
    logic [3:0] gear_sel;
    logic [2:0] gk;
    logic [9:0] cell_x;
    logic       cell_lit;
    logic [5:0] cell_code;
    cx = captured_x_q;
    cy = captured_line_q;
    ctl_hit = 1'b0;
    ctl_rgb = 24'h181838;
    gk = 3'd7;
    cell_x = 10'd0;
    cell_lit = 1'b0;
    cell_code = 6'd0;
    b_st = (cy >= 9'd20) && (cy < 9'd28);
    b_ga = (cy >= 9'd29) && (cy < 9'd37);
    b_br = (cy >= 9'd38) && (cy < 9'd46);
    b_cl = (cy >= 9'd47) && (cy < 9'd55);
    b_in = (cy >= 9'd56) && (cy < 9'd64);
    b_sn = (cy >= 9'd65) && (cy < 9'd73);
    b_pk = (cy >= 9'd74) && (cy < 9'd82);
    row = b_st ? (cy[2:0] - 3'd4) :          // 20 -> 0
          b_ga ? (cy[2:0] - 3'd5) :          // 29 -> 0
          b_br ? (cy[2:0] - 3'd6) :          // 38 -> 0
          b_cl ? (cy[2:0] - 3'd7) :          // 47 -> 0
          b_sn ? (cy[2:0] - 3'd1) :          // 65 -> 0
          b_pk ? (cy[2:0] - 3'd2) :          // 74 -> 0
                 cy[2:0];                    // 56 -> 0
    // The wheel track is 128 px wide like the bars, so the dot always stays
    // inside the panel (32 counts per pixel over the full 12-bit range).
    dot_x = 10'd32 + {3'b000, ctl_wheel[11:5]};
    raw_dot_x = 10'd32 + {3'b000, ctl_axis_lx[7:1]};
    bar_len = b_ga ? ctl_len_ga : b_br ? ctl_len_br : ctl_len_cl;
    bar_rgb = b_ga ? 24'h30d030 : b_br ? 24'he03030 : 24'h4060ff;
    gear_sel = ~ctl_gear_n;
    if (controls_enable_i && (cx >= CTL_PX0) && (cx < CTL_PX1) &&
        (cy >= CTL_PY0) && (cy < CTL_PY1)) begin
      ctl_hit = 1'b1;
      // Two-letter labels at x 8..23.
      if (cx < 10'd24) begin
        cell_code = (cx < 10'd16)
                  ? (b_st ? 6'd22 : b_ga ? 6'd17 : b_br ? 6'd11 : b_cl ? 6'd12 :
                     b_sn ? (ctl_gsp_timeout_valid ? 6'd17 : 6'd22) :
                     b_pk ? (ctl_gsp_timeout_valid ? 6'd17 : 6'd13) : 6'd17)
                  : (b_st ? 6'd23 : b_ga ? 6'd10 : b_br ? 6'd21 : b_cl ? 6'd19 :
                     b_sn ? (ctl_gsp_timeout_valid ? 6'd13 : 6'd20) :
                     b_pk ? (ctl_gsp_timeout_valid ? 6'd1 : 6'd18) : 6'd21);
        if (ctl_glyph_px(cell_code, cx, (cx < 10'd16) ? 10'd8 : 10'd16, row))
          ctl_rgb = 24'hf8f8f8;
      end else if (b_st) begin
        if ((cx >= 10'd32) && (cx < 10'd160)) begin
          // Yellow is the exact 12-bit value returned to the game. Cyan is
          // MiSTer's raw P1 left-X axis before deadzone/end-stop handling.
          // Seeing both separately identifies a host mapping or centering
          // problem without guessing from the car's movement.
          if ((row >= 3'd4) && (row <= 3'd6) &&
              (cx + 10'd2 >= dot_x) && (cx <= dot_x + 10'd2))
            ctl_rgb = 24'hf0e020;                       // wheel dot
          else if ((row >= 3'd1) && (row <= 3'd2) &&
                   (cx + 10'd2 >= raw_dot_x) && (cx <= raw_dot_x + 10'd2))
            ctl_rgb = 24'h40c0ff;                       // raw-axis dot
          else if ((cx == 10'd96) && (row >= 3'd1) && (row <= 3'd5))
            ctl_rgb = 24'hb0b0c0;                       // centre tick
          else if (row == 3'd3)
            ctl_rgb = 24'h707080;                       // track
        end
      end else if (b_ga || b_br || b_cl) begin
        if ((cx >= 10'd32) && (cx < 10'd160) && (row >= 3'd1) && (row <= 3'd6))
          ctl_rgb = ((cx - 10'd32) < {2'b00, bar_len}) ? bar_rgb : 24'h202030;
      end else if (b_in) begin
        // Gear cells N 1 2 3 4 at x 32,42,52,62,72 (8 px each).
        if ((cx >= 10'd32) && (cx < 10'd80)) begin
          if (cx < 10'd40) begin gk = 3'd0; cell_x = 10'd32; end
          else if ((cx >= 10'd42) && (cx < 10'd50)) begin gk = 3'd1; cell_x = 10'd42; end
          else if ((cx >= 10'd52) && (cx < 10'd60)) begin gk = 3'd2; cell_x = 10'd52; end
          else if ((cx >= 10'd62) && (cx < 10'd70)) begin gk = 3'd3; cell_x = 10'd62; end
          else if ((cx >= 10'd72) && (cx < 10'd80)) begin gk = 3'd4; cell_x = 10'd72; end
          if (gk != 3'd7) begin
            cell_lit = (gk == 3'd0) ? (gear_sel == 4'b0000) : gear_sel[gk - 3'd1];
            cell_code = (gk == 3'd0) ? 6'd20 : {3'b000, gk};
            if (ctl_glyph_px(cell_code, cx - 10'd1, cell_x, row))   // 1-px left margin
              ctl_rgb = cell_lit ? 24'h181838 : 24'h9090a0;
            else
              ctl_rgb = cell_lit ? 24'hf0f0f0 : 24'h202030;
          end
        end else if ((cx >= 10'd96) && (cx < 10'd104)) begin
          if (ctl_glyph_px(6'd18, cx, 10'd96, row)) ctl_rgb = 24'hf8f8f8;     // K
        end else if ((cx >= 10'd104) && (cx < 10'd112)) begin
          ctl_rgb = ctl_key ? 24'hf08020 : 24'h202030;
        end else if ((cx >= 10'd120) && (cx < 10'd128)) begin
          if (ctl_glyph_px(6'd10, cx, 10'd120, row)) ctl_rgb = 24'hf8f8f8;    // A
        end else if ((cx >= 10'd128) && (cx < 10'd136)) begin
          ctl_rgb = ctl_abort ? 24'he03030 : 24'h202030;
        end else if ((cx >= 10'd144) && (cx < 10'd152)) begin
          if (ctl_glyph_px(6'd12, cx, 10'd144, row)) ctl_rgb = 24'hf8f8f8;    // C
        end else if ((cx >= 10'd152) && (cx < 10'd160)) begin
          ctl_rgb = ctl_coin ? 24'hf0f0f0 : 24'h202030;
        end else if ((cx >= 10'd168) && (cx < 10'd176)) begin
          if (ctl_glyph_px(6'd28, cx, 10'd168, row)) ctl_rgb = 24'hf8f8f8;    // V
        end else if ((cx >= 10'd176) && (cx < 10'd184)) begin
          ctl_rgb = ctl_service ? 24'hf0f0f0 : 24'h202030;
        end
      end else if (b_sn) begin
        if (ctl_gsp_timeout_valid) begin
          // "GP": timeout PC (24 cyan bits), core state (6 yellow bits),
          // and a magenta valid lamp.  Four screen pixels encode each bit.
          if ((cx >= 10'd32) && (cx < 10'd128) && (row >= 3'd1) && (row <= 3'd6)) begin
            ctl_rgb = ctl_gsp_timeout_pc[5'd23 - ((cx - 10'd32) >> 2)]
                      ? 24'h30d0d0 : 24'h202030;
          end else if ((cx >= 10'd132) && (cx < 10'd156) && (row >= 3'd1) && (row <= 3'd6)) begin
            ctl_rgb = ctl_gsp_timeout_state[3'd5 - ((cx - 10'd132) >> 2)]
                      ? 24'hf0e020 : 24'h202030;
          end else if ((cx >= 10'd184) && (cx < 10'd192)) begin
            ctl_rgb = 24'hf0a0f0;
          end
        end else begin
        // "SN": previous frame's split line, latest split line seen during
        // the last ~1 second, misses during that second, total misses, and a
        // sticky red fault lamp. Nine-bit line values use four pixels/bit.
        if ((cx >= 10'd32) && (cx < 10'd68) && (row >= 3'd1) && (row <= 3'd6)) begin
          ctl_rgb = split_line_last_q[4'd8 - ((cx - 10'd32) >> 2)]
                    ? 24'h30d0d0 : 24'h202030;
        end else if ((cx >= 10'd72) && (cx < 10'd108) && (row >= 3'd1) && (row <= 3'd6)) begin
          ctl_rgb = split_line_recent_max_q[4'd8 - ((cx - 10'd72) >> 2)]
                    ? 24'h40c0ff : 24'h202030;
        end else if ((cx >= 10'd112) && (cx < 10'd144) && (row >= 3'd1) && (row <= 3'd6)) begin
          ctl_rgb = split_miss_rate_q[3'd7 - ((cx - 10'd112) >> 2)]
                    ? 24'he05050 : 24'h202030;
        end else if ((cx >= 10'd148) && (cx < 10'd180) && (row >= 3'd1) && (row <= 3'd6)) begin
          ctl_rgb = split_miss_count_q[3'd7 - ((cx - 10'd148) >> 2)]
                    ? 24'hf0e020 : 24'h202030;
        end else if ((cx >= 10'd184) && (cx < 10'd192)) begin
          ctl_rgb = split_miss_q ? 24'hf02020 : 24'h402020;
        end
        end
      end else if (b_pk) begin
        if (ctl_gsp_timeout_valid) begin
          // "G1": faulting instruction word (16 magenta bits), elapsed ms
          // since the preceding HINT (16 red bits), and illegal-opcode lamp.
          if ((cx >= 10'd32) && (cx < 10'd96) && (row >= 3'd1) && (row <= 3'd6)) begin
            ctl_rgb = ctl_gsp_timeout_instr[4'd15 - ((cx - 10'd32) >> 2)]
                      ? 24'hf0a0f0 : 24'h202030;
          end else if ((cx >= 10'd100) && (cx < 10'd164) && (row >= 3'd1) && (row <= 3'd6)) begin
            ctl_rgb = ctl_gsp_timeout_hint_age[4'd15 - ((cx - 10'd100) >> 2)]
                      ? 24'he05050 : 24'h202030;
          end else if ((cx >= 10'd168) && (cx < 10'd176)) begin
            ctl_rgb = ctl_gsp_timeout_illegal ? 24'hf02020 : 24'h204020;
          end
        end else begin
        // "PK": peak DAC excursion since reset (8 cells), COM RAM write
        // count (8 cells, saturating), and whether the DSP has interrupted
        // the sound 68000.  All three are sticky, so they can be read long
        // after the event that set them.
        if ((cx >= 10'd32) && (cx < 10'd64) && (row >= 3'd1) && (row <= 3'd6)) begin
          ctl_rgb = ctl_dac_peak[3'd7 - cx[4:2]] ? 24'h30d0d0 : 24'h202030;
        end else if ((cx >= 10'd68) && (cx < 10'd100) && (row >= 3'd1) && (row <= 3'd6)) begin
          ctl_rgb = ctl_comram_count[3'd7 - cx[4:2]] ? 24'hf0e020 : 24'h202030;
        end else if ((cx >= 10'd104) && (cx < 10'd112)) begin
          ctl_rgb = ctl_dsp_irq_seen ? 24'h30d030 : 24'h602020;
        end else if ((cx >= 10'd156) && (cx < 10'd188) && (row >= 3'd1) && (row <= 3'd6)) begin
          // Sound 68000 program page: which routine it is executing.
          ctl_rgb = ctl_pc_page[3'd7 - cx[4:2]] ? 24'hf0a0f0 : 24'h202030;
        end else if ((cx >= 10'd120) && (cx < 10'd152)) begin
          // Which analog axes have ever moved: LX, LY, RX, RY.  Squeeze a
          // trigger and the axis it drives lights up here for good.
          ctl_rgb = ctl_axis_moved[cx[4:3]] ? 24'h40c0ff : 24'h202030;
        end
        end
      end
    end
  end
  logic [7:0] pixel_index_q;
  assign line_o = captured_line_q;
  assign x_o    = captured_x_q;
  logic [1:0] programmed_hblank_sync_q;
  logic [1:0] programmed_vblank_sync_q;
  logic programmed_hblank_prev_q;
  logic programmed_vblank_prev_q;
  logic programmed_frame_pending_q;
  logic raster_line_advanced_q;

  wire raw_hactive_start = programmed_hblank_prev_q &&
                                  !programmed_hblank_sync_q[1];
  logic [1:0] source_hsync_sync_q;
  logic source_hsync_prev_q, sync_origin_valid_q;
  logic [12:0] sync_origin_delay_q;
  wire source_hsync_start = source_hsync_sync_q[1] && !source_hsync_prev_q;
  wire programmed_hactive_start = COCKPIT && sync_origin_valid_q ?
                                  sync_origin_delay_q == 13'd1 : raw_hactive_start;
  // HSYNC starts at TMS HCOUNT=0. One VCLK is four pixels / twelve
  // core clocks. Use the frame viewport's HEB, never a split's live HEB.
  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      source_hsync_sync_q <= '0;
      source_hsync_prev_q <= 1'b0;
      sync_origin_valid_q <= 1'b0;
      sync_origin_delay_q <= '0;
    end else begin
      source_hsync_sync_q <= {source_hsync_sync_q[0], video_hsync_i};
      source_hsync_prev_q <= source_hsync_sync_q[1];
      if (source_hsync_start && COCKPIT) begin
        sync_origin_valid_q <= 1'b1;
        sync_origin_delay_q <= 13'((sync_viewport_heblnk_q + 16'd1) * 12);
      end else if (sync_origin_delay_q != 0)
        sync_origin_delay_q <= sync_origin_delay_q - 1'b1;
    end
  end

  wire programmed_vactive_start = programmed_vblank_prev_q &&
                                  !programmed_vblank_sync_q[1];

  // Palette/fine-scroll state follows the matching buffered pixel row. A live
  // HACTIVE sample can belong to the following GSP row while SDRAM is still
  // publishing the preceding one; that mismatch makes the dashboard both
  // green and horizontally wrapped for a frame.
  logic [1:0] display_palette_bank;
  logic [3:0] display_fine_scroll;

  assign pixel_offset = COCKPIT ? ({1'b0, visible_x_q} + {10'd0, !display_fine_scroll[0]})
                      : {2'b00, active_col_q, 1'b0}
                      + {1'b0, visible_x_q}
                      - 11'd7
                      + {7'd0, display_fine_scroll};
  assign pixel_word_addr = {2'b00, pixel_offset[8:1]};
  logic [9:0] palette_addr_q;
  assign palette_addr_o = COCKPIT
    ? {display_palette_bank, pixel_byte_select_q ? row_word_q[15:8] : row_word_q[7:0]}
    : palette_addr_q;
  assign row_read_addr = {active_bank_q, serial_line_q, pixel_word_addr[7:0]};

  always_ff @(posedge core_clk_i) begin
    if (core_rst_i) begin
      consumed_toggle_q <= 1'b0;
      completed_slices_sync1_q <= 8'd0;
      completed_slices_sync2_q <= 8'd0;
      publish_sync_q     <= 2'b00;
      publish_seen_q     <= 1'b0;
      accepted_sync_q    <= 2'b00;
      accepted_seen_q    <= 1'b0;
      pending_bank_q     <= 1'b0;
      pending_col_q      <= 8'd0;
      pending_palette_bank_q <= 2'd0;
      pending_fine_scroll_q  <= 4'd0;
      pending_clip_left_q <= 10'd0;
      display_clip_left_q <= 10'd0;
      pending_lcstrt_q <= 2'd0;
      active_lcstrt_q <= 2'd0;
      serial_line_q <= 2'd0;
      active_bank_q      <= 1'b0;
      active_col_q       <= 8'd0;
      pending_row_q      <= 1'b0;
      row_valid_q        <= 1'b0;
      raster_h_q         <= 10'd0;
      raster_v_q         <= 9'd0;
      visible_x_q        <= 10'd0;
      pixel_phase_q      <= 2'd0;
      pixel_byte_select_q <= 1'b0;
      palette_addr_q     <= 10'd0;
      pixel_index_q      <= 8'd0;
      ce_pix_o           <= 1'b0;
      hsync_o            <= 1'b0;
      vsync_o            <= 1'b0;
      hblank_o           <= 1'b1;
      vblank_o           <= 1'b1;
      rgb_o              <= 24'd0;
      frame_start_o      <= 1'b0;
      loaded_srfaddr_o   <= 14'd0;
      loaded_org_o       <= 1'b0;
      loaded_col_o       <= 8'd0;
      loaded_row_select_o <= 8'd0;
      loaded_row_or_o    <= 16'd0;
      loaded_row_nonzero_o <= 11'd0;
      row_valid_o        <= 1'b0;
      screen_ack_o       <= 1'b0;
      programmed_hblank_sync_q <= 2'b11;
      programmed_vblank_sync_q <= 2'b11;
      programmed_hblank_prev_q <= 1'b1;
      programmed_vblank_prev_q <= 1'b1;
      programmed_frame_pending_q <= 1'b0;
      raster_line_advanced_q <= 1'b0;
      display_palette_bank <= 2'd0;
      display_fine_scroll <= 4'd0;
    end else begin
      completed_slices_sync1_q <= completed_slices_q;
      completed_slices_sync2_q <= completed_slices_sync1_q;
      publish_sync_q <= {publish_sync_q[0], published_toggle_q};
      accepted_sync_q <= {accepted_sync_q[0], accepted_toggle_q};
      programmed_hblank_sync_q <= {
        programmed_hblank_sync_q[0], video_hblank_i
      };
      programmed_vblank_sync_q <= {
        programmed_vblank_sync_q[0], video_vblank_i
      };
      programmed_hblank_prev_q <= programmed_hblank_sync_q[1];
      programmed_vblank_prev_q <= programmed_vblank_sync_q[1];
      ce_pix_o       <= 1'b0;
      frame_start_o  <= 1'b0;
      screen_ack_o   <= 1'b0;

      if (programmed_vactive_start)
        programmed_frame_pending_q <= 1'b1;

      if (accepted_sync_q[1] != accepted_seen_q) begin
        accepted_seen_q <= accepted_sync_q[1];
        screen_ack_o <= 1'b1;
      end

      if (publish_sync_q[1] != publish_seen_q) begin
        publish_seen_q <= publish_sync_q[1];
        pending_bank_q <= published_bank_q;
        pending_col_q  <= published_col_q;
        pending_palette_bank_q <= published_palette_bank_q;
        pending_fine_scroll_q  <= published_fine_scroll_q;
        pending_clip_left_q <= published_clip_left_q;
        pending_lcstrt_q <= published_lcstrt_q;
        pending_row_q  <= 1'b1;
        loaded_srfaddr_o <= published_srfaddr_q;
        loaded_org_o <= published_org_q;
        loaded_col_o <= published_col_q;
        loaded_row_select_o <= published_row_select_q;
        loaded_row_or_o <= published_row_or_q;
        loaded_row_nonzero_o <= published_row_nonzero_q;
      end

      pixel_phase_q <= (COCKPIT && pixel_phase_q == 2'd2) ? 2'd0 : pixel_phase_q + 2'd1;
      case (pixel_phase_q)
        2'd0: begin
          pixel_byte_select_q <= pixel_offset[0];
          captured_blank_q  <= !video_enable_i || (raster_h_q >= H_VISIBLE) ||
                               (raster_v_q >= V_VISIBLE) || !row_valid_q ||
                               (COCKPIT && visible_x_q < display_clip_left_q);
          captured_hblank_q <= raster_h_q >= H_VISIBLE;
          captured_vblank_q <= raster_v_q >= V_VISIBLE;
          captured_hsync_q  <= (raster_h_q >= (COCKPIT ? 10'd532 : 10'd536)) &&
                               (raster_h_q < (COCKPIT ? 10'd596 : 10'd600));
          captured_vsync_q  <= (raster_v_q >= (COCKPIT ? 9'd388 : 9'd292)) &&
                               (raster_v_q < (COCKPIT ? 9'd392 : 9'd296));
          captured_x_q      <= visible_x_q;
          captured_line_q   <= raster_v_q;
        end
        2'd1: begin
          pixel_index_q  <= pixel_byte_select_q
                          ? row_word_q[15:8] : row_word_q[7:0];
          palette_addr_q <= {display_palette_bank,
                             pixel_byte_select_q
                             ? row_word_q[15:8] : row_word_q[7:0]};
        end
        2'd2: begin
          ctl_hit_q <= ctl_hit;
          ctl_rgb_q <= ctl_rgb;
        end
        default: ;
      endcase
      if (pixel_phase_q == (COCKPIT ? 2'd2 : 2'd3)) begin
          ce_pix_o  <= 1'b1;
          hsync_o   <= captured_hsync_q;
          vsync_o   <= captured_vsync_q;
          hblank_o  <= captured_hblank_q;
          vblank_o  <= captured_vblank_q;
          if (captured_blank_q) begin
            rgb_o <= 24'h000000;
`ifdef HD_VIDEO_DIAGNOSTICS
          end else if (captured_line_q < 16 && captured_x_q < 508) begin
            rgb_o <= video_debug_bit_q ? 24'h00d040 : 24'hb01818;
`endif
          end else if (COCKPIT ? ctl_hit : ctl_hit_q) begin
            rgb_o <= COCKPIT ? ctl_rgb : ctl_rgb_q;
          end else if (diagnostic_enable_i &&
                       (captured_line_q < 9'd16) &&
                       (captured_x_q < 10'd512)) begin
            // arcade_video accepts every other source pixel on real hardware.
            // Repeat each bit across four source pixels so all 128 cells survive
            // capture, and repeat the same page vertically for robust sampling.
            rgb_o <= diagnostic_i[captured_x_q[8:2]]
                   ? 24'h00d040 : 24'hb01818;
          end else begin
            rgb_o <= {palette_lo_i[15:8], palette_lo_i[7:0],
                      palette_hi_i[7:0]};
          end
          if (!captured_hblank_q && !captured_vblank_q)
            visible_x_q <= visible_x_q + 10'd1;

          if (raster_h_q == H_LAST) begin
            raster_line_advanced_q <= 1'b1;
            raster_h_q  <= 10'd0;
            visible_x_q <= 10'd0;
            if (raster_v_q == V_LAST) begin
              raster_v_q    <= 9'd0;
              frame_start_o <= 1'b1;
            end else begin
              raster_v_q <= raster_v_q + 9'd1;
            end

            // A completed Compact screen transfer replaces exactly one
            // 512-pixel (256-word) serial row at a line boundary.
            // Multiline groups advance only on programmed HACTIVE. The
            // fallback raster edge can occur in the same line and must not
            // consume a second slice or promote a group before that edge.
            if (pending_row_q && pending_lcstrt_q == 2'd0) begin
              consumed_toggle_q <= publish_seen_q;
              active_bank_q  <= pending_bank_q;
              active_lcstrt_q <= pending_lcstrt_q;
              serial_line_q <= 2'd0;
              active_col_q   <= pending_col_q;
              display_palette_bank <= pending_palette_bank_q;
              display_fine_scroll  <= pending_fine_scroll_q;
              display_clip_left_q <= pending_clip_left_q;
              pending_row_q  <= 1'b0;
              row_valid_q    <= 1'b1;
              row_valid_o    <= 1'b1;
            end
          end else begin
            raster_h_q <= raster_h_q + 10'd1;
          end
      end

      // The fixed MiSTer raster is a blanking-rotated form of the ROM's
      // programmed 323x308 timing. Lock its active-line origin to the real
      // TMS34010 HBLANK edge, and start row zero on the first active line.
      // Without this phase lock, register programming after reset leaves an
      // arbitrary vertical rotation that cuts the lower text off-screen.
      if (programmed_hactive_start) begin
        // An HACTIVE edge can arrive a core cycle before the local wrap.
        // Resetting H alone then loses this line's vertical increment.
        // Count it here only if the local raster has not already done so.
        // A simultaneous local wrap makes the same assignment, never two.
        raster_line_advanced_q <= 1'b0;
        if (!raster_line_advanced_q) begin
          if (raster_v_q == V_LAST) begin
            raster_v_q <= 9'd0;
            frame_start_o <= 1'b1;
          end else begin
            raster_v_q <= raster_v_q + 9'd1;
          end
        end
        raster_h_q    <= 10'd0;
        visible_x_q   <= 10'd0;
        pixel_phase_q <= 2'd0;
        // The TMS HBLANK falling edge is the real serializer's row boundary.
        // Promote a completed physical row here as well as at the local
        // 646-clock boundary.  At the programmed frame edge these two phase
        // references can differ by one line; waiting only for raster_h=645
        // leaves the previous frame's bottom row active for visible line 0,
        // which the HDMI scaler expands into a coloured band at the top.
        if (pending_row_q && !(pending_lcstrt_q != 2'd0 &&
              (programmed_frame_pending_q || programmed_vactive_start))) begin
          consumed_toggle_q <= publish_seen_q;
          active_bank_q <= pending_bank_q;
          active_lcstrt_q <= pending_lcstrt_q;
          serial_line_q <= 2'd0;
          active_col_q  <= pending_col_q;
          display_palette_bank <= pending_palette_bank_q;
          display_fine_scroll  <= pending_fine_scroll_q;
          display_clip_left_q <= pending_clip_left_q;
          pending_row_q <= 1'b0;
          row_valid_q   <= 1'b1;
          row_valid_o   <= 1'b1;
        end else if (!pending_row_q && row_valid_q && active_lcstrt_q != 2'd0 &&
                     !programmed_frame_pending_q && !programmed_vactive_start &&
                     !programmed_vblank_sync_q[1]) begin
          // LCSTRT counts the lines supplied by one screen transfer. Never
          // reread slice zero for all four service lines (the b96 font fault).
          // Saturate if a following transfer is late rather than read an
          // unwritten slice outside this descriptor.
          if (serial_line_q < active_lcstrt_q && (!COCKPIT ||
              completed_slices_sync2_q[{active_bank_q, 2'(serial_line_q + 2'd1)}]))
            serial_line_q <= serial_line_q + 2'd1;
        end
        if (programmed_frame_pending_q || programmed_vactive_start) begin
          // The 4461 transfer for source row zero is issued at this edge and
          // completes during the following line.  Starting visible row zero
          // immediately therefore reuses the preceding frame's final row for
          // one line.  Use the last blank line as a one-line serializer
          // pre-roll; its boundary promotes the newly loaded row and performs
          // the normal 307 -> 0 wrap/frame pulse.  This preserves the fixed
          // 646x308 output timing while making the first visible line row 0.
          raster_v_q                  <= V_LAST;
          frame_start_o               <= 1'b0;
          programmed_frame_pending_q  <= 1'b0;
        end
      end
    end
  end

`ifdef HD_VIDEO_DIAGNOSTICS
`include "rtl/harddrivin_scanout_debug.svh"
`endif
  logic unused_overflow;
  logic [7:0] unused_pixel_index;
  logic [4:0] unused_programmed_video;
  assign unused_overflow = scan_overflow_i;
  assign unused_pixel_index = pixel_index_q;
  assign unused_programmed_video = {
    video_hsync_i, video_vsync_i, video_hblank_i, video_vblank_i,
    video_blank_i
  };
endmodule

`default_nettype wire

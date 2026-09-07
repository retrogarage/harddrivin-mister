`default_nettype none
// Hard Drivin' "driver sound" board (Atari 136052-1150 family).
//
// A 68000 at 8 MHz runs the sound sequencer from a 64 KiB EPROM pair, loads
// the TMS32010 program into a 4 KiW RAM, fills a 512-word communication RAM,
// and releases the DSP through an LS259 latch.  The DSP streams 8-bit
// samples from four serial EPROMs (a byte per IN 0, auto-incrementing
// address written through OUT 6/7), mixes them and writes the 12-bit DAC on
// OUT 0 at 20 kHz, pacing itself on BIO.  The main CPU talks to the 68000
// through a data latch and two flags.  Register semantics follow the
// schematics as documented by MAME's harddriv_a.cpp; everything runs on the
// 48 MHz core clock.  EPROM images are read through two DDR ROM clients
// (harddrivin_ddr_rom request/ack toggles).
module harddrivin_sound_board (
  input  logic        clk_i,
  input  logic        rst_i,
  input  logic        board_reset_i,   // pulse: main CPU sound reset
  // Main CPU latch interface.
  input  logic        main_wr_i,       // pulse: main CPU wrote 0x840000
  input  logic [15:0] main_wdata_i,
  input  logic        main_rd_i,       // pulse: main CPU read 0x840000
  output logic [15:0] main_rdata_o,    // sound -> main data latch
  output logic        mainflag_o,      // main -> sound data pending
  output logic        soundflag_o,     // sound -> main data pending
  // 68000 EPROM pair (64 KiB) in on-chip RAM, written by the MiSTer ROM
  // download (its own clock).  The sound CPU fetches every instruction from
  // it, so it must not share the DDR port with the main board.
  input  logic        rom_wr_clk_i,
  input  logic        rom_wr_en_i,
  input  logic [14:0] rom_wr_addr_i,
  input  logic [15:0] rom_wr_data_i,
  // Serial sample EPROMs (256 KiB) through a DDR ROM client.
  output logic [20:1] ser_rom_addr_o,
  output logic        ser_rom_req_o,
  input  logic        ser_rom_ack_i,
  input  logic [15:0] ser_rom_data_i,
  // 12-bit DAC, sign-extended to 16 bits.
  output logic signed [15:0] audio_o,
  // Diagnostics.
  output logic [23:1] cpu_addr_o,
  output logic [11:0] dsp_addr_o,
  output logic        dsp_running_o,
  output logic [11:0] dac_o,          // last DAC value
  output logic        dac_write_o,
  // Protocol diagnostics: the last command the main CPU sent, whether the
  // 68000 has enabled and is writing the DSP's communication RAM, and the
  // last sample byte streamed from the serial EPROMs.
  output logic [15:0] last_cmd_o,
  output logic        cramen_o,
  output logic        comram_write_o,
  output logic [7:0]  ser_byte_o,
  // Sticky evidence that survives until it is read: the largest DAC
  // excursion since reset, how many words the 68000 has written into the
  // DSP's communication RAM, and whether the DSP has ever interrupted it.
  output logic [7:0]  dac_peak_o,
  output logic [7:0]  comram_count_o,
  output logic        dsp_irq_seen_o,
  // Where the sound 68000 actually is: the page of its last instruction
  // fetch.  If it is stuck in a wait loop this pins the routine down in the
  // ROM disassembly.  The COM RAM counter now counts in units of 256 words
  // so its rate is visible instead of saturating.
  output logic [7:0]  cpu_pc_page_o
);
  // DDR byte offset (relative to 0x30000000) of the sample image in the MRA.
  localparam logic [20:1] SER_ROM_WORD_BASE  = 20'ha8000;   // byte 0x150000

  // ---------------------------------------------------------------------
  // Reset and clock enables
  // ---------------------------------------------------------------------
  logic [7:0] cpu_reset_cnt_q;
  logic       cpu_reset;
  logic [2:0] phi_div_q;
  logic       en_phi1, en_phi2;

  always_ff @(posedge clk_i) begin
    if (rst_i || board_reset_i) cpu_reset_cnt_q <= 8'd255;
    else if (cpu_reset_cnt_q != 8'd0) cpu_reset_cnt_q <= cpu_reset_cnt_q - 8'd1;
  end
  assign cpu_reset = (cpu_reset_cnt_q != 8'd0);

  // 8 MHz 68000: one CPU clock every six core clocks.
  always_ff @(posedge clk_i) begin
    if (rst_i) phi_div_q <= 3'd0;
    else phi_div_q <= (phi_div_q == 3'd5) ? 3'd0 : phi_div_q + 3'd1;
  end
  assign en_phi1 = (phi_div_q == 3'd0);
  assign en_phi2 = (phi_div_q == 3'd3);

  // 20 MHz TMS32010 CLKIN: five enables per twelve core clocks.
  logic [3:0] dsp_div_q;
  logic       dsp_tick;
  always_ff @(posedge clk_i) begin
    if (rst_i) dsp_div_q <= 4'd0;
    else dsp_div_q <= (dsp_div_q == 4'd11) ? 4'd0 : dsp_div_q + 4'd1;
  end
  assign dsp_tick = (dsp_div_q == 4'd0) || (dsp_div_q == 4'd2) ||
                    (dsp_div_q == 4'd5) || (dsp_div_q == 4'd7) ||
                    (dsp_div_q == 4'd10);

  // ---------------------------------------------------------------------
  // Communication latches and control state
  // ---------------------------------------------------------------------
  logic [15:0] maindata_q;    // main -> sound
  logic [15:0] sounddata_q;   // sound -> main
  logic        mainflag_q;
  logic        soundflag_q;
  logic        irq68k_q;
  logic [7:0]  latch_q;       // LS259: 3 CRAMEN, 4 RES320 (1 = run), 7 LED
  logic        mute_q;
  logic [11:0] dac_q;
  logic [19:0] ser_offs_q;    // serial EPROM / COM RAM stream address
  logic        bio_flag_q;
  logic [11:0] bio_div_q;

  assign main_rdata_o = sounddata_q;
  assign mainflag_o   = mainflag_q;
  assign soundflag_o  = soundflag_q;

  // ---------------------------------------------------------------------
  // 68000
  // ---------------------------------------------------------------------
  logic        cpu_rwn, cpu_asn, cpu_ldsn, cpu_udsn;
  logic [2:0]  cpu_fc;
  logic        cpu_dtackn, cpu_vpan;
  logic [2:0]  cpu_ipln;
  logic [15:0] cpu_din, cpu_dout;
  logic [23:1] cpu_addr;
  logic        cpu_as;
  logic        cpu_as_q;
  logic        cpu_write_done_q;
  logic        cpu_read_done_q;
  logic        cpu_write_strobe;
  logic        cpu_read_strobe;

  fx68k u_cpu (
    .clk(clk_i), .HALTn(1'b1), .extReset(cpu_reset), .pwrUp(rst_i),
    .enPhi1(en_phi1), .enPhi2(en_phi2),
    .eRWn(cpu_rwn), .ASn(cpu_asn), .LDSn(cpu_ldsn), .UDSn(cpu_udsn),
    .E(), .VMAn(), .FC0(cpu_fc[0]), .FC1(cpu_fc[1]), .FC2(cpu_fc[2]),
    .BGn(), .oRESETn(), .oHALTEDn(),
    .DTACKn(cpu_dtackn), .VPAn(cpu_vpan), .BERRn(1'b1), .BRn(1'b1),
    .BGACKn(1'b1), .IPL0n(cpu_ipln[0]), .IPL1n(cpu_ipln[1]),
    .IPL2n(cpu_ipln[2]), .iEdb(cpu_din), .oEdb(cpu_dout), .eab(cpu_addr)
  );

  assign cpu_addr_o = cpu_addr;
  assign cpu_as = !cpu_asn && (cpu_fc != 3'b111);

  // Address decode.
  logic sel_rom, sel_data, sel_latch, sel_port320, sel_status, sel_dsp_ram;
  logic sel_dsp_ports, sel_comram, sel_ram, sel_local;
  assign sel_rom       = (cpu_addr[23:17] == 7'd0);
  assign sel_data      = (cpu_addr[23:12] == 12'hff0);
  assign sel_latch     = (cpu_addr[23:12] == 12'hff1);
  assign sel_port320   = (cpu_addr[23:12] == 12'hff2);
  assign sel_status    = (cpu_addr[23:12] == 12'hff3);
  assign sel_dsp_ram   = (cpu_addr[23:13] == 11'h7fa);   // ff4000-ff5fff
  assign sel_dsp_ports = (cpu_addr[23:13] == 11'h7fb);   // ff6000-ff7fff
  assign sel_comram    = (cpu_addr[23:14] == 10'h3fe);   // ff8000-ffbfff
  assign sel_ram       = (cpu_addr[23:14] == 10'h3ff);   // ffc000-ffffff
  assign sel_local     = !sel_rom;

  // Autovectored interrupts: level 3 = DSP request, level 1 = main data.
  logic [2:0] cpu_ipl;
  assign cpu_ipl  = irq68k_q ? 3'd3 : (mainflag_q ? 3'd1 : 3'd0);
  assign cpu_ipln = ~cpu_ipl;
  assign cpu_vpan = !(!cpu_asn && (cpu_fc == 3'b111));

  // One read/write action per bus cycle.
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      cpu_as_q         <= 1'b0;
      cpu_write_done_q <= 1'b0;
      cpu_read_done_q  <= 1'b0;
    end else begin
      cpu_as_q <= cpu_as;
      if (!cpu_as) begin
        cpu_write_done_q <= 1'b0;
        cpu_read_done_q  <= 1'b0;
      end else begin
        if (cpu_write_strobe) cpu_write_done_q <= 1'b1;
        if (cpu_read_strobe)  cpu_read_done_q  <= 1'b1;
      end
    end
  end
  assign cpu_write_strobe = cpu_as && !cpu_rwn && (!cpu_udsn || !cpu_ldsn) &&
                            !cpu_write_done_q;
  assign cpu_read_strobe  = cpu_as && cpu_rwn && !cpu_read_done_q;

  // EPROM in on-chip RAM: the word is valid one clock after the address, and
  // the 68000 holds AS for several clocks, so the read never waits.
  logic        rom_ready;
  logic        rom_rd_q;
  logic [15:0] rom_word;
  harddrivin_dual_clock_word_ram #(.ADDR_WIDTH(15)) u_code_rom (
    .wr_clk_i(rom_wr_clk_i), .wr_en_i(rom_wr_en_i),
    .wr_addr_i(rom_wr_addr_i), .wr_data_i(rom_wr_data_i),
    .rd_clk_i(clk_i), .rd_addr_i(cpu_addr[15:1]), .rd_data_o(rom_word)
  );
  always_ff @(posedge clk_i) begin
    if (rst_i) rom_rd_q <= 1'b0;
    else if (!cpu_as) rom_rd_q <= 1'b0;
    else if (sel_rom) rom_rd_q <= 1'b1;
  end
  assign rom_ready = rom_rd_q;
  // A 68000 read of DSP port 0 (serial EPROM byte) waits for the stream
  // prefetch like the DSP does; everything else answers at once.
  logic cpu_port0_wait;
  assign cpu_port0_wait = sel_dsp_ports && cpu_rwn && (cpu_addr[3:1] == 3'd0) &&
                          !ser_ready;
  assign cpu_dtackn = !(cpu_as && ((sel_local && !cpu_port0_wait) || rom_ready));

  // ---------------------------------------------------------------------
  // Memories
  // ---------------------------------------------------------------------
  // 68000 work RAM, 8 KiW, two byte lanes.
  logic [7:0]  ram_hi [0:8191];
  logic [7:0]  ram_lo [0:8191];
  logic [7:0]  ram_hi_q, ram_lo_q;
  always_ff @(posedge clk_i) begin
    if (cpu_write_strobe && sel_ram && !cpu_udsn) ram_hi[cpu_addr[13:1]] <= cpu_dout[15:8];
    ram_hi_q <= ram_hi[cpu_addr[13:1]];
  end
  always_ff @(posedge clk_i) begin
    if (cpu_write_strobe && sel_ram && !cpu_ldsn) ram_lo[cpu_addr[13:1]] <= cpu_dout[7:0];
    ram_lo_q <= ram_lo[cpu_addr[13:1]];
  end

  // TMS32010 program RAM, 4 KiW: port A = 68000 (byte lanes), port B = DSP
  // (fetch, TBLR, TBLW).  Two true-dual-port block RAMs, one per byte lane
  // (a hand-written two-write-port array is not inferred as RAM and costs
  // 65 kbit of registers).
  logic [11:0] dsp_addr;
  logic        dsp_menn, dsp_denn, dsp_wen;
  logic [15:0] dsp_din, dsp_dout;
  logic        dsp_prog_we;
  logic [15:0] dsp_ram_hi_a_w, dsp_ram_lo_a_w, dsp_ram_hi_b_w, dsp_ram_lo_b_w;
  logic [7:0]  dsp_ram_hi_a_q, dsp_ram_lo_a_q;
  logic [7:0]  dsp_ram_hi_b_q, dsp_ram_lo_b_q;
  harddrivin_true_dual_port_ram #(.ADDR_WIDTH(12)) u_dsp_ram_hi (
    .clk_i(clk_i),
    .en_a_i(1'b1), .we_a_i(cpu_write_strobe && sel_dsp_ram && !cpu_udsn),
    .addr_a_i(cpu_addr[12:1]), .wdata_a_i({8'h00, cpu_dout[15:8]}), .rdata_a_o(dsp_ram_hi_a_w),
    .en_b_i(1'b1), .we_b_i(dsp_prog_we),
    .addr_b_i(dsp_addr), .wdata_b_i({8'h00, dsp_dout[15:8]}), .rdata_b_o(dsp_ram_hi_b_w)
  );
  harddrivin_true_dual_port_ram #(.ADDR_WIDTH(12)) u_dsp_ram_lo (
    .clk_i(clk_i),
    .en_a_i(1'b1), .we_a_i(cpu_write_strobe && sel_dsp_ram && !cpu_ldsn),
    .addr_a_i(cpu_addr[12:1]), .wdata_a_i({8'h00, cpu_dout[7:0]}), .rdata_a_o(dsp_ram_lo_a_w),
    .en_b_i(1'b1), .we_b_i(dsp_prog_we),
    .addr_b_i(dsp_addr), .wdata_b_i({8'h00, dsp_dout[7:0]}), .rdata_b_o(dsp_ram_lo_b_w)
  );
  assign dsp_ram_hi_a_q = dsp_ram_hi_a_w[7:0];
  assign dsp_ram_lo_a_q = dsp_ram_lo_a_w[7:0];
  assign dsp_ram_hi_b_q = dsp_ram_hi_b_w[7:0];
  assign dsp_ram_lo_b_q = dsp_ram_lo_b_w[7:0];
  logic unused_dsp_ram_hi_bits;
  assign unused_dsp_ram_hi_bits = ^{dsp_ram_hi_a_w[15:8], dsp_ram_lo_a_w[15:8],
                                    dsp_ram_hi_b_w[15:8], dsp_ram_lo_b_w[15:8]};

  // Communication RAM, 512 words: port A = 68000 (when CRAMEN), port B = DSP.
  logic [15:0] comram [0:511];
  logic [15:0] comram_a_q, comram_b_q;
  logic        comram_we;
  logic [15:0] comram_wdata;
  assign comram_we = cpu_write_strobe && sel_comram && latch_q[3];
  assign comram_wdata = {cpu_udsn ? comram_a_q[15:8] : cpu_dout[15:8],
                         cpu_ldsn ? comram_a_q[7:0]  : cpu_dout[7:0]};
  always_ff @(posedge clk_i) begin
    if (comram_we) comram[cpu_addr[9:1]] <= comram_wdata;
    comram_a_q <= comram[cpu_addr[9:1]];
    comram_b_q <= comram[ser_offs_q[8:0]];
  end

  // ---------------------------------------------------------------------
  // Serial EPROM stream through the DDR client
  // ---------------------------------------------------------------------
  logic [19:1] ser_addr_q;
  logic        ser_issued_q;
  logic        ser_ready;
  logic [7:0]  ser_byte;
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      ser_rom_req_o  <= 1'b0;
      ser_rom_addr_o <= '0;
      ser_addr_q     <= '0;
      ser_issued_q   <= 1'b0;
    end else if ((ser_rom_req_o == ser_rom_ack_i) &&
                 (!ser_issued_q || (ser_addr_q != ser_offs_q[19:1]))) begin
      ser_issued_q   <= 1'b1;
      ser_addr_q     <= ser_offs_q[19:1];
      ser_rom_addr_o <= SER_ROM_WORD_BASE + {1'b0, ser_offs_q[19:1]};
      ser_rom_req_o  <= !ser_rom_req_o;
    end
  end
  assign ser_ready = ser_issued_q && (ser_rom_req_o == ser_rom_ack_i) &&
                     (ser_addr_q == ser_offs_q[19:1]);
  assign ser_byte  = ser_offs_q[0] ? ser_rom_data_i[7:0] : ser_rom_data_i[15:8];

  // ---------------------------------------------------------------------
  // TMS32010
  // ---------------------------------------------------------------------
  logic dsp_rs_n;
  logic dsp_cen;
  logic dsp_stall;
  logic dsp_men_q, dsp_den_q, dsp_we_q;
  logic dsp_fetch_seen_q;
  logic [2:0] dsp_read_port_q;
  logic dsp_port_read_end;     // IN cycle completed (DEN rising)
  logic dsp_port_write;        // OUT cycle (WE falling)
  logic dsp_bio_poll;
  logic [2:0] dsp_port;
  logic [15:0] dsp_port_rdata;

  assign dsp_rs_n = !(cpu_reset || !latch_q[4]);
  assign dsp_running_o = dsp_rs_n;
  assign dsp_addr_o = dsp_addr;
  assign dsp_port = dsp_addr[2:0];
  // IKA32010 returns AOUT to the next program address before DEN rises.
  // Retain the peripheral address from the start of the IN transaction so
  // its data and side effects cannot be decoded as PC[2:0] on the final
  // sampling cycle.
  wire [2:0] dsp_read_port = dsp_den_q ? dsp_addr[2:0] : dsp_read_port_q;
  // Hold the DSP while a serial EPROM byte is still in flight.
  assign dsp_stall = !dsp_denn && (dsp_read_port == 3'd0) && !ser_ready;
  assign dsp_cen = dsp_tick && !dsp_stall;

  IKA32010 u_dsp (
    .i_EMUCLK(clk_i), .i_CLKIN_PCEN(dsp_cen), .o_CLKOUT(), .o_CLKOUT_PCEN(),
    .o_CLKOUT_NCEN(), .i_RS_n(dsp_rs_n), .o_MEN_n(dsp_menn),
    .o_DEN_n(dsp_denn), .o_WE_n(dsp_wen), .o_AOUT(dsp_addr), .i_DIN(dsp_din),
    .o_DOUT(dsp_dout), .o_DOUT_OE(), .i_BIO_n(!bio_flag_q),
    .o_BIO_POLL(dsp_bio_poll), .i_INT_n(1'b1)
  );

  always_ff @(posedge clk_i) begin
    if (rst_i || !dsp_rs_n) begin
      dsp_men_q <= 1'b1;
      dsp_den_q <= 1'b1;
      dsp_we_q  <= 1'b1;
      dsp_fetch_seen_q <= 1'b0;
      dsp_read_port_q <= 3'd0;
    end else begin
      dsp_men_q <= dsp_menn;
      dsp_den_q <= dsp_denn;
      dsp_we_q  <= dsp_wen;
      if (!dsp_denn && dsp_den_q) dsp_read_port_q <= dsp_addr[2:0];
      if (dsp_menn && !dsp_men_q) dsp_fetch_seen_q <= 1'b1;
    end
  end
  // A DSP held in reset must not drive the DAC, the stream address or the
  // 68000 interrupt: its WE_n line is not driven while RS_n is low.
  assign dsp_port_read_end = dsp_rs_n && dsp_fetch_seen_q &&
                             dsp_denn && !dsp_den_q;
  assign dsp_port_write    = dsp_rs_n && dsp_fetch_seen_q &&
                             !dsp_wen && dsp_we_q && (dsp_addr[11:3] == 9'd0);
  assign dsp_prog_we       = dsp_rs_n && dsp_fetch_seen_q &&
                             !dsp_wen && dsp_we_q && (dsp_addr[11:3] != 9'd0);

  // Port reads (IN): 0 = serial EPROM byte << 7, 1 = COM RAM word, 2 = 0.
  always_comb begin
    unique case (dsp_read_port)
      3'd0:    dsp_port_rdata = {1'b0, ser_byte, 7'b0000000};
      3'd1:    dsp_port_rdata = comram_b_q;
      default: dsp_port_rdata = 16'h0000;
    endcase
    dsp_din = dsp_denn ? {dsp_ram_hi_b_q, dsp_ram_lo_b_q} : dsp_port_rdata;
  end

  // 68000 access to the DSP ports (ff6000-ff7fff) shares the port logic.
  logic        cpu_port_read;
  logic        cpu_port_write;
  logic [2:0]  cpu_port;
  logic [15:0] cpu_port_rdata;
  assign cpu_port       = cpu_addr[3:1];
  // Port read side effects (stream advance) apply when the cycle ends, after
  // the CPU has latched the data; the address is still valid then.
  assign cpu_port_read  = cpu_as_q && !cpu_as && cpu_rwn && sel_dsp_ports;
  assign cpu_port_write = cpu_write_strobe && sel_dsp_ports;
  always_comb begin
    unique case (cpu_port)
      3'd0:    cpu_port_rdata = {1'b0, ser_byte, 7'b0000000};
      3'd1:    cpu_port_rdata = comram_b_q;
      default: cpu_port_rdata = 16'h0000;
    endcase
  end

  // Port writes from either master.
  logic        port_write;
  logic [2:0]  port_write_idx;
  logic [15:0] port_write_data;
  assign port_write      = dsp_port_write || cpu_port_write;
  assign port_write_idx  = dsp_port_write ? dsp_port : cpu_port;
  assign port_write_data = dsp_port_write ? dsp_dout : cpu_dout;

  // 20 kHz BIO pacing.  A ready event remains asserted until the DSP
  // actually executes BIOZ.  It must not be cleared by a DAC write: the
  // firmware contains other OUT 0 instructions between BIOZ polls, and
  // clearing readiness there makes the next poll miss its slot and sends
  // the mixer down the wrong control path.  This is the handshake formed by
  // the board's CLKOUT-clocked LS74 on /320BIO -> /BIOS.
  always_ff @(posedge clk_i) begin
    if (rst_i || !dsp_rs_n) begin
      bio_div_q <= 12'd0;
    end else if (dsp_bio_poll && bio_flag_q) begin
      bio_div_q <= 12'd0;
    end else if (!bio_flag_q) begin
      if (bio_div_q != 12'd2399) bio_div_q <= bio_div_q + 12'd1;
    end
  end

  // ---------------------------------------------------------------------
  // Control registers
  // ---------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      maindata_q  <= 16'h0000;
      sounddata_q <= 16'h0000;
      mainflag_q  <= 1'b0;
      soundflag_q <= 1'b0;
      irq68k_q    <= 1'b0;
      latch_q     <= 8'h00;
      mute_q      <= 1'b0;
      dac_q       <= 12'd0;
      ser_offs_q  <= 20'd0;
      bio_flag_q  <= 1'b0;
    end else begin
      // Main CPU side.
      if (main_wr_i) begin
        maindata_q <= main_wdata_i;
        mainflag_q <= 1'b1;
      end
      if (main_rd_i) soundflag_q <= 1'b0;

      // Sound CPU side.
      if (cpu_read_strobe && sel_data) mainflag_q <= 1'b0;
      if (cpu_write_strobe && sel_data) begin
        if (!cpu_udsn) sounddata_q[15:8] <= cpu_dout[15:8];
        if (!cpu_ldsn) sounddata_q[7:0]  <= cpu_dout[7:0];
        soundflag_q <= 1'b1;
      end
      if (cpu_write_strobe && sel_latch) latch_q[cpu_addr[3:1]] <= cpu_addr[4];
      if (cpu_write_strobe && sel_status) irq68k_q <= 1'b0;

      // DSP stream address advances after every port 0/1 read.
      if ((dsp_port_read_end && (dsp_read_port_q[2:1] == 2'b00)) ||
          (cpu_port_read && (cpu_port[2:1] == 2'b00)))
        ser_offs_q <= ser_offs_q + 20'd1;

      if (bio_div_q == 12'd2399) bio_flag_q <= 1'b1;
      if (dsp_bio_poll && bio_flag_q) bio_flag_q <= 1'b0;

      if (port_write) begin
        unique case (port_write_idx)
          3'd0: begin
            dac_q <= port_write_data[15:4];
          end
          3'd4: mute_q   <= port_write_data[0];
          3'd5: irq68k_q <= 1'b1;
          3'd6: ser_offs_q[19:16] <= port_write_data[3:0];
          3'd7: ser_offs_q[15:0]  <= port_write_data[15:0];
          default: ;
        endcase
      end

      if (board_reset_i) begin
        mainflag_q  <= 1'b0;
        soundflag_q <= 1'b0;
        dac_q       <= 12'd0;
      end
      if (!dsp_rs_n) begin
        irq68k_q   <= 1'b0;
        bio_flag_q <= 1'b0;
        dac_q      <= 12'd0;
      end
      if (cpu_reset) latch_q <= 8'h00;
    end
  end

  // A reset DSP must not leave the external DAC holding or emitting its
  // previous sample. This also prevents a reset/error path from turning a
  // stale non-zero sample into the loud constant tone heard on hardware.
  assign audio_o = (mute_q || cpu_reset || !dsp_rs_n)
                 ? 16'sd0 : $signed({dac_q, 4'b0000});
  assign dac_o = dac_q;
  assign last_cmd_o     = maindata_q;
  assign cramen_o       = latch_q[3];
  assign comram_write_o = comram_we;
  assign ser_byte_o     = ser_byte;

  logic [7:0]  dac_peak_q;
  logic [7:0]  comram_count_q;
  logic [7:0]  comram_frac_q;
  logic        dsp_irq_seen_q;
  logic [11:0] dac_abs;
  // The DAC word is signed 12-bit two's complement. Keep all twelve bits
  // while taking its magnitude so the -2048 endpoint becomes a saturated
  // peak instead of wrapping to zero in this diagnostic counter.
  assign dac_abs = dac_q[11] ? (12'd0 - dac_q) : dac_q;
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      dac_peak_q     <= 8'd0;
      comram_count_q <= 8'd0;
      comram_frac_q  <= 8'd0;
      dsp_irq_seen_q <= 1'b0;
    end else begin
      if (dac_abs[11]) dac_peak_q <= 8'hff;
      else if (dac_abs[10:3] > dac_peak_q) dac_peak_q <= dac_abs[10:3];
      if (comram_we) begin
        comram_frac_q <= comram_frac_q + 8'd1;
        if ((comram_frac_q == 8'd255) && !(&comram_count_q))
          comram_count_q <= comram_count_q + 8'd1;
      end
      if (port_write && (port_write_idx == 3'd5)) dsp_irq_seen_q <= 1'b1;
    end
  end
  assign dac_peak_o     = dac_peak_q;
  assign comram_count_o = comram_count_q;
  assign dsp_irq_seen_o = dsp_irq_seen_q;

  logic [7:0] cpu_pc_page_q;
  always_ff @(posedge clk_i) begin
    if (rst_i) cpu_pc_page_q <= 8'h00;
    else if (cpu_as && cpu_rwn && (cpu_fc[1:0] == 2'b10)) cpu_pc_page_q <= cpu_addr[16:9];
  end
  assign cpu_pc_page_o = cpu_pc_page_q;
  assign dac_write_o = port_write && (port_write_idx == 3'd0);

  // ---------------------------------------------------------------------
  // 68000 read data
  // ---------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    unique case (1'b1)
      sel_rom:       cpu_din <= cpu_addr[16] ? 16'h0000 : rom_word;
      sel_data:      cpu_din <= maindata_q;
      sel_latch:     cpu_din <= 16'h0000;
      sel_port320:   cpu_din <= 16'h0000;
      sel_status:    cpu_din <= {mainflag_q, soundflag_q, 1'b1, 1'b0, 12'h000};
      sel_dsp_ram:   cpu_din <= {dsp_ram_hi_a_q, dsp_ram_lo_a_q};
      sel_dsp_ports: cpu_din <= cpu_port_rdata;
      sel_comram:    cpu_din <= latch_q[3] ? comram_a_q : 16'hffff;
      sel_ram:       cpu_din <= {ram_hi_q, ram_lo_q};
      default:       cpu_din <= 16'hffff;
    endcase
  end

  logic unused_signals;
  assign unused_signals = dsp_menn;
endmodule
`default_nettype wire

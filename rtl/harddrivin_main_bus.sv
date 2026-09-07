`default_nettype none

// Board-accurate 68010 startup bus slice.  In addition to ROM and RAM, this
// implements the SP-327 NBUS control decoder and the direct GSP/MSP host
// windows.  The 74LS259 is unusual: A3:A1 select an output and A4 is the data
// bit, so its state is controlled entirely by the write address.
module harddrivin_main_bus #(
  // 1: model the driver-sound board's latch handshake locally (command echo
  //    after ~100 us).  0: the real board owns the flags and data latches.
  parameter bit SOUND_STUB = 1,
  parameter bit COCKPIT = 0,
  // Development only: preload the battery-backed settings RAM (zero-power
  // RAM, 0xff4000-0xff4fff) so the ROM boots as a calibrated cabinet.  A
  // release must leave these empty and require the player's calibration.
  parameter        ZERO_RAM_INIT_HI_FILE = "",
  parameter        ZERO_RAM_INIT_LO_FILE = ""
) (
  input  logic        clk_i,
  input  logic        rst_i,
  input  logic        cycle_i,
  input  logic [31:0] addr_i,
  input  logic [15:0] wdata_i,
  input  logic [1:0]  busstate_i,
  input  logic        nuds_i,
  input  logic        nlds_i,
  output logic [15:0] rdata_o,
  output logic        ready_o,

  output logic [19:1] rom_addr_o,
  output logic        rom_req_o,
  input  logic        rom_ack_i,
  input  logic [15:0] rom_data_i,

  input  logic [7:0]  switch1_i,      // 0x60c000 low byte: DIAGN, /HSYNC, /VSYNC, EOC12, EOC8, self-test, coin2, coin1
  input  logic [7:0]  sw1_i,
  // NVRAM side port (MiSTer .nvm download/upload through ioctl), clk_i
  // domain: one zero-RAM byte access per request level, taken on a cycle
  // the 68010 does not use the RAM; even byte addresses are the high lane.
  input  logic        nvram_req_i,
  input  logic        nvram_we_i,
  input  logic [11:0] nvram_addr_i,
  input  logic [7:0]  nvram_wdata_i,
  output logic        nvram_ack_o,     // pulse: done, nvram_rdata_o valid
  output logic [7:0]  nvram_rdata_o,
  output logic        nvram_dirty_o,   // pulse: the 68010 wrote the zero RAM
  // Driver sound board latches (used when SOUND_STUB == 0).
  output logic        snd_wr_o,        // pulse: 0x840000 written (wdata_i)
  output logic        snd_rd_o,        // pulse: 0x840000 read
  output logic        snd_reset_o,     // pulse: 0x84c000 write / 0x604000 read
  input  logic [15:0] snd_rdata_i,     // sound -> main data latch
  input  logic        snd_mainflag_i,
  input  logic        snd_soundflag_i,          // 0x60c000 high byte: SW1 #8 (bit 0) .. SW1 #1 (bit 7), 1 = off
  input  logic [15:0] compact_port1_i,
  input  logic [7:0]  accelerator_i,
  input  logic [7:0]  clutch_i,
  input  logic [7:0]  seat_i,
  input  logic [7:0]  shifter_x_i,
  input  logic [7:0]  shifter_y_i,
  input  logic [7:0]  brake_i,
  input  logic [11:0] brake12_i,
  input  logic [11:0] steering_i,       // requested absolute controller position
  output logic [11:0] steering_counter_o, // Compact byte-counter position seen by ROM
  output logic        watchdog_clear_o,
  output logic        irq_clear_o,
  output logic [7:0]  control_latch_o,
  output logic        gsp_reset_release_o,
  output logic        msp_reset_release_o,

  output logic        gsp_host_req_o,
  output logic        gsp_host_we_o,
  output logic [1:0]  gsp_host_reg_o,
  output logic [1:0]  gsp_host_be_o,
  output logic [15:0] gsp_host_wdata_o,
  input  logic [15:0] gsp_host_rdata_i,
  input  logic        gsp_host_ack_i,

  // ADSP board window 0x800000-0x83ffff (sheet 1 decoder: A23:A18 = 100000).
  output logic        adsp_req_o,
  output logic        adsp_we_o,
  output logic [17:1] adsp_addr_o,
  output logic [15:0] adsp_wdata_o,
  input  logic [15:0] adsp_rdata_i,
  input  logic        adsp_ack_i,

  output logic        msp_host_req_o,
  output logic        msp_host_we_o,
  output logic [1:0]  msp_host_reg_o,
  output logic [1:0]  msp_host_be_o,
  output logic [15:0] msp_host_wdata_o,
  input  logic [15:0] msp_host_rdata_i,
  input  logic        msp_host_ack_i,

  output logic [23:0] last_fetch_addr_o,
  output logic [23:0] last_read_addr_o,
  output logic [23:0] last_write_addr_o,
  output logic [31:0] fetch_count_o,
  output logic [31:0] read_count_o,
  output logic [31:0] write_count_o,
  output logic [7:0]  diagnostic_o
);
  localparam logic [1:0] BUS_FETCH = 2'b00;
  localparam logic [1:0] BUS_IDLE  = 2'b01;
  localparam logic [1:0] BUS_READ  = 2'b10;
  localparam logic [1:0] BUS_WRITE = 2'b11;

  logic done_cycle_q;
  logic rom_pending_q;
  logic local_pending_q;
  logic local_zero_q;
  logic host_pending_q;
  logic adsp_pending_q;
  logic adsp_write_q;
  logic host_msp_q;
  logic host_write_q;
  logic pending_cycle_q;
  logic [19:1] pending_rom_addr_q;
  logic [3:0] vector_match_q;
  logic reset_pc_fetch_q;
  logic execution_fetch_q;
  logic gsp_host_seen_q;
  logic msp_host_seen_q;
  logic [7:0] control_latch_q;

  logic active_cycle;
  logic rom_select;
  logic zero_ram_select;
  logic main_ram_select;
  logic nbus_select;
  logic nbus_probe_select;
  logic nbus_latch_select;
  logic nbus_watchdog_select;
  logic nbus_switch_irq_select;
  logic gsp_host_select;
  logic msp_host_select;
  logic host_select;
  logic compact_wheel_select;
  logic compact_wheel_reset_select;
  logic compact_port1_select;
  logic compact_adc8_select;
  logic compact_adc12_select;
  logic adsp_select;
  logic duart_select;
  logic snd_select;
  logic snd_data_select;
  logic snd_status_select;
  logic snd_reset_select;
  logic new_cycle;
  logic local_read_start;
  logic local_write_start;
  logic zero_ram_cpu_write_allowed;
  logic main_ram_en;
  logic zero_ram_en;
  logic [15:0] main_ram_rdata;
  logic [15:0] zero_ram_rdata;
  logic [7:0] adc_control_q;
  logic [11:0] adc12_data_q;
  logic [11:0] wheel_position_q;
  logic [11:0] wheel_sample_next;
  logic [11:0] wheel_distance;
  localparam logic [11:0] WHEEL_MAX_SAMPLE_DELTA = 12'h07f;
  // The cabinet wheel is physically absolute, but the Compact CPU sees only
  // its low byte at 0x400000 plus an edge latch at the 0xf00 wrap.  The ROM
  // subtracts consecutive low-byte samples as a signed value (harddrivc
  // 0x01cf3e).  A gamepad can jump farther between reads than a real wheel;
  // exposing that jump directly aliases to the opposite direction.  Advance
  // toward the absolute controller target by the largest unambiguous signed
  // delta on each real read.  At the 244 Hz cabinet rate, centre-to-lock
  // completes in at most 16 reads (about 66 ms).
  always_comb begin
    wheel_sample_next = wheel_position_q;
    wheel_distance = 12'd0;
    if (steering_i > wheel_position_q) begin
      wheel_distance = steering_i - wheel_position_q;
      wheel_sample_next = (wheel_distance > WHEEL_MAX_SAMPLE_DELTA)
                        ? wheel_position_q + WHEEL_MAX_SAMPLE_DELTA
                        : steering_i;
    end else if (steering_i < wheel_position_q) begin
      wheel_distance = wheel_position_q - steering_i;
      wheel_sample_next = (wheel_distance > WHEEL_MAX_SAMPLE_DELTA)
                        ? wheel_position_q - WHEEL_MAX_SAMPLE_DELTA
                        : steering_i;
    end
  end
  assign steering_counter_o = COCKPIT ? steering_i : wheel_position_q;
  logic wheel_edge_q;
  logic wheel_calibration_mode_q;
  logic [7:0] adc8_data;
  logic [15:0] compact_port1_data;

  // Sound board handshake model.  Until the driver sound board is emulated
  // the main CPU still needs the latch protocol it uses at every game start:
  // it resets the board, writes a command and waits for the sound CPU to
  // echo it back (the firmware's reset code and its main-data interrupt
  // handler both read the command latch and write the value straight
  // back). The status word mirrors the board/MAME hd68k_snd_status_r:
  // {mainflag, soundflag, 14'h1fff}. Bit 13 is low on this interface.
  localparam int unsigned SND_ECHO_CLOCKS = 4800;  // ~100 us at 48 MHz
  logic        snd_mainflag_q;
  logic        snd_soundflag_q;
  logic [15:0] snd_cmd_q;
  logic [15:0] snd_data_q;
  logic        snd_echo_pending_q;
  logic [12:0] snd_echo_timer_q;
  logic        snd_reset_pulse;
  logic        snd_cmd_write;
  logic        snd_data_read;

  assign active_cycle = busstate_i != BUS_IDLE;
  assign rom_select = (addr_i[23:20] == 4'h0);
  assign zero_ram_select = (addr_i[23:12] == 12'hff4);
  assign main_ram_select = (addr_i[23:15] == 9'h1ff);
  // SP-327 sheet 4: A23:A21=011 selects NBUS. Compact assigns the four
  // A15:A14 regions to an unused/open aperture, the addressable latch,
  // watchdog clear, and the shared switch-read/IRQ-clear aperture.
  assign nbus_select = (addr_i[23:21] == 3'b011);
  assign nbus_probe_select      = nbus_select && (addr_i[15:14] == 2'b00);
  assign nbus_latch_select      = nbus_select && (addr_i[15:14] == 2'b01);
  assign nbus_watchdog_select   = nbus_select && (addr_i[15:14] == 2'b10);
  assign nbus_switch_irq_select = nbus_select && (addr_i[15:14] == 2'b11);

  // HSBUS is A23:A21=110.  The lower decoder selects GSP at A15:A14=00
  // and MSP at 01; A20:A16 are aliases on the original board.
  assign gsp_host_select = (addr_i[23:21] == 3'b110) &&
                           (addr_i[15:14] == 2'b00);
  assign msp_host_select = (addr_i[23:21] == 3'b110) &&
                           (addr_i[15:14] == 2'b01);
  assign host_select = gsp_host_select || msp_host_select;
  assign compact_wheel_select = !COCKPIT && addr_i[23:1] == 23'h200000;
  assign compact_wheel_reset_select = !COCKPIT && addr_i[23:1] == 23'h204000;
  assign compact_port1_select = addr_i[23:19] == 5'b10101;
  assign compact_adc8_select = addr_i[23:19] == 5'b10110;
  assign compact_adc12_select = addr_i[23:19] == 5'b10111;
  assign adsp_select = addr_i[23:18] == 6'b100000;
  // MC68681 DUART at 0xff0000-0xff001f on the upper byte lane (MAME umask
  // 0xff00).  Only the counter/timer is modelled: the game programs ACR =
  // 0x30 (counter mode, X1/16 = 230.4 kHz), preloads 0xffff, starts it and
  // reads CUR/CLR later to measure elapsed time.  Status reads report an
  // idle serial link; no interrupt is generated.
  assign duart_select = (addr_i[23:5] == 19'h7f800);
  // Driver sound board latches (MAME harddriv_sound_board_device):
  // 0x840000 data (main<->sound), 0x844000 status, 0x84c000 sound reset.
  assign snd_select        = addr_i[23:16] == 8'h84;
  assign snd_data_select   = snd_select && (addr_i[15:14] == 2'b00);
  assign snd_status_select = snd_select && (addr_i[15:14] == 2'b01);
  assign snd_reset_select  = snd_select && (addr_i[15:14] == 2'b11);
  assign new_cycle = (cycle_i != done_cycle_q) && !rom_pending_q &&
                     !local_pending_q && !host_pending_q && !adsp_pending_q;
  assign local_read_start = new_cycle && (busstate_i == BUS_READ) &&
                            (main_ram_select || zero_ram_select);
  assign local_write_start = new_cycle && (busstate_i == BUS_WRITE) &&
                             (main_ram_select || zero_ram_select);
  // MAME hd68k_zram_w and SP-327 sheet 4 agree: the battery-backed zero
  // RAM is writable only with ZP1 low and ZP2 high.  Reads are never gated.
  // Ignoring these latch outputs lets ordinary protected writes corrupt the
  // cabinet calibration and bookkeeping values.
  assign zero_ram_cpu_write_allowed = !control_latch_q[4] &&
                                      control_latch_q[5];
  assign main_ram_en = (local_read_start || local_write_start) &&
                       main_ram_select;
  logic cpu_zero_access;
  logic nvram_go;
  logic nvram_pend_q;
  logic nvram_done_q;
  assign cpu_zero_access = !rst_i && (local_read_start || local_write_start) &&
                           zero_ram_select;
  // The side port keeps working while the core is held in reset: MiSTer
  // downloads the .nvm image with the core reset asserted, and the 68010's
  // own RAM accesses are quiet then (their state is reset).
  assign nvram_go = nvram_req_i && !nvram_pend_q && !nvram_done_q &&
                    (rst_i || (!local_pending_q && !local_read_start && !local_write_start));
  assign zero_ram_en = cpu_zero_access || nvram_go;
  initial begin
    nvram_pend_q  = 1'b0;
    nvram_done_q  = 1'b0;
    nvram_ack_o   = 1'b0;
    nvram_rdata_o = 8'h00;
    nvram_dirty_o = 1'b0;
  end
  wire [7:0] adc_control_next = !nlds_i ? wdata_i[7:0]
      : (COCKPIT ? adc_control_q : wdata_i[15:8]);
  always_ff @(posedge clk_i) begin
    nvram_pend_q  <= nvram_go;
    nvram_ack_o   <= nvram_pend_q;
    if (nvram_pend_q)
      nvram_rdata_o <= nvram_addr_i[0] ? zero_ram_rdata[7:0] : zero_ram_rdata[15:8];
    if (nvram_pend_q) nvram_done_q <= 1'b1;
    else if (!nvram_req_i) nvram_done_q <= 1'b0;
    nvram_dirty_o <= !rst_i && local_write_start && zero_ram_select &&
                     zero_ram_cpu_write_allowed;
  end
  assign ready_o = cycle_i == done_cycle_q;
  assign control_latch_o = control_latch_q;
  assign gsp_reset_release_o = control_latch_q[6];
  assign msp_reset_release_o = control_latch_q[7];
  assign diagnostic_o = {
    msp_host_seen_q,
    gsp_host_seen_q,
    control_latch_q[7] && control_latch_q[6],
    reset_pc_fetch_q && execution_fetch_q,
    vector_match_q
  };

  wire [7:0] cockpit_wheel8 = steering_i <= 12'h010 ? 8'h10 :
    steering_i >= 12'hff0 ? 8'hf0 :
    8'h10 + 8'((32'(steering_i - 12'h010) * 32'd224) / 32'd4064);
  always_comb begin
    unique case (adc_control_q[2:0])
      3'd0: adc8_data = accelerator_i;
      3'd1: adc8_data = clutch_i;
      // racedrivc is the Compact cabinet input map used by harddrivc.
      // Channels 2..5 and 7 are physically unused and read high.  The
      // upright Hard Drivin' seat/H-pattern potentiometers are not fitted.
      3'd2: adc8_data = COCKPIT ? seat_i : 8'hff;
      3'd3: adc8_data = COCKPIT ? shifter_y_i : 8'hff;
      3'd4: adc8_data = COCKPIT ? shifter_x_i : 8'hff;
      3'd5: adc8_data = COCKPIT ? cockpit_wheel8 : 8'hff;
      3'd6: adc8_data = COCKPIT ? 8'h80 : brake_i;
      3'd7: adc8_data = COCKPIT ? 8'h80 : 8'hff;
      default: adc8_data = 8'hff;
    endcase

    compact_port1_data = COCKPIT ? (compact_port1_i | 16'hfff8) : compact_port1_i;
    if (!COCKPIT && wheel_edge_q) compact_port1_data[14] = 1'b0;
  end

  harddrivin_lane_ram #(.ADDR_WIDTH(14)) u_main_ram (
    .clk_i(clk_i), .en_i(main_ram_en), .addr_i(addr_i[14:1]),
    .we_hi_i(local_write_start && main_ram_select && !nuds_i),
    .we_lo_i(local_write_start && main_ram_select && !nlds_i),
    .wdata_i(wdata_i), .rdata_o(main_ram_rdata)
  );

  harddrivin_lane_ram #(
    .ADDR_WIDTH(11),
    .INIT_HI_FILE(ZERO_RAM_INIT_HI_FILE), .INIT_LO_FILE(ZERO_RAM_INIT_LO_FILE)
  ) u_zero_ram (
    .clk_i(clk_i), .en_i(zero_ram_en),
    .addr_i(cpu_zero_access ? addr_i[11:1] : nvram_addr_i[11:1]),
    .we_hi_i((cpu_zero_access && local_write_start &&
              zero_ram_cpu_write_allowed && !nuds_i) ||
             (nvram_go && nvram_we_i && !nvram_addr_i[0])),
    .we_lo_i((cpu_zero_access && local_write_start &&
              zero_ram_cpu_write_allowed && !nlds_i) ||
             (nvram_go && nvram_we_i && nvram_addr_i[0])),
    .wdata_i(cpu_zero_access ? wdata_i : {nvram_wdata_i, nvram_wdata_i}),
    .rdata_o(zero_ram_rdata)
  );

  logic        snd_mainflag_eff, snd_soundflag_eff;
  logic [15:0] snd_rdata_eff;
  assign snd_mainflag_eff  = SOUND_STUB ? snd_mainflag_q  : snd_mainflag_i;
  assign snd_soundflag_eff = SOUND_STUB ? snd_soundflag_q : snd_soundflag_i;
  assign snd_rdata_eff     = SOUND_STUB ? snd_data_q      : snd_rdata_i;
  assign snd_wr_o    = snd_cmd_write;
  assign snd_rd_o    = snd_data_read;
  assign snd_reset_o = snd_reset_pulse;
  logic [7:0]  duart_acr_q;
  logic [15:0] duart_preload_q;
  logic [15:0] duart_count_q;
  logic        duart_running_q;
  logic        duart_ready_q;
  logic [25:0] duart_phase_q;
  logic        duart_tick;
  logic [7:0]  duart_rdata;
  logic        duart_read;
  logic        duart_write;
  logic [3:0]  duart_reg;
  assign duart_reg   = addr_i[4:1];
  assign duart_read  = new_cycle && (busstate_i == BUS_READ) && duart_select;
  assign duart_write = new_cycle && (busstate_i == BUS_WRITE) && duart_select && !nuds_i;
  // 230.4 kHz from 48 MHz: one tick every 208.33 clocks on average.
  assign duart_tick = (duart_phase_q >= 26'd47769600);
  always_comb begin
    unique case (duart_reg)
      4'd1, 4'd9: duart_rdata = 8'h0c;                 // SRA/SRB: TxRDY, TxEMT
      4'd5:       duart_rdata = {4'b0000, duart_ready_q, 3'b000};   // ISR
      4'd6:       duart_rdata = duart_count_q[15:8];   // CUR
      4'd7:       duart_rdata = duart_count_q[7:0];    // CLR
      default:    duart_rdata = 8'h00;
    endcase
  end
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      duart_acr_q     <= 8'h00;
      duart_preload_q <= 16'hffff;
      duart_count_q   <= 16'hffff;
      duart_running_q <= 1'b0;
      duart_ready_q   <= 1'b0;
      duart_phase_q   <= 26'd0;
    end else begin
      duart_phase_q <= duart_tick ? (duart_phase_q + 26'd230400 - 26'd48000000)
                                  : (duart_phase_q + 26'd230400);
      if (duart_running_q && duart_tick) begin
        duart_count_q <= duart_count_q - 16'd1;
        if (duart_count_q == 16'd0) duart_ready_q <= 1'b1;
      end
      if (duart_write) begin
        unique case (duart_reg)
          4'd4: duart_acr_q           <= wdata_i[15:8];
          4'd6: duart_preload_q[15:8] <= wdata_i[15:8];
          4'd7: duart_preload_q[7:0]  <= wdata_i[15:8];
          default: ;
        endcase
      end
      if (duart_read) begin
        unique case (duart_reg)
          4'd14: begin                     // start counter command
            duart_count_q   <= duart_preload_q;
            duart_running_q <= 1'b1;
            duart_ready_q   <= 1'b0;
          end
          4'd15: begin                     // stop counter command
            duart_running_q <= 1'b0;
            duart_ready_q   <= 1'b0;
          end
          default: ;
        endcase
      end
    end
  end
  logic unused_duart_acr;
  assign unused_duart_acr = ^duart_acr_q;

  assign snd_reset_pulse = new_cycle &&
                           ((busstate_i == BUS_WRITE && snd_reset_select) ||
                            (!COCKPIT && busstate_i == BUS_READ && nbus_latch_select));
  assign snd_cmd_write = new_cycle && (busstate_i == BUS_WRITE) &&
                         snd_data_select;
  assign snd_data_read = new_cycle && (busstate_i == BUS_READ) &&
                         snd_data_select;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      snd_mainflag_q     <= 1'b0;
      snd_soundflag_q    <= 1'b0;
      snd_cmd_q          <= 16'h0000;
      snd_data_q         <= 16'h0000;
      snd_echo_pending_q <= 1'b0;
      snd_echo_timer_q   <= 13'd0;
    end else begin
      if (snd_echo_pending_q) begin
        if (snd_echo_timer_q == 13'd0) begin
          // The sound CPU reads the command (clearing mainflag) and echoes it.
          snd_echo_pending_q <= 1'b0;
          snd_mainflag_q     <= 1'b0;
          snd_soundflag_q    <= 1'b1;
          snd_data_q         <= snd_cmd_q;
        end else begin
          snd_echo_timer_q <= snd_echo_timer_q - 13'd1;
        end
      end
      if (snd_data_read) snd_soundflag_q <= 1'b0;
      if (snd_cmd_write) begin
        snd_cmd_q          <= wdata_i;
        snd_mainflag_q     <= 1'b1;
        snd_echo_pending_q <= 1'b1;
        snd_echo_timer_q   <= 13'(SND_ECHO_CLOCKS);
      end
      if (snd_reset_pulse) begin
        // hd68k_snd_reset_w / hd68k_sound_reset_r: both flags drop and the
        // firmware's reset code echoes whatever the command latch holds.
        snd_mainflag_q     <= 1'b0;
        snd_soundflag_q    <= 1'b0;
        snd_echo_pending_q <= 1'b1;
        snd_echo_timer_q   <= 13'(SND_ECHO_CLOCKS);
      end
    end
  end

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      done_cycle_q      <= 1'b0;
      rom_pending_q     <= 1'b0;
      local_pending_q   <= 1'b0;
      local_zero_q      <= 1'b0;
      host_pending_q    <= 1'b0;
      adsp_pending_q    <= 1'b0;
      adsp_write_q      <= 1'b0;
      adsp_req_o        <= 1'b0;
      adsp_we_o         <= 1'b0;
      adsp_addr_o       <= 17'd0;
      adsp_wdata_o      <= 16'h0000;
      host_msp_q        <= 1'b0;
      host_write_q      <= 1'b0;
      pending_cycle_q   <= 1'b0;
      pending_rom_addr_q <= 19'd0;
      rom_addr_o        <= 19'd0;
      rom_req_o         <= 1'b0;
      watchdog_clear_o  <= 1'b0;
      irq_clear_o       <= 1'b0;
      control_latch_q   <= 8'h00;
      gsp_host_req_o    <= 1'b0;
      gsp_host_we_o     <= 1'b0;
      gsp_host_reg_o    <= 2'b00;
      gsp_host_be_o     <= 2'b00;
      gsp_host_wdata_o  <= 16'h0000;
      msp_host_req_o    <= 1'b0;
      msp_host_we_o     <= 1'b0;
      msp_host_reg_o    <= 2'b00;
      msp_host_be_o     <= 2'b00;
      msp_host_wdata_o  <= 16'h0000;
      adc_control_q     <= 8'h00;
      adc12_data_q      <= 12'h800;
      wheel_position_q  <= 12'h800;
      wheel_edge_q      <= 1'b0;
      wheel_calibration_mode_q <= 1'b0;
      rdata_o           <= 16'hffff;
      last_fetch_addr_o <= 24'd0;
      last_read_addr_o  <= 24'd0;
      last_write_addr_o <= 24'd0;
      fetch_count_o     <= 32'd0;
      read_count_o      <= 32'd0;
      write_count_o     <= 32'd0;
      vector_match_q    <= 4'd0;
      reset_pc_fetch_q  <= 1'b0;
      execution_fetch_q <= 1'b0;
      gsp_host_seen_q   <= 1'b0;
      msp_host_seen_q   <= 1'b0;
    end else begin
      watchdog_clear_o <= 1'b0;
      irq_clear_o      <= 1'b0;

      // The Compact wheel edge latch (port 1 bit 14, cleared by a write to
      // 0x408000) is needed while the cabinet records its four steering-limit
      // samples.  Our gamepad adapter already transports every low-byte delta
      // losslessly (each step is <= 127), so exposing the physical wheel's
      // index edge afterward makes the ROM apply a second correction.  That
      // correction is cumulative: a right-lock traversal leaves a centered
      // controller steering farther left on every periodic update.  Follow
      // the target ROM's calibration call precisely: 0x019c64 is its entry;
      // 0x01921e is the instruction after it returns to the caller.
      if ((cycle_i != done_cycle_q) && !rom_pending_q && !local_pending_q &&
          !host_pending_q && !adsp_pending_q &&
          (busstate_i == BUS_FETCH)) begin
        if (addr_i[23:0] == 24'h019c64)
          wheel_calibration_mode_q <= 1'b1;
        else if (addr_i[23:0] == 24'h01921e) begin
          wheel_calibration_mode_q <= 1'b0;
          wheel_edge_q             <= 1'b0;
        end
      end

      // Sample the edge only inside that calibration call. Movement that the
      // game has not read remains invisible, exactly as on the board/MAME.
      if (new_cycle && (busstate_i == BUS_READ) && compact_wheel_select) begin
        if (wheel_calibration_mode_q &&
            ((wheel_position_q >= 12'hf00) !=
             (wheel_sample_next >= 12'hf00)))
          wheel_edge_q <= 1'b1;
        wheel_position_q <= wheel_sample_next;
      end

      if (local_pending_q) begin
        rdata_o         <= local_zero_q ? zero_ram_rdata : main_ram_rdata;
        done_cycle_q    <= pending_cycle_q;
        local_pending_q <= 1'b0;
        read_count_o    <= read_count_o + 32'd1;
      end

      if (rom_pending_q && (rom_ack_i == rom_req_o)) begin
        rdata_o       <= rom_data_i;
        done_cycle_q  <= pending_cycle_q;
        rom_pending_q <= 1'b0;
        read_count_o  <= read_count_o + 32'd1;

        case (pending_rom_addr_q)
          19'h00000: vector_match_q[0] <= rom_data_i == 16'hffff;
          19'h00001: vector_match_q[1] <= rom_data_i == 16'hc000;
          19'h00002: vector_match_q[2] <= rom_data_i == 16'h0000;
          19'h00003: vector_match_q[3] <= rom_data_i == 16'h021c;
          default: ;
        endcase
        if (pending_rom_addr_q == 19'h0010e)
          reset_pc_fetch_q <= 1'b1;
        if (pending_rom_addr_q > 19'h00110)
          execution_fetch_q <= 1'b1;
      end

      if (adsp_pending_q && adsp_ack_i) begin
        if (!adsp_write_q) begin
          rdata_o      <= adsp_rdata_i;
          read_count_o <= read_count_o + 32'd1;
        end else begin
          write_count_o <= write_count_o + 32'd1;
        end
        done_cycle_q   <= pending_cycle_q;
        adsp_pending_q <= 1'b0;
        adsp_req_o     <= 1'b0;
      end

      if (host_pending_q &&
          ((host_msp_q && msp_host_ack_i) ||
           (!host_msp_q && gsp_host_ack_i))) begin
        if (!host_write_q) begin
          rdata_o      <= host_msp_q ? msp_host_rdata_i : gsp_host_rdata_i;
          read_count_o <= read_count_o + 32'd1;
        end else begin
          write_count_o <= write_count_o + 32'd1;
        end
        done_cycle_q  <= pending_cycle_q;
        host_pending_q <= 1'b0;
        if (host_msp_q) begin
          msp_host_req_o  <= 1'b0;
          msp_host_seen_q <= 1'b1;
        end else begin
          gsp_host_req_o  <= 1'b0;
          gsp_host_seen_q <= 1'b1;
        end
      end

      if ((cycle_i != done_cycle_q) && !rom_pending_q && !local_pending_q &&
          !host_pending_q && !adsp_pending_q) begin
        // Preserve the CPU-visible address of every real external cycle.
        // These taps are also useful after an open-bus access, where the
        // normal read data alone cannot identify a missing board decode.
        unique case (busstate_i)
          BUS_FETCH: begin
            last_fetch_addr_o <= addr_i[23:0];
            fetch_count_o     <= fetch_count_o + 32'd1;
          end
          BUS_READ:  last_read_addr_o  <= addr_i[23:0];
          BUS_WRITE: last_write_addr_o <= addr_i[23:0];
          default: ;
        endcase

        if (!active_cycle) begin
          rdata_o      <= 16'hffff;
          done_cycle_q <= cycle_i;
        end else if ((busstate_i == BUS_FETCH || busstate_i == BUS_READ) &&
                     rom_select) begin
          rom_addr_o         <= addr_i[19:1];
          pending_rom_addr_q <= addr_i[19:1];
          pending_cycle_q    <= cycle_i;
          rom_req_o          <= !rom_req_o;
          rom_pending_q      <= 1'b1;
        end else if (busstate_i == BUS_READ) begin
          if (main_ram_select) begin
            local_pending_q <= 1'b1;
            local_zero_q    <= 1'b0;
            pending_cycle_q <= cycle_i;
          end else if (zero_ram_select) begin
            local_pending_q <= 1'b1;
            local_zero_q    <= 1'b1;
            pending_cycle_q <= cycle_i;
          end else if (host_select) begin
            pending_cycle_q <= cycle_i;
            host_pending_q  <= 1'b1;
            host_msp_q      <= msp_host_select;
            host_write_q    <= 1'b0;
            if (msp_host_select) begin
              msp_host_req_o   <= 1'b1;
              msp_host_we_o    <= 1'b0;
              msp_host_reg_o   <= {addr_i[3], !addr_i[2]};
              msp_host_be_o    <= {!nuds_i, !nlds_i};
              msp_host_wdata_o <= wdata_i;
            end else begin
              gsp_host_req_o   <= 1'b1;
              gsp_host_we_o    <= 1'b0;
              gsp_host_reg_o   <= {addr_i[3], !addr_i[2]};
              gsp_host_be_o    <= {!nuds_i, !nlds_i};
              gsp_host_wdata_o <= wdata_i;
            end
          end else if (adsp_select) begin
            pending_cycle_q <= cycle_i;
            adsp_pending_q  <= 1'b1;
            adsp_write_q    <= 1'b0;
            adsp_req_o      <= 1'b1;
            adsp_we_o       <= 1'b0;
            adsp_addr_o     <= addr_i[17:1];
            adsp_wdata_o    <= wdata_i;
          end else if (compact_wheel_select) begin
            rdata_o      <= {wheel_sample_next[7:0], 8'hff};
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (compact_port1_select) begin
            rdata_o      <= compact_port1_data;
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (compact_adc8_select) begin
            rdata_o      <= {8'hff, adc8_data};
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (compact_adc12_select) begin
            // MAME hd68k_adc12_r returns the selected byte with ZEROS in the
            // unused bits (the handler is mapped 16 bits wide).  Returning
            // ones there corrupts the wheel value whenever the game reads
            // this port as a word and combines the two halves.
            rdata_o <= adc_control_q[7]
                     ? {12'h000, adc12_data_q[11:8]}
                     : {8'h00, adc12_data_q[7:0]};
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (duart_select) begin
            rdata_o      <= {duart_rdata, 8'hff};
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (snd_data_select) begin
            rdata_o      <= snd_rdata_eff;
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (snd_status_select) begin
            rdata_o      <= {snd_mainflag_eff, snd_soundflag_eff, 14'h1fff};
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (COCKPIT ? nbus_probe_select : nbus_switch_irq_select) begin
            rdata_o      <= {sw1_i, switch1_i};
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else if (nbus_select) begin
            // The remaining NBUS devices are strobes or write-only.  They do
            // not drive the data bus on the PCB.
            rdata_o      <= 16'hffff;
            read_count_o <= read_count_o + 32'd1;
            done_cycle_q <= cycle_i;
          end else begin
            rdata_o         <= 16'hffff;
            read_count_o    <= read_count_o + 32'd1;
            done_cycle_q    <= cycle_i;
          end
        end else if (busstate_i == BUS_WRITE) begin
          if (adsp_select) begin
            pending_cycle_q <= cycle_i;
            adsp_pending_q  <= 1'b1;
            adsp_write_q    <= 1'b1;
            adsp_req_o      <= 1'b1;
            adsp_we_o       <= 1'b1;
            adsp_addr_o     <= addr_i[17:1];
            adsp_wdata_o    <= wdata_i;
          end else if (host_select) begin
            pending_cycle_q <= cycle_i;
            host_pending_q  <= 1'b1;
            host_msp_q      <= msp_host_select;
            host_write_q    <= 1'b1;
            if (msp_host_select) begin
              msp_host_req_o   <= 1'b1;
              msp_host_we_o    <= 1'b1;
              msp_host_reg_o   <= {addr_i[3], !addr_i[2]};
              msp_host_be_o    <= {!nuds_i, !nlds_i};
              msp_host_wdata_o <= wdata_i;
            end else begin
              gsp_host_req_o   <= 1'b1;
              gsp_host_we_o    <= 1'b1;
              gsp_host_reg_o   <= {addr_i[3], !addr_i[2]};
              gsp_host_be_o    <= {!nuds_i, !nlds_i};
              gsp_host_wdata_o <= wdata_i;
            end
          end else begin
            if (nbus_latch_select)
              control_latch_q[addr_i[3:1]] <= addr_i[4];
            if (nbus_watchdog_select) watchdog_clear_o <= 1'b1;
            if (nbus_switch_irq_select) irq_clear_o <= 1'b1;
            if (compact_wheel_reset_select) wheel_edge_q <= 1'b0;
            if (compact_adc12_select) begin
              // MAME COMBINE_DATA: an upper-lane write does not replace the
              // ADC control bits in the low byte on the cockpit board.
              if (!nlds_i) adc_control_q <= wdata_i[7:0];
              else if (!nuds_i && !COCKPIT) adc_control_q <= wdata_i[15:8];
              if (adc_control_next[6]) begin
                case (adc_control_next[5:4])
                  2'd0: adc12_data_q <= steering_i;
                  2'd1: if (COCKPIT) adc12_data_q <= brake12_i;
                  default: if (COCKPIT) adc12_data_q <= 12'hfff;
                endcase
              end
            end
            write_count_o <= write_count_o + 32'd1;
            done_cycle_q  <= cycle_i;
          end
        end else begin
          rdata_o      <= 16'hffff;
          done_cycle_q <= cycle_i;
        end
      end
    end
  end

  logic unused_nbus_probe;
  assign unused_nbus_probe = nbus_probe_select;
endmodule

`default_nettype wire

// Hard Drivin' cockpit/compact top for the MiSTer framework.
module emu (
  `include "sys/emu_ports.vh"
);
`ifdef HD_COCKPIT
  localparam bit COCKPIT = 1;
`else
  localparam bit COCKPIT = 0;
`endif
  assign ADC_BUS = 'Z;
  assign USER_OUT = '1;
  assign {UART_RTS, UART_TXD, UART_DTR} = 3'b000;
  assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

  assign VGA_F1 = 1'b0;
  assign VGA_SCALER = 1'b0;
  assign VGA_DISABLE = 1'b0;
  assign HDMI_FREEZE = 1'b0;
  assign HDMI_BLACKOUT = 1'b0;
  assign HDMI_BOB_DEINT = 1'b0;
  // Driver sound board DAC (mono); signed samples.
  logic signed [15:0] sound_audio;
  assign AUDIO_S = 1'b1;
  assign AUDIO_L = sound_audio;
  assign AUDIO_R = sound_audio;
  assign AUDIO_MIX = 2'b00;
  assign LED_DISK = 2'b00;
  assign LED_POWER = 2'b00;
  assign BUTTONS = 2'b00;
  wire [127:0] status;
  // Compact hardware uses a 512x288 raster on a 4:3 cabinet monitor. Keep
  // that physical presentation by default, while exposing the square-pixel
  // raster for capture/debugging.
  assign VIDEO_ARX = status[2] ? (COCKPIT ? 13'd127 : 13'd16) : 13'd4;
  assign VIDEO_ARY = status[2] ? (COCKPIT ? 13'd96 : 13'd9)  : 13'd3;

  localparam CONF_STR = {
`ifdef HD_COCKPIT
    "Hard Drivin' Cockpit;;",
`else
    "Hard Drivin' Compact (feasibility);;",
`endif
    "DIP;",
    "-;",
    "T[0],Reset;",
    "R[0],Reset and close OSD;",
`ifdef HD_COCKPIT
    "O[6],Pedals,Sticks,Buttons;",
`else
    "O[6],Pedals,Sticks,Triggers;",
`endif
    "O[7],Gearbox,Sequential,H-pattern;",
`ifdef HD_COCKPIT
    "O[8],Controller,Gamepad,Wheel;",
`endif
    "O[9],Shifter input,Player 1,Player 2;",
    "O[4],Live controls display,Off,On;",
    "O[2],Aspect ratio,Original 4:3,Raw pixels;",
    "-;",
    "O[3],Service mode (self-test),Off,On;",
    "-;",
    // Build stamp: bump this for every image handed to hardware so the OSD
    // says which one is loaded.
`ifdef HD_COCKPIT
    "V,v0.9-b134-frame-parity"
`else
    "V,v0.9-b97"
`endif
  };

  wire [1:0] buttons;
  wire ioctl_download;
  wire ioctl_upload, ioctl_rd;
  wire forced_scandoubler;
  wire [21:0] gamma_bus;
  wire [31:0] joystick_0;
  wire [31:0] joystick_1;
  wire [15:0] joystick_l_analog_0, joystick_l_analog_1;
  wire [15:0] joystick_r_analog_0, joystick_r_analog_1;
  wire        ioctl_wr;
  wire [24:0] ioctl_addr;
  wire [7:0]  ioctl_dout;
  wire [15:0] ioctl_index;

  hps_io #(.CONF_STR(CONF_STR)) u_hps_io (
    .clk_sys(CLK_50M), .HPS_BUS(HPS_BUS), .EXT_BUS(),
    .gamma_bus(gamma_bus), .forced_scandoubler(forced_scandoubler),
    .buttons(buttons), .status(status), .status_menumask(16'd0),
    .joystick_0(joystick_0), .joystick_1(joystick_1),
    .joystick_l_analog_0(joystick_l_analog_0), .joystick_l_analog_1(joystick_l_analog_1),
    .joystick_r_analog_0(joystick_r_analog_0), .joystick_r_analog_1(joystick_r_analog_1),
    .ioctl_download(ioctl_download), .ioctl_wait(nvram_ioctl_wait),
    .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr), .ioctl_dout(ioctl_dout),
    .ioctl_index(ioctl_index),
    .ioctl_upload(ioctl_upload), .ioctl_upload_req(nvram_save_req),
    .ioctl_upload_index(8'd2), .ioctl_din(nvram_ioctl_din), .ioctl_rd(ioctl_rd)
  );

  // ---- controls: MAME racedrivc port layout (the Compact set uses it) ----
  // MRA buttons (joystick_0 bits 4..11): Gas, Brake, Clutch, Abort, Gear down,
  // Gear up, Key (start), Coin; bits 12..15 select gears 1..4. Steering uses only the horizontal axis (or
  // Left/Right as digital end stops). Pedals use their named buttons or the
  // configured analog axes selected by the Pedals OSD setting. Up and Down
  // are not game controls.
  // This is a one-seat cabinet.  Only MiSTer's player-1 controls may drive
  // it: merging player 2 made an idle second controller (or an uncentred
  // axis) steer and press controls unexpectedly.
  wire [31:0] joy = joystick_0;
  wire btn_gas       = joy[4];
  wire btn_brake     = joy[5];
  wire btn_clutch    = joy[6];
  wire btn_abort     = joy[7];
  wire btn_gear_down = joy[8];
  wire btn_gear_up   = joy[9];
  wire btn_key       = joy[10];
  wire btn_coin      = joy[11];
  wire service_mode  = status[3];

  // The barcode-style internal diagnostic remains hard-disabled in release
  // builds.  The compact controls panel is a separate user-selectable aid.
  wire diagnostic_overlay_live = 1'b0;

  // ---- sound board 68000 EPROM (MRA rom index 1, 64 KiB) ----------------
  // The MRA interleaves 3122/3121 into big-endian words; the download
  // arrives a byte at a time, so pair them and write the on-chip ROM.
  logic [7:0]  snd_rom_hi_q = 8'h00;
  logic        snd_rom_wr_q = 1'b0;
  logic [14:0] snd_rom_addr_q = 15'd0;
  logic [15:0] snd_rom_data_q = 16'h0000;
  always_ff @(posedge CLK_50M) begin
    snd_rom_wr_q <= 1'b0;
    if (ioctl_wr && (ioctl_index == 16'd1)) begin
      if (!ioctl_addr[0]) begin
        snd_rom_hi_q <= ioctl_dout;
      end else begin
        snd_rom_addr_q <= ioctl_addr[15:1];
        snd_rom_data_q <= {snd_rom_hi_q, ioctl_dout};
        snd_rom_wr_q   <= 1'b1;
      end
    end
  end

  // ---- NVRAM (.nvm, ioctl index 2): the 200e/210e calibration EEPROMs ----
  wire        nvram_ioctl_wait;
  wire [7:0]  nvram_ioctl_din;
  wire        nvram_save_req;
  wire        nvram_req_level;
  wire        nvram_we;
  wire [11:0] nvram_addr;
  wire [7:0]  nvram_wdata;
  wire        nvram_ack_core;
  wire [7:0]  nvram_rdata_core;
  wire        nvram_dirty_core;
  // DIP switches from the MRA <switches> block (ioctl index 254): byte 0 =
  // SW1 (bit 0 = SW1:8 .. bit 7 = SW1:1, 1 = off, MAME's port 0 high byte),
  // byte 1 bit 0 = the DIAGN jumper (1 = off).
  logic [7:0] dip_sw1_q = 8'hff;
  logic [7:0] dip_misc_q = 8'h01;
  always_ff @(posedge CLK_50M) begin
    if (ioctl_wr && (ioctl_index == 16'd254) && (ioctl_addr[24:1] == 24'd0)) begin
      if (ioctl_addr[0]) dip_misc_q <= ioctl_dout;
      else               dip_sw1_q  <= ioctl_dout;
    end
  end

  // ---- analog pedals ----------------------------------------------------
  // The pedals are 8-bit ADC channels on the PCB, so drive them from analog
  // axes whenever one is deflected: left-stick Y and right-stick Y provide
  // the two directions of the gas/brake pedal pair, right-stick X the clutch.
  // Buttons stay as a full-press override, and every axis keeps a deadzone
  // so a resting stick reads zero.  The controls overlay shows the raw axes,
  // so a pad or wheel can be mapped by watching them.
  localparam [7:0] PEDAL_DEADZONE = 8'd12;
  function automatic [7:0] pedal_travel(input [7:0] raw, input pos_dir);
    logic [7:0] mag;
    logic [9:0] scaled;
    begin
      // raw is unsigned with 0x80 at rest; pos_dir selects the direction.
      if (pos_dir) mag = (raw > (8'h80 + PEDAL_DEADZONE)) ? (raw - 8'h80 - PEDAL_DEADZONE) : 8'd0;
      else         mag = (raw < (8'h80 - PEDAL_DEADZONE)) ? (8'h80 - PEDAL_DEADZONE - raw) : 8'd0;
      scaled = ({2'b00, mag} << 1) + {4'b0000, mag[7:2]};   // about x2.25 -> full scale
      pedal_travel = (scaled > 10'd255) ? 8'hff : scaled[7:0];
    end
  endfunction
  function automatic [7:0] pedal_max(input [7:0] a, input [7:0] b);
    pedal_max = (a > b) ? a : b;
  endfunction

  wire [15:0] l_analog = joystick_l_analog_0;
  wire [15:0] r_analog = joystick_r_analog_0;
  wire [7:0] axis_lx = l_analog[7:0]  ^ 8'h80;
  wire [7:0] axis_ly = l_analog[15:8] ^ 8'h80;
  wire [7:0] axis_rx = r_analog[7:0]  ^ 8'h80;
  wire [7:0] axis_ry = r_analog[15:8] ^ 8'h80;

  // Sticky evidence of which analog axes MiSTer actually delivers: a flag
  // sets the first time an axis leaves its resting value, so wiggling a
  // stick or squeezing a trigger shows on the overlay which axis it drives.
  logic [3:0] axis_moved_q = 4'd0;
  always_ff @(posedge CLK_50M) begin
    if ((axis_lx > 8'h90) || (axis_lx < 8'h70)) axis_moved_q[0] <= 1'b1;
    if ((axis_ly > 8'h90) || (axis_ly < 8'h70)) axis_moved_q[1] <= 1'b1;
    if ((axis_rx > 8'h90) || (axis_rx < 8'h70)) axis_moved_q[2] <= 1'b1;
    if ((axis_ry > 8'h90) || (axis_ry < 8'h70)) axis_moved_q[3] <= 1'b1;
  end

  // MiSTer normalises every analog axis to -127..127 around its mid point,
  // so an analog TRIGGER (which rests at one end of its travel) arrives as
  // an axis pinned near 0x00 that rises to 0xff when pressed.  "Sticks"
  // reads the Y axes as a bipolar pedal pair; "Triggers" reads three axes
  // directly as pedal travel, which is what a trigger or a wheel's pedal
  // set produces.  The overlay's axis row shows all four raw axes, so the
  // right assignment can be seen at a glance.
  wire wheel_mode = COCKPIT && status[8];
  // Stock Main sends mapped gamepad buttons and the ordinary stick axes.
  // Cockpit uses stock stick axes or mapped button levels. In particular,
  // a centred right stick must never be interpreted as half-pressed pedals.
  wire cockpit_button_pedals = COCKPIT && status[6];
  wire pedals_are_triggers = !COCKPIT && status[6];

  // Main's opt-in trigger transport has fixed endpoints: signed -127 is
  // released, +127 is fully pressed. After XOR 0x80 those are 1 and 255.
  // Do not learn a resting value from input history: a pedal held when the
  // core starts must retain its real position, and releasing either pedal
  // must not change the other's scale. Expand 0..254 to 0..255 at the top.
  function automatic [7:0] trigger_travel(input [7:0] raw);
    trigger_travel = (raw == 8'hff) ? 8'hff :
                     (raw > 8'd0) ? raw - 8'd1 : 8'd0;
  endfunction

  // Retain Compact's legacy third-axis clutch mapping separately. Cockpit
  // uses its mapped clutch button, independent of both analog sticks.
  logic [7:0] axis_floor_ly = 8'hff;
  always_ff @(posedge CLK_50M) begin
    if (axis_ly < axis_floor_ly) axis_floor_ly <= axis_ly;
  end
  // MiSTer's wheel layout is LX=steering, LY=gas, RX=clutch, RY=brake.
  // Wheel pedals are signed 0 (released) to -127 (fully pressed).
  function automatic [7:0] wheel_travel(input [7:0] raw);
    wheel_travel = raw >= 8'h80 ? 8'd0 : raw <= 8'd1 ? 8'hff
                   : ((8'h80 - raw) << 1);
  endfunction
  wire [7:0] axis_gas    = wheel_mode ? wheel_travel(axis_ly) : cockpit_button_pedals ? 8'd0 : pedals_are_triggers ? trigger_travel(axis_rx)
                         : COCKPIT ? pedal_travel(axis_ry, 1'b0) : pedal_max(pedal_travel(axis_ry, 1'b0), pedal_travel(axis_ly, 1'b0));
  wire [7:0] axis_brake  = wheel_mode ? wheel_travel(axis_ry) : cockpit_button_pedals ? 8'd0 : pedals_are_triggers ? trigger_travel(axis_ry)
                         : COCKPIT ? pedal_travel(axis_ry, 1'b1) : pedal_max(pedal_travel(axis_ry, 1'b1), pedal_travel(axis_ly, 1'b1));
  wire [7:0] axis_clutch = wheel_mode ? wheel_travel(axis_rx) : COCKPIT ? 8'd0 : pedals_are_triggers ? ((axis_ly > axis_floor_ly) ? axis_ly - axis_floor_ly : 8'd0)
                         : pedal_travel(axis_rx, 1'b1);

  // MiSTer's pad mapper can assert the named digital button for the same
  // physical trigger that is also supplying an analog axis.  In trigger
  // mode the axis must remain authoritative or any touch becomes full-scale.
  wire [7:0] gas_value    = (!wheel_mode && !pedals_are_triggers && btn_gas)    ? 8'hff : axis_gas;
  wire [7:0] clutch_value = ((COCKPIT || !pedals_are_triggers) && btn_clutch) ? 8'hff : axis_clutch;
  wire [7:0] brake_force  = (!wheel_mode && !pedals_are_triggers && btn_brake)  ? 8'hff : axis_brake;

  wire reset = RESET | status[0] | buttons[1] | ioctl_download;
  wire ce_pix;
  wire hblank, vblank, hsync, vsync;
  wire [23:0] rgb;
  wire [31:0] debug;
  wire clk_core;

  harddrivin_nvram_bridge u_nvram_bridge (
    .hps_clk_i(CLK_50M),
    .ioctl_download_i(ioctl_download), .ioctl_upload_i(ioctl_upload),
    .ioctl_wr_i(ioctl_wr), .ioctl_rd_i(ioctl_rd), .ioctl_index_i(ioctl_index),
    .ioctl_addr_i(ioctl_addr[11:0]), .ioctl_dout_i(ioctl_dout),
    .ioctl_wait_o(nvram_ioctl_wait), .ioctl_din_o(nvram_ioctl_din),
    .ioctl_upload_req_o(nvram_save_req),
    .core_clk_i(clk_core), .core_req_o(nvram_req_level),
    .core_we_o(nvram_we), .core_addr_o(nvram_addr),
    .core_wdata_o(nvram_wdata), .core_ack_i(nvram_ack_core),
    .core_rdata_i(nvram_rdata_core), .core_dirty_i(nvram_dirty_core)
  );

  logic [3:0] gear_state_q;   // one-hot 1st..4th (port 1 bits 8..11, active low), 0 = neutral
  // MAME declares the cabinet Key as momentary START1, without PORT_TOGGLE.
  // Calibration waits for it to be released after each turn; latching it on
  // leaves the ROM on its deliberately blank transition page.
  // Bring-up builds can traverse cabinet calibration unattended. This is
  // hard-disabled in the release so the player performs the cabinet's real
  // key turns and control movements; there is no release OSD switch that can
  // accidentally enable the synthetic calibration sequence.
  wire DEVELOPMENT_BOOT_ASSIST = 1'b0;
  // The first boot starts from the target set's original 200E/210E PROM
  // dumps.  MiSTer then downloads/saves the player's own 4 KiB .nvm image,
  // preserving the cabinet's real calibration procedure and later results.
  logic [25:0] boot_pot_timer_q;
  logic [27:0] boot_brake_timer_q;
  logic [27:0] boot_shifter_timer_q;
  logic [27:0] boot_seat_timer_q;
  logic [27:0] boot_steering_timer_q;
  logic [1:0] boot_pot_prompt_sync_q;
  logic [1:0] boot_brake_sync_q;
  logic [1:0] boot_shifter_sync_q;
  logic [1:0] boot_seat_sync_q;
  logic [1:0] boot_calibration_sync_q;
  logic boot_pot_prompt_seen_q;
  logic boot_brake_seen_q;
  logic boot_shifter_seen_q;
  logic boot_seat_seen_q;
  logic boot_calibration_seen_q;
  logic [12:0] boot_steering_ramp;
  logic [11:0] boot_steering;

  // A shifter on the wheel base shares player 1. A separate USB shifter
  // can use player 2 without letting that device steer or apply pedals.
  wire [31:0] shifter_joy = status[9] ? joystick_1 : joystick_0;
  harddrivin_gearbox u_gearbox (
    .clk_i(CLK_50M), .rst_i(reset), .h_pattern_i(status[7]),
    .up_i(shifter_joy[9]), .down_i(shifter_joy[8]),
    .positions_i(shifter_joy[15:12]), .gear_o(gear_state_q)
  );

  // Compact startup follows the physical cabinet procedure: centered pot
  // baseline, force brake, two shifter corners, two seat endpoints, and the
  // four-turn steering sweep. Each stage is armed by the ROM's observed PC
  // window so slow rendering or host startup cannot shift the sequence.
  logic boot_pot_prompt_active;
  logic boot_brake_active;
  logic boot_shifter_active;
  logic boot_seat_active;
  logic boot_calibration_active;
  always_ff @(posedge CLK_50M) begin
    if (reset) begin
      boot_pot_timer_q        <= 26'd0;
      boot_brake_timer_q      <= 28'd0;
      boot_shifter_timer_q    <= 28'd0;
      boot_seat_timer_q       <= 28'd0;
      boot_steering_timer_q   <= 28'd0;
      boot_pot_prompt_sync_q  <= 2'b00;
      boot_brake_sync_q       <= 2'b00;
      boot_shifter_sync_q     <= 2'b00;
      boot_seat_sync_q        <= 2'b00;
      boot_calibration_sync_q <= 2'b00;
      boot_pot_prompt_seen_q  <= 1'b0;
      boot_brake_seen_q       <= 1'b0;
      boot_shifter_seen_q     <= 1'b0;
      boot_seat_seen_q        <= 1'b0;
      boot_calibration_seen_q <= 1'b0;
    end else begin
      boot_pot_prompt_sync_q <= {
        boot_pot_prompt_sync_q[0], boot_pot_prompt_active
      };
      boot_brake_sync_q <= {
        boot_brake_sync_q[0], boot_brake_active
      };
      boot_shifter_sync_q <= {
        boot_shifter_sync_q[0], boot_shifter_active
      };
      boot_seat_sync_q <= {
        boot_seat_sync_q[0], boot_seat_active
      };
      boot_calibration_sync_q <= {
        boot_calibration_sync_q[0], boot_calibration_active
      };

      if (!boot_pot_prompt_seen_q) begin
        boot_pot_timer_q <= 26'd0;
        if (boot_pot_prompt_sync_q[1])
          boot_pot_prompt_seen_q <= 1'b1;
      end else if (!(&boot_pot_timer_q)) begin
        boot_pot_timer_q <= boot_pot_timer_q + 26'd1;
      end

      if (!boot_brake_seen_q) begin
        boot_brake_timer_q <= 28'd0;
        if (boot_brake_sync_q[1]) boot_brake_seen_q <= 1'b1;
      end else if (!(&boot_brake_timer_q)) begin
        boot_brake_timer_q <= boot_brake_timer_q + 28'd1;
      end

      if (!boot_shifter_seen_q) begin
        boot_shifter_timer_q <= 28'd0;
        if (boot_shifter_sync_q[1]) boot_shifter_seen_q <= 1'b1;
      end else if (!(&boot_shifter_timer_q)) begin
        boot_shifter_timer_q <= boot_shifter_timer_q + 28'd1;
      end

      if (!boot_seat_seen_q) begin
        boot_seat_timer_q <= 28'd0;
        if (boot_seat_sync_q[1]) boot_seat_seen_q <= 1'b1;
      end else if (!(&boot_seat_timer_q)) begin
        boot_seat_timer_q <= boot_seat_timer_q + 28'd1;
      end

      if (!boot_calibration_seen_q) begin
        boot_steering_timer_q <= 28'd0;
        if (boot_calibration_sync_q[1])
          boot_calibration_seen_q <= 1'b1;
      end else if (!(&boot_steering_timer_q)) begin
        boot_steering_timer_q <= boot_steering_timer_q + 28'd1;
      end
    end
  end
  wire boot_pot_turn_key = DEVELOPMENT_BOOT_ASSIST &&
                           boot_pot_prompt_seen_q &&
                           (boot_pot_timer_q >= 26'd25_000_000) &&
                           (boot_pot_timer_q <  26'd50_000_000);
  wire boot_shifter_turn_key = DEVELOPMENT_BOOT_ASSIST &&
                        boot_shifter_seen_q &&
                       (((boot_shifter_timer_q >= 28'd50_000_000) &&
                         (boot_shifter_timer_q <  28'd75_000_000)) ||
                        ((boot_shifter_timer_q >= 28'd125_000_000) &&
                         (boot_shifter_timer_q <  28'd150_000_000)));
  wire boot_seat_turn_key = DEVELOPMENT_BOOT_ASSIST &&
                        boot_seat_seen_q &&
                       (((boot_seat_timer_q >= 28'd50_000_000) &&
                         (boot_seat_timer_q <  28'd75_000_000)) ||
                        ((boot_seat_timer_q >= 28'd125_000_000) &&
                         (boot_seat_timer_q <  28'd150_000_000)));
  wire boot_steering_turn_key = DEVELOPMENT_BOOT_ASSIST &&
                       boot_calibration_seen_q &&
                       (boot_steering_timer_q >= 28'd150_000_000) &&
                       (boot_steering_timer_q <  28'd200_000_000);
  wire boot_turn_key = boot_pot_turn_key || boot_shifter_turn_key ||
                       boot_seat_turn_key || boot_steering_turn_key;

  // Absolute wheel target: no input filter or artificial slew. The selected
  // player's left analog X maps to all 12 target bits immediately; the main
  // bus presents that target through the cabinet's modulo-256 wheel counter
  // at the fastest direction-safe rate. MiSTer Left/Right remain digital
  // end-stop fallbacks.
  wire [11:0] player_steering, pad_steering, wheel_steering;
  assign player_steering = wheel_mode ? wheel_steering : pad_steering;
  harddrivin_steering u_steering (
    .axis_i(l_analog[7:0]),
    .right_i(joy[0]),
    .left_i(joy[1]),
    .position_o(pad_steering)
  );

  harddrivin_steering #(.DEADZONE(0)) u_wheel_steering (
    .axis_i(l_analog[7:0]), .right_i(1'b0), .left_i(1'b0),
    .position_o(wheel_steering)
  );

  // Start near one lock, sweep through the complete counter range, hold long
  // enough for the ROM's four samples, then return to center before the key.
  always_comb begin
    boot_steering_ramp = 13'h010;
    boot_steering      = 12'h800;
    if (boot_steering_timer_q < 28'd25_000_000) begin
      boot_steering = 12'h010;
    end else if (boot_steering_timer_q < 28'd100_000_000) begin
      boot_steering_ramp = 13'h010 +
                           13'((boot_steering_timer_q - 28'd25_000_000) >> 14);
      if (boot_steering_ramp > 13'hff0)
        boot_steering = 12'hff0;
      else
        boot_steering = boot_steering_ramp[11:0];
    end else if (boot_steering_timer_q < 28'd150_000_000) begin
      boot_steering_ramp =
        13'((boot_steering_timer_q - 28'd100_000_000) >> 14);
      if (boot_steering_ramp > 13'h7f0)
        boot_steering = 12'h800;
      else
        boot_steering = 12'hff0 - boot_steering_ramp[11:0];
    end
  end
  wire [11:0] steering = (DEVELOPMENT_BOOT_ASSIST &&
                          boot_calibration_seen_q &&
                          (boot_steering_timer_q < 28'd200_000_000))
                       ? boot_steering : player_steering;

  // The force-brake transducer (8-bit ADC channel 6) reads 0xf0 released
  // and falls to 0x20 under full pressure (MAME racedrivc 8BADC.6: MINMAX
  // 0x20..0xf0, reversed, rest 0xf0 - the values MAME's saved calibration
  // expects).  The ROM's calibration needs at least a 0x10 excursion
  // followed by a return to within four counts of the baseline.
  wire [16:0] brake_drop    = brake_force * 9'd209;                     // 0..255 -> 0..0xd0
  wire [7:0]  player_brake  = 8'hf0 - brake_drop[15:8];
  wire [7:0] boot_brake = ((boot_brake_timer_q >= 28'd75_000_000) &&
                           (boot_brake_timer_q <  28'd125_000_000))
                         ? 8'h20 : 8'hf0;
  wire [7:0] brake = (DEVELOPMENT_BOOT_ASSIST && boot_brake_seen_q &&
                      (boot_brake_timer_q < 28'd150_000_000))
                   ? boot_brake : player_brake;

  // These values belong to the earlier cockpit input hardware and are kept
  // only for the optional development calibration sequencer.  harddrivc uses
  // MAME's racedrivc map: its ADC0809 channels 2..5 are physically unused;
  // ordinary gear selection is the maintained digital port below.
  logic [7:0] player_shifter_x;
  logic [7:0] player_shifter_y;
  always_comb begin
    player_shifter_x = 8'h80;
    player_shifter_y = 8'h80;
    unique case (gear_state_q)
      4'b0001: begin player_shifter_x = 8'h20; player_shifter_y = 8'he0; end
      4'b0010: begin player_shifter_x = 8'h20; player_shifter_y = 8'h20; end
      4'b0100: begin player_shifter_x = 8'he0; player_shifter_y = 8'he0; end
      4'b1000: begin player_shifter_x = 8'he0; player_shifter_y = 8'h20; end
      default: ;
    endcase
  end
  wire boot_shifter_second = boot_shifter_timer_q >= 28'd100_000_000;
  wire [7:0] shifter_x = (DEVELOPMENT_BOOT_ASSIST &&
                          boot_shifter_seen_q &&
                          (boot_shifter_timer_q < 28'd175_000_000))
                       ? (boot_shifter_second ? 8'he0 : 8'h20)
                       : player_shifter_x;
  wire [7:0] shifter_y = (DEVELOPMENT_BOOT_ASSIST &&
                          boot_shifter_seen_q &&
                          (boot_shifter_timer_q < 28'd175_000_000))
                       ? (boot_shifter_second ? 8'he0 : 8'h20)
                       : player_shifter_y;

  // Development-only legacy seat sweep.  The Compact racedrivc input map
  // leaves ADC channel 2 unconnected, so the board bus does not expose this.
  wire [7:0] seat = (DEVELOPMENT_BOOT_ASSIST && boot_seat_seen_q &&
                     (boot_seat_timer_q < 28'd175_000_000))
                  ? ((boot_seat_timer_q < 28'd100_000_000) ? 8'h20 : 8'he0)
                  : 8'h80;
  // MAME racedrivc a80000: bit 0 Abort, bit 1 Key, bit 2 aux coin, bits
  // 8..11 gears 1st..4th (all active low); bit 14 = wheel edge (the bus).
  wire [15:0] compact_port1 = {
    4'b1111,
    ~gear_state_q[3], ~gear_state_q[2],
    ~gear_state_q[1], ~gear_state_q[0],
    5'b11111,
    1'b1, ~(btn_key | boot_turn_key), ~btn_abort
  };

  harddrivin_milestone #(
    .COCKPIT(COCKPIT),
    .DIAG_FULL(1'b0),   // verification builds keep the GSP boot, runtime and DI-latency pages
    .ZERO_RAM_INIT_HI_FILE(COCKPIT ? "generated/zram_cockpit_200e.hex" : "generated/zram_200e.hex"),
    .ZERO_RAM_INIT_LO_FILE(COCKPIT ? "generated/zram_cockpit_210e.hex" : "generated/zram_210e.hex")
  ) u_milestone (
    .clk_50_i(CLK_50M), .rst_i(reset), .clk_core_o(clk_core), .ce_pix_o(ce_pix),
    .hblank_o(hblank), .vblank_o(vblank), .hsync_o(hsync),
    .vsync_o(vsync), .rgb_o(rgb), .debug_o(debug),
    .diagnostic_enable_i(diagnostic_overlay_live),
    .controls_overlay_i(status[4]),
    .analog_axes_i({axis_ry, axis_rx, axis_ly, axis_lx}),
    .axis_moved_i(axis_moved_q),
    .pot_prompt_active_o(boot_pot_prompt_active),
    .brake_calibration_active_o(boot_brake_active),
    .shifter_calibration_active_o(boot_shifter_active),
    .seat_calibration_active_o(boot_seat_active),
    .calibration_active_o(boot_calibration_active),
    .coin1_i(btn_coin), .coin2_i(1'b0),
    .service_i(service_mode), .diag_jumper_i(dip_misc_q[0]), .sw1_i(dip_sw1_q),
    .compact_port1_i(compact_port1),
    .accelerator_i(gas_value),
    .clutch_i(clutch_value), .seat_i(seat), .shifter_x_i(shifter_x),
    .shifter_y_i(shifter_y), .brake_i(brake),
    .steering_i(steering), .brake12_i(12'hfff - {brake_force, brake_force[7:4]}), .audio_o(sound_audio),
    .snd_rom_wr_clk_i(CLK_50M), .snd_rom_wr_en_i(snd_rom_wr_q),
    .snd_rom_wr_addr_i(snd_rom_addr_q), .snd_rom_wr_data_i(snd_rom_data_q),
    .nvram_req_i(nvram_req_level), .nvram_we_i(nvram_we), .nvram_addr_i(nvram_addr),
    .nvram_wdata_i(nvram_wdata), .nvram_ack_o(nvram_ack_core),
    .nvram_rdata_o(nvram_rdata_core), .nvram_dirty_o(nvram_dirty_core),
    .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
    .DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT),
    .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
    .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
    .SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA),
    .SDRAM_CLK(SDRAM_CLK), .SDRAM_CKE(SDRAM_CKE),
    .SDRAM_DQML(SDRAM_DQML), .SDRAM_DQMH(SDRAM_DQMH),
    .SDRAM_nCS(SDRAM_nCS), .SDRAM_nCAS(SDRAM_nCAS),
    .SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nWE(SDRAM_nWE)
  );
  assign DDRAM_CLK = clk_core;

  arcade_video #(.WIDTH(COCKPIT ? 508 : 512), .DW(24), .GAMMA(1)) u_video (
    .clk_video(clk_core), .ce_pix(ce_pix), .RGB_in(rgb),
    .HBlank(hblank), .VBlank(vblank), .HSync(hsync), .VSync(vsync),
    .CLK_VIDEO(CLK_VIDEO), .CE_PIXEL(CE_PIXEL),
    .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B),
    .VGA_HS(VGA_HS), .VGA_VS(VGA_VS), .VGA_DE(VGA_DE), .VGA_SL(VGA_SL),
    .fx(3'd0), .forced_scandoubler(forced_scandoubler), .gamma_bus(gamma_bus)
  );

  assign LED_USER = debug[22] ^ debug[9];

  logic unused_inputs;
  always_comb unused_inputs = CLK_AUDIO ^ SD_MISO ^ SD_CD ^ UART_CTS ^
                              UART_RXD ^ UART_DSR ^ ^USER_IN ^ OSD_STATUS ^
                              ^HDMI_WIDTH ^ ^HDMI_HEIGHT;
endmodule

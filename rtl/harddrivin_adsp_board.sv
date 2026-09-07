`default_nettype none

// Atari Hard Drivin' ADSP board (drawing A044421) as seen from the 68010 and
// from the ADSP-2100 data-memory bus.  Sheet references are to that drawing.
//
// 68010 window 0x800000-0x83ffff (sheet 1 decoder 7B on A17:A15):
//   000  /PROGMEM   0x800000  8Kx24 program RAM, one 24-bit word per 32-bit
//                             longword (upper word carries bits 23:16)
//   001  /DATAMEM   0x808000  8Kx16 data memory, word address A13:A1
//   010  /BUFFERMEM 0x810000  sequential output memory, bank BCON, A13:A1
//   011  /ASEL      0x818000  sheet 2 decoder 7C on BA07:05:
//                               000 /LATCHES 74LS259 7K: A3:A1 select, A4 data
//                                   Q7 /ADSPRES  Q6 /ADSPHALT  Q5 /ADSPBR
//                                   Q3 /BCON     Q0-Q2,Q4 LEDs/unused
//                               011 /CGINT   clears the DIRQ flop
//   111  /READSTAT  0x838000  sheet 3 buffer 8F: D0 /DIRQ, D1 XFLAG, D2 1
//
// ADSP data-memory ports (sheet 2 decoders 7J/5K, DMA13=1, DMA2:0):
//   0x2000 read   /SIMBUF sequential input EPROM word, counter post-increments
//   0x2001 write  /SIMLD  load the 16-bit sequential input counter
//   0x2002 write  /SOMD   sequential output word, counter post-increments
//   (order verified against the ADSP program itself: PC 0x33/0x34 do
//   DM(0x2003)=0 then DM(0x2002)=0 (output address, then output word),
//   PC 0x90..0x95 do DM(0x2001)=addr then AR=DM(0x2000) (input counter,
//   then input word); MAME hdadsp_special_w agrees.  The first RTL had
//   0x2001/0x2002 swapped, which left the polygon buffer empty ->
//   BAD POLY BUFF ERROR.)
//   0x2003 write  /SOMLD  load the sequential output counter
//   0x2005 write  /XOUT   XFLAG <= DMD0
//   0x2006 write  /GINT   set DIRQ (68010 interrupt level 2)
//   0x2007 write  /MP     MPAGE <= DMD0 (selects the EPROM pair)
// Sheet 4/5: SIM EPROMs 10H/10K (MPAGE=0) and 10J/10L (MPAGE=1), low/high
// byte.  Sheets 5-8: two 8Kx16 output banks; the 68010 reads bank BCON while
// the ADSP counters address the other bank.
module harddrivin_adsp_board #(
  // MAC rounding rule of the ADSP-2100 core: 0 = manual 2.3.2.6 (default),
  // 1 = MAME's variant (replay benches only, to stay word-exact with the oracle).
  parameter integer MAC_ROUND_MAME = 0
) (
  input  logic        clk_i,
  input  logic        rst_i,

  // 68010 access: held request, one-cycle acknowledge.
  input  logic        host_req_i,
  input  logic        host_we_i,
  input  logic [17:1] host_addr_i,
  input  logic [15:0] host_wdata_i,
  output logic [15:0] host_rdata_o,
  output logic        host_ack_o,
  output logic        irq_o,

  // Sequential input EPROMs, served from DDR (prepare_roms.py
  // adsp_sequential_data.bin order at DDR byte 0x30100000).  Toggle
  // request/acknowledge: sim_addr_o is the 17-bit word {MPAGE, counter}.
  output logic        sim_req_o,
  output logic [16:0] sim_addr_o,
  input  logic        sim_ack_i,
  input  logic [15:0] sim_data_i,

  // Diagnostics.
  output logic [13:0] adsp_pc_o,
  output logic        adsp_running_o,
  output logic        adsp_illegal_o,
  output logic [7:0]  latch_o,
  output logic        dirq_o,
  output logic        xflag_o,
  output logic        mpage_o,
  output logic [15:0] gint_count_o,
  output logic [15:0] upload_count_o,
  output logic [15:0] upload_sum_lo_o,   // sum16 of the A1=1 (low) upload words
  output logic [15:0] upload_xor_lo_o,
  output logic [15:0] upload_sum_hi_o,   // sum16 of the A1=0 (high byte) words
  output logic [15:0] upload_xor_hi_o,
  output logic [15:0] sim_read_count_o,
  output logic [15:0] som_write_count_o
);
  // ---- Control latch (74LS259 7K), cleared by power-on reset ----
  logic [7:0] latch_q;
  logic adspres_n, adsphalt_n, adspbr_n, bcon;
  assign adspres_n  = latch_q[7];
  assign adsphalt_n = latch_q[6];
  assign adspbr_n   = latch_q[5];
  assign bcon       = !latch_q[3];
  assign latch_o    = latch_q;

  logic dirq_q, xflag_q, mpage_q;
  assign irq_o = dirq_q;
  assign dirq_o = dirq_q;
  assign xflag_o = xflag_q;
  assign mpage_o = mpage_q;

  // ---- 68010 decode ----
  logic sel_prog, sel_data, sel_buffer, sel_asel, sel_readstat;
  logic sel_latches, sel_cgint;
  assign sel_prog     = host_addr_i[17:15] == 3'b000;
  assign sel_data     = host_addr_i[17:15] == 3'b001;
  assign sel_buffer   = host_addr_i[17:15] == 3'b010;
  assign sel_asel     = host_addr_i[17:15] == 3'b011;
  assign sel_readstat = host_addr_i[17:15] == 3'b111;
  assign sel_latches  = sel_asel && (host_addr_i[7:5] == 3'b000);
  assign sel_cgint    = sel_asel && (host_addr_i[7:5] == 3'b011);

  logic host_pending_q;
  logic host_req_q;
  logic host_start;
  assign host_start = host_req_i && !host_req_q;

  // ---- ADSP core ----
  logic        core_rst, core_halt;
  logic        prog_we;
  logic prog_re, prog_ready;
  logic [23:0] prog_rdata;
  logic [12:0] prog_read_addr_q;
  logic prog_read_low_q, prog_read_sample_q;
  logic [7:0]  prog_hi_q;
  logic        dm_req, dm_we, dm_ready;
  logic [13:0] dm_addr;
  logic [15:0] dm_wdata, dm_rdata;
  logic [13:0] adsp_pc;
  logic        adsp_running, adsp_illegal;
  assign core_rst  = rst_i || !adspres_n;
  // The physical /BR input grants the 68010 exclusive access to the ADSP
  // program/data buses by stopping the processor at its next instruction
  // boundary.  Treat it exactly like /HALT here.  Ignoring /BR allows the
  // host and ADSP to touch shared memory concurrently during driven-game
  // command transfers, which can corrupt a polygon job and strand the main
  // CPU in its ADSP timeout path.
  assign core_halt = !adsphalt_n || !adspbr_n || prog_re;
  assign prog_re = host_pending_q && host_kind_q == 3'd5;
  assign prog_we   = host_start && host_we_i && sel_prog && host_addr_i[1];
  assign adsp_pc_o = adsp_pc;
  assign adsp_running_o = adsp_running;
  assign adsp_illegal_o = adsp_illegal;

  adsp2100_core #(.MAC_ROUND_MAME(MAC_ROUND_MAME)) u_core (
    .clk_i(clk_i), .rst_i(core_rst), .halt_i(core_halt),
    .prog_we_i(prog_we), .prog_re_i(prog_re),
    .prog_ready_o(prog_ready), .prog_rdata_o(prog_rdata),
    .prog_addr_i(prog_re ? prog_read_addr_q : host_addr_i[14:2]),
    .prog_wdata_i({prog_hi_q, host_wdata_i}),
    .dm_req_o(dm_req), .dm_we_o(dm_we), .dm_addr_o(dm_addr),
    .dm_wdata_o(dm_wdata), .dm_rdata_i(dm_rdata), .dm_ready_i(dm_ready),
    .pc_o(adsp_pc), .instruction_o(), .running_o(adsp_running),
    .illegal_o(adsp_illegal)
  );

  // ---- Data memory 8Kx16, port A = ADSP, port B = 68010 ----
  logic dm_is_port, dm_busy_q;
  logic [15:0] dm_ram_rdata_a, dm_ram_rdata_b;
  assign dm_is_port = dm_addr[13];

  harddrivin_true_dual_port_ram #(.ADDR_WIDTH(13)) u_dm_ram (
    .clk_i(clk_i),
    .en_a_i(dm_req && !dm_is_port && !dm_busy_q), .we_a_i(dm_we),
    .addr_a_i(dm_addr[12:0]), .wdata_a_i(dm_wdata), .rdata_a_o(dm_ram_rdata_a),
    .en_b_i(host_start && sel_data), .we_b_i(host_we_i),
    .addr_b_i(host_addr_i[13:1]), .wdata_b_i(host_wdata_i), .rdata_b_o(dm_ram_rdata_b)
  );

  // ---- Sequential input EPROMs: DDR client ----
  logic [15:0] sim_ctr_q;
  logic        sim_wait_q;      // DDR read outstanding for the current port read
  logic        sim_done_q;      // data captured, acknowledge the ADSP next cycle
  logic [15:0] sim_data_q;
  assign sim_addr_o = {mpage_q, sim_ctr_q};

  // ---- Sequential output memory: two 8Kx16 banks ----
  logic [12:0] som_ctr_q;
  logic        som_write;
  logic [15:0] som_rdata0, som_rdata1;
  assign som_write = dm_req && dm_we && dm_is_port && (dm_addr[2:0] == 3'd2) && !dm_busy_q;
  harddrivin_dual_port_word_ram #(.ADDR_WIDTH(13)) u_som0 (
    .clk_a_i(clk_i), .en_a_i(som_write && bcon), .we_a_i(1'b1),
    .addr_a_i(som_ctr_q), .wdata_a_i(dm_wdata), .rdata_a_o(),
    .clk_b_i(clk_i), .en_b_i(host_start && sel_buffer), .addr_b_i(host_addr_i[13:1]),
    .rdata_b_o(som_rdata0)
  );
  harddrivin_dual_port_word_ram #(.ADDR_WIDTH(13)) u_som1 (
    .clk_a_i(clk_i), .en_a_i(som_write && !bcon), .we_a_i(1'b1),
    .addr_a_i(som_ctr_q), .wdata_a_i(dm_wdata), .rdata_a_o(),
    .clk_b_i(clk_i), .en_b_i(host_start && sel_buffer), .addr_b_i(host_addr_i[13:1]),
    .rdata_b_o(som_rdata1)
  );

  // ---- ADSP data bus response ----
  // RAM and port writes: one registered cycle.  Sequential-input reads wait
  // for the DDR word; the core holds its request meanwhile.
  logic dm_sim_read;
  assign dm_sim_read = dm_is_port && !dm_we && (dm_addr[2:0] == 3'd0);
  assign dm_ready = dm_sim_read ? sim_done_q : dm_busy_q;
  always_comb begin
    if (!dm_is_port) dm_rdata = dm_ram_rdata_a;
    else if (dm_addr[2:0] == 3'd0) dm_rdata = sim_data_q;
    else dm_rdata = 16'h0000;
  end

  logic [2:0] host_kind_q;   // 0 none, 1 data, 2 buffer, 3 status, 4 other
  logic [15:0] host_status;
  assign host_status = {13'h1fff, 1'b1, xflag_q, !dirq_q};

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      latch_q <= 8'h00;
      dirq_q <= 1'b0;
      xflag_q <= 1'b0;
      mpage_q <= 1'b0;
      prog_hi_q <= 8'h00;
      dm_busy_q <= 1'b0;
      sim_ctr_q <= 16'd0;
      sim_wait_q <= 1'b0;
      sim_done_q <= 1'b0;
      sim_data_q <= 16'h0000;
      sim_req_o <= 1'b0;
      som_ctr_q <= 13'd0;
      host_pending_q <= 1'b0;
      host_req_q <= 1'b0;
      host_ack_o <= 1'b0;
      host_rdata_o <= 16'hffff;
      host_kind_q <= 3'd0;
      prog_read_addr_q <= '0;
      prog_read_low_q <= 1'b0;
      prog_read_sample_q <= 1'b0;
      gint_count_o <= 16'd0;
      upload_count_o <= 16'd0;
      upload_sum_lo_o <= 16'd0; upload_xor_lo_o <= 16'd0;
      upload_sum_hi_o <= 16'd0; upload_xor_hi_o <= 16'd0;
      sim_read_count_o <= 16'd0;
      som_write_count_o <= 16'd0;
    end else begin
      host_ack_o <= 1'b0;
      host_req_q <= host_req_i;

      // ADSP-side access: request accepted on the first cycle, acknowledged
      // with data on the next.  Port side effects happen on the acknowledge.
      dm_busy_q <= dm_req && !dm_busy_q && !dm_sim_read;
      // Sequential-input read: one DDR word per port read, then increment.
      sim_done_q <= 1'b0;
      if (dm_req && dm_sim_read && !sim_wait_q && !sim_done_q) begin
        sim_req_o  <= !sim_req_o;
        sim_wait_q <= 1'b1;
      end else if (sim_wait_q && (sim_ack_i == sim_req_o)) begin
        sim_wait_q <= 1'b0;
        sim_done_q <= 1'b1;
        sim_data_q <= sim_data_i;
        sim_ctr_q  <= sim_ctr_q + 16'd1;
        sim_read_count_o <= sim_read_count_o + 16'd1;
      end
      if (dm_req && !dm_busy_q && dm_is_port && dm_we) begin
        begin
          case (dm_addr[2:0])
            3'd1: sim_ctr_q <= dm_wdata;
            3'd2: begin som_ctr_q <= som_ctr_q + 13'd1; som_write_count_o <= som_write_count_o + 16'd1; end
            3'd3: som_ctr_q <= dm_wdata[12:0];
            3'd5: xflag_q <= dm_wdata[0];
            3'd6: begin dirq_q <= 1'b1; gint_count_o <= gint_count_o + 16'd1; end
            3'd7: mpage_q <= dm_wdata[0];
            default: ;
          endcase
        end
      end

      // 68010-side access.
      if (host_start) begin
        host_pending_q <= 1'b1;
        host_kind_q <= sel_data ? 3'd1 : sel_buffer ? 3'd2 : sel_readstat ? 3'd3 : (sel_prog && !host_we_i) ? 3'd5 : 3'd4;
        prog_read_addr_q <= host_addr_i[14:2];
        prog_read_low_q <= host_addr_i[1];
        prog_read_sample_q <= 1'b0;
        if (host_we_i) begin
          if (sel_prog) begin
            if (!host_addr_i[1]) begin
              prog_hi_q <= host_wdata_i[7:0];
              upload_sum_hi_o <= upload_sum_hi_o + host_wdata_i;
              upload_xor_hi_o <= upload_xor_hi_o ^ host_wdata_i;
            end else begin
              upload_count_o  <= upload_count_o + 16'd1;
              upload_sum_lo_o <= upload_sum_lo_o + host_wdata_i;
              upload_xor_lo_o <= upload_xor_lo_o ^ host_wdata_i;
            end
          end
          if (sel_latches) latch_q[host_addr_i[3:1]] <= host_addr_i[4];
          if (sel_cgint) dirq_q <= 1'b0;
        end
      end else if (host_pending_q) begin
        // Port A belongs to instruction fetch until the ADSP reaches its
        // halted boundary. Then allow one synchronous RAM read before ACK.
        if (host_kind_q == 3'd5 && !prog_read_sample_q) begin
          if (prog_ready) prog_read_sample_q <= 1'b1;
        end else begin
        host_pending_q <= 1'b0;
        host_ack_o <= 1'b1;
        case (host_kind_q)
          3'd1: host_rdata_o <= dm_ram_rdata_b;
          3'd2: host_rdata_o <= bcon ? som_rdata1 : som_rdata0;
          3'd3: host_rdata_o <= host_status;
          3'd5: host_rdata_o <= prog_read_low_q ? prog_rdata[15:0] : {8'd0, prog_rdata[23:16]};
          default: host_rdata_o <= 16'hffff;
        endcase
        end
      end
    end
  end
endmodule

`default_nettype wire

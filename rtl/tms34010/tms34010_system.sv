// -----------------------------------------------------------------------------
// tms34010_system.sv
//
// Synthesizable functional-system wrapper for the TMS34010 core and its
// integrated abstract memory fabric. This is the first boundary at which
// CPU/graphics, screen refresh, DRAM refresh, and host-indirect traffic all
// converge on one controller request.
//
// Physical host strobes/HRDY/CDC and original local-bus pin phases remain
// outside this wrapper. The dedicated VCLK enters explicitly and its video
// CDC is owned below the core I/O block. The controller-facing cycle kind and
// payload remain explicit for the pin wrapper.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_system
  import tms34010_pkg::*;
#(
  parameter bit PIXEL_OPS = 1'b1,  // Task 0180
  parameter bit VIDEO = 1'b1,      // Task 0188
  parameter bit FAST_BLIT = 1'b0,  // see tms34010_core
  // 0: no DRAM refresh cycles on the local bus (a controller that refreshes
  // its own memory, e.g. SDRAM, does not need them; MAME charges none)
  parameter bit DRAM_REFRESH = 1'b1
) (
  input  logic                              clk,
  input  logic                              vclk_i,
  input  logic                              rst,
  input  logic                              vclk_rst_i,
  input  logic                              video_hsync_n_i,
  input  logic                              video_vsync_n_i,

  input  logic                              run_emu_n_i,
  output logic                              emua_n_o,

  input  logic                              hcs_n_i,
  input  logic                              host_req_i,
  input  logic                              host_we_i,
  input  host_reg_sel_t                     host_reg_i,
  input  logic [1:0]                        host_be_i,
  input  local_word_t                       host_wdata_i,
  output local_word_t                       host_rdata_o,
  output logic                              host_ack_o,
  output logic                              host_busy_o,
  output logic                              hint_n_o,

  input  logic                              lint1_n_i,
  input  logic                              lint2_n_i,
  input  logic                              dpyint_set_i,

  input  logic                              hold_req_i,
  output logic                              hold_ack_o,

  output logic                              video_hsync_o,
  output logic                              video_vsync_o,
  output logic                              video_hblank_o,
  output logic                              video_vblank_o,
  output logic                              video_blank_o,
  output logic                              video_hsync_oe_o,
  output logic                              video_vsync_oe_o,

  output logic                              cycle_req_o,
  output local_cycle_kind_t                 cycle_kind_o,
  output logic [ADDR_WIDTH-1:0]             cycle_addr_o,
  output local_word_t                       cycle_wdata_o,
  output local_word_t                       cycle_io_rdata_o,
  output logic                              cycle_iaq_o,
  output logic [13:0]                       cycle_srfaddr_o,
  output logic [15:0]                       cycle_dpytap_o,
  output logic                              cycle_screen_org_o,
  output logic [7:0]                        cycle_dram_row_o,
  input  local_word_t                       cycle_rdata_i,
  input  logic                              cycle_ack_i,

  output core_state_t                       state_o,
  output logic [ADDR_WIDTH-1:0]             pc_o,
  output instr_word_t                       instr_word_o,
  output logic                              illegal_opcode_o,
  output local_word_t hstctll_diag_o
);

  logic                              cpu_field_req;
  logic                              cpu_field_we;
  logic [ADDR_WIDTH-1:0]             cpu_field_addr;
  logic [FIELD_SIZE_WIDTH-1:0]       cpu_field_size;
  logic [DATA_WIDTH-1:0]             cpu_field_wdata;
  logic [DATA_WIDTH-1:0]             cpu_field_rdata;
  logic                              cpu_field_ack;
  logic                              cpu_field_iaq;
  logic                              cpu_field_srt;
  logic                              cpu_field_is_io;
  logic                              cpu_field_io_we;
  local_word_t                       cpu_field_io_rdata;
  local_word_t                       cpu_field_io_rdata_held;

  logic                              host_mem_req;
  logic                              host_mem_we;
  logic [ADDR_WIDTH-1:0]             host_mem_addr;
  local_word_t                       host_mem_wdata;
  logic                              host_mem_is_io;
  local_word_t                       host_mem_io_rdata;
  local_word_t                       host_mem_rdata;
  logic                              host_mem_ack;

  logic                              refresh_req;
  logic [7:0]                        refresh_row;
  logic                              refresh_cbr;

  logic                              screen_req;
  logic                              screen_ack;
  logic [13:0]                       screen_srfaddr;
  logic [15:0]                       screen_dpytap;
  logic                              screen_org;

  tms34010_core #(.PIXEL_OPS(PIXEL_OPS), .VIDEO(VIDEO), .FAST_BLIT(FAST_BLIT)) u_core (
    .clk                     (clk),
    .vclk_i                  (vclk_i),
    .rst                     (rst),
    .vclk_rst_i              (vclk_rst_i),
    .video_hsync_n_i         (video_hsync_n_i),
    .video_vsync_n_i         (video_vsync_n_i),
    .mem_req                 (cpu_field_req),
    .mem_we                  (cpu_field_we),
    .mem_addr                (cpu_field_addr),
    .mem_size                (cpu_field_size),
    .mem_wdata               (cpu_field_wdata),
    .mem_iaq                 (cpu_field_iaq),
    .mem_srt                 (cpu_field_srt),
    .mem_is_io               (cpu_field_is_io),
    .mem_io_we               (cpu_field_io_we),
    .mem_io_rdata            (cpu_field_io_rdata),
    .mem_io_rdata_held       (cpu_field_io_rdata_held),
    .mem_rdata               (cpu_field_rdata),
    .mem_ack                 (cpu_field_ack),
    .run_emu_n_i             (run_emu_n_i),
    .emua_n_o                (emua_n_o),
    .hcs_n_i                 (hcs_n_i),
    .host_req_i              (host_req_i),
    .host_we_i               (host_we_i),
    .host_reg_i              (host_reg_i),
    .host_be_i               (host_be_i),
    .host_wdata_i            (host_wdata_i),
    .host_rdata_o            (host_rdata_o),
    .host_ack_o              (host_ack_o),
    .host_busy_o             (host_busy_o),
    .hint_n_o                (hint_n_o),
    .host_mem_req_o          (host_mem_req),
    .host_mem_we_o           (host_mem_we),
    .host_mem_addr_o         (host_mem_addr),
    .host_mem_wdata_o        (host_mem_wdata),
    .host_mem_is_io_o        (host_mem_is_io),
    .host_mem_io_rdata_o     (host_mem_io_rdata),
    .host_mem_rdata_i        (host_mem_rdata),
    .host_mem_ack_i          (host_mem_ack),
    .lint1_n_i               (lint1_n_i),
    .lint2_n_i               (lint2_n_i),
    .dpyint_set_i            (dpyint_set_i),
    .refresh_req_o           (refresh_req),
    .refresh_row_o           (refresh_row),
    .refresh_cbr_o           (refresh_cbr),
    .video_hsync_o           (video_hsync_o),
    .video_vsync_o           (video_vsync_o),
    .video_hblank_o          (video_hblank_o),
    .video_vblank_o          (video_vblank_o),
    .video_blank_o           (video_blank_o),
    .video_hsync_oe_o        (video_hsync_oe_o),
    .video_vsync_oe_o        (video_vsync_oe_o),
    .screen_refresh_req_o    (screen_req),
    .screen_refresh_ack_i    (screen_ack),
    .screen_refresh_srfaddr_o(screen_srfaddr),
    .screen_refresh_dpytap_o (screen_dpytap),
    .screen_refresh_org_o    (screen_org),
    .state_o                 (state_o),
    .pc_o                    (pc_o),
    .instr_word_o            (instr_word_o),
    .illegal_opcode_o        (illegal_opcode_o),
    .hstctll_diag_o(hstctll_diag_o)
  );

  // The live I/O read view remains a core output for direct users; the
  // fabric consumes the registered copy (Task 0177).
  logic unused_cpu_field_io_rdata;
  assign unused_cpu_field_io_rdata = ^cpu_field_io_rdata;

  tms34010_memory_fabric u_memory_fabric (
    .clk                (clk),
    .rst                (rst),
    .cpu_field_req_i    (cpu_field_req),
    .cpu_field_we_i     (cpu_field_we),
    .cpu_field_addr_i   (cpu_field_addr),
    .cpu_field_size_i   (cpu_field_size),
    .cpu_field_wdata_i  (cpu_field_wdata),
    .cpu_field_iaq_i    (cpu_field_iaq),
    .cpu_field_srt_i    (cpu_field_srt),
    .cpu_field_is_io_i  (cpu_field_is_io),
    .cpu_field_io_we_i  (cpu_field_io_we),
    .cpu_field_io_rdata_held_i(cpu_field_io_rdata_held),
    .cpu_field_rdata_o  (cpu_field_rdata),
    .cpu_field_ack_o    (cpu_field_ack),
    .host_req_i         (host_mem_req),
    .host_we_i          (host_mem_we),
    .host_addr_i        (host_mem_addr),
    .host_wdata_i       (host_mem_wdata),
    .host_is_io_i       (host_mem_is_io),
    .host_io_rdata_i    (host_mem_io_rdata),
    .host_rdata_o       (host_mem_rdata),
    .host_ack_o         (host_mem_ack),
    .screen_req_i       (screen_req),
    .screen_srfaddr_i   (screen_srfaddr),
    .screen_dpytap_i    (screen_dpytap),
    .screen_org_i       (screen_org),
    .screen_ack_o       (screen_ack),
    .dram_req_i         (DRAM_REFRESH ? refresh_req : 1'b0),
    .dram_row_i         (refresh_row),
    .dram_cbr_i         (refresh_cbr),
    .hold_req_i         (hold_req_i),
    .hold_ack_o         (hold_ack_o),
    .cycle_req_o        (cycle_req_o),
    .cycle_kind_o       (cycle_kind_o),
    .cycle_addr_o       (cycle_addr_o),
    .cycle_wdata_o      (cycle_wdata_o),
    .cycle_io_rdata_o   (cycle_io_rdata_o),
    .cycle_iaq_o        (cycle_iaq_o),
    .cycle_srfaddr_o    (cycle_srfaddr_o),
    .cycle_dpytap_o     (cycle_dpytap_o),
    .cycle_screen_org_o (cycle_screen_org_o),
    .cycle_dram_row_o   (cycle_dram_row_o),
    .cycle_rdata_i      (cycle_rdata_i),
    .cycle_ack_i        (cycle_ack_i)
  );

endmodule : tms34010_system

`default_nettype wire

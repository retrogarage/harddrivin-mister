// -----------------------------------------------------------------------------
// tms34010_bus_arbiter.sv
//
// Fixed-priority local-memory arbitration for the TMS34010 memory clients.
//
// User's Guide §11.3, page 11-4 specifies this descending priority:
//   1. external HOLD
//   2. screen refresh
//   3. DRAM refresh
//   4. host indirect access
//   5. CPU access
//
// A registered owner holds every selected cycle through acknowledgement, so a
// newly arriving higher-priority request cannot truncate an active cycle. CPU
// partial-word read/modify/write pairs are reserved across their read-to-write
// boundary. HOLD is the only exception: when accepted in that boundary, the
// arbiter pulses cpu_restart_o so the field sequencer repeats the entire pair.
// CPU I/O takes precedence over its captured SRT tag; otherwise SRT maps a
// pixel read/write to the explicit VRAM MTR/RTM cycle kind.
//
// Screen, host, and CPU clients hold their requests and address/write payloads
// stable until acknowledgement. A host I/O read's live internal word is
// sampled when it wins arbitration. DRAM refresh is a one-clock event, so this
// block captures one pending row/mode until the physical controller completes
// it. The controller-facing cycle is likewise held with stable payload until
// cycle_ack_i.
//
// Resource plan:
//   - one six-state registered owner;
//   - one CPU RMW reservation bit;
//   - one pending DRAM-refresh bit plus its 8-bit row and CBR mode;
//   - no RAM, DSP, derived clock, tristate, or combinational acknowledge loop.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_bus_arbiter
  import tms34010_pkg::*;
(
  input  logic                          clk,
  input  logic                          rst,

  input  logic                          hold_req_i,
  output logic                          hold_ack_o,

  input  logic                          screen_req_i,
  input  logic [13:0]                   screen_srfaddr_i,
  input  logic [15:0]                   screen_dpytap_i,
  input  logic                          screen_org_i,
  output logic                          screen_ack_o,

  input  logic                          dram_req_i,
  input  logic [7:0]                    dram_row_i,
  input  logic                          dram_cbr_i,
  output logic                          dram_ack_o,

  input  logic                          host_req_i,
  input  logic                          host_we_i,
  input  logic [ADDR_WIDTH-1:0]         host_addr_i,
  input  local_word_t                   host_wdata_i,
  input  logic                          host_io_i,
  input  local_word_t                   host_io_rdata_i,
  output local_word_t                   host_rdata_o,
  output logic                          host_ack_o,

  input  logic                          cpu_req_i,
  input  logic                          cpu_we_i,
  input  logic [ADDR_WIDTH-1:0]         cpu_addr_i,
  input  local_word_t                   cpu_wdata_i,
  input  logic                          cpu_io_i,
  input  local_word_t                   cpu_io_rdata_i,
  input  logic                          cpu_iaq_i,
  input  logic                          cpu_srt_i,
  input  logic                          cpu_rmw_lock_i,
  output local_word_t                   cpu_rdata_o,
  output logic                          cpu_ack_o,
  output logic                          cpu_restart_o,

  output logic                          cycle_req_o,
  output local_cycle_kind_t             cycle_kind_o,
  output logic [ADDR_WIDTH-1:0]         cycle_addr_o,
  output local_word_t                   cycle_wdata_o,
  output local_word_t                   cycle_io_rdata_o,
  output logic                          cycle_iaq_o,
  output logic [13:0]                   cycle_srfaddr_o,
  output logic [15:0]                   cycle_dpytap_o,
  output logic                          cycle_screen_org_o,
  output logic [7:0]                    cycle_dram_row_o,
  input  local_word_t                   cycle_rdata_i,
  input  logic                          cycle_ack_i
);

  typedef enum logic [2:0] {
    ARB_IDLE   = 3'd0,
    ARB_HOLD   = 3'd1,
    ARB_SCREEN = 3'd2,
    ARB_DRAM   = 3'd3,
    ARB_HOST   = 3'd4,
    ARB_CPU    = 3'd5
  } arb_state_t;

  arb_state_t state_q, state_d;

  // A CPU reservation is created only when its partial-word read is actually
  // selected. Thus a newly offered CPU operation never bypasses an already
  // pending higher-priority client merely because its lock signal is high.
  logic cpu_rmw_active_q;

  // DRAM refresh arrives as a pulse rather than a held request.
  logic       dram_pending_q;
  logic [7:0] dram_row_q;
  logic       dram_cbr_q;

  logic select_cpu;
  logic select_host;
  logic retire_dram;
  logic capture_dram;
  local_word_t host_io_rdata_q;

  assign retire_dram =
      (state_q == ARB_DRAM) && cycle_ack_i;
  assign capture_dram =
      dram_req_i && (!dram_pending_q || retire_dram);

  // Registered-owner FSM. Selection occurs only while idle; every physical
  // cycle that has begun therefore completes regardless of later arrivals.
  always_comb begin
    state_d    = state_q;
    select_cpu = 1'b0;
    select_host = 1'b0;

    unique case (state_q)
      ARB_IDLE: begin
        if (hold_req_i) begin
          state_d = ARB_HOLD;
        end else if (cpu_rmw_active_q) begin
          state_d    = ARB_CPU;
          select_cpu = 1'b1;
        end else if (screen_req_i) begin
          state_d = ARB_SCREEN;
        end else if (dram_pending_q || dram_req_i) begin
          state_d = ARB_DRAM;
        end else if (host_req_i) begin
          state_d = ARB_HOST;
          select_host = 1'b1;
        end else if (cpu_req_i) begin
          state_d    = ARB_CPU;
          select_cpu = 1'b1;
        end
      end

      ARB_HOLD: begin
        if (!hold_req_i) state_d = ARB_IDLE;
      end

      ARB_SCREEN,
      ARB_DRAM,
      ARB_HOST,
      ARB_CPU: begin
        if (cycle_ack_i) state_d = ARB_IDLE;
      end

      default: state_d = ARB_IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) state_q <= ARB_IDLE;
    else     state_q <= state_d;
  end

  // Host-indirect I/O read data can originate in a live on-chip register.
  // Sample it when the host wins arbitration so the complete physical command
  // remains stable through controller/CDC stalls.
  always_ff @(posedge clk) begin
    if (rst) begin
      host_io_rdata_q <= '0;
    end else if ((state_q == ARB_IDLE) && select_host && host_io_i) begin
      host_io_rdata_q <= host_io_rdata_i;
    end
  end

  // HOLD accepted between a partial read and its write cancels the
  // reservation. The field sequencer observes the combinational restart pulse
  // on this edge and returns to its read state before HOLD is acknowledged.
  assign cpu_restart_o =
      (state_q == ARB_IDLE) && hold_req_i && cpu_rmw_active_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      cpu_rmw_active_q <= 1'b0;
    end else if (cpu_restart_o) begin
      cpu_rmw_active_q <= 1'b0;
    end else if ((state_q == ARB_CPU) && cycle_ack_i
                 && cpu_rmw_active_q && cpu_we_i) begin
      cpu_rmw_active_q <= 1'b0;
    end else if ((state_q == ARB_IDLE) && select_cpu
                 && cpu_req_i && cpu_rmw_lock_i) begin
      cpu_rmw_active_q <= 1'b1;
    end
  end

  // One-entry refresh event capture. If a new event coincides with retiring
  // the old one, the new row/mode wins and remains pending for another cycle.
  always_ff @(posedge clk) begin
    if (rst) begin
      dram_pending_q <= 1'b0;
      dram_row_q     <= '0;
      dram_cbr_q     <= 1'b0;
    end else begin
      if (retire_dram) dram_pending_q <= 1'b0;
      if (capture_dram) begin
        dram_pending_q <= 1'b1;
        dram_row_q     <= dram_row_i;
        dram_cbr_q     <= dram_cbr_i;
      end
    end
  end

  // Client responses. Read data is meaningful only with the corresponding
  // acknowledgement, but direct wiring avoids storage with no architectural
  // lifetime.
  assign host_rdata_o = cycle_rdata_i;
  assign cpu_rdata_o  = cycle_rdata_i;

  always_comb begin
    hold_ack_o    = 1'b0;
    screen_ack_o  = 1'b0;
    dram_ack_o    = 1'b0;
    host_ack_o    = 1'b0;
    cpu_ack_o     = 1'b0;

    cycle_req_o      = 1'b0;
    cycle_kind_o     = LOCAL_CYCLE_WORD_READ;
    cycle_addr_o     = '0;
    cycle_wdata_o    = '0;
    cycle_io_rdata_o = '0;
    cycle_iaq_o      = 1'b0;
    cycle_srfaddr_o  = '0;
    cycle_dpytap_o   = '0;
    cycle_screen_org_o = 1'b0;
    cycle_dram_row_o = '0;

    unique case (state_q)
      ARB_HOLD: begin
        hold_ack_o = 1'b1;
      end

      ARB_SCREEN: begin
        cycle_req_o     = 1'b1;
        cycle_kind_o    = LOCAL_CYCLE_SCREEN_REFRESH;
        cycle_srfaddr_o = screen_srfaddr_i;
        cycle_dpytap_o  = screen_dpytap_i;
        cycle_screen_org_o = screen_org_i;
        screen_ack_o    = cycle_ack_i;
      end

      ARB_DRAM: begin
        cycle_req_o      = 1'b1;
        cycle_kind_o     = dram_cbr_q
                         ? LOCAL_CYCLE_DRAM_CBR
                         : LOCAL_CYCLE_DRAM_RAS;
        cycle_dram_row_o = dram_row_q;
        dram_ack_o       = cycle_ack_i;
      end

      ARB_HOST: begin
        cycle_req_o   = 1'b1;
        if (host_io_i) begin
          cycle_kind_o = host_we_i
                       ? LOCAL_CYCLE_IO_WRITE
                       : LOCAL_CYCLE_IO_READ;
        end else begin
          cycle_kind_o = host_we_i
                       ? LOCAL_CYCLE_WORD_WRITE
                       : LOCAL_CYCLE_WORD_READ;
        end
        cycle_addr_o  = host_addr_i;
        cycle_wdata_o = host_wdata_i;
        cycle_io_rdata_o = host_io_i ? host_io_rdata_q : '0;
        host_ack_o    = cycle_ack_i;
      end

      ARB_CPU: begin
        cycle_req_o   = 1'b1;
        if (cpu_io_i) begin
          cycle_kind_o = cpu_we_i
                       ? LOCAL_CYCLE_IO_WRITE
                       : LOCAL_CYCLE_IO_READ;
        end else if (cpu_srt_i) begin
          cycle_kind_o = cpu_we_i
                       ? LOCAL_CYCLE_PIXEL_RTM
                       : LOCAL_CYCLE_PIXEL_MTR;
        end else begin
          cycle_kind_o = cpu_we_i
                       ? LOCAL_CYCLE_WORD_WRITE
                       : LOCAL_CYCLE_WORD_READ;
        end
        cycle_addr_o  = cpu_addr_i;
        cycle_wdata_o = cpu_wdata_i;
        cycle_io_rdata_o = cpu_io_rdata_i;
        cycle_iaq_o   = (cpu_io_i || cpu_srt_i) ? 1'b0 : cpu_iaq_i;
        cpu_ack_o     = cycle_ack_i;
      end

      ARB_IDLE: ;
      default: ;
    endcase
  end

endmodule : tms34010_bus_arbiter

`default_nettype wire

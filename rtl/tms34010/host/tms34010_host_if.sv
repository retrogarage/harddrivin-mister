// -----------------------------------------------------------------------------
// tms34010_host_if.sv
//
// Synchronous host-register and indirect-local-memory engine.
//
// The 1988 TMS34010 User's Guide §10.2 and §10.3.3 define four host-visible
// 16-bit registers. HSTADRL/HSTADRH form one word-aligned bit address;
// HSTDATA is a prefetch/write buffer; HSTCTL is owned by the surrounding I/O
// register block. Host address completion and HSTDATA accesses initiate held
// 16-bit local-word cycles according to LBL, INCR, and INCW. Processor-side
// accesses of HSTADR/HSTDATA alter only the registers and never initiate a
// local cycle (§10.3.3.4).
//
// This module is entirely in the core clock domain. Its host request/ack
// boundary represents transactions that have crossed the Task 0153
// asynchronous pin wrapper; this module itself does not claim
// HCS/HREAD/HWRITE/HRDY timing.
// The local-word request and payload remain stable until acknowledgement.
//
// Resource plan:
//   - 32-bit HSTADR and 16-bit HSTDATA architectural registers;
//   - one two-state request FSM and one captured 16-bit local-word payload;
//   - no RAM, DSP, derived clock, or combinational ready/ack loop.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_host_if
  import tms34010_pkg::*;
(
  input  logic                      clk,
  input  logic                      rst,

  // Completed-cycle synchronous host-register request. The requester holds
  // req and payload stable until ack, then deasserts req before the next
  // transaction. be_i[0] selects bits 7:0; be_i[1] selects bits 15:8.
  input  logic                      host_req_i,
  input  logic                      host_we_i,
  input  host_reg_sel_t             host_reg_i,
  input  logic [1:0]                host_be_i,
  input  local_word_t               host_wdata_i,
  output local_word_t               host_rdata_o,
  output logic                      host_ack_o,
  output logic                      host_busy_o,

  // HSTCTL is stored by tms34010_io_regs. Writes pass through on the host
  // acceptance edge; the shared high-byte fields also control this engine.
  output logic                      ctl_we_o,
  output logic [1:0]                ctl_be_o,
  output local_word_t               ctl_wdata_o,
  input  local_word_t               ctl_rdata_i,
  input  local_word_t               hstctlh_i,

  // Processor-side HSTADR/HSTDATA access. A processor write is a completed
  // on-chip I/O write pulse and has no indirect-memory side effect.
  input  logic                      cpu_we_i,
  input  host_reg_sel_t             cpu_reg_i,   // live select: processor READ mux only
  // Task 0179: the processor WRITE select comes from the I/O block's
  // registered request address, so the register-file -> ALU address path
  // does not run into hstadr_q/hstdata_q through the live address decode.
  input  host_reg_sel_t             cpu_wreg_i,
  input  local_word_t               cpu_wdata_i,
  output local_word_t               cpu_rdata_o,
  // Parallel read view avoids encoding/decoding the CPU I/O index twice.
  output logic [47:0]               cpu_words_o,

  // The host-indirect client's completed on-chip I/O access to HSTADR or
  // HSTDATA. It reaches the same architectural storage as the processor port;
  // the read view is independent so a waiting CPU request remains observable.
  input  logic                      indirect_io_we_i,
  input  host_reg_sel_t             indirect_io_reg_i,
  input  local_word_t               indirect_io_wdata_i,
  output local_word_t               indirect_io_rdata_o,

  // Held aligned 16-bit local-memory client.
  output logic                      local_req_o,
  output logic                      local_we_o,
  output logic [ADDR_WIDTH-1:0]     local_addr_o,
  output local_word_t               local_wdata_o,
  input  local_word_t               local_rdata_i,
  input  logic                      local_ack_i
);

  typedef enum logic [0:0] {
    HOST_MEM_IDLE    = 1'b0,
    HOST_MEM_REQUEST = 1'b1
  } host_mem_state_t;

  host_mem_state_t state_q, state_d;

  // Architectural storage and the accepted host read response.
  logic [ADDR_WIDTH-1:0] hstadr_q, hstadr_d;
  local_word_t           hstdata_q, hstdata_d;
  local_word_t           host_rdata_q;

  // One outstanding host transaction is recognized until req returns low.
  logic host_seen_q;
  logic host_accept;

  // Captured local transaction payload. These registers justify state across
  // an arbitrarily stalled local request.
  logic [ADDR_WIDTH-1:0] local_addr_q;
  logic                  local_we_q;
  local_word_t           local_wdata_q;
  logic                  incw_after_q;

  logic [ADDR_WIDTH-1:0] host_addr_merged;
  local_word_t           host_data_merged;
  logic                  last_data_byte;
  logic                  last_address_byte;
  logic                  start_addr_read;
  logic                  start_data_read;
  logic                  start_data_write;
  logic                  start_local_cycle;
  logic [ADDR_WIDTH-1:0] incr_read_addr;

  assign host_accept = host_req_i
                    && !host_seen_q
                    && (state_q == HOST_MEM_IDLE);

  assign last_data_byte =
      hstctlh_i[HSTCTL_LBL_BIT] ? host_be_i[0] : host_be_i[1];
  assign last_address_byte =
      hstctlh_i[HSTCTL_LBL_BIT]
        ? ((host_reg_i == HOST_REG_HSTADRL) && host_be_i[0])
        : ((host_reg_i == HOST_REG_HSTADRH) && host_be_i[1]);

  assign start_addr_read = host_accept && host_we_i
                        && last_address_byte;
  assign start_data_read = host_accept && !host_we_i
                        && (host_reg_i == HOST_REG_HSTDATA)
                        && last_data_byte;
  assign start_data_write = host_accept && host_we_i
                         && (host_reg_i == HOST_REG_HSTDATA)
                         && last_data_byte;
  assign start_local_cycle =
      start_addr_read || start_data_read || start_data_write;

  assign incr_read_addr =
      hstctlh_i[HSTCTL_INCR_BIT]
        ? (hstadr_q + ADDR_WIDTH'(LOCAL_WORD_WIDTH))
        : hstadr_q;

  // Host byte merging is combinational so a last-byte access launches the
  // local cycle with the just-completed address/data value.
  always_comb begin
    host_addr_merged = hstadr_q;
    if (host_reg_i == HOST_REG_HSTADRL) begin
      if (host_be_i[0]) host_addr_merged[7:0]  = host_wdata_i[7:0];
      if (host_be_i[1]) host_addr_merged[15:8] = host_wdata_i[15:8];
    end else if (host_reg_i == HOST_REG_HSTADRH) begin
      if (host_be_i[0]) host_addr_merged[23:16] = host_wdata_i[7:0];
      if (host_be_i[1]) host_addr_merged[31:24] = host_wdata_i[15:8];
    end
    host_addr_merged[LOCAL_WORD_ADDR_LSB-1:0] = '0;

    host_data_merged = hstdata_q;
    if (host_be_i[0]) host_data_merged[7:0]  = host_wdata_i[7:0];
    if (host_be_i[1]) host_data_merged[15:8] = host_wdata_i[15:8];
  end

  // HSTADR update priority for otherwise-unspecified simultaneous accesses:
  // local INCW completion, processor write, completed host-indirect I/O write,
  // then accepted direct-host access. Explicit register accesses therefore
  // win an automatic completion update. The device documents simultaneous
  // processor/host access as invalid; this order keeps simulation deterministic.
  always_comb begin
    hstadr_d = hstadr_q;

    if ((state_q == HOST_MEM_REQUEST) && local_ack_i && incw_after_q)
      hstadr_d = local_addr_q + ADDR_WIDTH'(LOCAL_WORD_WIDTH);

    if (cpu_we_i && (cpu_wreg_i == HOST_REG_HSTADRL)) begin
      hstadr_d[15:0] = cpu_wdata_i;
      hstadr_d[LOCAL_WORD_ADDR_LSB-1:0] = '0;
    end else if (cpu_we_i && (cpu_wreg_i == HOST_REG_HSTADRH)) begin
      hstadr_d[31:16] = cpu_wdata_i;
    end

    if (indirect_io_we_i
        && (indirect_io_reg_i == HOST_REG_HSTADRL)) begin
      hstadr_d[15:0] = indirect_io_wdata_i;
      hstadr_d[LOCAL_WORD_ADDR_LSB-1:0] = '0;
    end else if (indirect_io_we_i
                 && (indirect_io_reg_i == HOST_REG_HSTADRH)) begin
      hstadr_d[31:16] = indirect_io_wdata_i;
    end

    if (host_accept && host_we_i
        && ((host_reg_i == HOST_REG_HSTADRL)
            || (host_reg_i == HOST_REG_HSTADRH))) begin
      hstadr_d = host_addr_merged;
    end else if (start_data_read && hstctlh_i[HSTCTL_INCR_BIT]) begin
      hstadr_d = incr_read_addr;
    end
  end

  // HSTDATA update priority mirrors HSTADR: explicit I/O accesses override a
  // returning prefetch, and an accepted direct-host write has final priority.
  always_comb begin
    hstdata_d = hstdata_q;

    if ((state_q == HOST_MEM_REQUEST) && local_ack_i && !local_we_q)
      hstdata_d = local_rdata_i;

    if (cpu_we_i && (cpu_wreg_i == HOST_REG_HSTDATA))
      hstdata_d = cpu_wdata_i;

    if (indirect_io_we_i
        && (indirect_io_reg_i == HOST_REG_HSTDATA))
      hstdata_d = indirect_io_wdata_i;

    if (host_accept && host_we_i
        && (host_reg_i == HOST_REG_HSTDATA))
      hstdata_d = host_data_merged;
  end

  // Local-memory request FSM. A new side effect is captured only while idle;
  // the request cannot be dropped or have its payload changed before ack.
  always_comb begin
    state_d = state_q;
    unique case (state_q)
      HOST_MEM_IDLE: begin
        if (start_local_cycle) state_d = HOST_MEM_REQUEST;
      end

      HOST_MEM_REQUEST: begin
        if (local_ack_i) state_d = HOST_MEM_IDLE;
      end

      default: state_d = HOST_MEM_IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) state_q <= HOST_MEM_IDLE;
    else     state_q <= state_d;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      hstadr_q  <= '0;
      hstdata_q <= '0;
    end else begin
      hstadr_q  <= hstadr_d;
      hstdata_q <= hstdata_d;
    end
  end

  // Host request acceptance and registered response.
  always_ff @(posedge clk) begin
    if (rst) begin
      host_seen_q  <= 1'b0;
      host_ack_o   <= 1'b0;
      host_rdata_q <= '0;
    end else begin
      host_ack_o <= host_accept;
      if (!host_req_i) host_seen_q <= 1'b0;
      else if (host_accept) host_seen_q <= 1'b1;

      if (host_accept) begin
        unique case (host_reg_i)
          HOST_REG_HSTADRL: host_rdata_q <= hstadr_q[15:0];
          HOST_REG_HSTADRH: host_rdata_q <= hstadr_q[31:16];
          HOST_REG_HSTDATA: host_rdata_q <= hstdata_q;
          HOST_REG_HSTCTL:  host_rdata_q <= ctl_rdata_i;
          default:          host_rdata_q <= '0;
        endcase
      end
    end
  end

  // Capture the complete local payload on the host acceptance edge.
  always_ff @(posedge clk) begin
    if (rst) begin
      local_addr_q  <= '0;
      local_we_q    <= 1'b0;
      local_wdata_q <= '0;
      incw_after_q  <= 1'b0;
    end else if (start_addr_read) begin
      local_addr_q  <= host_addr_merged;
      local_we_q    <= 1'b0;
      local_wdata_q <= '0;
      incw_after_q  <= 1'b0;
    end else if (start_data_read) begin
      local_addr_q  <= incr_read_addr;
      local_we_q    <= 1'b0;
      local_wdata_q <= '0;
      incw_after_q  <= 1'b0;
    end else if (start_data_write) begin
      local_addr_q  <= hstadr_q;
      local_we_q    <= 1'b1;
      local_wdata_q <= host_data_merged;
      incw_after_q  <= hstctlh_i[HSTCTL_INCW_BIT];
    end
  end

  // Processor-visible register reads do not cause indirect local cycles.
  assign cpu_words_o = {hstdata_q, hstadr_q};

  always_comb begin
    cpu_rdata_o = '0;
    unique case (cpu_reg_i)
      HOST_REG_HSTADRL: cpu_rdata_o = hstadr_q[15:0];
      HOST_REG_HSTADRH: cpu_rdata_o = hstadr_q[31:16];
      HOST_REG_HSTDATA: cpu_rdata_o = hstdata_q;
      default: ;
    endcase
  end

  // A host-indirect I/O cycle can address these same three registers while
  // the processor-facing selector independently follows the core address.
  always_comb begin
    indirect_io_rdata_o = '0;
    unique case (indirect_io_reg_i)
      HOST_REG_HSTADRL: indirect_io_rdata_o = hstadr_q[15:0];
      HOST_REG_HSTADRH: indirect_io_rdata_o = hstadr_q[31:16];
      HOST_REG_HSTDATA: indirect_io_rdata_o = hstdata_q;
      default: ;
    endcase
  end

  // HSTCTL pass-through is an acceptance-edge pulse. The surrounding I/O
  // block retains the complementary field-ownership rules from Task 0142.
  assign ctl_we_o    = host_accept && host_we_i
                    && (host_reg_i == HOST_REG_HSTCTL);
  assign ctl_be_o    = host_be_i;
  assign ctl_wdata_o = host_wdata_i;

  assign host_rdata_o = host_rdata_q;
  assign host_busy_o  = (state_q != HOST_MEM_IDLE);

  assign local_req_o   = (state_q == HOST_MEM_REQUEST);
  assign local_we_o    = local_we_q;
  assign local_addr_o  = local_addr_q;
  assign local_wdata_o = local_wdata_q;

endmodule : tms34010_host_if

`default_nettype wire

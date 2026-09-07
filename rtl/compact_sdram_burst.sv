// Four-word burst SDRAM controller for Compact VRAM.
//
// Derived from the GPL-3.0 MiST/MiSTer controllers already reused by this
// project (Till Harbaum/Sorgelig). It retains their 100 MHz timing model,
// refresh interval, CAS latency, auto-precharge policy, and altddio clock
// output, while exposing an explicit four-word read transaction interface.

`default_nettype none

module compact_sdram_burst (
  input  logic        init_i,
  input  logic        clk_i,

  inout  wire  [15:0] SDRAM_DQ,
  output logic [12:0] SDRAM_A,
  output logic        SDRAM_DQML,
  output logic        SDRAM_DQMH,
  output logic [1:0]  SDRAM_BA,
  output logic        SDRAM_nCS,
  output logic        SDRAM_nWE,
  output logic        SDRAM_nRAS,
  output logic        SDRAM_nCAS,
  output logic        SDRAM_CKE,
  output logic        SDRAM_CLK,

  input  logic        req_i,
  input  logic        we_i,
  input  logic [23:0] addr_i,
  input  logic [63:0] wdata_i,     // four words: a write is one BL=4 burst at the aligned column
  input  logic [7:0]  be_i,        // byte enables per word (word k: be_i[2k+1:2k]); masked beats use DQM
  output logic        accept_o,
  output logic        initialized_o,
  output logic        done_o,
  output logic [63:0] rdata_o
);
  localparam logic [2:0] CMD_NOP          = 3'b111;
  localparam logic [2:0] CMD_ACTIVE       = 3'b011;
  localparam logic [2:0] CMD_READ         = 3'b101;
  localparam logic [2:0] CMD_WRITE        = 3'b100;
  localparam logic [2:0] CMD_PRECHARGE    = 3'b010;
  localparam logic [2:0] CMD_AUTO_REFRESH = 3'b001;
  localparam logic [2:0] CMD_LOAD_MODE    = 3'b000;

  // BL=4, sequential, CAS=2, burst writes (four data beats, DQM per beat).
  localparam logic [12:0] MODE = {3'b000, 1'b0, 2'b00, 3'd2, 1'b0, 3'b010};
  localparam integer STARTUP_PRECHARGE = 12000;
  localparam integer STARTUP_REFRESH_1 = 12008;
  localparam integer STARTUP_REFRESH_2 = 12016;
  localparam integer STARTUP_MODE       = 12024;
  localparam integer STARTUP_DONE       = 12032;
  localparam integer REFRESH_CYCLES     = 780;

  typedef enum logic [4:0] {
    STARTUP,
    IDLE,
    ACTIVE_WAIT,
    ISSUE_COLUMN,
    READ_WAIT_1,
    READ_WAIT_2,
    READ_WAIT_3,
    READ_BEAT_0,
    READ_BEAT_1,
    READ_BEAT_2,
    READ_BEAT_3,
    WRITE_BEAT_1,
    WRITE_BEAT_2,
    WRITE_BEAT_3,
    WRITE_RECOVER,
    REFRESH_RECOVER
  } state_t;

  state_t state;
  logic [2:0] command;
  logic [13:0] startup_count;
  logic [13:0] refresh_count;
  logic [3:0] recover_count;
  logic saved_we;
  logic [23:0] saved_addr;
  logic [63:0] saved_wdata;
  logic [7:0] saved_be;
  logic [15:0] dq_sample;
  logic [15:0] dq_out;
  logic dq_oe;

  assign SDRAM_CKE  = 1'b1;
  assign SDRAM_nCS  = 1'b0;
  assign SDRAM_nRAS = command[2];
  assign SDRAM_nCAS = command[1];
  assign SDRAM_nWE  = command[0];
  assign SDRAM_DQMH = SDRAM_A[12];
  assign SDRAM_DQML = SDRAM_A[11];
  assign SDRAM_DQ = dq_oe ? dq_out : 16'hzzzz;

  always_ff @(posedge clk_i) begin
    command <= CMD_NOP;
    dq_oe <= 1'b0;
    accept_o <= 1'b0;
    done_o <= 1'b0;
    dq_sample <= SDRAM_DQ;
    refresh_count <= refresh_count + 14'd1;

    if (init_i) begin
      state <= STARTUP;
      startup_count <= 14'd0;
      refresh_count <= 14'd0;
      command <= CMD_NOP;
      SDRAM_A <= 13'd0;
      SDRAM_BA <= 2'd0;
      accept_o <= 1'b0;
      initialized_o <= 1'b0;
      done_o <= 1'b0;
    end else begin
      case (state)
        STARTUP: begin
          startup_count <= startup_count + 14'd1;
          SDRAM_A <= 13'd0;
          SDRAM_BA <= 2'd0;
          if (startup_count == STARTUP_PRECHARGE) begin
            command <= CMD_PRECHARGE;
            SDRAM_A[10] <= 1'b1;
          end else if (startup_count == STARTUP_REFRESH_1 ||
                       startup_count == STARTUP_REFRESH_2) begin
            command <= CMD_AUTO_REFRESH;
          end else if (startup_count == STARTUP_MODE) begin
            command <= CMD_LOAD_MODE;
            SDRAM_A <= MODE;
          end else if (startup_count == STARTUP_DONE) begin
            state <= IDLE;
            initialized_o <= 1'b1;
            refresh_count <= 14'd0;
          end
        end

        IDLE: begin
          if (refresh_count >= REFRESH_CYCLES) begin
            command <= CMD_AUTO_REFRESH;
            refresh_count <= 14'd0;
            recover_count <= 4'd7;
            state <= REFRESH_RECOVER;
          end else if (req_i) begin
            // Acceptance is a pulse, not an idle/ready level. The requester
            // holds req_i through refresh and may release it only after this
            // edge has captured the complete transaction payload.
            accept_o <= 1'b1;
            saved_we <= we_i;
            saved_addr <= addr_i;
            saved_wdata <= wdata_i;
            saved_be <= be_i;
            command <= CMD_ACTIVE;
            // Four banks x 8192 rows x 512 16-bit columns.
            SDRAM_BA <= addr_i[23:22];
            SDRAM_A <= addr_i[21:9];
            state <= ACTIVE_WAIT;
          end
        end

        ACTIVE_WAIT: begin
          state <= ISSUE_COLUMN;
        end

        ISSUE_COLUMN: begin
          SDRAM_BA <= saved_addr[23:22];
          // A12:A11 are byte masks, A10 requests auto precharge, A9 is
          // unused for a 512-column device, and A8:A0 select the column.
          // A write bursts the four words of the aligned line: the WRITE
          // command carries word 0, the next three edges carry words 1-3
          // with their own DQM (A12:A11) masks.
          SDRAM_A <= {saved_we ? ~saved_be[1:0] : 2'b00, 2'b10, saved_we ? {saved_addr[8:2], 2'b00} : saved_addr[8:0]};
          if (saved_we) begin
            command <= CMD_WRITE;
            dq_out <= saved_wdata[15:0];
            dq_oe <= 1'b1;
            state <= WRITE_BEAT_1;
          end else begin
            command <= CMD_READ;
            state <= READ_WAIT_1;
          end
        end

        // Read-data sampling point. The READ command is registered at edge
        // E0 and drives the pins during [E0, E1). The inverted SDRAM_CLK
        // latches it half a cycle later; with CAS latency 2 the first beat
        // becomes valid after the second SDRAM edge that follows. The proven
        // MiSTer controller this block is derived from (deps/alice_mister/
        // rtl/sdram.sv: data_ready_delay shifted from STATE_OPEN_2) samples
        // the pins at edge E3. dq_sample is a one-edge pin register, so the
        // state that consumes it for beat 0 must run during [E3, E4), i.e.
        // three wait states after ISSUE_COLUMN. Two wait states captured the
        // pins at E2, one edge before the device drives beat 0, which shifted
        // every returned word down by one beat (word k read back beat k-1).
        READ_WAIT_1: state <= READ_WAIT_2;
        READ_WAIT_2: state <= READ_WAIT_3;
        READ_WAIT_3: state <= READ_BEAT_0;
        READ_BEAT_0: begin
          rdata_o[15:0] <= dq_sample;
          state <= READ_BEAT_1;
        end
        READ_BEAT_1: begin
          rdata_o[31:16] <= dq_sample;
          state <= READ_BEAT_2;
        end
        READ_BEAT_2: begin
          rdata_o[47:32] <= dq_sample;
          state <= READ_BEAT_3;
        end
        READ_BEAT_3: begin
          rdata_o[63:48] <= dq_sample;
          done_o <= 1'b1;
          state <= IDLE;
        end

        WRITE_BEAT_1: begin
          SDRAM_A[12:11] <= ~saved_be[3:2];
          dq_out <= saved_wdata[31:16];
          dq_oe <= 1'b1;
          state <= WRITE_BEAT_2;
        end
        WRITE_BEAT_2: begin
          SDRAM_A[12:11] <= ~saved_be[5:4];
          dq_out <= saved_wdata[47:32];
          dq_oe <= 1'b1;
          state <= WRITE_BEAT_3;
        end
        WRITE_BEAT_3: begin
          SDRAM_A[12:11] <= ~saved_be[7:6];
          dq_out <= saved_wdata[63:48];
          dq_oe <= 1'b1;
          recover_count <= 4'd3;
          state <= WRITE_RECOVER;
        end

        WRITE_RECOVER: begin
          if (recover_count == 0) begin
            done_o <= 1'b1;
            state <= IDLE;
          end else begin
            recover_count <= recover_count - 4'd1;
          end
        end

        REFRESH_RECOVER: begin
          if (recover_count == 0) state <= IDLE;
          else recover_count <= recover_count - 4'd1;
        end

        default: state <= STARTUP;
      endcase
    end
  end

  altddio_out #(
    .extend_oe_disable("OFF"),
    .intended_device_family("Cyclone V"),
    .invert_output("OFF"),
    .lpm_hint("UNUSED"),
    .lpm_type("altddio_out"),
    .oe_reg("UNREGISTERED"),
    .power_up_high("OFF"),
    .width(1)
  ) sdramclk_ddr (
    .datain_h(1'b0),
    .datain_l(1'b1),
    .outclock(clk_i),
    .dataout(SDRAM_CLK),
    .aclr(1'b0),
    .aset(1'b0),
    .oe(1'b1),
    .outclocken(1'b1),
    .sclr(1'b0),
    .sset(1'b0)
  );
endmodule

`default_nettype wire

// -----------------------------------------------------------------------------
// tms34010_host_bus.sv
//
// Asynchronous original-pin host bus to synchronous four-register bridge.
//
// A physical access exists while HCS, exactly one of HREAD/HWRITE, and at
// least one of HLDS/HUDS are active low. The combined access level is
// synchronized into the core domain. HRDY is forced low immediately while
// that request crosses, so HFS, direction, byte enables, and write data meet
// a bundled-data MCP contract and remain stable until they are captured.
//
// The host must retain every active access until HRDY returns high and must
// leave the combined access inactive long enough for the two-flop
// synchronizer to re-arm before beginning another access. Final FPGA timing
// constraints must prove that requirement against the host strobe minima.
//
// Spec sources:
//   - 1988 TMS34010 User's Guide §2.2, Table 2-2, pages 2-5/2-6.
//   - §10.3, pages 10-4 through 10-10.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_host_bus
  import tms34010_pkg::*;
(
  input  logic              clk,
  input  logic              rst,

  input  logic              hcs_n_i,
  input  logic              hread_n_i,
  input  logic              hwrite_n_i,
  input  logic              hlds_n_i,
  input  logic              huds_n_i,
  input  logic [1:0]        hfs_i,
  input  local_word_t       hd_i,
  output local_word_t       hd_o,
  output logic [1:0]        hd_oe_o,
  output logic              hrdy_o,

  output logic              host_req_o,
  output logic              host_we_o,
  output host_reg_sel_t     host_reg_o,
  output logic [1:0]        host_be_o,
  output local_word_t       host_wdata_o,
  input  local_word_t       host_rdata_i,
  input  logic              host_ack_i,
  input  logic              host_busy_i
);

  logic access_async;
  logic hcs_active_async;
  logic access_sync;
  logic hcs_active_sync;
  logic legal_read_async;
  logic legal_write_async;
  logic [1:0] byte_enable_async;

  logic access_seen_q;
  logic response_ready_q;
  local_word_t response_data_q;
  logic hcs_seen_q;
  logic [1:0] ctl_wait_q;

  assign legal_read_async  = !hread_n_i && hwrite_n_i;
  assign legal_write_async = hread_n_i && !hwrite_n_i;
  assign byte_enable_async = {!huds_n_i, !hlds_n_i};
  assign access_async =
      !hcs_n_i
      && (legal_read_async || legal_write_async)
      && (|byte_enable_async);
  assign hcs_active_async = !hcs_n_i;

  // These are level crossings. The external host holds both levels until
  // HRDY permits completion; the remaining pins are the associated stable
  // bundled payload captured only after access_sync arrives.
  tms34010_sync_bit #(.RESET_VALUE(1'b0)) u_access_sync (
    .clk     (clk),
    .rst     (rst),
    .async_i (access_async),
    .sync_o  (access_sync)
  );

  tms34010_sync_bit #(.RESET_VALUE(1'b0)) u_hcs_sync (
    .clk     (clk),
    .rst     (rst),
    .async_i (hcs_active_async),
    .sync_o  (hcs_active_sync)
  );

  // Capture one coherent transaction after the synchronized active level.
  // Request and payload then remain stable until the synchronous engine
  // acknowledges them. The response remains stable until the physical strobe
  // ends and that inactive level has crossed back into the core.
  always_ff @(posedge clk) begin
    if (rst) begin
      access_seen_q  <= 1'b0;
      response_ready_q <= 1'b0;
      response_data_q  <= '0;
      host_req_o     <= 1'b0;
      host_we_o      <= 1'b0;
      host_reg_o     <= HOST_REG_HSTADRL;
      host_be_o      <= 2'b00;
      host_wdata_o   <= '0;
    end else if (!access_sync) begin
      access_seen_q    <= 1'b0;
      response_ready_q <= 1'b0;
      host_req_o       <= 1'b0;
    end else begin
      if (!access_seen_q) begin
        access_seen_q  <= 1'b1;
        host_req_o     <= 1'b1;
        host_we_o      <= legal_write_async;
        host_reg_o     <= host_reg_sel_t'(hfs_i);
        host_be_o      <= byte_enable_async;
        host_wdata_o   <= hd_i;
      end

      if (host_req_o && host_ack_i) begin
        host_req_o       <= 1'b0;
        response_ready_q <= 1'b1;
        response_data_q  <= host_rdata_i;
      end
    end
  end

  // HSTCTL always generates a visible wait interval from HCS alone. HFS must
  // be stable before HCS falls, as required by §10.3.2. Two core clocks are a
  // deterministic functional realization; A0043 defers proof of the
  // specified one-to-two-local-clock interval to the final PLL/timing work.
  always_ff @(posedge clk) begin
    if (rst) begin
      hcs_seen_q <= 1'b0;
      ctl_wait_q <= 2'd0;
    end else if (!hcs_active_sync) begin
      hcs_seen_q <= 1'b0;
      ctl_wait_q <= 2'd0;
    end else if (!hcs_seen_q) begin
      hcs_seen_q <= 1'b1;
      ctl_wait_q <= (hfs_i == HOST_REG_HSTCTL) ? 2'd2 : 2'd0;
    end else if (ctl_wait_q != 2'd0) begin
      ctl_wait_q <= ctl_wait_q - 2'd1;
    end
  end

  // Once this access has a response, HRDY must remain high through its first
  // inactive strobe even if the accepted HSTDATA/HSTADR operation made the
  // indirect engine busy. That busy state blocks the following access.
  always_comb begin
    hrdy_o = 1'b1;
    if (!hcs_n_i) begin
      if (access_async && response_ready_q) begin
        // A fast backend response cannot shorten HSTCTL's mandatory wait.
        hrdy_o = !((hfs_i == HOST_REG_HSTCTL)
                   && (!hcs_seen_q || (ctl_wait_q != 2'd0)));
      end else if (host_busy_i) begin
        hrdy_o = 1'b0;
      end else if ((hfs_i == HOST_REG_HSTCTL)
                   && (!hcs_seen_q || (ctl_wait_q != 2'd0))) begin
        hrdy_o = 1'b0;
      end else if (access_async) begin
        hrdy_o = 1'b0;
      end
    end
  end

  assign hd_o = response_data_q;
  assign hd_oe_o =
      (response_ready_q && !hcs_n_i && legal_read_async)
        ? host_be_o
        : 2'b00;

endmodule : tms34010_host_bus

`default_nettype wire

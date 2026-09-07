`default_nettype none

// Two-client, scanout-priority adapter for the proven single-word MiSTer
// SDRAM controller. Compact VRAM occupies only 512 KiB (256K x 16 here).
module compact_vram_arbiter (
  input  logic        clk_i,
  input  logic        rst_i,

  input  logic        scan_req_i,
  input  logic [17:0] scan_addr_i,
  output logic        scan_ack_o,
  output logic [15:0] scan_data_o,
  output logic        scan_overflow_o,

  input  logic        gsp_req_i,
  input  logic        gsp_we_i,
  input  logic [1:0]  gsp_be_i,
  input  logic [17:0] gsp_addr_i,
  input  logic [15:0] gsp_wdata_i,
  output logic        gsp_ack_o,
  output logic [15:0] gsp_rdata_o,

  output logic [24:0] sdram_addr_o,
  output logic [15:0] sdram_din_o,
  output logic [1:0]  sdram_be_o,
  output logic        sdram_rd_o,
  output logic        sdram_we_o,
  input  logic [15:0] sdram_dout_i,
  input  logic        sdram_ready_i,

  output logic [31:0] scan_words_o,
  output logic [31:0] gsp_words_o
);
  typedef enum logic [1:0] {IDLE, WAIT_BUSY, WAIT_DONE} state_t;
  state_t state;
  logic scan_pending;
  logic [17:0] scan_addr;
  logic gsp_pending;
  logic gsp_armed;
  logic gsp_we;
  logic [1:0] gsp_be;
  logic [17:0] gsp_addr;
  logic [15:0] gsp_wdata;
  logic active_scan;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state           <= IDLE;
      scan_pending    <= 1'b0;
      gsp_pending     <= 1'b0;
      gsp_armed       <= 1'b1;
      scan_ack_o      <= 1'b0;
      gsp_ack_o       <= 1'b0;
      scan_overflow_o <= 1'b0;
      sdram_rd_o      <= 1'b0;
      sdram_we_o      <= 1'b0;
      sdram_be_o      <= 2'b11;
      scan_words_o    <= 32'd0;
      gsp_words_o     <= 32'd0;
      active_scan     <= 1'b0;
    end else begin
      scan_ack_o <= 1'b0;
      gsp_ack_o  <= 1'b0;
      sdram_rd_o <= 1'b0;
      sdram_we_o <= 1'b0;

      if (scan_req_i) begin
        if (scan_pending) scan_overflow_o <= 1'b1;
        else begin
          scan_pending <= 1'b1;
          scan_addr    <= scan_addr_i;
        end
      end

      if (!gsp_req_i) gsp_armed <= 1'b1;
      if (gsp_req_i && gsp_armed && !gsp_pending) begin
        gsp_pending <= 1'b1;
        gsp_armed   <= 1'b0;
        gsp_we      <= gsp_we_i;
        gsp_be      <= gsp_be_i;
        gsp_addr    <= gsp_addr_i;
        gsp_wdata   <= gsp_wdata_i;
      end

      case (state)
        IDLE: begin
          if (scan_pending) begin
            active_scan  <= 1'b1;
            scan_pending <= 1'b0;
            sdram_addr_o <= {6'd0, scan_addr, 1'b0};
            sdram_din_o  <= 16'd0;
            sdram_be_o   <= 2'b11;
            sdram_rd_o   <= 1'b1;
            state        <= WAIT_BUSY;
          end else if (gsp_pending) begin
            active_scan <= 1'b0;
            gsp_pending <= 1'b0;
            sdram_addr_o <= {6'd0, gsp_addr, 1'b0};
            sdram_din_o  <= gsp_wdata;
            sdram_be_o   <= gsp_be;
            sdram_rd_o   <= !gsp_we;
            sdram_we_o   <= gsp_we;
            state        <= WAIT_BUSY;
          end
        end
        WAIT_BUSY: if (!sdram_ready_i) state <= WAIT_DONE;
        WAIT_DONE: if (sdram_ready_i) begin
          if (active_scan) begin
            scan_data_o <= sdram_dout_i;
            scan_ack_o  <= 1'b1;
            scan_words_o <= scan_words_o + 32'd1;
          end else begin
            gsp_rdata_o <= sdram_dout_i;
            gsp_ack_o   <= 1'b1;
            gsp_words_o <= gsp_words_o + 32'd1;
          end
          state <= IDLE;
        end
        default: state <= IDLE;
      endcase
    end
  end
endmodule

`default_nettype wire

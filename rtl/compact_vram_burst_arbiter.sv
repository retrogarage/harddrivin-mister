`default_nettype none

// Two-client Compact VRAM adapter with independent four-word read caches.
// Sequential scan traffic converts to one SDRAM burst per four words. GSP
// reads get the same coalescing while writes remain single-word transactions.
module compact_vram_burst_arbiter #(parameter int ADDR_W = 18) (
  input  logic        clk_i,
  input  logic        rst_i,

  input  logic        scan_req_i,
  input  logic [ADDR_W-1:0] scan_addr_i,
  output logic        scan_ack_o,
  output logic [15:0] scan_data_o,
  output logic        scan_overflow_o,

  input  logic        gsp_req_i,
  input  logic        gsp_we_i,
  input  logic [7:0]  gsp_be_i,      // byte enables of the four words of the line
  input  logic [ADDR_W-1:0] gsp_addr_i,
  input  logic [63:0] gsp_wdata_i,   // a write is the whole 4-word line at gsp_addr_i[ADDR_W-1:2]
  output logic        gsp_ack_o,
  output logic [15:0] gsp_rdata_o,
  output logic [63:0] gsp_rdata64_o,   // the whole 4-word burst holding gsp_rdata_o

  output logic        mem_req_o,
  output logic        mem_we_o,
  output logic [23:0] mem_addr_o,
  output logic [63:0] mem_wdata_o,
  output logic [7:0]  mem_be_o,
  input  logic        mem_accept_i,
  input  logic        mem_done_i,
  input  logic [63:0] mem_rdata_i,

  output logic [31:0] scan_words_o,
  output logic [31:0] gsp_words_o,
  output logic [31:0] read_bursts_o,
  output logic [31:0] write_words_o,
  output logic [7:0]  debug_o
);
  typedef enum logic [1:0] {IDLE, WAIT_ACCEPT, WAIT_MEMORY} state_t;
  state_t state;

  logic scan_pending;
  logic [ADDR_W-1:0] scan_address;
  logic gsp_pending;
  logic gsp_armed;
  logic gsp_write;
  logic [7:0]  gsp_be;
  logic [ADDR_W-1:0] gsp_address;
  logic [63:0] gsp_wdata;

  logic scan_cache_valid;
  logic [ADDR_W-3:0] scan_cache_tag;
  logic [63:0] scan_cache_data;
  logic gsp_cache_valid;
  logic [ADDR_W-3:0] gsp_cache_tag;
  logic [63:0] gsp_cache_data;

  logic active_scan;
  logic active_write;
  logic [1:0] active_select;
  logic [ADDR_W-3:0] active_tag;

  // Stable bring-up visibility for a transaction that stops between the
  // level-held GSP request and the one-cycle SDRAM completion. This has no
  // influence on arbitration and can be removed after hardware closure.
  assign debug_o = {
    state == WAIT_MEMORY, scan_pending, gsp_pending, gsp_armed,
    active_scan, active_write, gsp_req_i, mem_accept_i
  };

  function automatic logic [15:0] select_word(
    input logic [63:0] data,
    input logic [1:0] index
  );
    case (index)
      2'd0: select_word = data[15:0];
      2'd1: select_word = data[31:16];
      2'd2: select_word = data[47:32];
      default: select_word = data[63:48];
    endcase
  endfunction

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state             <= IDLE;
      scan_pending      <= 1'b0;
      gsp_pending       <= 1'b0;
      gsp_armed         <= 1'b1;
      scan_cache_valid  <= 1'b0;
      gsp_cache_valid   <= 1'b0;
      scan_ack_o        <= 1'b0;
      gsp_ack_o         <= 1'b0;
      scan_overflow_o   <= 1'b0;
      mem_req_o         <= 1'b0;
      mem_we_o          <= 1'b0;
      mem_be_o          <= 8'hff;
      scan_words_o      <= 32'd0;
      gsp_words_o       <= 32'd0;
      read_bursts_o     <= 32'd0;
      write_words_o     <= 32'd0;
      active_scan       <= 1'b0;
      active_write      <= 1'b0;
    end else begin
      scan_ack_o <= 1'b0;
      gsp_ack_o  <= 1'b0;
      mem_req_o  <= 1'b0;

      if (scan_req_i) begin
        if (scan_pending) begin
          scan_overflow_o <= 1'b1;
        end else begin
          scan_pending <= 1'b1;
          scan_address <= scan_addr_i;
        end
      end

      if (!gsp_req_i) gsp_armed <= 1'b1;
      if (gsp_req_i && gsp_armed && !gsp_pending) begin
        gsp_pending <= 1'b1;
        gsp_armed   <= 1'b0;
        gsp_write   <= gsp_we_i;
        gsp_be      <= gsp_be_i;
        gsp_address <= gsp_addr_i;
        gsp_wdata   <= gsp_wdata_i;
      end

      // A cached scan word needs no SDRAM transaction. Deliver it while
      // unrelated memory work is in flight, but never alongside completion
      // of a write which invalidates that cache line.
      if (scan_pending && scan_cache_valid &&
          scan_cache_tag == scan_address[ADDR_W-1:2] &&
          !(state == WAIT_MEMORY && active_write && mem_done_i &&
            active_tag == scan_cache_tag)) begin
        scan_data_o <= select_word(scan_cache_data, scan_address[1:0]);
        scan_ack_o <= 1'b1;
        scan_words_o <= scan_words_o + 32'd1;
        scan_pending <= 1'b0;
      end

      case (state)
        IDLE: begin
          if (scan_pending) begin
            if (!(scan_cache_valid && scan_cache_tag == scan_address[ADDR_W-1:2])) begin
              mem_req_o      <= 1'b1;
              mem_we_o       <= 1'b0;
              mem_addr_o     <= {{(24-ADDR_W){1'b0}}, scan_address[ADDR_W-1:2], 2'b00};
              mem_wdata_o    <= 64'd0;
              mem_be_o       <= 8'hff;
              active_scan    <= 1'b1;
              active_write   <= 1'b0;
              active_select  <= scan_address[1:0];
              active_tag     <= scan_address[ADDR_W-1:2];
              scan_pending   <= 1'b0;
              read_bursts_o  <= read_bursts_o + 32'd1;
              state           <= WAIT_ACCEPT;
            end
          end else if (gsp_pending) begin
            if (!gsp_write && gsp_cache_valid && gsp_cache_tag == gsp_address[ADDR_W-1:2]) begin
              gsp_rdata_o <= select_word(gsp_cache_data, gsp_address[1:0]);
              gsp_rdata64_o <= gsp_cache_data;
              gsp_ack_o   <= 1'b1;
              gsp_words_o <= gsp_words_o + 32'd1;
              gsp_pending <= 1'b0;
            end else begin
              mem_req_o      <= 1'b1;
              mem_we_o       <= gsp_write;
              mem_addr_o     <= {{(24-ADDR_W){1'b0}}, gsp_address[ADDR_W-1:2], 2'b00};
              mem_wdata_o    <= gsp_wdata;
              mem_be_o       <= gsp_write ? gsp_be : 8'hff;
              active_scan    <= 1'b0;
              active_write   <= gsp_write;
              active_select  <= gsp_address[1:0];
              active_tag     <= gsp_address[ADDR_W-1:2];
              gsp_pending    <= 1'b0;
              if (gsp_write) write_words_o <= write_words_o + 32'd1;
              else read_bursts_o <= read_bursts_o + 32'd1;
              state <= WAIT_ACCEPT;
            end
          end
        end

        // req/accept is a true valid handshake. Keep the complete command
        // asserted while the SDRAM controller services a refresh, and do not
        // wait for completion until the controller explicitly accepts it.
        WAIT_ACCEPT: begin
          mem_req_o <= 1'b1;
          if (mem_accept_i) begin
            mem_req_o <= 1'b0;
            state <= WAIT_MEMORY;
          end
        end

        WAIT_MEMORY: begin
          if (mem_done_i) begin
            if (active_write) begin
              gsp_ack_o   <= 1'b1;
              gsp_words_o <= gsp_words_o + 32'd1;
              if (scan_cache_tag == active_tag) scan_cache_valid <= 1'b0;
              if (gsp_cache_tag == active_tag) gsp_cache_valid <= 1'b0;
            end else if (active_scan) begin
              scan_cache_valid <= 1'b1;
              scan_cache_tag   <= active_tag;
              scan_cache_data  <= mem_rdata_i;
              scan_data_o      <= select_word(mem_rdata_i, active_select);
              scan_ack_o       <= 1'b1;
              scan_words_o     <= scan_words_o + 32'd1;
            end else begin
              gsp_cache_valid <= 1'b1;
              gsp_cache_tag   <= active_tag;
              gsp_cache_data  <= mem_rdata_i;
              gsp_rdata_o     <= select_word(mem_rdata_i, active_select);
              gsp_rdata64_o   <= mem_rdata_i;
              gsp_ack_o       <= 1'b1;
              gsp_words_o     <= gsp_words_o + 32'd1;
            end
            state <= IDLE;
          end
        end

        // Recover to a quiescent state if the encoded state is ever upset.
        // Pending client requests remain queued and will be serviced from
        // IDLE on the following cycle.
        default: begin
          state        <= IDLE;
          mem_req_o    <= 1'b0;
          active_scan  <= 1'b0;
          active_write <= 1'b0;
        end
      endcase
    end
  end
endmodule

`default_nettype wire

// -----------------------------------------------------------------------------
// tms34010_field_sequencer.sv
//
// Synthesizable TMS34010 architectural-field to physical-word sequencer.
//
// The CPU side presents one right-justified 1..32-bit field at an arbitrary
// 32-bit bit address. The memory side presents aligned 16-bit word cycles.
// The sequencer implements the seven minimum-cycle alignment cases specified
// by the 1988 TMS34010 User's Guide §4.1, pages 4-2 through 4-5:
//
//   - reads issue one, two, or three ascending word reads;
//   - fully covered words are written directly;
//   - each partially covered word is read, merged, then written;
//   - bits outside the requested field are preserved.
//
// `word_rmw_lock_o` is asserted from the partial-word read request through
// its matching write acknowledgement. User's Guide §11.3, page 11-4 makes
// that pair indivisible while allowing higher-priority cycles between
// different words of a multiword field. HOLD is the sole exception: if it is
// accepted after the partial read but before the write is issued, a one-cycle
// `word_restart_i` returns the sequencer to the read so the complete RMW pair
// restarts after HOLD release.
//
// Both interfaces use request/acknowledge flow control. A request and all of
// its payload signals remain stable until acknowledgement. Consecutive word
// requests may be back-to-back: after an acknowledged word, the next cycle
// may retain `word_req_o` while changing to the next acknowledged payload.
// -----------------------------------------------------------------------------

`default_nettype none

module tms34010_field_sequencer
  import tms34010_pkg::*;
(
  input  logic                              clk,
  input  logic                              rst,

  input  logic                              field_req_i,
  input  logic                              field_we_i,
  input  logic [ADDR_WIDTH-1:0]             field_addr_i,
  input  logic [FIELD_SIZE_WIDTH-1:0]       field_size_i,
  input  logic [DATA_WIDTH-1:0]             field_wdata_i,
  output logic [DATA_WIDTH-1:0]             field_rdata_o,
  output logic                              field_ack_o,

  output logic                              word_req_o,
  output logic                              word_we_o,
  output logic [ADDR_WIDTH-1:0]             word_addr_o,
  output local_word_t                       word_wdata_o,
  input  local_word_t                       word_rdata_i,
  input  logic                              word_ack_i,
  input  logic                              word_restart_i,
  output logic                              word_rmw_lock_o
);

  localparam int unsigned WORD_INDEX_WIDTH = $clog2(FIELD_MAX_WORDS);
  localparam logic [FIELD_WINDOW_WIDTH-1:0] FIELD_ONE =
      {{(FIELD_WINDOW_WIDTH-1){1'b0}}, 1'b1};

  typedef enum logic [2:0] {
    FIELD_IDLE         = 3'd0,
    FIELD_READ_WORD    = 3'd1,
    FIELD_WRITE_SELECT = 3'd2,
    FIELD_WRITE_READ   = 3'd3,
    FIELD_WRITE_WORD   = 3'd4,
    FIELD_RESPONSE     = 3'd5
  } field_state_t;

  field_state_t state_q, state_d;

  // Justification (a): these values describe the field transaction while its
  // physical word cycles execute over later clocks.
  logic [ADDR_WIDTH-1:0]       field_addr_q;
  logic [FIELD_SIZE_WIDTH-1:0] field_size_q;
  logic [DATA_WIDTH-1:0]       field_wdata_q;

  // Justification (a): selects the current physical word across a multiword
  // transaction. Two bits cover the specified maximum of three words.
  logic [WORD_INDEX_WIDTH-1:0] word_index_q;

  // Justification (a): accumulated physical words are needed after their read
  // acknowledgement to assemble the final right-justified field response.
  logic [FIELD_WINDOW_WIDTH-1:0] read_window_q;
  logic [DATA_WIDTH-1:0]         field_rdata_q;

  // Justification (a): a partial-word write must retain the acknowledged old
  // word across the indivisible read-to-write transition.
  local_word_t rmw_word_q;

  logic                              field_size_valid_i;
  logic [5:0]                        field_end_offset;
  logic [WORD_INDEX_WIDTH-1:0]       last_word_index;
  logic [FIELD_WINDOW_WIDTH-1:0]     unshifted_mask;
  logic [FIELD_WINDOW_WIDTH-1:0]     field_mask;
  logic [FIELD_WINDOW_WIDTH-1:0]     field_data_window;
  logic [FIELD_WINDOW_WIDTH-1:0]     read_window_next;
  logic [FIELD_WINDOW_WIDTH-1:0]     read_window_shifted;
  local_word_t                       current_word_mask;
  local_word_t                       current_word_data;
  local_word_t                       merged_word_data;

  function automatic local_word_t window_word(
    input logic [FIELD_WINDOW_WIDTH-1:0] window,
    input logic [WORD_INDEX_WIDTH-1:0]   index
  );
    local_word_t result;
    begin
      result = '0;
      unique case (index)
        2'd0: result = window[15:0];
        2'd1: result = window[31:16];
        2'd2: result = window[47:32];
        default: ;
      endcase
      return result;
    end
  endfunction

  function automatic logic [FIELD_WINDOW_WIDTH-1:0] replace_window_word(
    input logic [FIELD_WINDOW_WIDTH-1:0] window,
    input logic [WORD_INDEX_WIDTH-1:0]   index,
    input local_word_t                   word
  );
    logic [FIELD_WINDOW_WIDTH-1:0] result;
    begin
      result = window;
      unique case (index)
        2'd0: result[15:0]  = word;
        2'd1: result[31:16] = word;
        2'd2: result[47:32] = word;
        default: ;
      endcase
      return result;
    end
  endfunction

  assign field_size_valid_i =
      (field_size_i >= FIELD_SIZE_WIDTH'(1))
      && (field_size_i <= FIELD_SIZE_WIDTH'(DATA_WIDTH));

  // Offset of the field's final bit within the three-word window. Values
  // 0..46 select final word indices 0..2 through bits [5:4].
  assign field_end_offset =
      {2'b00, field_addr_q[LOCAL_WORD_ADDR_LSB-1:0]}
      + field_size_q - FIELD_SIZE_WIDTH'(1);
  assign last_word_index = field_end_offset[5:4];

  always_comb begin
    unshifted_mask = '0;
    if ((field_size_q >= FIELD_SIZE_WIDTH'(1))
        && (field_size_q <= FIELD_SIZE_WIDTH'(DATA_WIDTH))) begin
      unshifted_mask = (FIELD_ONE << field_size_q) - FIELD_ONE;
    end

    field_mask = unshifted_mask
               << field_addr_q[LOCAL_WORD_ADDR_LSB-1:0];
    field_data_window =
        {{(FIELD_WINDOW_WIDTH-DATA_WIDTH){1'b0}}, field_wdata_q}
        << field_addr_q[LOCAL_WORD_ADDR_LSB-1:0];
    current_word_mask = window_word(field_mask, word_index_q);
    current_word_data = window_word(field_data_window, word_index_q);
    merged_word_data = (rmw_word_q & ~current_word_mask)
                     | (current_word_data & current_word_mask);

    read_window_next = replace_window_word(
        read_window_q, word_index_q, word_rdata_i);
    read_window_shifted =
        read_window_next >> field_addr_q[LOCAL_WORD_ADDR_LSB-1:0];
  end

  // State register.
  always_ff @(posedge clk) begin
    if (rst) state_q <= FIELD_IDLE;
    else     state_q <= state_d;
  end

  // Transaction payload registers.
  always_ff @(posedge clk) begin
    if (rst) begin
      field_addr_q  <= '0;
      field_size_q  <= '0;
      field_wdata_q <= '0;
    end else if ((state_q == FIELD_IDLE) && field_req_i) begin
      field_addr_q  <= field_addr_i;
      field_size_q  <= field_size_i;
      field_wdata_q <= field_wdata_i;
    end
  end

  // Physical-word index.
  always_ff @(posedge clk) begin
    if (rst) begin
      word_index_q <= '0;
    end else if ((state_q == FIELD_IDLE) && field_req_i) begin
      word_index_q <= '0;
    end else if ((state_q == FIELD_READ_WORD) && word_ack_i
                 && (word_index_q != last_word_index)) begin
      word_index_q <= word_index_q + WORD_INDEX_WIDTH'(1);
    end else if ((state_q == FIELD_WRITE_WORD) && word_ack_i
                 && (word_index_q != last_word_index)) begin
      word_index_q <= word_index_q + WORD_INDEX_WIDTH'(1);
    end
  end

  // Read assembly window.
  always_ff @(posedge clk) begin
    if (rst) begin
      read_window_q <= '0;
    end else if ((state_q == FIELD_IDLE) && field_req_i) begin
      read_window_q <= '0;
    end else if ((state_q == FIELD_READ_WORD) && word_ack_i) begin
      read_window_q <= read_window_next;
    end
  end

  // Final right-justified read response.
  always_ff @(posedge clk) begin
    if (rst) begin
      field_rdata_q <= '0;
    end else if ((state_q == FIELD_IDLE) && field_req_i) begin
      field_rdata_q <= '0;
    end else if ((state_q == FIELD_READ_WORD) && word_ack_i
                 && (word_index_q == last_word_index)) begin
      field_rdata_q <= read_window_shifted[DATA_WIDTH-1:0]
                     & unshifted_mask[DATA_WIDTH-1:0];
    end
  end

  // Old word retained for a partial-word merge.
  always_ff @(posedge clk) begin
    if (rst) begin
      rmw_word_q <= '0;
    end else if ((state_q == FIELD_WRITE_READ) && word_ack_i) begin
      rmw_word_q <= word_rdata_i;
    end
  end

  // Word address/output decode. All physical addresses are aligned to the
  // LSB of their 16-bit word and advance in ascending address order.
  always_comb begin
    word_addr_o = {
        field_addr_q[ADDR_WIDTH-1:LOCAL_WORD_ADDR_LSB],
        {LOCAL_WORD_ADDR_LSB{1'b0}}
    };
    unique case (word_index_q)
      2'd0: ;
      2'd1: word_addr_o = word_addr_o + ADDR_WIDTH'(LOCAL_WORD_WIDTH);
      2'd2: word_addr_o = word_addr_o + ADDR_WIDTH'(2 * LOCAL_WORD_WIDTH);
      default: ;
    endcase
  end

  // Two-block FSM: physical requests and response acknowledgement are
  // combinational state outputs; all payload is registered above.
  always_comb begin
    state_d         = state_q;
    field_ack_o     = 1'b0;
    word_req_o      = 1'b0;
    word_we_o       = 1'b0;
    word_wdata_o    = merged_word_data;
    word_rmw_lock_o = 1'b0;

    unique case (state_q)
      FIELD_IDLE: begin
        if (field_req_i) begin
          if (!field_size_valid_i)
            state_d = FIELD_RESPONSE;
          else if (field_we_i)
            state_d = FIELD_WRITE_SELECT;
          else
            state_d = FIELD_READ_WORD;
        end
      end

      FIELD_READ_WORD: begin
        word_req_o = 1'b1;
        if (word_ack_i) begin
          if (word_index_q == last_word_index)
            state_d = FIELD_RESPONSE;
        end
      end

      FIELD_WRITE_SELECT: begin
        if (current_word_mask == {LOCAL_WORD_WIDTH{1'b1}})
          state_d = FIELD_WRITE_WORD;
        else
          state_d = FIELD_WRITE_READ;
      end

      FIELD_WRITE_READ: begin
        word_req_o      = 1'b1;
        word_rmw_lock_o = 1'b1;
        if (word_ack_i) state_d = FIELD_WRITE_WORD;
      end

      FIELD_WRITE_WORD: begin
        if (word_restart_i
            && (current_word_mask != {LOCAL_WORD_WIDTH{1'b1}})) begin
          // HOLD may interrupt the otherwise-indivisible pair only between
          // cycles. Suppress the not-yet-issued write and reacquire old data.
          state_d = FIELD_WRITE_READ;
        end else begin
          word_req_o   = 1'b1;
          word_we_o    = 1'b1;
          word_wdata_o = merged_word_data;
          if (current_word_mask != {LOCAL_WORD_WIDTH{1'b1}})
            word_rmw_lock_o = 1'b1;
          if (word_ack_i) begin
            if (word_index_q == last_word_index)
              state_d = FIELD_RESPONSE;
            else
              state_d = FIELD_WRITE_SELECT;
          end
        end
      end

      FIELD_RESPONSE: begin
        field_ack_o = 1'b1;
        state_d     = FIELD_IDLE;
      end

      default: state_d = FIELD_IDLE;
    endcase
  end

  assign field_rdata_o = field_rdata_q;

endmodule : tms34010_field_sequencer

`default_nettype wire

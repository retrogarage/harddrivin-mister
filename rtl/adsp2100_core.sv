`default_nettype none

// Clean-room ADSP-2100 execution core for the Atari Hard Drivin' ADSP board.
//
// Sources: Analog Devices ADSP-2100 User's Manual (1989): chapter 2
// (ALU, MAC, shifter, division primitives), chapter 3 (DAG modulo
// addressing, PX exchange), chapter 4 (sequencer, loops, counter, stacks,
// status registers), chapter 6 (instruction semantics) and Appendix A
// (instruction coding).  MAME was used only as a behavioral replay oracle.
//
// Scope: every instruction type the uploaded Hard Drivin' program executes
// (Appendix A types 1-21, 23, 24, 26; RTI as a plain return because the
// board pulls all four IRQ pins high).  Unimplemented: interrupts, TRAP,
// bit-reverse addressing, the secondary register bank, MSTAT mode control
// (the program never writes MSTAT) and the instruction cache (a timing aid
// only).  Executing an unsupported word sets illegal_o and behaves as NOP.
//
// The real part executes one instruction per four 32 MHz clocks.  This core
// spends six 48 MHz cycles per instruction, the same 8 MHz instruction
// rate, which gives the synchronous program RAM, the external data-memory
// handshake, and the registered multiplier explicit cycles.
module adsp2100_core #(
  // 0: unbiased rounding on the 40-bit result as specified in section
  //    2.3.2.6 of the manual (midpoint clears bit 16 of the result).
  // 1: MAME-compatible variant (midpoint test on the shifted product only)
  //    used by the replay testbench to stay in lock-step with the oracle.
  parameter integer MAC_ROUND_MAME = 0,
  // 0: manual 4.2.4 case 2 — a taken jump/call/return at the loop end
  //    suppresses every end-of-loop action.  1: MAME still pops the loop,
  //    PC and counter stacks when the termination condition is true (replay
  //    oracle behaviour, kept only for lock-step verification).
  parameter integer LOOP_END_MAME = 0
) (
  input  logic        clk_i,
  input  logic        rst_i,
  input  logic        halt_i,

  // 68010 program upload.  The host holds the core in reset/halt meanwhile.
  input  logic        prog_we_i,
  input  tri0         prog_re_i,
  output logic        prog_ready_o,
  output logic [23:0] prog_rdata_o,
  input  logic [12:0] prog_addr_i,
  input  logic [23:0] prog_wdata_i,

  // Data-memory bus.  Board glue owns RAM and the 0x2000-0x2007 ports.  A
  // request stays asserted until dm_ready_i; read data is sampled with it.
  output logic        dm_req_o,
  output logic        dm_we_o,
  output logic [13:0] dm_addr_o,
  output logic [15:0] dm_wdata_o,
  input  logic [15:0] dm_rdata_i,
  input  logic        dm_ready_i,

  output logic [13:0] pc_o,
  output logic [23:0] instruction_o,
  output logic        running_o,
  output logic        illegal_o
);
  localparam logic [2:0] ST_FETCH      = 3'd0;
  localparam logic [2:0] ST_FETCH_WAIT = 3'd1;
  localparam logic [2:0] ST_DECODE     = 3'd2;
  localparam logic [2:0] ST_MEMORY     = 3'd3;
  localparam logic [2:0] ST_COMMIT     = 3'd4;
  localparam logic [2:0] ST_PAD        = 3'd5;

  // DREG codes (Appendix A): AX0 AX1 MX0 MX1 AY0 AY1 MY0 MY1 SI SE AR MR0
  // MR1 MR2 SR0 SR1.
  localparam logic [3:0] R_AX0 = 4'h0, R_AX1 = 4'h1, R_MX0 = 4'h2, R_MX1 = 4'h3;
  localparam logic [3:0] R_AY0 = 4'h4, R_AY1 = 4'h5, R_MY0 = 4'h6, R_MY1 = 4'h7;
  localparam logic [3:0] R_SI  = 4'h8, R_SE  = 4'h9, R_AR  = 4'ha, R_MR0 = 4'hb;
  localparam logic [3:0] R_MR1 = 4'hc, R_MR2 = 4'hd, R_SR0 = 4'he, R_SR1 = 4'hf;

  logic [2:0]  state_q;
  logic [13:0] pc_q;
  logic [23:0] instruction_q;

  logic [15:0] dreg_q [0:15];
  logic [15:0] af_q;
  logic [15:0] mf_q;
  logic [4:0]  sb_q;
  logic [7:0]  px_q;
  logic [13:0] ireg_q [0:7];
  logic [13:0] mreg_q [0:7];
  logic [13:0] lreg_q [0:7];
  logic [7:0]  astat_q;   // AZ AN AV AC AS AQ MV SS
  logic [3:0]  mstat_q;
  logic [3:0]  imask_q;
  logic [4:0]  icntl_q;
  logic [13:0] cntr_q;
  logic        cntr_valid_q;
  logic [13:0] count_stack_q [0:3];
  logic [2:0]  count_sp_q;
  logic [13:0] pc_stack_q [0:15];
  logic [4:0]  pc_sp_q;
  logic [13:0] loop_end_q [0:3];
  logic [3:0]  loop_term_q [0:3];
  logic [2:0]  loop_sp_q;
  logic        illegal_q;

  // Program memory (8K x 24).  Port A: fetch / host upload.  Port B: data.
  logic [23:0] pm_fetch_rdata;
  logic [23:0] pm_data_rdata;
  logic [12:0] pm_b_addr;
  logic        pm_b_we;
  logic [23:0] pm_b_wdata;

  // ---------------------------------------------------------------------
  // Decode (all combinational from instruction_q and the register file).
  // ---------------------------------------------------------------------
  logic [7:0] op;
  logic t_dual, t_imm_dm_w, t_direct_dm, t_alu_dm, t_alu_pm, t_ldreg, t_lreg;
  logic t_alu_move, t_cond_alu, t_jump, t_do, t_shift_dm, t_shift_pm;
  logic t_shift_move, t_shift_imm, t_cond_shift, t_move, t_ijump, t_ret;
  logic t_modify, t_divq, t_divs, t_stack, t_nop, supported;

  assign op = instruction_q[23:16];
  assign t_nop        = (instruction_q == 24'd0);
  assign t_dual       = (op[7:6] == 2'b11);
  assign t_imm_dm_w   = (op[7:5] == 3'b101);
  assign t_direct_dm  = (op[7:5] == 3'b100);
  assign t_alu_dm     = (op[7:5] == 3'b011);
  assign t_alu_pm     = (op[7:4] == 4'b0101);
  assign t_ldreg      = (op[7:4] == 4'b0100);
  assign t_lreg       = (op[7:4] == 4'b0011);
  assign t_alu_move   = (op[7:3] == 5'b00101);
  assign t_cond_alu   = (op[7:3] == 5'b00100);
  assign t_jump       = (op[7:3] == 5'b00011);
  assign t_do         = (op[7:2] == 6'b000101);
  assign t_shift_dm   = (op[7:1] == 7'b0001001);
  assign t_shift_pm   = (op == 8'h11);
  assign t_shift_move = (op == 8'h10);
  assign t_shift_imm  = (op == 8'h0f);
  assign t_cond_shift = (op == 8'h0e);
  assign t_move       = (op == 8'h0d);
  assign t_ijump      = (op == 8'h0b);
  assign t_ret        = (op == 8'h0a);
  assign t_modify     = (op == 8'h09);
  assign t_divq       = (op == 8'h07);
  assign t_divs       = (op == 8'h06);
  assign t_stack      = (op == 8'h04);
  assign supported = t_nop || t_dual || t_imm_dm_w || t_direct_dm || t_alu_dm ||
                     t_alu_pm || t_ldreg || t_lreg || t_alu_move || t_cond_alu ||
                     t_jump || t_do || t_shift_dm || t_shift_pm || t_shift_move ||
                     t_shift_imm || t_cond_shift || t_move || t_ijump || t_ret ||
                     t_modify || t_divq || t_divs || t_stack;

  // Computation fields shared by the ALU/MAC types.
  logic [4:0] amf;
  logic [1:0] yop;
  logic [2:0] xop;
  logic       z_fb;
  logic       has_compute;      // ALU or MAC operation present
  logic       compute_cond;     // gated by IF condition for type 9
  logic [3:0] cond_code;
  logic       cond_true;
  assign amf   = instruction_q[17:13];
  assign yop   = instruction_q[12:11];
  assign xop   = instruction_q[10:8];
  assign z_fb  = instruction_q[18];
  assign cond_code = instruction_q[3:0];
  assign has_compute = (t_dual || t_alu_dm || t_alu_pm || t_alu_move || t_cond_alu)
                       && (amf != 5'd0);

  // Shifter fields shared by the shift types.
  logic [3:0] sf;
  logic       has_shift;
  assign sf = instruction_q[14:11];
  assign has_shift = t_shift_dm || t_shift_pm || t_shift_move || t_shift_imm || t_cond_shift;

  // ---------------------------------------------------------------------
  // Register read helpers.
  // ---------------------------------------------------------------------
  function automatic logic [15:0] read_dreg(input logic [3:0] idx);
    begin
      case (idx)
        R_SE:    read_dreg = {{8{dreg_q[R_SE][7]}}, dreg_q[R_SE][7:0]};
        R_MR2:   read_dreg = {{8{dreg_q[R_MR2][7]}}, dreg_q[R_MR2][7:0]};
        default: read_dreg = dreg_q[idx];
      endcase
    end
  endfunction

  function automatic logic [15:0] read_reg(input logic [1:0] group, input logic [3:0] idx);
    logic [2:0] dag;
    begin
      read_reg = 16'd0;
      case (group)
        2'd0: read_reg = read_dreg(idx);
        2'd1, 2'd2: begin
          dag = {group[1], idx[1:0]};
          case (idx[3:2])
            2'd0: read_reg = {2'd0, ireg_q[dag]};
            2'd1: read_reg = {{2{mreg_q[dag][13]}}, mreg_q[dag]};
            2'd2: read_reg = {2'd0, lreg_q[dag]};
            default: read_reg = 16'd0;
          endcase
        end
        2'd3: begin
          case (idx)
            4'h0: read_reg = {8'd0, astat_q};
            4'h1: read_reg = {12'd0, mstat_q};
            4'h2: read_reg = {8'd0, (loop_sp_q == 3'd0), 1'b0, 1'b1, 1'b0,
                              (count_sp_q == 3'd0), 1'b0, (pc_sp_q == 5'd0), 1'b0};
            4'h3: read_reg = {12'd0, imask_q};
            4'h4: read_reg = {11'd0, icntl_q};
            4'h5: read_reg = {2'd0, cntr_q};
            4'h6: read_reg = {{11{sb_q[4]}}, sb_q};
            4'h7: read_reg = {8'd0, px_q};
            default: read_reg = 16'd0;
          endcase
        end
      endcase
    end
  endfunction

  // ALU / MAC X operand (Appendix A X codes; group differs for ALU vs MAC).
  function automatic logic [15:0] read_x(input logic [2:0] sel, input logic is_mac);
    begin
      case (sel)
        3'd0: read_x = is_mac ? dreg_q[R_MX0] : dreg_q[R_AX0];
        3'd1: read_x = is_mac ? dreg_q[R_MX1] : dreg_q[R_AX1];
        3'd2: read_x = dreg_q[R_AR];
        3'd3: read_x = dreg_q[R_MR0];
        3'd4: read_x = dreg_q[R_MR1];
        3'd5: read_x = read_dreg(R_MR2);
        3'd6: read_x = dreg_q[R_SR0];
        default: read_x = dreg_q[R_SR1];
      endcase
    end
  endfunction

  function automatic logic [15:0] read_y(input logic [1:0] sel, input logic is_mac);
    begin
      case (sel)
        2'd0: read_y = is_mac ? dreg_q[R_MY0] : dreg_q[R_AY0];
        2'd1: read_y = is_mac ? dreg_q[R_MY1] : dreg_q[R_AY1];
        2'd2: read_y = is_mac ? mf_q : af_q;
        default: read_y = 16'd0;
      endcase
    end
  endfunction

  function automatic logic condition_true(input logic [3:0] c);
    logic lt;
    begin
      lt = astat_q[1] ^ astat_q[2];
      case (c)
        4'h0: condition_true = astat_q[0];
        4'h1: condition_true = !astat_q[0];
        4'h2: condition_true = !(lt || astat_q[0]);
        4'h3: condition_true = lt || astat_q[0];
        4'h4: condition_true = lt;
        4'h5: condition_true = !lt;
        4'h6: condition_true = astat_q[2];
        4'h7: condition_true = !astat_q[2];
        4'h8: condition_true = astat_q[3];
        4'h9: condition_true = !astat_q[3];
        4'ha: condition_true = astat_q[4];
        4'hb: condition_true = !astat_q[4];
        4'hc: condition_true = astat_q[6];
        4'hd: condition_true = !astat_q[6];
        4'he: condition_true = !(cntr_valid_q && (cntr_q == 14'd1)) && cntr_valid_q;
        default: condition_true = 1'b1;
      endcase
    end
  endfunction

  assign cond_true = condition_true(cond_code);
  assign compute_cond = t_cond_alu ? cond_true : 1'b1;

  // ---------------------------------------------------------------------
  // ALU.
  // ---------------------------------------------------------------------
  logic        alu_is_mac;
  logic [15:0] alu_x, alu_y;
  logic [15:0] alu_result;
  logic alu_az, alu_an, alu_av, alu_ac, alu_flags_valid;
  assign alu_is_mac = (amf != 5'd0) && (amf < 5'd16);
  assign alu_x = read_x(xop, alu_is_mac);
  assign alu_y = read_y(yop, alu_is_mac);

  always_comb begin
    logic [16:0] sum;
    logic cin;
    sum = 17'd0;
    cin = astat_q[3];
    alu_result = 16'd0;
    alu_ac = 1'b0;
    alu_av = 1'b0;
    alu_flags_valid = 1'b1;
    case (amf)
      5'd16: alu_result = alu_y;
      5'd17: begin sum = {1'b0, alu_y} + 17'd1; alu_result = sum[15:0]; alu_ac = sum[16];
                   alu_av = !alu_y[15] && alu_result[15]; end
      5'd18, 5'd19: begin
        sum = {1'b0, alu_x} + {1'b0, alu_y} + {16'd0, (amf == 5'd18) ? cin : 1'b0};
        alu_result = sum[15:0]; alu_ac = sum[16];
        alu_av = !(alu_x[15] ^ alu_y[15]) && (alu_result[15] ^ alu_x[15]);
      end
      5'd20: alu_result = ~alu_y;
      5'd21: begin sum = {1'b0, ~alu_y} + 17'd1; alu_result = sum[15:0]; alu_ac = sum[16];
                   alu_av = (alu_y == 16'h8000); end
      5'd22, 5'd23: begin
        sum = {1'b0, alu_x} + {1'b0, ~alu_y} + {16'd0, (amf == 5'd22) ? cin : 1'b1};
        alu_result = sum[15:0]; alu_ac = sum[16];
        alu_av = (alu_x[15] ^ alu_y[15]) && (alu_result[15] ^ alu_x[15]);
      end
      5'd24: begin sum = {1'b0, alu_y} + 17'h0ffff; alu_result = sum[15:0]; alu_ac = sum[16];
                   alu_av = alu_y[15] && !alu_result[15]; end
      5'd25, 5'd26: begin
        sum = {1'b0, alu_y} + {1'b0, ~alu_x} + {16'd0, (amf == 5'd26) ? cin : 1'b1};
        alu_result = sum[15:0]; alu_ac = sum[16];
        alu_av = (alu_y[15] ^ alu_x[15]) && (alu_result[15] ^ alu_y[15]);
      end
      5'd27: alu_result = ~alu_x;
      5'd28: alu_result = alu_x & alu_y;
      5'd29: alu_result = alu_x | alu_y;
      5'd30: alu_result = alu_x ^ alu_y;
      5'd31: begin alu_result = alu_x[15] ? (~alu_x + 16'd1) : alu_x; alu_av = (alu_x == 16'h8000); end
      default: alu_flags_valid = 1'b0;
    endcase
    alu_az = (alu_result == 16'd0);
    alu_an = alu_result[15];
  end

  // ---------------------------------------------------------------------
  // MAC.  Operands are read at DECODE and the product registered at MEMORY
  // so COMMIT only performs the 40-bit accumulate.
  // ---------------------------------------------------------------------
  logic        mac_x_signed, mac_y_signed;
  logic [32:0] mac_product;      // signed 33-bit product of sign-extended operands
  logic signed [32:0] product_q;
  logic [39:0] mr_q;             // {MR2, MR1, MR0}
  logic [39:0] mac_p40, mac_sum, mac_rounded, mac_result;
  logic        mac_sub, mac_accumulate, mac_round, mac_mv;
  assign mr_q = {dreg_q[R_MR2][7:0], dreg_q[R_MR1], dreg_q[R_MR0]};
  always_comb begin
    // AMF 1-3: RND (signed x signed); 4-7 / 8-11 / 12-15: SS SU US UU.
    case (amf[1:0])
      2'd0: begin mac_x_signed = 1'b1; mac_y_signed = 1'b1; end
      2'd1: begin mac_x_signed = 1'b1; mac_y_signed = 1'b0; end
      2'd2: begin mac_x_signed = 1'b0; mac_y_signed = 1'b1; end
      default: begin mac_x_signed = 1'b0; mac_y_signed = 1'b0; end
    endcase
    if (amf <= 5'd3) begin mac_x_signed = 1'b1; mac_y_signed = 1'b1; end
    mac_round = (amf >= 5'd1) && (amf <= 5'd3);
    mac_accumulate = (amf[4:2] == 3'b010) || (amf[4:2] == 3'b011) || (amf == 5'd2) || (amf == 5'd3);
    mac_sub = (amf[4:2] == 3'b011) || (amf == 5'd3);
    mac_product = $signed({mac_x_signed & alu_x[15], alu_x}) * $signed({mac_y_signed & alu_y[15], alu_y});
    // Product format adjust: sign-extend to 40 bits and shift left one.
    mac_p40 = {{7{product_q[32]}}, product_q[31:0], 1'b0};
    if (!mac_accumulate) mac_sum = mac_p40;
    else if (mac_sub) mac_sum = mr_q - mac_p40;
    else mac_sum = mr_q + mac_p40;
    // Rounding at bit 15.  Manual (2.3.2.6): a midpoint *result* clears bit
    // 16 after the add.  MAME tests the midpoint on the shifted product and
    // clears bit 16 of the rounded result (fitted to 400 recorded samples).
    mac_rounded = mac_sum + 40'h0000008000;
    if (MAC_ROUND_MAME == 0) begin
      if (mac_sum[15:0] == 16'h8000) mac_rounded[16] = 1'b0;
    end else begin
      if (mac_p40[15:0] == 16'h8000) mac_rounded[16] = 1'b0;
    end
    mac_result = mac_round ? mac_rounded : mac_sum;
    mac_mv = !((mac_result[39:31] == 9'h000) || (mac_result[39:31] == 9'h1ff));
  end

  // ---------------------------------------------------------------------
  // Shifter.
  // ---------------------------------------------------------------------
  logic [15:0] sh_in;
  logic signed [8:0] sh_code;
  logic sh_ext, sh_lo, sh_or, sh_norm, sh_exp, sh_expadj;
  logic [47:0] sh_base;
  logic [47:0] sh_array;
  logic [31:0] sh_out;
  logic [7:0]  se_q;
  logic [4:0]  exp_count;    // leading redundant sign bits (HI) 0..15
  logic [4:0]  exp_lo_count; // leading bits equal to SS (LO) 0..16
  logic [7:0]  exp_result;
  logic [4:0]  exp_neg;
  logic        exp_ss;
  assign se_q = dreg_q[R_SE][7:0];
  // Shifter X code 0 selects SI (Appendix A: "X0 (SI for Shifter)").
  assign sh_in = (xop == 3'd0) ? dreg_q[R_SI] : read_x(xop, 1'b0);
  assign sh_norm = (sf[3:2] == 2'b10);
  assign sh_exp = (sf[3:2] == 2'b11) && (sf[1:0] != 2'b11);
  assign sh_expadj = (sf == 4'hf);
  assign sh_lo = sf[1];
  assign sh_or = sf[0];
  always_comb begin
    integer i;
    logic found;
    // Control code: immediate for type 15, SE otherwise, -SE for NORM.
    if (t_shift_imm) sh_code = $signed({instruction_q[7], instruction_q[7:0]});
    else if (sh_norm) sh_code = -$signed({se_q[7], se_q});
    else sh_code = $signed({se_q[7], se_q});
    // Extension bit: arithmetic/normalize use the input sign, logical zero.
    sh_ext = (sf[3:2] == 2'b01) ? sh_in[15] : (sh_norm ? (sh_lo ? 1'b0 : sh_in[15]) : 1'b0);
    if (sh_lo) sh_base = {{32{sh_ext}}, sh_in};
    else sh_base = {{16{sh_ext}}, sh_in, 16'h0000};
    if (sh_code >= 0) begin
      if (sh_code >= 9'sd48) sh_array = 48'd0;
      else sh_array = sh_base << sh_code[5:0];
    end else begin
      if (sh_code <= -9'sd48) sh_array = {48{sh_ext}};
      else sh_array = $signed(sh_base) >>> ((-sh_code) & 9'h3f);
    end
    sh_out = sh_array[31:0];
    if (sh_or) sh_out = sh_out | {dreg_q[R_SR1], dreg_q[R_SR0]};

    // Exponent detector (Table 2.5).
    exp_count = 5'd0;
    found = 1'b0;
    for (i = 14; i >= 0; i = i - 1) begin
      if (!found) begin
        if (sh_in[i] == sh_in[15]) exp_count = exp_count + 5'd1;
        else found = 1'b1;
      end
    end
    exp_lo_count = 5'd0;
    found = 1'b0;
    for (i = 15; i >= 0; i = i - 1) begin
      if (!found) begin
        if (sh_in[i] == astat_q[7]) exp_lo_count = exp_lo_count + 5'd1;
        else found = 1'b1;
      end
    end
    exp_ss = sh_in[15];
    exp_neg = -exp_count;
    if (sf == 4'hc) exp_result = -{3'd0, exp_count};                          // EXP (HI)
    else if (sf == 4'hd) begin                                                 // EXP (HIX)
      if (astat_q[2]) begin exp_result = 8'd1; exp_ss = !sh_in[15]; end
      else exp_result = -{3'd0, exp_count};
    end else exp_result = 8'd241 - {3'd0, exp_lo_count};                       // EXP (LO): -15 - k
  end

  // ---------------------------------------------------------------------
  // Memory operand decode.
  // ---------------------------------------------------------------------
  logic        mem_dm_read, mem_dm_write, mem_pm_read, mem_pm_write;
  logic [2:0]  dm_i_sel, dm_m_sel, pm_i_sel, pm_m_sel;
  logic [13:0] dm_addr, pm_addr;
  logic [15:0] dm_wdata;
  logic [3:0]  dm_dreg, pm_dreg;
  logic [15:0] pm_wdata16;
  always_comb begin
    mem_dm_read = 1'b0; mem_dm_write = 1'b0; mem_pm_read = 1'b0; mem_pm_write = 1'b0;
    dm_i_sel = 3'd0; dm_m_sel = 3'd0; pm_i_sel = 3'd4; pm_m_sel = 3'd4;
    dm_addr = 14'd0; pm_addr = 14'd0; dm_wdata = 16'd0;
    dm_dreg = instruction_q[7:4]; pm_dreg = instruction_q[7:4];
    pm_wdata16 = read_dreg(instruction_q[7:4]);
    if (t_dual) begin
      mem_dm_read = 1'b1; mem_pm_read = 1'b1;
      dm_i_sel = {1'b0, instruction_q[3:2]}; dm_m_sel = {1'b0, instruction_q[1:0]};
      pm_i_sel = {1'b1, instruction_q[7:6]}; pm_m_sel = {1'b1, instruction_q[5:4]};
      dm_dreg = {2'b00, instruction_q[19:18]};            // DD: AX0 AX1 MX0 MX1
      pm_dreg = {2'b01, instruction_q[21:20]};                      // PD: AY0 AY1 MY0 MY1
      dm_addr = ireg_q[dm_i_sel]; pm_addr = ireg_q[pm_i_sel];
    end else if (t_imm_dm_w) begin
      mem_dm_write = 1'b1;
      dm_i_sel = {instruction_q[20], instruction_q[3:2]}; dm_m_sel = {instruction_q[20], instruction_q[1:0]};
      dm_addr = ireg_q[dm_i_sel]; dm_wdata = instruction_q[19:4];
    end else if (t_direct_dm) begin
      mem_dm_read = !instruction_q[20]; mem_dm_write = instruction_q[20];
      dm_addr = instruction_q[17:4];
      dm_wdata = read_reg(instruction_q[19:18], instruction_q[3:0]);
    end else if (t_alu_dm) begin
      mem_dm_read = !instruction_q[19]; mem_dm_write = instruction_q[19];
      dm_i_sel = {instruction_q[20], instruction_q[3:2]}; dm_m_sel = {instruction_q[20], instruction_q[1:0]};
      dm_addr = ireg_q[dm_i_sel]; dm_wdata = read_dreg(instruction_q[7:4]);
    end else if (t_shift_dm) begin
      // Type 12: G at bit 16, D at bit 15 (Appendix A).
      mem_dm_read = !instruction_q[15]; mem_dm_write = instruction_q[15];
      dm_i_sel = {instruction_q[16], instruction_q[3:2]}; dm_m_sel = {instruction_q[16], instruction_q[1:0]};
      dm_addr = ireg_q[dm_i_sel]; dm_wdata = read_dreg(instruction_q[7:4]);
    end else if (t_alu_pm) begin
      mem_pm_read = !instruction_q[19]; mem_pm_write = instruction_q[19];
      pm_i_sel = {1'b1, instruction_q[3:2]}; pm_m_sel = {1'b1, instruction_q[1:0]};
      pm_addr = ireg_q[pm_i_sel];
    end else if (t_shift_pm) begin
      // Type 13: D at bit 15.
      mem_pm_read = !instruction_q[15]; mem_pm_write = instruction_q[15];
      pm_i_sel = {1'b1, instruction_q[3:2]}; pm_m_sel = {1'b1, instruction_q[1:0]};
      pm_addr = ireg_q[pm_i_sel];
    end
  end

  // DAG post-modify with modulo addressing (chapter 3.2.2): see dag_next_m.

  // ---------------------------------------------------------------------
  // External DM bus and program RAM.
  // ---------------------------------------------------------------------
  logic in_memory;
  // HALT is sampled at an instruction boundary.  An assertion while a memory
  // instruction is in flight must not withdraw its request or discard its
  // commit; doing either leaves a partially executed instruction behind.
  assign in_memory = !rst_i && (state_q == ST_MEMORY);
  assign dm_req_o = in_memory && (mem_dm_read || mem_dm_write);
  assign dm_we_o = mem_dm_write;
  assign dm_addr_o = dm_addr;
  assign dm_wdata_o = dm_wdata;

  assign pm_b_addr = pm_addr[12:0];
  assign pm_b_we = !rst_i && (state_q == ST_COMMIT) && mem_pm_write;
  assign pm_b_wdata = {pm_wdata16, px_q};

  assign prog_ready_o = rst_i || state_q == ST_FETCH;
  assign prog_rdata_o = pm_fetch_rdata;
  adsp2100_program_ram u_program_ram (
    .clk_i(clk_i),
    .a_addr_i((prog_we_i || (prog_re_i && prog_ready_o)) ? prog_addr_i : pc_q[12:0]), .a_we_i(prog_we_i),
    .a_wdata_i(prog_wdata_i), .a_rdata_o(pm_fetch_rdata),
    .b_addr_i(pm_b_addr), .b_we_i(pm_b_we), .b_wdata_i(pm_b_wdata),
    .b_rdata_o(pm_data_rdata)
  );

  // ---------------------------------------------------------------------
  // Sequencing and commit.
  // ---------------------------------------------------------------------
  logic [13:0] seq_pc;
  logic at_loop_end;
  logic [1:0] loop_push_idx;
  logic [1:0] loop_top_idx;
  logic [1:0] count_push_idx;
  logic [1:0] count_top_idx;
  logic [3:0] pc_push_idx;
  logic [3:0] pc_top_idx;
  assign seq_pc = pc_q + 14'd1;
  // The architectural counters need an extra bit to represent a full stack,
  // while every physical array index must remain in range even when a guarded
  // expression is evaluated speculatively by a simulator or synthesizer.
  assign loop_push_idx  = loop_sp_q[1:0];
  assign loop_top_idx   = loop_sp_q[1:0] - 2'd1;
  assign count_push_idx = count_sp_q[1:0];
  assign count_top_idx  = count_sp_q[1:0] - 2'd1;
  assign pc_push_idx    = pc_sp_q[3:0];
  assign pc_top_idx     = pc_sp_q[3:0] - 4'd1;
  assign at_loop_end = (loop_sp_q != 3'd0) && (pc_q == loop_end_q[loop_top_idx]);

  task automatic write_dreg(input logic [3:0] idx, input logic [15:0] value);
    begin
      dreg_q[idx] <= value;
      if (idx == R_MR1) dreg_q[R_MR2] <= {{8{value[15]}}, {8{value[15]}}};
    end
  endtask

  task automatic write_reg(input logic [1:0] group, input logic [3:0] idx, input logic [15:0] value);
    logic [2:0] dag;
    begin
      // Fully assign the task-local selector even in register groups that do
      // not consume it. This is combinational decode metadata, not state.
      dag = 3'd0;
      case (group)
        2'd0: write_dreg(idx, value);
        2'd1, 2'd2: begin
          dag = {group[1], idx[1:0]};
          case (idx[3:2])
            2'd0: ireg_q[dag] <= value[13:0];
            2'd1: mreg_q[dag] <= value[13:0];
            2'd2: lreg_q[dag] <= value[13:0];
            default: illegal_q <= 1'b1;
          endcase
        end
        2'd3: begin
          case (idx)
            4'h0: astat_q <= value[7:0];
            4'h1: mstat_q <= value[3:0];
            4'h3: imask_q <= value[3:0];
            4'h4: icntl_q <= value[4:0];
            4'h5: begin
              if (cntr_valid_q) begin
                if (count_sp_q < 3'd4) begin
                  count_stack_q[count_push_idx] <= cntr_q;
                  count_sp_q <= count_sp_q + 3'd1;
                end
              end
              cntr_q <= value[13:0];
              cntr_valid_q <= 1'b1;
            end
            4'h6: sb_q <= value[4:0];
            4'h7: px_q <= value[7:0];
            default: illegal_q <= 1'b1;
          endcase
        end
      endcase
    end
  endtask

  task automatic pop_counter();
    begin
      if (count_sp_q != 3'd0) begin
        cntr_q <= count_stack_q[count_top_idx];
        count_sp_q <= count_sp_q - 3'd1;
        cntr_valid_q <= 1'b1;
      end else begin
        cntr_valid_q <= 1'b0;
      end
    end
  endtask

  integer ri;
  always_ff @(posedge clk_i) begin
    logic [13:0] next_pc;
    logic explicit_flow;
    logic taken;
    logic terminate;
    logic ce_now;
    logic [15:0] dm_data;
    logic [23:0] pm_data;
    logic [15:0] ar_value;
    logic [15:0] dx, dy, r;
    logic aq;

    if (rst_i) begin
      state_q <= ST_FETCH;
      pc_q <= 14'd4;
      instruction_q <= 24'd0;
      astat_q <= 8'd0; mstat_q <= 4'd0; imask_q <= 4'd0; icntl_q <= 5'd0;
      cntr_q <= 14'd0; cntr_valid_q <= 1'b0; count_sp_q <= 3'd0;
      pc_sp_q <= 5'd0; loop_sp_q <= 3'd0;
      af_q <= 16'd0; mf_q <= 16'd0; sb_q <= 5'd0; px_q <= 8'd0;
      illegal_q <= 1'b0;
      product_q <= 33'd0;
      for (ri = 0; ri < 16; ri = ri + 1) dreg_q[ri] <= 16'd0;
      for (ri = 0; ri < 8; ri = ri + 1) begin
        ireg_q[ri] <= 14'd0; mreg_q[ri] <= 14'd0; lreg_q[ri] <= 14'd0;
      end
    end else if ((halt_i || prog_re_i) && (state_q == ST_FETCH)) begin
      // Stay between instructions until /HALT and /BR are both released.
      state_q <= ST_FETCH;
    end else begin
      case (state_q)
        ST_FETCH: state_q <= ST_FETCH_WAIT;
        ST_FETCH_WAIT: begin
          instruction_q <= pm_fetch_rdata;
          state_q <= ST_DECODE;
        end
        ST_DECODE: begin
          if (!supported) illegal_q <= 1'b1;
          state_q <= ST_MEMORY;
        end
        ST_MEMORY: begin
          product_q <= $signed(mac_product);
          if (!(mem_dm_read || mem_dm_write) || dm_ready_i) state_q <= ST_COMMIT;
        end
        ST_COMMIT: begin
          next_pc = seq_pc;
          explicit_flow = 1'b0;
          taken = 1'b0;
          dm_data = dm_rdata_i;
          pm_data = pm_data_rdata;
          ce_now = cntr_valid_q && (cntr_q == 14'd1);

          // ---- Computation (reads old registers, writes at the end) ----
          if (has_compute && compute_cond) begin
            if (alu_is_mac) begin
              if (z_fb) mf_q <= mac_result[31:16];
              else begin
                dreg_q[R_MR0] <= mac_result[15:0];
                dreg_q[R_MR1] <= mac_result[31:16];
                dreg_q[R_MR2] <= {{8{mac_result[39]}}, mac_result[39:32]};
              end
              astat_q[6] <= mac_mv;
            end else begin
              ar_value = alu_result;
              if (mstat_q[3] && alu_av && alu_flags_valid)
                ar_value = alu_result[15] ? 16'h7fff : 16'h8000;
              if (z_fb) af_q <= alu_result;
              else dreg_q[R_AR] <= ar_value;
              if (alu_flags_valid) begin
                astat_q[0] <= alu_az;
                astat_q[1] <= alu_an;
                astat_q[2] <= mstat_q[2] ? (astat_q[2] || alu_av) : alu_av;
                astat_q[3] <= alu_ac;
                if (amf == 5'd31) astat_q[4] <= alu_x[15];
              end
            end
          end

          // ---- Shifter ----
          if (has_shift && (!t_cond_shift || cond_true)) begin
            if (sh_expadj) begin
              if ($signed(exp_neg) > $signed(sb_q)) sb_q <= exp_neg;
            end else if (sh_exp) begin
              if (sf != 4'he || se_q == 8'hf1)
                dreg_q[R_SE] <= {8'd0, exp_result};
              if (sf != 4'he) astat_q[7] <= exp_ss;
            end else begin
              dreg_q[R_SR0] <= sh_out[15:0];
              dreg_q[R_SR1] <= sh_out[31:16];
            end
          end

          // ---- Division primitives ----
          if (t_divs) begin
            dx = read_x(xop, 1'b0);
            dy = read_y(yop, 1'b0);
            aq = dx[15] ^ dy[15];
            astat_q[5] <= aq;
            af_q <= {dy[14:0], dreg_q[R_AY0][15]};
            dreg_q[R_AY0] <= {dreg_q[R_AY0][14:0], aq};
          end
          if (t_divq) begin
            dx = read_x(xop, 1'b0);
            r = astat_q[5] ? (af_q + dx) : (af_q - dx);
            aq = r[15] ^ dx[15];
            astat_q[5] <= aq;
            af_q <= {r[14:0], dreg_q[R_AY0][15]};
            dreg_q[R_AY0] <= {dreg_q[R_AY0][14:0], !aq};
          end

          // ---- Memory results and DAG updates ----
          if (mem_dm_read) begin
            if (t_direct_dm) write_reg(instruction_q[19:18], instruction_q[3:0], dm_data);
            else write_dreg(dm_dreg, dm_data);
          end
          if (mem_pm_read) begin
            write_dreg(pm_dreg, pm_data[23:8]);
            px_q <= pm_data[7:0];
          end
          if (t_dual) begin
            ireg_q[dm_i_sel] <= dag_next_m(dm_i_sel, dm_m_sel);
            ireg_q[pm_i_sel] <= dag_next_m(pm_i_sel, pm_m_sel);
          end else if (mem_dm_read || mem_dm_write) begin
            if (!t_direct_dm) ireg_q[dm_i_sel] <= dag_next_m(dm_i_sel, dm_m_sel);
          end else if (mem_pm_read || mem_pm_write) begin
            ireg_q[pm_i_sel] <= dag_next_m(pm_i_sel, pm_m_sel);
          end
          if (t_modify) begin
            ireg_q[{instruction_q[4], instruction_q[3:2]}] <=
              dag_next_m({instruction_q[4], instruction_q[3:2]}, {instruction_q[4], instruction_q[1:0]});
          end

          // ---- Register loads and moves ----
          if (t_ldreg) write_dreg(instruction_q[3:0], instruction_q[19:4]);
          if (t_lreg) write_reg(instruction_q[19:18], instruction_q[3:0], {2'd0, instruction_q[17:4]});
          if (t_move) write_reg(instruction_q[11:10], instruction_q[7:4],
                                read_reg(instruction_q[9:8], instruction_q[3:0]));
          if (t_alu_move || t_shift_move) write_dreg(instruction_q[7:4], read_dreg(instruction_q[3:0]));

          // ---- Flow control ----
          if (t_jump && cond_true) begin
            taken = 1'b1;
            if (instruction_q[18]) begin
              if (pc_sp_q < 5'd16) begin
                pc_stack_q[pc_push_idx] <= seq_pc;
                pc_sp_q <= pc_sp_q + 5'd1;
              end else illegal_q <= 1'b1;
            end
            next_pc = instruction_q[17:4];
          end
          if (t_jump && (cond_code == 4'he) && cntr_valid_q) begin
            // A conditional jump testing CE also decrements the counter.
            if (cntr_q == 14'd1) pop_counter();
            else cntr_q <= cntr_q - 14'd1;
          end
          if (t_ijump && cond_true) begin
            taken = 1'b1;
            if (instruction_q[4]) begin
              if (pc_sp_q < 5'd16) begin
                pc_stack_q[pc_push_idx] <= seq_pc;
                pc_sp_q <= pc_sp_q + 5'd1;
              end else illegal_q <= 1'b1;
            end
            next_pc = ireg_q[{1'b1, instruction_q[7:6]}];
          end
          if (t_ret && cond_true) begin
            taken = 1'b1;
            if (pc_sp_q != 5'd0) begin
              next_pc = pc_stack_q[pc_top_idx];
              pc_sp_q <= pc_sp_q - 5'd1;
            end else illegal_q <= 1'b1;
          end
          if (t_stack) begin
            if (instruction_q[4] && pc_sp_q != 5'd0) pc_sp_q <= pc_sp_q - 5'd1;
            if (instruction_q[3] && loop_sp_q != 3'd0) loop_sp_q <= loop_sp_q - 3'd1;
            if (instruction_q[2]) pop_counter();
          end
          explicit_flow = taken;

          if (t_do) begin
            if (loop_sp_q < 3'd4 && pc_sp_q < 5'd16) begin
              loop_end_q[loop_push_idx] <= instruction_q[17:4];
              loop_term_q[loop_push_idx] <= instruction_q[3:0];
              loop_sp_q <= loop_sp_q + 3'd1;
              pc_stack_q[pc_push_idx] <= seq_pc;
              pc_sp_q <= pc_sp_q + 5'd1;
            end else illegal_q <= 1'b1;
          end else if (at_loop_end && !t_stack &&
                       (!explicit_flow || LOOP_END_MAME != 0)) begin
            terminate = termination_true(loop_term_q[loop_top_idx]);
            if (explicit_flow) begin
              // MAME-compatibility only: pop the stacks on a true termination
              // even though the explicit branch is taken.
              if (terminate) begin
                loop_sp_q <= loop_sp_q - 3'd1;
                if (!t_ret) begin
                  if (pc_sp_q != 5'd0) pc_sp_q <= pc_sp_q - 5'd1;
                end
                if (loop_term_q[loop_top_idx] == 4'he && cntr_valid_q) pop_counter();
              end
            end else begin
              if (loop_term_q[loop_top_idx] == 4'he && cntr_valid_q) begin
                if (cntr_q == 14'd1) pop_counter();
                else cntr_q <= cntr_q - 14'd1;
              end
              if (terminate) begin
                loop_sp_q <= loop_sp_q - 3'd1;
                if (pc_sp_q != 5'd0) pc_sp_q <= pc_sp_q - 5'd1;
                else illegal_q <= 1'b1;
              end else begin
                if (pc_sp_q != 5'd0) next_pc = pc_stack_q[pc_top_idx];
                else illegal_q <= 1'b1;
              end
            end
          end

          pc_q <= next_pc;
          state_q <= ST_PAD;
        end
        ST_PAD: state_q <= ST_FETCH;
        default: state_q <= ST_FETCH;
      endcase
    end
  end

  function automatic logic termination_true(input logic [3:0] term);
    begin
      case (term)
        4'he: termination_true = cntr_valid_q && (cntr_q == 14'd1);
        4'hf: termination_true = 1'b1;
        default: termination_true = condition_true(term ^ 4'h1);
      endcase
    end
  endfunction

  function automatic logic [13:0] dag_next_m(input logic [2:0] isel, input logic [2:0] msel);
    logic [13:0] i, m, l, base, sum, span, mask;
    integer k;
    begin
      i = ireg_q[isel]; m = mreg_q[msel]; l = lreg_q[isel];
      sum = i + m;
      if (l == 14'd0) dag_next_m = sum;
      else begin
        mask = 14'd0;
        for (k = 0; k < 14; k = k + 1) if (l > ((14'd1 << k) - 14'd1)) mask[k] = 1'b1;
        base = i & ~mask;
        span = sum - base;
        if (m[13]) begin
          if (span >= l) dag_next_m = sum + l;   // wrapped below base (span went negative)
          else dag_next_m = sum;
        end else begin
          if (span >= l) dag_next_m = sum - l;
          else dag_next_m = sum;
        end
      end
    end
  endfunction

  assign pc_o = pc_q;
  assign instruction_o = instruction_q;
  assign running_o = !rst_i && !((halt_i || prog_re_i) && (state_q == ST_FETCH));
  assign illegal_o = illegal_q;

  logic unused;
  assign unused = ^{imask_q, icntl_q, mac_p40[0]};
endmodule

`default_nettype wire

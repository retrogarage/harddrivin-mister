// -----------------------------------------------------------------------------
// tms34010_decode.sv
//
// Combinational instruction decoder.
//
// Recognition is current through Task 0135. The authoritative per-instruction
// implementation/test ledger is docs/instruction_coverage.md; shared decoded
// instruction classes and control fields are in rtl/tms34010_pkg.sv.
//
// Flag policy and encodings cite SPVU001A Appendix A (Instruction Set
// summary chart, page A-14) and corresponding `docs/assumptions.md`
// entries (A0011-A0014).
//
// Spec sources cited:
//   third_party/TMS34010_Info/tools/assembler/
//     TMS34010_Assembly_Language_Tools_Users_Guide_SPVU004.pdf
//     (the assembler manual; the assembled listings on pages ~1356, 3823,
//      3898 show the bit-level encoding for MOVI IW. Cross-referenced
//      against `MOVI pbuf_sz, A4 → 0x09C4 0005` and
//      `MOVI array_size, A2 → 0x09C2 0x0640`.)
//   third_party/TMS34010_Info/bibliography/hdl-reimplementation/
//     02-instruction-set.md  §"Encoding shape" + §"Move and load/store".
//
// Synthesis notes:
//   - One `always_comb` block.
//   - Safe defaults at the top: illegal output if no arm matches.
//   - No `/`, no `%`, no loops, no `initial`.
// -----------------------------------------------------------------------------

`default_nettype none
module tms34010_decode
  import tms34010_pkg::*;
#(
  // Task 0180: 0 removes the pixel-array instructions (FILL, PIXBLT, DRAV,
  // LINE) by decoding them as illegal opcodes, so an instance that never
  // executes them (the Hard Drivin' MSP) drops the pixel-processing
  // datapath and its ~20 ns write paths.
  parameter bit PIXEL_OPS = 1'b1
) (
  input  instr_word_t    instr,
  output decoded_instr_t decoded
);

  // Fixed-width views of the top opcode bits.
  logic [9:0] top10;
  logic [6:0] top7;
  logic [5:0] top6;
  assign top10 = instr[INSTR_WORD_WIDTH-1:6];
  assign top7  = instr[INSTR_WORD_WIDTH-1:9];
  assign top6  = instr[INSTR_WORD_WIDTH-1:10];

  // User's Guide §8.7/Table 8-6 reserved ranges. Keep architectural illegal
  // detection separate from `decoded.illegal`: the latter also marks valid
  // ISA encodings whose implementation has not landed yet.
  logic architectural_illegal;
  assign architectural_illegal =
       (instr[15:8] == 8'h02)
    || (instr[15:8] == 8'h04)
    || (instr[15:8] == 8'h08)
    || (instr[15:8] == 8'h0A)
    || (instr[15:8] == 8'h0C)
    || (instr[15:8] == 8'h0E)
    || (instr[15:10] == 6'b00_1101) // 0x3400 through 0x37FF
    || (instr[15:12] == 4'h7)
    || (instr[15:9] == 7'b100_1111) // 0x9E00 through 0x9FFF
    || (instr[15:9] == 7'b101_1111) // 0xBE00 through 0xBFFF
    || ((instr[15:11] == 5'b110_11) && (instr[10:8] != 3'b111))
    || (instr[15:9] == 7'b111_1111); // 0xFE00 through 0xFFFF

  // Opcode prefixes (each cited from SPVU001A Appendix A page A-14).
  localparam logic [9:0] MOVI_TOP10   = 10'b00_0010_0111;
  localparam logic [5:0] MOVK_TOP6    = 6'b00_0110;
  // K-family arithmetic shares the top-4 prefix 4'b0001; bits[11:10]
  // select the operation (00=ADDK, 01=SUBK, 10=MOVK, 11=BTST K).
  localparam logic [5:0] ADDK_TOP6    = 6'b00_0100;  // chart: 0001 00KK KKKR DDDD
  localparam logic [5:0] SUBK_TOP6    = 6'b00_0101;  // chart: 0001 01KK KKKR DDDD
  // Single-register unary family: bits[15:7] = 9'b000000111 (= 0x007);
  // bits[6:5] picks sub-op: 00=ABS, 01=NEG, 10=NEGB, 11=NOT.
  localparam logic [8:0] UNARY_TOP9    = 9'b0000_0011_1;
  // 16-bit-immediate arithmetic family. All share bits[15:5] (11-bit
  // prefix), bottom 5 bits are {R, Rd[3:0]}. The immediate value
  // follows in the next 16-bit word, sign-extended to 32 bits.
  localparam logic [10:0] ADDI_IW_TOP11 = 11'b0000_1011_000;  // ADDI IW
  localparam logic [10:0] SUBI_IW_TOP11 = 11'b0000_1011_111;  // SUBI IW
  localparam logic [10:0] CMPI_IW_TOP11 = 11'b0000_1011_010;  // CMPI IW

  // K-form shift family. Each is a top-6 prefix; bits[9:5] = K (5-bit
  // shift amount); bit[4] = R; bits[3:0] = Rd.
  localparam logic [5:0] SLA_K_TOP6  = 6'b001000;  // 0010 00KK KKKR DDDD
  localparam logic [5:0] SLL_K_TOP6  = 6'b001001;  // 0010 01KK KKKR DDDD
  localparam logic [5:0] SRA_K_TOP6  = 6'b001010;  // 0010 10KK KKKR DDDD
  localparam logic [5:0] SRL_K_TOP6  = 6'b001011;  // 0010 11KK KKKR DDDD
  localparam logic [5:0] RL_K_TOP6   = 6'b001100;  // 0011 00KK KKKR DDDD

  // IL-form immediate family. 11-bit prefix; bottom 5 bits = {R, Rd}.
  // 32-bit immediate follows in two 16-bit words (LO, HI).
  localparam logic [10:0] ADDI_IL_TOP11 = 11'b0000_1011_001;  // ADDI IL
  localparam logic [10:0] CMPI_IL_TOP11 = 11'b0000_1011_011;  // CMPI IL
  localparam logic [10:0] ANDI_IL_TOP11 = 11'b0000_1011_100;  // ANDI/ANDNI IL
  localparam logic [10:0] ORI_IL_TOP11  = 11'b0000_1011_101;  // ORI  IL
  localparam logic [10:0] XORI_IL_TOP11 = 11'b0000_1011_110;  // XORI IL
  localparam logic [10:0] SUBI_IL_TOP11 = 11'b0000_1101_000;  // SUBI IL (different base!)

  // MOVE Rs, Rd (register-to-register). Per SPVU001A page 12-126 and the
  // summary table (verified against BOTH the 1986 first edition and the
  // 1988 User's Guide), the encoding is `0100 11MS SSSR DDDD` (base
  // 0x4C00). This is NOT a field move: the full 32 bits are copied and
  // the field size has no effect, so there is no F bit here.
  //   - M = instr[9]: 0 = both registers in one file, 1 = cross-file.
  //   - R = instr[4]: when M=0, the file for BOTH registers; when M=1,
  //                   the SOURCE file (the destination is the other file).
  // This is the only MOVE that crosses register files (e.g. MOVE A0,B1 =
  // 0x4E01, the object code in Figure 12-3). The earlier code decoded
  // reg-to-reg MOVE at 0x9000 (`1001 00FS`), which is actually
  // MOVE Rs,*Rd+ (postincrement to memory) — see assumptions.md A0020.
  localparam logic [5:0] MOVE_RR_TOP6 = 6'b010011;  // 0100 11MS SSSR DDDD

  // MOVE field, register <-> indirect (no increment/decrement). Per
  // SPVU001A pages 12-127 (Rs,*Rd) and 12-135 (*Rs,Rd). The pointer
  // register holds a BIT address; Rs and Rd are in the same file (R bit).
  //   MOVE Rs,*Rd : 1000 00FS SSSR DDDD  (top6 = 100000) — store
  //   MOVE *Rs,Rd : 1000 01FS SSSR DDDD  (top6 = 100001) — load
  // F=instr[9] selects FS0/FE0 vs FS1/FE1 in ST. Task 0059 implements
  // ONLY the field-size-32, word-aligned case (FS selected = 32, where
  // sign/zero extension is a no-op and the access maps to the existing
  // 32-bit aligned memory path). Other field sizes / unaligned pointers
  // are deferred (assumptions.md A0020 / Phase 6).
  localparam logic [5:0] MOVE_STORE_TOP6 = 6'b100000;  // MOVE Rs,*Rd
  localparam logic [5:0] MOVE_LOAD_TOP6  = 6'b100001;  // MOVE *Rs,Rd
  localparam logic [5:0] MOVE_M2M_TOP6   = 6'b100010;  // MOVE *Rs,*Rd (ind->ind)
  // Auto inc/dec indirect-to-indirect (Task 0062). Both pointers step by
  // the field size (±32 at FS=32).
  //   MOVE *Rs+,*Rd+ : 1001 10FS  (top6 = 100110, 0x9800) postincrement
  //   MOVE -*Rs,-*Rd : 1010 10FS  (top6 = 101010, 0xA800) predecrement
  localparam logic [5:0] MOVE_M2M_POSTINC_TOP6 = 6'b100110;
  localparam logic [5:0] MOVE_M2M_PREDEC_TOP6  = 6'b101010;

  // Register-indirect with 16-bit signed offset (Task 0064). The effective
  // bit-address is pointer + sign_extend(offset16); the offset is the 2nd
  // instruction word. The pointer register is unchanged.
  //   MOVE Rs,*Rd(off) : 1011 00FS  (top6 = 101100, 0xB000) store
  //   MOVE *Rs(off),Rd : 1011 01FS  (top6 = 101101, 0xB400) load
  localparam logic [5:0] MOVE_OFF_STORE_TOP6 = 6'b101100;
  localparam logic [5:0] MOVE_OFF_LOAD_TOP6  = 6'b101101;
  // Offset source to postincrement destination (Task 0126):
  //   MOVE *Rs(off),*Rd+ : 1101 00FS (top6 = 110100, 0xD000)
  localparam logic [5:0] MOVE_OFF_M2M_PI_TOP6 = 6'b110100;
  // Absolute source to postincrement destination (Task 0127):
  //   MOVE @SAddr,*Rd+ : 1101 01F0 000R DDDD + source address LO/HI
  localparam logic [5:0] MOVE_ABS_M2M_PI_TOP6 = 6'b110101;

  // Auto inc/dec indirect MOVE (Task 0060). Same field semantics as the
  // plain forms; the pointer register steps by the field size (±32 at
  // FS=32). Postincrement uses the pointer then adds; predecrement
  // subtracts first and uses the new value.
  //   MOVE Rs,*Rd+ : 1001 00FS  (top6 = 100100) store, postincrement
  //   MOVE *Rs+,Rd : 1001 01FS  (top6 = 100101) load,  postincrement
  //   MOVE Rs,-*Rd : 1010 00FS  (top6 = 101000) store, predecrement
  //   MOVE -*Rs,Rd : 1010 01FS  (top6 = 101001) load,  predecrement
  localparam logic [5:0] MOVE_STORE_POSTINC_TOP6 = 6'b100100;
  localparam logic [5:0] MOVE_LOAD_POSTINC_TOP6  = 6'b100101;
  localparam logic [5:0] MOVE_STORE_PREDEC_TOP6  = 6'b101000;
  localparam logic [5:0] MOVE_LOAD_PREDEC_TOP6   = 6'b101001;

  // MOVB (move byte) — a special form of MOVE with the field size fixed at 8
  // (SPVU001A 12-118ff). No FS field in the encoding; loads always
  // sign-extend the byte to 32 bits. MOVB has no auto inc/dec forms. These
  // The first 7 forms map directly onto the existing MOVE field datapaths
  // with FS forced to 8 (decoded.force_byte). The store/load indirect forms
  // differ in bit[9], so they are matched on top7 (bits[15:9]). The remaining
  // two forms use the shared multiword M2M path added in Task 0120.
  //   MOVB Rs,*Rd          : 1000 110S  (top7 = 1000110, 0x8C00) store
  //   MOVB *Rs,Rd          : 1000 111S  (top7 = 1000111, 0x8E00) load
  //   MOVB *Rs,*Rd         : 1001 110S  (top7 = 1001110, 0x9C00) ind->ind
  //   MOVB Rs,*Rd(off)     : 1010 110S  (top7 = 1010110, 0xAC00) off store
  //   MOVB *Rs(off),Rd     : 1010 111S  (top7 = 1010111, 0xAE00) off load
  // The two absolute forms live in the 0000 01.. field-op family (sub-op
  // instr[7:5]=111), distinguished from each other by bit[9]:
  //   MOVB Rs,@DAddr       : 0000 0101 111R SSSS (0x05E0) abs store (bit9=0)
  //   MOVB @SAddr,Rd       : 0000 0111 111R DDDD (0x07E0) abs load  (bit9=1)
  //   MOVB *Rs(SOff),*Rd(DOff)  : 1011 110S (0xBC00) offset-to-offset
  //   MOVB @SAddr,@DAddr        : 0000 0011 0100 0000 (0x0340) abs-to-abs
  localparam logic [6:0] MOVB_STORE_TOP7    = 7'b1000_110;
  localparam logic [6:0] MOVB_LOAD_TOP7     = 7'b1000_111;
  localparam logic [6:0] MOVB_M2M_TOP7      = 7'b1001_110;
  localparam logic [6:0] MOVB_OFF_STORE_TOP7 = 7'b1010_110;
  localparam logic [6:0] MOVB_OFF_LOAD_TOP7  = 7'b1010_111;
  localparam logic [6:0] MOVB_OFF_M2M_TOP7   = 7'b1011_110;
  // Three-word offset-to-offset MOVE:
  //   1011 10FS SSSR DDDD + signed source offset + signed destination offset.
  localparam logic [5:0] MOVE_OFF_M2M_TOP6   = 6'b101110;
  localparam instr_word_t MOVB_ABS_M2M_OPCODE = 16'h0340;
  // Five-word absolute-to-absolute MOVE (User's Guide pp. 12-157/158):
  //   0000 01 F1 1100 0000 + source LO/HI + destination LO/HI.
  // Bit 9 selects F0/F1; both encodings share this masked value.
  localparam instr_word_t MOVE_ABS_M2M_MASK   = 16'hFDFF;
  localparam instr_word_t MOVE_ABS_M2M_OPCODE = 16'h05C0;

  // PIXT (pixel transfer), LINEAR forms — a pixel-size (PSIZE) field move
  // (SPVU001A 12-? / encoding table). The XY forms (1111 000S/001S/010S) need
  // XY->linear address conversion and are deferred. Linear forms:
  //   PIXT Rs,*Rd  : 1111 100S (top7 = 1111100, 0xF800) store
  //   PIXT *Rs,Rd  : 1111 101S (top7 = 1111101, 0xFA00) load
  //   PIXT *Rs,*Rd : 1111 110S (top7 = 1111110, 0xFC00) indirect-to-indirect
  localparam logic [6:0] PIXT_STORE_TOP7 = 7'b1111_100;
  localparam logic [6:0] PIXT_LOAD_TOP7  = 7'b1111_101;
  localparam logic [6:0] PIXT_M2M_TOP7   = 7'b1111_110;
  // XY-addressed PIXT: the pointer holds an XY value (converted to linear by
  // the core's xy_addr path). PIXT Rs,*Rd.XY (1111 000S, 0xF000) store /
  // PIXT *Rs.XY,Rd (1111 001S, 0xF200) load. The XY-to-XY M2M (1111 010S,
  // 0xF400) needs dual conversion and is deferred.
  localparam logic [6:0] PIXT_XY_STORE_TOP7 = 7'b1111_000;
  localparam logic [6:0] PIXT_XY_LOAD_TOP7  = 7'b1111_001;
  localparam logic [6:0] PIXT_XY_M2M_TOP7   = 7'b1111_010;  // *Rs.XY,*Rd.XY (0xF400)

  // MOVX / MOVY — move the X (low 16) or Y (high 16) half of Rs into the
  // same half of Rd, leaving the other half untouched. Per SPVU001A pages
  // 12-162/12-163. Encodings 1110 110S (MOVX) / 1110 111S (MOVY) SSSR DDDD.
  // Rs and Rd same file; all flags Unaffected. (Task 0065.)
  localparam logic [6:0] MOVX_TOP7    = 7'b1110_110;
  localparam logic [6:0] MOVY_TOP7    = 7'b1110_111;
  // CVXYL — convert the XY address in Rs to a linear address in Rd (SPVU001A
  // 12-59). Encoding 1110 100S SSSR DDDD. Same file; all flags Unaffected.
  localparam logic [6:0] CVXYL_TOP7   = 7'b1110_100;

  // ADDXY / SUBXY — add/subtract the X (low 16) and Y (high 16) halves of
  // Rs and Rd independently (no carry between halves). Per SPVU001A pages
  // 12-41 (ADDXY) / 12-252 (SUBXY). Encodings 1110 000S (ADDXY) /
  // 1110 001S (SUBXY) SSSR DDDD. Status bits encode graphics-clipping
  // info (computed in the core's XY datapath). Same-file. (Task 0066.)
  localparam logic [6:0] ADDXY_TOP7   = 7'b1110_000;
  localparam logic [6:0] SUBXY_TOP7   = 7'b1110_001;
  // DRAV Rs,Rd — draw COLOR1 at Rd's XY then Rd += Rs (XY). Encoding
  // `1111 011S SSSR DDDD` (base 0xF600), same reg-reg field layout as ADDXY.
  localparam logic [6:0] DRAV_TOP7    = 7'b1111_011;
  // CMPXY — nondestructive XY compare (Rd unchanged). Sets status as if
  // RdX-RsX / RdY-RsY were computed. Per SPVU001A page 12-55. Encoding
  // 1110 010S SSSR DDDD. (Task 0067.)
  localparam logic [6:0] CMPXY_TOP7   = 7'b1110_010;
  // CPW — compare point (Rs) to window (WSTART=B5, WEND=B6); the 4-bit
  // out-of-window code lands in Rd. Per SPVU001A page 12-57. Encoding
  // 1110 011S SSSR DDDD. (Task 0070.)
  localparam logic [6:0] CPW_TOP7     = 7'b1110_011;
  // MPYS / MPYU — multiply Rs (field, FS1 bits) by the 32 bits in Rd,
  // producing a 32..64-bit result. Per SPVU001A pages 12-164 (MPYS) /
  // 12-166 (MPYU). Encodings 0101 110S (MPYS) / 0101 111S (MPYU) SSSR DDDD.
  // Task 0071 implements FS1=32 (full 32-bit Rs); field-size-n deferred.
  localparam logic [6:0] MPYS_TOP7    = 7'b0101_110;
  localparam logic [6:0] MPYU_TOP7    = 7'b0101_111;
  // DIVU — unsigned divide. Per SPVU001A page 12-69. Encoding
  // 0101 101S SSSR DDDD. Multi-cycle (the core runs tms34010_divider).
  localparam logic [6:0] DIVU_TOP7    = 7'b0101_101;
  // MODU — unsigned modulo (Rd mod Rs -> Rd). Per SPVU001A page 12-113.
  // Encoding 0110 111S SSSR DDDD. Reuses the divider (remainder result).
  localparam logic [6:0] MODU_TOP7    = 7'b0110_111;
  // DIVS / MODS — signed divide / modulo (SPVU001A 12-63 / 12-112). The
  // divider stays unsigned; the core feeds |operands| and sign-conditions
  // the results. Encodings 0101 100S (DIVS) / 0110 110S (MODS) SSSR DDDD.
  localparam logic [6:0] DIVS_TOP7    = 7'b0101_100;
  localparam logic [6:0] MODS_TOP7    = 7'b0110_110;
  localparam logic [6:0] ADD_RR_TOP7  = 7'b0100_000;  // chart: 0100 000S SSSR DDDD
  localparam logic [6:0] ADDC_RR_TOP7 = 7'b0100_001;  // chart: 0100 001S SSSR DDDD
  localparam logic [6:0] SUB_RR_TOP7  = 7'b0100_010;  // chart: 0100 010S SSSR DDDD
  localparam logic [6:0] SUBB_RR_TOP7 = 7'b0100_011;  // chart: 0100 011S SSSR DDDD
  localparam logic [6:0] AND_RR_TOP7  = 7'b0101_000;  // chart: 0101 000S SSSR DDDD
  localparam logic [6:0] ANDN_RR_TOP7 = 7'b0101_001;  // chart: 0101 001S SSSR DDDD
  localparam logic [6:0] OR_RR_TOP7   = 7'b0101_010;  // chart: 0101 010S SSSR DDDD
  localparam logic [6:0] XOR_RR_TOP7  = 7'b0101_011;  // chart: 0101 011S SSSR DDDD
  localparam logic [6:0] CMP_RR_TOP7  = 7'b0100_100;  // chart: 0100 100S SSSR DDDD

  // PUSHST — push status register onto stack. Per SPVU001A summary
  // table page A-16: single fixed encoding 0x01E0
  // (`0000 0001 1110 0000`). Sets SP -= 32 then writes ST as a
  // 32-bit word to mem[new_SP]. Status bits unaffected.
  localparam instr_word_t PUSHST_OPCODE = 16'h01E0;
  // POPST — pop status register from stack. Per SPVU001A summary
  // table page A-16: single fixed encoding 0x01C0
  // (`0000 0001 1100 0000`). Reads 32-bit ST from mem[SP], then
  // increments SP by 32. Status bits all written by the loaded value.
  localparam instr_word_t POPST_OPCODE  = 16'h01C0;

  // CALL Rs — Call Subroutine Indirect. Per SPVU001A page 12-47 +
  // summary table line 27018. Encoding `0000 1001 001R DDDD`:
  //   top11 = 11'b00001001_001 = 0x049
  //   bit[4] = R (file of Rs)
  //   bits[3:0] = Rs index
  // Semantics:
  //   SP -= 32
  //   mem[new SP] = PC'   (where PC' = address of next instruction)
  //   PC = Rs              (with bottom 4 bits forced to 0 for word align)
  // Status bits all "Unaffected".
  localparam logic [10:0] CALL_RS_TOP11 = 11'b0000_1001_001;

  // RETI — Return from Interrupt. Per SPVU001A page 12-230 + summary
  // table line 27037. Single fixed encoding `0x0940`. Pops both ST
  // and PC from the stack (in that order) — two consecutive 32-bit
  // reads. SP increments by 32 after each pop (total +64).
  // All four status flag bits + IE come from the popped ST.
  localparam instr_word_t RETI_OPCODE   = 16'h0940;

  // TRAP N — Software Interrupt. Per SPVU001A page 12-252. Encoding
  // `0000 1001 000N NNNN` = `0x0900 | N`, where N at instr[4:0] is
  // the trap number (0..31). Action per spec:
  //   1) Push PC' (32-bit) at SP-32.
  //   2) Push ST  (32-bit) at SP-64.
  //   3) ST <- 0x00000010 (clears flags + IE; FS0 = 16, FS1 = 0).
  //   4) PC <- mem[0xFFFFFFE0 - N*32] (32-bit vector fetch).
  //   5) SP -= 64.
  // TRAP 0 is special-cased in the spec (skips the pushes); we defer
  // that case to a follow-up and accept N=0 as well-defined only for
  // N>=1 here. The pattern + mask catch the whole 0x0900..0x091F range.
  localparam logic [10:0] TRAP_TOP11      = 11'b0000_1001_000;

  // MMTM Rp, register list — Move Multiple to Memory. Per SPVU001A
  // page 12-111. Encoding `0000 1001 100R DDDD` (top11 =
  // 11'b00001001_100), where R=instr[4] is the register file and
  // DDDD=instr[3:0] is Rp's index. The second instruction word is a
  // 16-bit binary mask indicating which registers are in the list.
  // MMTM mask bit (15-N) selects R(N). Iteration order is lowest-order
  // register first per
  // the spec's "lowest order register is always saved first".
  // For each set bit, Rp is predecremented by 32 and the matching
  // register's 32-bit value is written. Final Rp is left pointing
  // at the lowest written address.
  localparam logic [10:0] MMTM_TOP11      = 11'b0000_1001_100;

  // MMFM Rp, register list — Move Multiple from Memory (the pop
  // counterpart of MMTM). Per SPVU001A page 12-109. Encoding
  // `0000 1001 101R DDDD` (top11 = 11'b00001001_101), with R=instr[4]
  // the register file and DDDD=instr[3:0] the Rp index. The second
  // instruction word uses MMFM's direct mapping: bit N selects R(N),
  // unlike MMTM's reverse mapping. Iteration order is
  // HIGHEST-order register first per the spec ("the highest order
  // register is always restored first"). For each set bit, a 32-bit
  // value is read from mem[Rp] into the register and Rp is then
  // post-incremented by 32 — including after the last register, so
  // final Rp = initial Rp + 32*count (points one word past the data).
  // All four status bits are Unaffected (simpler than MMTM).
  localparam logic [10:0] MMFM_TOP11      = 11'b0000_1001_101;

  // CALLA Address — Call Subroutine Absolute. Per SPVU001A page 12-48
  // + summary table line 27019. Single fixed opcode `0x0D5F` followed
  // by a 32-bit absolute target address. PC' is pushed; new PC = the
  // absolute address with bottom 4 bits forced to 0. Status unaffected.
  localparam instr_word_t CALLA_OPCODE  = 16'h0D5F;
  // CALLR Address — Call Subroutine Relative. Per SPVU001A page 12-49
  // + summary table line 27020. Single fixed opcode `0x0D3F` followed
  // by a 16-bit signed word-offset. PC' is pushed; new PC =
  // PC' + sign_extend(disp16) × 16. Status unaffected.
  localparam instr_word_t CALLR_OPCODE  = 16'h0D3F;

  // RETS [N] — Return from Subroutine. Per SPVU001A page 12-231 +
  // summary table line 27036. Encoding `0000 1001 011N NNNN`:
  //   top11 = 11'b00001001_011 = 0x04B
  //   bits[4:0] = N (5-bit unsigned arg-pop count, 0..31)
  // Semantics:
  //   PC <- mem[SP]   (32-bit pop)
  //   SP <- SP + 32 + 16*N
  // Status bits all "Unaffected". RETS without N = RETS 0.
  localparam logic [10:0] RETS_TOP11    = 11'b0000_1001_011;

  // DINT / EINT — single-fixed-encoding interrupt-enable control.
  // Per SPVU001A summary table (page A-14):
  //   DINT = 0x0360  (0000 0011 0110 0000) — clear ST.IE (bit 21)
  //   EINT = 0x0D60  (0000 1101 0110 0000) — set ST.IE
  // Status flag bits (N, C, Z, V) all unaffected. Implemented via
  // a full ST-write that reads current ST, modifies bit 21, writes back.
  localparam instr_word_t DINT_OPCODE = 16'h0360;
  localparam instr_word_t EINT_OPCODE = 16'h0D60;

  // EXGF Rd, F — Exchange Field Definition. Per SPVU001A page 12-77 +
  // summary table line 26954. Encoding `1101 01F1 000R DDDD`:
  //   bits[15:10] = 6'b110101  (= 0x35)
  //   bit[9]      = F  (selector: 0 = FS0/FE0; 1 = FS1/FE1)
  //   bit[8]      = 1  (constant)
  //   bits[7:5]   = 000 (sub-op)
  //   bit[4]      = R  (file)
  //   bits[3:0]   = Rd index
  // The instruction atomically swaps Rd's low 6 bits with the F-selected
  // FE:FS pair (1 + 5 bits) in ST. Rd's upper 26 bits are cleared.
  // All four status bits (N, C, Z, V) "Unaffected" per spec.
  localparam logic [5:0] EXGF_TOP6  = 6'b110101;

  // SETF FS, FE, F — Set Field Parameters. Per SPVU001A page 12-237 +
  // summary table line 26978. Encoding `0000 01F1 01FE FFFF`:
  //   bits[15:10] = 6'b000001  (top6 of the field-size-config family)
  //   bit[9]      = F  (selector: 0 = update FS0/FE0; 1 = update FS1/FE1)
  //   bit[8]      = 1  (constant — distinguishes from JUMP/family at bits[15:8]=0x01)
  //   bits[7:6]   = 2'b01 (SETF sub-op marker; SEXT/ZEXT use 00x here)
  //   bit[5]      = FE (new FE value)
  //   bits[4:0]   = FS (new FS value; 5'b00000 → field-size 32)
  // Status bits all "Unaffected" per spec.
  localparam logic [5:0] SETF_TOP6  = 6'b000001;

  // LMO Rs, Rd (Leftmost-One priority encoder). Per SPVU001A page 12-108
  // + summary table line 26955: encoding `0110 101S SSSR DDDD`
  // (top7 = 7'b0110_101 = 0x35). Rs and Rd same file; result =
  // 31 - bit_pos(leftmost-1 in Rs) in bottom 5 bits; Z = (Rs == 0).
  // N, C, V Unaffected — uses the wb_flag_mask added in Task 0037.
  localparam logic [6:0] LMO_RR_TOP7  = 7'b0110_101;

  // Shift Rs-form family. Per SPVU001A summary table page A-15:
  //   SLA Rs, Rd : 0110 000S SSSR DDDD   (top7 = 7'b0110_000)
  //   SLL Rs, Rd : 0110 001S SSSR DDDD   (top7 = 7'b0110_001)
  //   SRA Rs, Rd : 0110 010S SSSR DDDD   (top7 = 7'b0110_010)
  //   SRL Rs, Rd : 0110 011S SSSR DDDD   (top7 = 7'b0110_011)
  //   RL  Rs, Rd : 0110 100S SSSR DDDD   (top7 = 7'b0110_100)
  //
  // The shift amount comes from Rs[4:0]. Per pages 12-244/12-246, SRA/SRL
  // use the 2's complement of those five bits, so the core's amount mux
  // negates Rs[4:0] for the right-shift opcodes (A0019).
  localparam logic [6:0] SLA_RR_TOP7  = 7'b0110_000;
  localparam logic [6:0] SLL_RR_TOP7  = 7'b0110_001;
  localparam logic [6:0] SRA_RR_TOP7  = 7'b0110_010;
  localparam logic [6:0] SRL_RR_TOP7  = 7'b0110_011;
  localparam logic [6:0] RL_RR_TOP7   = 7'b0110_100;

  // JRcc short form: chart row "1100 code xxxx xxxx" with any cc.
  // bits[15:12] = 4'b1100; bits[11:8] = cc (4 bits); bits[7:0] = signed
  // 8-bit displacement. The two low-byte values 0x00 and 0x80 are reserved
  // (long-relative and absolute-form markers respectively).
  localparam logic [3:0] JRCC_TOP4 = 4'b1100;

  // NOP: single full encoding, no operand fields. Per SPVU001A page
  // 12-170 instruction-summary table: NOP = 0000 0011 0000 0000 = 0x0300
  // (A0021). Distinct from the unary family at 0000 0011 1xxx xxxx
  // (top9 = 9'b000000111); NOP's top9 = 9'b000000110, so no collision.
  localparam instr_word_t NOP_OPCODE = 16'h0300;
  localparam instr_word_t EMU_OPCODE = 16'h0100;

  // JUMP Rs (register-indirect jump). Per SPVU001A page 12-98 +
  // summary table line 13852: encoding `0000 0001 011R DDDD` —
  // top11 = 11'b00000001_011. Semantics: Rs → PC, with the bottom
  // 4 bits of PC forced to 0 (word alignment).
  localparam logic [10:0] JUMP_RS_TOP11 = 11'b0000_0001_011;

  // DSJ family: two-word instructions with a 16-bit signed
  // displacement following the opcode word. Per SPVU001A pages
  // 12-70..12-73 + summary table lines 27021-27023:
  //   DSJ   Rd, Address : 0000 1101 100R DDDD + 16-bit offset
  //   DSJEQ Rd, Address : 0000 1101 101R DDDD + 16-bit offset
  //   DSJNE Rd, Address : 0000 1101 110R DDDD + 16-bit offset
  // All three: status bits unaffected. Rd is decremented (when the
  // pre-condition holds), and the branch is taken iff the
  // post-decrement Rd is nonzero. Target math: PC' + offset*16,
  // where PC' is the address of the instruction following the
  // two-word DSJ — i.e., pc_value at CORE_WRITEBACK after both
  // FETCH/IMM_LO advances.
  localparam logic [10:0] DSJ_TOP11   = 11'b0000_1101_100;
  localparam logic [10:0] DSJEQ_TOP11 = 11'b0000_1101_101;
  localparam logic [10:0] DSJNE_TOP11 = 11'b0000_1101_110;

  // Status-register manipulation: single-fixed-encoding opcodes
  // CLRC / SETC (clear/set carry) and the two-register GETST / PUTST.
  // Per SPVU001A summary table (page A-14):
  //   CLRC  = 16'h0320  ("0000 0011 0010 0000")
  //   SETC  = 16'h0DE0  ("0000 1101 1110 0000")
  //   GETST Rd : top11 = 11'b00000001_100 = 0x00C  ⇒ base 0x0180
  //   PUTST Rs : top11 = 11'b00000001_101 = 0x00D  ⇒ base 0x01A0
  localparam instr_word_t CLRC_OPCODE   = 16'h0320;
  localparam instr_word_t SETC_OPCODE   = 16'h0DE0;
  localparam logic [10:0] GETST_TOP11   = 11'b0000_0001_100;
  localparam logic [10:0] PUTST_TOP11   = 11'b0000_0001_101;

  // GETPC, EXGPC, REV — small PC/revision register-context ops.
  // Per SPVU001A summary table page A-16:
  //   GETPC Rd  : 0000 0001 010R DDDD  (top11 = 11'b00000001_010 = 0x00A)
  //   EXGPC Rd  : 0000 0001 001R DDDD  (top11 = 11'b00000001_001 = 0x009)
  //   REV   Rd  : 0000 0000 001R DDDD  (top11 = 11'b00000000_001 = 0x001)
  // REV value: the format and worked example on page 12-233 produce
  // `REV A1 → 0x00000008` (A0025 resolved).
  localparam logic [10:0] GETPC_TOP11   = 11'b0000_0001_010;
  localparam logic [10:0] EXGPC_TOP11   = 11'b0000_0001_001;
  localparam logic [10:0] REV_TOP11     = 11'b0000_0000_001;

  // BTST K, Rd and BTST Rs, Rd (Test Register Bit). Per SPVU001A
  // pages 12-46/12-47 + summary table lines 26942/26943:
  //   BTST K, Rd  : 0001 11KK KKKR DDDD   (top6 = 6'b000111 = 0x07)
  //   BTST Rs, Rd : 0100 101S SSSR DDDD   (top7 = 7'b0100_101)
  // Both forms test a single bit of Rd (selected by K or by Rs[4:0])
  // and set Z = !bit; N, C, V are Unaffected per spec — the wb_flag_mask
  // blocks updates to those three flags.
  localparam logic [5:0] BTST_K_TOP6   = 6'b000111;
  localparam logic [6:0] BTST_RR_TOP7  = 7'b0100_101;

  // DSJS Rd, Address (Decrement-and-Skip-Jump Short — single-word).
  // Per SPVU001A page 12-74 + summary table line 13844:
  //   `0011 1Dxx xxxR DDDD` — top5 = 5'b00111.
  // Bit[10] = D (direction: 0=forward, 1=backward).
  // Bits[9:5] = 5-bit unsigned offset (words from PC').
  // Bit[4] = file (R). Bits[3:0] = Rd index.
  localparam logic [4:0]  DSJS_TOP5  = 5'b0011_1;

  // Reg-reg ops use bits[8:5] for Rs index.
  reg_idx_t rs_idx_from_instr;
  assign rs_idx_from_instr = instr[8:5];

  // Top-11 view of the encoding (for ADDI/SUBI/CMPI IW which share an
  // 11-bit prefix).
  logic [10:0] top11;
  assign top11 = instr[INSTR_WORD_WIDTH-1:5];

  // Reg-field decoders (5 bits = file + 4-bit index).
  reg_file_t reg_file_from_instr;
  reg_idx_t  reg_idx_from_instr;
  assign reg_file_from_instr = reg_file_t'(instr[4]);
  assign reg_idx_from_instr  = instr[3:0];

  always_comb begin
    // -----------------------------------------------------------------------
    // Safe defaults: report ILLEGAL with all execution metadata cleared.
    // -----------------------------------------------------------------------
    decoded.illegal         = 1'b1;
    decoded.illegal_trap    = architectural_illegal;
    decoded.iclass          = INSTR_ILLEGAL;
    decoded.rd_file         = REG_FILE_A;
    decoded.rd_idx          = '0;
    decoded.rs_idx          = '0;
    decoded.needs_imm16     = 1'b0;
    decoded.needs_imm32     = 1'b0;
    decoded.needs_imm64     = 1'b0;
    decoded.imm_sign_extend = 1'b0;
    decoded.alu_op          = ALU_OP_PASS_A;
    // rs_file defaults to the same file as rd_file (R bit). Only MOVE Rs,Rd
    // overrides it for cross-file moves; the core reads it only for MOVE_RR.
    decoded.rs_file         = reg_file_from_instr;
    decoded.move_mode       = MV_ADDR_NONE;   // only the indirect MOVE family sets this
    decoded.force_byte      = 1'b0;            // only MOVB sets this (force FS=8)
    decoded.force_pixel     = 1'b0;            // only PIXT sets this (force FS=PSIZE)
    decoded.xy_addr         = 1'b0;            // only XY-addressed PIXT sets this
    decoded.blt_src_xy      = 1'b0;            // PIXBLT source XY (XY,L / XY,XY)
    decoded.blt_dst_xy      = 1'b0;            // PIXBLT dest XY   (L,XY / XY,XY)
    decoded.blt_binary      = 1'b0;            // PIXBLT B (1-bit source color expand)
    decoded.shift_op        = SHIFT_OP_SLL;
    decoded.use_shifter     = 1'b0;
    decoded.k5              = '0;
    decoded.branch_cc       = '0;
    decoded.wb_reg_en       = 1'b0;
    decoded.wb_flags_en     = 1'b0;
    // Default flag-update mask = all-ones. Instructions that need
    // selective updates (BTST → Z only; ABS → all but C) override
    // this in their arms below.
    decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b1, z: 1'b1, v: 1'b1};
    decoded.needs_memory_op = 1'b0;

    // -----------------------------------------------------------------------
    // MOVI IW K, Rd
    // -----------------------------------------------------------------------
    if (top10 == MOVI_TOP10 && instr[5] == 1'b0) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVI_IW;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.needs_imm16     = 1'b1;
      decoded.needs_imm32     = 1'b0;
      decoded.imm_sign_extend = 1'b1;
      decoded.alu_op          = ALU_OP_PASS_B;   // pass the immediate through
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // -----------------------------------------------------------------------
    // MOVI IL K, Rd
    // -----------------------------------------------------------------------
    if (top10 == MOVI_TOP10 && instr[5] == 1'b1) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVI_IL;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.needs_imm16     = 1'b0;
      decoded.needs_imm32     = 1'b1;
      decoded.imm_sign_extend = 1'b0;   // 32-bit immediate is already full-width
      decoded.alu_op          = ALU_OP_PASS_B;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // -----------------------------------------------------------------------
    // MOVK K, Rd
    // -----------------------------------------------------------------------
    if (top6 == MOVK_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVK;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.k5              = instr[9:5];
      decoded.needs_imm16     = 1'b0;
      decoded.needs_imm32     = 1'b0;
      decoded.imm_sign_extend = 1'b0;
      decoded.alu_op          = ALU_OP_PASS_B;   // routed through ALU like MOVI
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;            // MOVK doesn't touch ST
    end

    // -----------------------------------------------------------------------
    // ADDK K, Rd  (K + Rd → Rd; encoded K=0 represents 32 per A0018)
    // -----------------------------------------------------------------------
    if (top6 == ADDK_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_ADDK;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.k5              = instr[9:5];
      decoded.alu_op          = ALU_OP_ADD;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // SUBK K, Rd  (Rd - K → Rd; encoded K=0 represents 32 per A0018)
    // -----------------------------------------------------------------------
    if (top6 == SUBK_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_SUBK;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.k5              = instr[9:5];
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // IW-form immediate arithmetic (ADDI, SUBI, CMPI)
    //
    // All three share the encoding shape:
    //   bits[15:5] = 11-bit prefix selecting the op
    //   bit[4]     = R   (file)
    //   bits[3:0]  = Rd index
    //   next word  = 16-bit extension (sign-extended to 32 bits)
    //
    // SUBI and CMPI extensions are the one's complement of the source-level
    // immediate; the execute operand mux complements them back after
    // extension. CMPI does not write Rd, matching CMP Rs,Rd.
    // -----------------------------------------------------------------------
    if (top11 == ADDI_IW_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_ADDI_IW;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.needs_imm16     = 1'b1;
      decoded.imm_sign_extend = 1'b1;
      decoded.alu_op          = ALU_OP_ADD;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    if (top11 == SUBI_IW_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_SUBI_IW;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.needs_imm16     = 1'b1;
      decoded.imm_sign_extend = 1'b1;
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    if (top11 == CMPI_IW_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_CMPI_IW;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.needs_imm16     = 1'b1;
      decoded.imm_sign_extend = 1'b1;
      decoded.alu_op          = ALU_OP_CMP;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // IL-form immediate family (32-bit immediate)
    //
    //   ADDI IL  =  0000 1011 001R DDDD + 32-bit imm
    //   SUBI IL  =  0000 1101 000R DDDD + 32-bit imm   (different base prefix!)
    //   CMPI IL  =  0000 1011 011R DDDD + 32-bit imm   (wb_reg_en = 0)
    //   ANDI/ANDNI IL = 0000 1011 100R DDDD + 32-bit extension
    //   ORI  IL  =  0000 1011 101R DDDD + 32-bit imm
    //   XORI IL  =  0000 1011 110R DDDD + 32-bit imm
    //
    // All reuse MOVI IL's CORE_FETCH_IMM_LO/HI path (needs_imm32=1,
    // imm_sign_extend=0). The 32-bit extension is already full-width. SUBI
    // and CMPI carry the one's complement of their source-level immediate.
    // -----------------------------------------------------------------------
    if (top11 == ADDI_IL_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_ADDI_IL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.needs_imm32 = 1'b1;
      decoded.alu_op      = ALU_OP_ADD;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
    end
    if (top11 == SUBI_IL_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SUBI_IL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.needs_imm32 = 1'b1;
      decoded.alu_op      = ALU_OP_SUB;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
    end
    if (top11 == CMPI_IL_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_CMPI_IL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.needs_imm32 = 1'b1;
      decoded.alu_op      = ALU_OP_CMP;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b1;
    end
    if (top11 == ANDI_IL_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_ANDI_IL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.needs_imm32 = 1'b1;
      // ANDI and ANDNI share this hardware opcode. The extension words are
      // complemented by the assembler for source-level ANDI and direct for
      // ANDNI; silicon always computes Rd & ~extension.
      decoded.alu_op      = ALU_OP_ANDN;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end
    if (top11 == ORI_IL_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_ORI_IL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.needs_imm32 = 1'b1;
      decoded.alu_op      = ALU_OP_OR;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end
    if (top11 == XORI_IL_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_XORI_IL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.needs_imm32 = 1'b1;
      decoded.alu_op      = ALU_OP_XOR;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // MOVE Rs, Rd  (register-to-register; cross-file capable)
    //
    // Encoding `0100 11MS SSSR DDDD` (base 0x4C00; corrected in Task 0058 —
    // see the localparam comment and assumptions.md A0020). Full 32-bit
    // copy (not a field move; no F bit). Status per SPVU001A 12-126:
    // N = data[31], Z = (data==0), V = 0, C Unaffected.
    // -----------------------------------------------------------------------
    if (top6 == MOVE_RR_TOP6) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_MOVE_RR;
      // M=instr[9], R=instr[4]. Source file is always R; destination file
      // is R when M=0 (same file) or the opposite file when M=1 (cross).
      decoded.rs_file     = reg_file_from_instr;                      // = R
      decoded.rd_file     = instr[9] ? reg_file_t'(~instr[4])         // M=1: ~R
                                     : reg_file_from_instr;           // M=0:  R
      decoded.rd_idx      = reg_idx_from_instr;                       // Rd = instr[3:0]
      decoded.rs_idx      = rs_idx_from_instr;                        // Rs = instr[8:5]
      decoded.alu_op      = ALU_OP_PASS_A;        // alu_a = rf_rs1_data (= Rs)
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      // Per SPVU001A page 12-126: N = data[31], Z = (data==0), V = 0,
      // C Unaffected. PASS_A already yields n/z/v correctly (v defaults 0);
      // mask C off so the move doesn't clobber the carry flag.
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // -----------------------------------------------------------------------
    // MOVX / MOVY — move the X (low 16) or Y (high 16) half of Rs into the
    // same half of Rd; the other half of Rd is unchanged. Same-file reg-reg
    // (top7 = 1110_110 / 1110_111). All flags Unaffected. The core composes
    // the result from Rs and the old Rd in the rf_wr_data mux. (Task 0065.)
    // -----------------------------------------------------------------------
    if (top7 == MOVX_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_MOVX;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;     // Rd (instr[3:0])
      decoded.rs_idx      = rs_idx_from_instr;      // Rs (instr[8:5])
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b0;
    end

    if (top7 == MOVY_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_MOVY;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b0;
    end

    // CVXYL — convert XY address (Rs) to linear address (Rd). The core's
    // CVXYL datapath reads OFFSET (B4) on read port 3 and CONVDP/PSIZE from
    // the I/O registers; result -> Rd. All status bits Unaffected.
    if (top7 == CVXYL_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_CVXYL;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;     // Rd (instr[3:0])
      decoded.rs_idx      = rs_idx_from_instr;      // Rs (instr[8:5]) = XY value
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b0;
    end

    // -----------------------------------------------------------------------
    // ADDXY / SUBXY — dual independent 16-bit add/sub on the X and Y halves.
    // Same-file reg-reg; all four status bits set per the XY datapath in the
    // core (full update, default wb_flag_mask). (Task 0066.)
    // -----------------------------------------------------------------------
    if (top7 == ADDXY_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_ADDXY;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;     // Rd (dest + source)
      decoded.rs_idx      = rs_idx_from_instr;      // Rs
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
    end

    if (top7 == SUBXY_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SUBXY;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
    end

    // DRAV Rs,Rd — Draw and Advance (SPVU001A page 12-67). Writes COLOR1 (B9)
    // to the pixel at Rd's XY address (PSIZE-bit, with PPOP/T/PMASK), then
    // advances Rd by Rs as an XY add (X+X, Y+Y, no carry between halves —
    // reuses the ADDXY datapath). Rd is written back; no flags for W=0 window
    // mode. Same-file reg-reg layout as ADDXY. The core applies the Task 0112
    // per-pixel window behavior when CONTROL.W is nonzero.
    if (top7 == DRAV_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_DRAV;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;     // Rd (XY dest + advanced)
      decoded.rs_idx          = rs_idx_from_instr;      // Rs (XY increment)
      decoded.wb_reg_en       = 1'b1;                   // Rd <- advanced XY
      decoded.needs_memory_op = 1'b1;                   // RMW pixel write
    end

    // LINE Z — Bresenham inner loop (SPVU001A page 12-99). Encoding
    // `1101 1111 Z001 1010` = 0xDF1A (Z=0) / 0xDF9A (Z=1); the Z bit (bit 7)
    // selects the d=0 treatment and is read directly from instr_word_q[7] in
    // the core. All operands are implied B-file registers — no rs/rd. The
    // engine writes back d(B0)/DADDR(B2)/COUNT(B10) itself, so wb_reg_en=0.
    if ((instr[15:8] == 8'hDF) && (instr[6:0] == 7'h1A)) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_LINE;
      decoded.wb_reg_en       = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // CMPXY: nondestructive — sets flags only, Rd unchanged (wb_reg_en=0).
    if (top7 == CMPXY_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_CMPXY;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b1;
    end

    // MPYS / MPYU — 32x32 multiply (FS1=32 scope). Rd is multiplicand and
    // destination; for even Rd the 64-bit result also lands in Rd+1. The
    // core latches the product and writes it back over 1 (odd Rd) or 2
    // (even Rd) cycles. MPYS sets N+Z; MPYU sets Z only.
    if (top7 == MPYS_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_MPYS;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;     // Rd (multiplicand + dest)
      decoded.rs_idx       = rs_idx_from_instr;      // Rs (multiplier)
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    if (top7 == MPYU_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_MPYU;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};  // Z only
    end

    // DIVU — unsigned divide (multi-cycle). Z + V (N Unaffected). The core
    // runs the divider in CORE_DIVIDE, then writes the quotient (and, for
    // even Rd, the remainder to Rd+1) at WRITEBACK.
    if (top7 == DIVU_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_DIVU;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b1};  // Z, V
    end

    // MODU — unsigned modulo (multi-cycle, reuses the divider). The
    // remainder is written to Rd. Z + V; the core masks Z off when Rs=0
    // (overflow) so it is "Unaffected" per SPVU001A page 12-113.
    if (top7 == MODU_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_MODU;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b1};  // Z, V
    end

    // DIVS — signed divide. Like DIVU but N is also defined by the quotient
    // sign and the instruction page's 0x80000000 result rule.
    if (top7 == DIVS_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_DIVS;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};  // N, Z, V
    end

    // MODS — signed modulo. The result is the signed remainder, but unlike
    // DIVS the instruction page defines N as Unaffected. Z is updated from
    // the remainder except when Rs=0; V reports quotient overflow.
    if (top7 == MODS_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_MODS;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b1};  // Z, V
    end

    // CPW: write the out-of-window code to Rd; set only V (point outside
    // window). The core reads WSTART (B5) / WEND (B6) via read ports 2/3.
    if (top7 == CPW_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_CPW;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b0, v: 1'b1};  // V only
    end

    // -----------------------------------------------------------------------
    // MOVE Rs, *Rd  — store field from Rs to mem[*Rd]  (SPVU001A 12-127)
    //
    // Rd holds the destination BIT address (a pointer, NOT written back);
    // Rs supplies the data. The transfer is a single memory write. All
    // status bits Unaffected. Task 0059: field-size-32, word-aligned only.
    // -----------------------------------------------------------------------
    if (top6 == MOVE_STORE_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_STORE;
      decoded.rd_file         = reg_file_from_instr;   // R bit (Rs & Rd same file)
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = pointer (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = data    (instr[8:5])
      decoded.wb_reg_en       = 1'b0;                  // pointer unchanged; no writeback
      decoded.wb_flags_en     = 1'b0;                  // all flags Unaffected
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MOVE *Rs, Rd  — load field from mem[*Rs] into Rd  (SPVU001A 12-135)
    //
    // Rs holds the source BIT address (a pointer); Rd receives the loaded
    // data. Single memory read. Implicit compare-to-0: N = data[31],
    // Z = (data==0), V = 0, C Unaffected. Task 0059: field-size-32,
    // word-aligned only (so sign/zero extension is a no-op).
    // -----------------------------------------------------------------------
    if (top6 == MOVE_LOAD_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_LOAD;
      decoded.rd_file         = reg_file_from_instr;   // R bit (Rs & Rd same file)
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = destination (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pointer     (instr[8:5])
      decoded.wb_reg_en       = 1'b1;                  // Rd <- loaded data
      decoded.wb_flags_en     = 1'b1;
      // N/Z/V update, C Unaffected (the loaded data drives the flags via
      // the flag_input mux in the core, not the ALU).
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MOVE Rs,*Rd(off) / *Rs(off),Rd  — register-indirect with signed 16-bit
    // offset (Task 0064). 2-word: opcode + offset. Effective bit-address =
    // pointer + sign_extend(offset16); the pointer register is unchanged.
    // SPVU001A 12-132 (store) / 12-147 (load). Rs=instr[8:5], Rd=instr[3:0].
    // Store: data=Rs, pointer=Rd, all flags Unaffected. Load: pointer=Rs,
    // dest=Rd, implicit compare-to-0 (N/Z). Field-size-32, word-aligned.
    // -----------------------------------------------------------------------
    if (top6 == MOVE_OFF_STORE_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_OFF_STORE;
      decoded.rd_file         = reg_file_from_instr;   // Rs & Rd same file
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = pointer (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = data    (instr[8:5])
      decoded.needs_imm16     = 1'b1;                  // fetch the 16-bit offset
      decoded.imm_sign_extend = 1'b1;                  // signed offset
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;                  // pointer unchanged; no writeback
      decoded.wb_flags_en     = 1'b0;                  // all flags Unaffected
    end

    if (top6 == MOVE_OFF_LOAD_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_OFF_LOAD;
      decoded.rd_file         = reg_file_from_instr;   // Rs & Rd same file
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = destination (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pointer     (instr[8:5])
      decoded.needs_imm16     = 1'b1;                  // fetch the 16-bit offset
      decoded.imm_sign_extend = 1'b1;                  // signed offset
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b1;                  // Rd <- loaded data
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // -----------------------------------------------------------------------
    // MOVE *Rs(offset),*Rd+ — signed-offset source to postincrement
    // destination (1988 User's Guide page 12-149).
    //
    // Read at Rs + sext(offset16), write through the original Rd, leave Rs
    // unchanged, then advance Rd by the F-selected field size. This is a
    // memory-to-memory transfer, so FE is irrelevant and all flags are
    // Unaffected.
    // -----------------------------------------------------------------------
    if (top6 == MOVE_OFF_M2M_PI_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_OFF_M2M_PI;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.needs_imm16     = 1'b1;
      decoded.imm_sign_extend = 1'b1;
      decoded.move_mode       = MV_ADDR_POSTINC;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MOVE @SAddress,*Rd+ — absolute source to postincrement destination
    // (1988 User's Guide page 12-155).
    //
    // The two following words hold the 32-bit source bit address, low word
    // first. Read one F-selected field there, write through the original Rd,
    // then advance Rd by FS. FE and all status bits are irrelevant.
    // -----------------------------------------------------------------------
    if (top6 == MOVE_ABS_M2M_PI_TOP6 && instr[8:5] == 4'b0000) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_ABS_M2M_PI;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.needs_imm32     = 1'b1;
      decoded.move_mode       = MV_ADDR_POSTINC;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MOVE *Rs, *Rd  — indirect-to-indirect field move  (SPVU001A 12-137)
    //
    // Read the field at mem[*Rs], write it to mem[*Rd]. Both Rs and Rd are
    // bit-address pointers (unchanged for the plain form). Two memory
    // transactions: read then write. All status bits Unaffected. Task 0061
    // implements the plain form only (no inc/dec), field-size-32,
    // word-aligned. The inc/dec forms (*Rs+,*Rd+ / -*Rs,-*Rd) are deferred.
    // -----------------------------------------------------------------------
    if (top6 == MOVE_M2M_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_M2M;
      decoded.rd_file         = reg_file_from_instr;   // Rs & Rd same file
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest pointer (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = src  pointer (instr[8:5])
      decoded.wb_reg_en       = 1'b0;                  // pointers unchanged; no writeback
      decoded.wb_flags_en     = 1'b0;                  // all flags Unaffected
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MOVE *Rs+,*Rd+ / -*Rs,-*Rd  — indirect-to-indirect with auto inc/dec
    // (Task 0062). Same two-transaction read-then-write as the plain form,
    // plus BOTH pointers step by ±32. The source pointer (Rs) is updated in
    // the core during CORE_MEMORY (after the read); the destination pointer
    // (Rd) is updated at WRITEBACK (hence wb_reg_en=1 here). FS=32,
    // word-aligned. SPVU001A 12-138 (postinc Rs==Rd: data is written to the
    // incremented location — handled in the core).
    // -----------------------------------------------------------------------
    if (top6 == MOVE_M2M_POSTINC_TOP6 || top6 == MOVE_M2M_PREDEC_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_M2M;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = src  pointer
      decoded.move_mode       = (top6 == MOVE_M2M_PREDEC_TOP6)
                              ? MV_ADDR_PREDEC : MV_ADDR_POSTINC;
      decoded.wb_reg_en       = 1'b1;                  // write Rd (pointer) back
      decoded.wb_flags_en     = 1'b0;                  // all flags Unaffected
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MOVE indirect with auto inc/dec (Task 0060). Same field semantics as
    // the plain forms above; the pointer register also steps by the field
    // size (±32 at FS=32). For STORES the pointer (Rd) is written back at
    // WRITEBACK (so wb_reg_en=1 here, unlike the plain store). For LOADS
    // the data still goes to Rd and the pointer (Rs) is updated via a
    // separate CORE_MEMORY-time write in the core. Task 0060 scope is
    // field-size-32, word-aligned (same limitation as Task 0059).
    // -----------------------------------------------------------------------
    if (top6 == MOVE_STORE_POSTINC_TOP6 || top6 == MOVE_STORE_PREDEC_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_STORE;
      decoded.rd_file         = reg_file_from_instr;   // Rs & Rd same file
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = pointer (written back)
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = data
      decoded.move_mode       = (top6 == MOVE_STORE_PREDEC_TOP6)
                              ? MV_ADDR_PREDEC : MV_ADDR_POSTINC;
      decoded.wb_reg_en       = 1'b1;                  // write updated pointer to Rd
      decoded.wb_flags_en     = 1'b0;                  // all flags Unaffected
      decoded.needs_memory_op = 1'b1;
    end

    if (top6 == MOVE_LOAD_POSTINC_TOP6 || top6 == MOVE_LOAD_PREDEC_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_LOAD;
      decoded.rd_file         = reg_file_from_instr;   // Rs & Rd same file
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = destination (data)
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pointer (updated)
      decoded.move_mode       = (top6 == MOVE_LOAD_PREDEC_TOP6)
                              ? MV_ADDR_PREDEC : MV_ADDR_POSTINC;
      decoded.wb_reg_en       = 1'b1;                  // Rd <- loaded data at WRITEBACK
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // K-form shift instructions
    //
    // All five share the encoding shape:
    //   bits[15:10] = 6-bit shift-op prefix (table below)
    //   bits[9:5]   = K for SLA/SLL/RL; 2's complement of K for SRA/SRL.
    //                  Every architectural count is 0..31, including zero.
    //   bit[4]      = R (file)
    //   bits[3:0]   = Rd index
    //
    //   SLA K, Rd  →  6'b001000   (shift left arithmetic)
    //   SLL K, Rd  →  6'b001001   (shift left logical)
    //   SRA K, Rd  →  6'b001010   (shift right arithmetic / sign-extend)
    //   SRL K, Rd  →  6'b001011   (shift right logical)
    //   RL  K, Rd  →  6'b001100   (rotate left)
    // -----------------------------------------------------------------------
    if (top6 == SLA_K_TOP6) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SLA_K;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = instr[9:5];
      decoded.shift_op    = SHIFT_OP_SLA;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b1, z: 1'b1, v: 1'b1};
    end
    if (top6 == SLL_K_TOP6) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SLL_K;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = instr[9:5];
      decoded.shift_op    = SHIFT_OP_SLL;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b1, v: 1'b0};
    end
    if (top6 == SRA_K_TOP6) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SRA_K;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = (~instr[9:5]) + 5'd1;
      decoded.shift_op    = SHIFT_OP_SRA;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b1, z: 1'b1, v: 1'b0};
    end
    if (top6 == SRL_K_TOP6) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SRL_K;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = (~instr[9:5]) + 5'd1;
      decoded.shift_op    = SHIFT_OP_SRL;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b1, v: 1'b0};
    end
    if (top6 == RL_K_TOP6) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_RL_K;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = instr[9:5];
      decoded.shift_op    = SHIFT_OP_RL;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // Single-register unary family
    //   ABS  Rd   bits[6:5] = 00  (deferred — V-on-MIN_INT spec subtlety)
    //   NEG  Rd   bits[6:5] = 01  (implemented; ALU_OP_NEG)
    //   NEGB Rd   bits[6:5] = 10  (deferred — needs C-input handling)
    //   NOT  Rd   bits[6:5] = 11  (implemented; ALU_OP_NOT)
    // -----------------------------------------------------------------------
    if (instr[15:7] == UNARY_TOP9) begin
      case (instr[6:5])
        2'b00: begin   // ABS: spec says C is "Unaffected" — mask blocks C.
          decoded.illegal         = 1'b0;
          decoded.iclass          = INSTR_ABS;
          decoded.rd_file         = reg_file_from_instr;
          decoded.rd_idx          = reg_idx_from_instr;
          decoded.alu_op          = ALU_OP_ABS;
          decoded.wb_reg_en       = 1'b1;
          decoded.wb_flags_en     = 1'b1;
          decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
        end
        2'b01: begin   // NEG
          decoded.illegal         = 1'b0;
          decoded.iclass          = INSTR_NEG;
          decoded.rd_file         = reg_file_from_instr;
          decoded.rd_idx          = reg_idx_from_instr;
          decoded.alu_op          = ALU_OP_NEG;
          decoded.wb_reg_en       = 1'b1;
          decoded.wb_flags_en     = 1'b1;
        end
        2'b10: begin   // NEGB  (Rd ← 0 - Rd - C; uses SUBB with a=0)
          decoded.illegal         = 1'b0;
          decoded.iclass          = INSTR_NEGB;
          decoded.rd_file         = reg_file_from_instr;
          decoded.rd_idx          = reg_idx_from_instr;
          decoded.alu_op          = ALU_OP_SUBB;
          decoded.wb_reg_en       = 1'b1;
          decoded.wb_flags_en     = 1'b1;
        end
        2'b11: begin   // NOT
          decoded.illegal         = 1'b0;
          decoded.iclass          = INSTR_NOT;
          decoded.rd_file         = reg_file_from_instr;
          decoded.rd_idx          = reg_idx_from_instr;
          decoded.alu_op          = ALU_OP_NOT;
          decoded.wb_reg_en       = 1'b1;
          decoded.wb_flags_en     = 1'b1;
          decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
        end
        // The 2-bit selector is fully enumerated above; the default keeps
        // the per-instruction defaults (set at the top of this always_comb)
        // and satisfies the "every case has a default" rule
        // (hdl-coding-guidelines 90 AP4/AP13, 14 §3.5).
        default: ;
      endcase
    end

    // -----------------------------------------------------------------------
    // Shift Rs-form family (SLA / SLL / SRA / SRL / RL Rs, Rd)
    //
    // Shift amount comes from rf_rs1_data[4:0]; the core's
    // shifter-amount mux applies the 2's-complement negation for
    // SRA/SRL per pages 12-244/12-246 (A0019).
    // -----------------------------------------------------------------------
    if (top7 == SLA_RR_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SLA_RR;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.shift_op    = SHIFT_OP_SLA;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b1, z: 1'b1, v: 1'b1};
    end
    if (top7 == SLL_RR_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SLL_RR;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.shift_op    = SHIFT_OP_SLL;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b1, v: 1'b0};
    end
    if (top7 == SRA_RR_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SRA_RR;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.shift_op    = SHIFT_OP_SRA;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b1, z: 1'b1, v: 1'b0};
    end
    if (top7 == SRL_RR_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_SRL_RR;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.shift_op    = SHIFT_OP_SRL;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b1, v: 1'b0};
    end
    if (top7 == RL_RR_TOP7) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_RL_RR;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.rs_idx      = rs_idx_from_instr;
      decoded.shift_op    = SHIFT_OP_RL;
      decoded.use_shifter = 1'b1;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // LMO Rs, Rd  (Leftmost-One priority encoder)
    //
    // Rd <- 31 - bit_pos(leftmost-1 in Rs)  (5 bits; upper 27 bits = 0)
    //   if Rs == 0: Rd <- 0; Z <- 1
    //   else:        Z <- 0
    // N, C, V "Unaffected" per SPVU001A page 12-108 — wb_flag_mask
    // gates them out.
    // -----------------------------------------------------------------------
    if (top7 == LMO_RR_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_LMO_RR;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // SETF FS, FE, F : update the field-size pair selected by F bit
    // (instr[9]).  Status bits unaffected. The core constructs the new
    // ST value by reading st_value, modifying the F-selected FS/FE
    // bits, and writing back via st_write_en. Operand extraction
    // happens in the core directly from instr_word_q (F, FE, FS).
    // -----------------------------------------------------------------------
    if (instr[15:10] == SETF_TOP6 && instr[8] && (instr[7:6] == 2'b01)) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_SETF;
      decoded.wb_reg_en    = 1'b0;
      decoded.wb_flags_en  = 1'b0;
    end

    // -----------------------------------------------------------------------
    // SEXT Rd, F : sign-extend the low FS<F> bits of Rd to 32 bits.
    //   Encoding: bits[15:10]=6'b000001, bit[9]=F, bit[8]=1,
    //             bits[7:5]=3'b000, bit[4]=R, bits[3:0]=Rd.
    //   Flags: N from result, Z from result; C and V "Unaffected".
    //
    // ZEXT Rd, F : zero-extend the low FS<F> bits of Rd to 32 bits.
    //   Same encoding shape but bits[7:5]=3'b001.
    //   Flags: Z only; N, C, V "Unaffected".
    //
    // F is read directly from instr_word_q[9] in the core; the SEXT/
    // ZEXT datapath there reads the F-selected FS field from st_value
    // and constructs the appropriate mask + extension.
    // -----------------------------------------------------------------------
    if (instr[15:10] == SETF_TOP6 && instr[8] && (instr[7:5] == 3'b000)) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_SEXT;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b0};
    end
    if (instr[15:10] == SETF_TOP6 && instr[8] && (instr[7:5] == 3'b001)) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_ZEXT;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // MOVE with absolute (32-bit) address. Same `0000 01F1` field-op family
    // as SETF/SEXT/ZEXT, with the sub-op in bits[7:5]:
    //   100 = MOVE Rs,@DAddr (store)   — SPVU001A 12-134
    //   101 = MOVE @SAddr,Rd (load)    — SPVU001A 12-153
    // The 32-bit absolute bit-address is the two words after the opcode
    // (LSBs first), fetched via the imm32 path. F (bit[9]) selects FS0/FE0
    // vs FS1/FE1; Task 0063 implements field-size-32, word-aligned only.
    // The register operand (Rs for store / Rd for load) is at instr[3:0].
    // -----------------------------------------------------------------------
    if (instr[15:10] == SETF_TOP6 && instr[8] && (instr[7:5] == 3'b100)) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_ABS_STORE;
      decoded.rd_file         = reg_file_from_instr;   // R bit
      decoded.rs_idx          = reg_idx_from_instr;    // Rs = data (instr[3:0])
      decoded.needs_imm32     = 1'b1;                  // fetch 32-bit DAddress
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;                  // no register writeback
      decoded.wb_flags_en     = 1'b0;                  // all flags Unaffected
    end

    if (instr[15:10] == SETF_TOP6 && instr[8] && (instr[7:5] == 3'b101)) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_ABS_LOAD;
      decoded.rd_file         = reg_file_from_instr;   // R bit
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest (instr[3:0])
      decoded.needs_imm32     = 1'b1;                  // fetch 32-bit SAddress
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b1;                  // Rd <- loaded data
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // -----------------------------------------------------------------------
    // MOVB (move byte, FS forced to 8) — Task 0080. Seven forms reuse the
    // MOVE field/offset/absolute datapaths with decoded.force_byte = 1, which
    // makes the core drive mem_size = 8 and (for loads) sign-extend the byte.
    // MOVB has no auto inc/dec, so move_mode stays MV_ADDR_NONE. Loads set
    // N/Z from the sign-extended byte; stores leave all flags Unaffected.
    // -----------------------------------------------------------------------
    if (top7 == MOVB_STORE_TOP7) begin            // MOVB Rs,*Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_STORE;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = pointer (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = data    (instr[8:5])
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == MOVB_LOAD_TOP7) begin             // MOVB *Rs,Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_LOAD;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest    (instr[3:0])
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pointer (instr[8:5])
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == MOVB_M2M_TOP7) begin              // MOVB *Rs,*Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_M2M;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = src  pointer
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == MOVB_OFF_STORE_TOP7) begin        // MOVB Rs,*Rd(off)
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_OFF_STORE;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = data
      decoded.needs_imm16     = 1'b1;
      decoded.imm_sign_extend = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
    end

    if (top7 == MOVB_OFF_LOAD_TOP7) begin         // MOVB *Rs(off),Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_OFF_LOAD;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pointer
      decoded.needs_imm16     = 1'b1;
      decoded.imm_sign_extend = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // MOVB *Rs(SOffset),*Rd(DOffset): opcode, signed source offset, signed
    // destination offset. The two operand words use the existing two-word
    // immediate fetch path but are interpreted independently by the core.
    if (top7 == MOVB_OFF_M2M_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVB_OFF_M2M;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.needs_imm32     = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
    end

    // MOVE *Rs(SOffset),*Rd(DOffset)[,F]: opcode followed by independent
    // signed source and destination offsets. F selects the architectural
    // field definition; neither pointer nor status is modified.
    if (top6 == MOVE_OFF_M2M_TOP6) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_OFF_M2M;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.needs_imm32     = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
    end

    // MOVB @SAddress,@DAddress: fixed opcode followed by source low/high,
    // then destination low/high. No register operand or writeback.
    if (instr == MOVB_ABS_M2M_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVB_ABS_M2M;
      decoded.force_byte      = 1'b1;
      decoded.needs_imm64     = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
    end

    // MOVE @SAddress,@DAddress[,F]: source low/high then destination
    // low/high. F chooses FS0/FS1. Registers and status are unaffected.
    if ((instr & MOVE_ABS_M2M_MASK) == MOVE_ABS_M2M_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_ABS_M2M;
      decoded.needs_imm64     = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
    end

    // MOVB absolute forms: 0000 01.. family, sub-op instr[7:5]=111,
    // store (bit9=0) / load (bit9=1).
    if (instr[15:10] == SETF_TOP6 && !instr[9] && instr[8] && (instr[7:5] == 3'b111)) begin
      decoded.illegal         = 1'b0;             // MOVB Rs,@DAddr (0x05E0)
      decoded.iclass          = INSTR_MOVE_ABS_STORE;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rs_idx          = reg_idx_from_instr;    // Rs = data (instr[3:0])
      decoded.needs_imm32     = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
    end

    if (instr[15:10] == SETF_TOP6 && instr[9] && instr[8] && (instr[7:5] == 3'b111)) begin
      decoded.illegal         = 1'b0;             // MOVB @SAddr,Rd (0x07E0)
      decoded.iclass          = INSTR_MOVE_ABS_LOAD;
      decoded.force_byte      = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest (instr[3:0])
      decoded.needs_imm32     = 1'b1;
      decoded.needs_memory_op = 1'b1;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b1, v: 1'b1};
    end

    // -----------------------------------------------------------------------
    // PIXT (pixel transfer, LINEAR forms) — Tasks 0083 and 0089–0092. A
    // pixel-size field
    // move: force_pixel makes the core use FS = PSIZE and zero-extend loads.
    // Reuses the MOVE field store/load/M2M datapaths. Store processing
    // (PPOP, transparency, and PMASK) is applied downstream in the core.
    //   PIXT Rs,*Rd  (0xF800): store — all flags Unaffected.
    //   PIXT *Rs,Rd  (0xFA00): load  — V = (pixel != 0); N/C/Z Undefined
    //                                  (masked off here).
    //   PIXT *Rs,*Rd (0xFC00): M2M   — all flags Unaffected.
    // -----------------------------------------------------------------------
    if (top7 == PIXT_STORE_TOP7) begin           // PIXT Rs,*Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_STORE;
      decoded.force_pixel     = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = linear pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pixel data
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == PIXT_LOAD_TOP7) begin            // PIXT *Rs,Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_LOAD;
      decoded.force_pixel     = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = destination
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = linear pointer
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      // Only V is defined (V = pixel != 0); N/C/Z are Undefined -> masked off.
      decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b0, v: 1'b1};
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == PIXT_M2M_TOP7) begin             // PIXT *Rs,*Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_M2M;
      decoded.force_pixel     = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = dest pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = src  pointer
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // XY-addressed PIXT (Task 0085): the pointer register holds an XY value;
    // the core's xy_addr path converts it to linear (CONVDP for the store's
    // dest pointer, CONVSP for the load's source pointer) before the pixel
    // field access. force_pixel ⇒ FS = PSIZE; same flag rules as linear PIXT.
    if (top7 == PIXT_XY_STORE_TOP7) begin        // PIXT Rs,*Rd.XY
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_STORE;
      decoded.force_pixel     = 1'b1;
      decoded.xy_addr         = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = XY dest pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = pixel data
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == PIXT_XY_LOAD_TOP7) begin         // PIXT *Rs.XY,Rd
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_LOAD;
      decoded.force_pixel     = 1'b1;
      decoded.xy_addr         = 1'b1;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = destination
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = XY source pointer
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b0, v: 1'b1};
      decoded.needs_memory_op = 1'b1;
    end

    if (top7 == PIXT_XY_M2M_TOP7) begin          // PIXT *Rs.XY,*Rd.XY
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MOVE_FIELD_M2M;
      decoded.force_pixel     = 1'b1;
      decoded.xy_addr         = 1'b1;               // src via CONVSP, dst via CONVDP
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;    // Rd = XY dest pointer
      decoded.rs_idx          = rs_idx_from_instr;     // Rs = XY source pointer
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // EXGF Rd, F : atomic swap Rd[5:0] ↔ FE<F>:FS<F> in ST. Rd's
    // upper 26 bits are cleared. Status unaffected.
    //
    // The core handles both halves of the swap in one CORE_WRITEBACK
    // cycle: rf_wr_data delivers the new Rd (= {26'b0, FE_old, FS_old}),
    // and st_write_en+st_write_data deliver the new ST (= current ST
    // with the F-selected slot replaced by Rd_old[5:0]).
    // -----------------------------------------------------------------------
    if (instr[15:10] == EXGF_TOP6 && instr[8] && (instr[7:5] == 3'b000)) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_EXGF;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b0;
    end

    // -----------------------------------------------------------------------
    // DINT : clear ST.IE (disable interrupts). Single fixed encoding 0x0360.
    // EINT : set   ST.IE (enable  interrupts). Single fixed encoding 0x0D60.
    // Status bits N, C, Z, V all unaffected. The core constructs the new
    // ST value by reading current st_value, clearing or setting bit 21,
    // and using the existing full-ST-write path.
    // -----------------------------------------------------------------------
    if (instr == DINT_OPCODE) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_DINT;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end
    if (instr == EINT_OPCODE) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_EINT;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // -----------------------------------------------------------------------
    // PUSHST — push ST onto the stack. Per SPVU001A summary table:
    //   SP <- SP - 32; mem[SP] <- ST  (32-bit write).
    // Single fixed encoding 0x01E0. Status bits Unaffected.
    //
    // ALU computes (SP - 32): alu_a = rf_rs1_data (reads A15 = SP via
    // the regfile's SP-alias); alu_b = 32 via a new core-side mux
    // entry; alu_op = SUB. Then the core fires a 32-bit memory write
    // to alu_result with mem_wdata = st_value; finally WRITEBACK
    // updates SP (rd_idx=15) with alu_result.
    // -----------------------------------------------------------------------
    if (instr == PUSHST_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PUSHST;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;     // write back to SP (= A15)
      decoded.rs_idx          = REG_SP_IDX;     // read SP via rs1 port
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // POPST — pop ST from stack. Per SPVU001A summary table:
    //   ST <- mem[SP]    (32-bit read)
    //   SP <- SP + 32
    // Single fixed encoding 0x01C0. All status bits are written by
    // the popped value (since the read covers all 32 bits of ST).
    //
    // ALU computes (SP + 32): alu_a = rf_rs1_data (= SP), alu_b = 32
    // via the same constant-32 mux entry PUSHST uses, alu_op = ADD.
    // The mem read uses the OLD SP (= rf_rs1_data) — not alu_result —
    // because we read BEFORE the increment. WRITEBACK then writes
    // the new SP via the regfile and the new ST via st_write_en
    // (st_write_data = mem_rdata).
    // -----------------------------------------------------------------------
    if (instr == POPST_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_POPST;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;
      decoded.rs_idx          = REG_SP_IDX;
      decoded.alu_op          = ALU_OP_ADD;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;          // ST update via st_write_en, not flag mask
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // CALL Rs — Call Subroutine Indirect.
    //   SP -= 32;  mem[SP] = PC';  PC = Rs  (with bottom 4 bits cleared)
    //
    // Setup: rs2 reads SP (rd_idx=15) so the ALU's swap-group can put
    // it on alu_a; rs1 reads Rs (rs_idx from instr[3:0]) so the PC-load
    // mux can read it via rf_rs1_data. ALU computes alu_a - alu_b =
    // SP - 32 via the constant-32 mux entry. The CORE_MEMORY arm
    // writes pc_value (= PC' at that point in the FSM) to mem[alu_result].
    // CORE_WRITEBACK then updates SP (rf_wr_en) and loads PC (pc_load_en).
    // Status bits all Unaffected.
    // -----------------------------------------------------------------------
    if (top11 == CALL_RS_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_CALL_RS;
      decoded.rd_file         = reg_file_from_instr;  // = R bit (file of Rs)
      decoded.rd_idx          = REG_SP_IDX;            // write back to SP
      decoded.rs_idx          = reg_idx_from_instr;    // Rs read via rs1 port
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // RETS [N] — Return from Subroutine. Inverse of CALL Rs (mostly).
    //   PC <- mem[SP];  SP <- SP + 32 + 16*N
    // N at instr[4:0]. Status bits all Unaffected.
    //
    // The mem-read uses the OLD SP value (rf_rs2_data, since both
    // ports read SP via rd_idx=15/rs_idx=15); the popped value goes
    // directly to pc_load_value in WRITEBACK. ALU computes
    // SP + 32 + 16*N via a custom alu_b mux entry that combines
    // 32'd32 with `decoded.k5` left-shifted by 4.
    // -----------------------------------------------------------------------
    if (top11 == RETS_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_RETS;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;
      decoded.rs_idx          = REG_SP_IDX;
      decoded.k5              = instr[4:0];       // N (0..31)
      decoded.alu_op          = ALU_OP_ADD;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // CALLA Address — Call Subroutine Absolute. 3-word instruction:
    //   opcode (0x0D5F) + 16-bit LO + 16-bit HI of the absolute
    //   target.  Same FSM flow as MOVI IL: opcode → IMM_LO → IMM_HI →
    //   EXECUTE → CORE_MEMORY (write PC') → WRITEBACK (load PC,
    //   write SP).  The target uses the existing `branch_target_jacc`
    //   datapath (absolute with bottom 4 bits cleared).
    // -----------------------------------------------------------------------
    if (instr == CALLA_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_CALLA;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;       // SP write back
      decoded.alu_op          = ALU_OP_SUB;       // SP - 32
      decoded.needs_imm32     = 1'b1;             // fetch the 32-bit address
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // CALLR Address — Call Subroutine Relative. 2-word instruction:
    //   opcode (0x0D3F) + 16-bit signed disp.  Same FSM flow as
    //   JRcc long: opcode → IMM_LO → EXECUTE → CORE_MEMORY (write
    //   PC') → WRITEBACK (load PC = PC' + sign_ext(disp16)*16, write SP).
    // -----------------------------------------------------------------------
    if (instr == CALLR_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_CALLR;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;
      decoded.alu_op          = ALU_OP_SUB;
      decoded.needs_imm16     = 1'b1;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // RETI — Return from Interrupt. Two-pop memory sequence:
    //   step 0: ST <- mem[SP];     SP intermediate = SP + 32
    //   step 1: PC <- mem[SP+32];  SP final        = SP + 64
    // The core's multi-step CORE_MEMORY sequence handles both reads,
    // latches the popped values, then WRITEBACK fires both the
    // st_write_en path (with the popped ST) and pc_load_en (with the
    // popped PC), plus the regfile write of the new SP.
    //
    // ALU computes the FINAL SP via SP + 64.  alu_a = SP via rs2;
    // alu_b = 64 via a new mux entry.
    // -----------------------------------------------------------------------
    if (instr == RETI_OPCODE) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_RETI;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;
      decoded.rs_idx          = REG_SP_IDX;
      decoded.alu_op          = ALU_OP_ADD;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;          // ST update via st_write_en
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // TRAP N
    //
    // Encoding `0000 1001 000N NNNN` (top11 = 11'b00001001_000); N is
    // the trap number at instr[4:0] (0..31). Semantics summarized at
    // the TRAP_TOP11 declaration above. Three-step memory transaction
    // sequenced by the core's `mem_op_step` counter.
    //
    // ALU is set up to compute SP - 64 (SUB SP, 64) so the regfile
    // writeback lands the post-push SP. alu_b = 64 reuses RETI's
    // mux entry — the only direction difference is alu_op.
    // -----------------------------------------------------------------------
    if (top11 == TRAP_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_TRAP;
      decoded.rd_file         = REG_FILE_A;
      decoded.rd_idx          = REG_SP_IDX;
      decoded.rs_idx          = REG_SP_IDX;
      decoded.k5              = instr[4:0];       // N (0..31)
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b0;          // ST update via st_write_en
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MMTM Rp, register list
    //
    // Top11 match on 11'b00001001_100. Rd carries the Rp index; the
    // second instruction word (fetched via the existing imm16 path) is
    // the 16-bit register-list mask. CORE_MEMORY iterates set bits in
    // mm_mask_q (lowest-order first per spec) and pushes each
    // matching register's 32-bit value, predecrementing the working
    // Rp by 32 per push. Final Rp is written back to the regfile slot
    // for the original Rp index.
    //
    // Per SPVU001A page 12-111, MMTM updates only N: "N is set to the
    // sign of the result of 0 - Rp" (the ORIGINAL Rp), with two stated
    // exceptions (Rp=0 -> N=1, Rp=0x80000000 -> N=0). That closed form
    // is exactly N = ~Rp[31]; the core computes it from the original Rp
    // in the flag_input mux. C/Z/V are "Unaffected", so the flag mask is
    // N-only (Task 0057).
    // -----------------------------------------------------------------------
    if (top11 == MMTM_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MMTM;
      decoded.rd_file         = reg_file_from_instr;   // R bit at instr[4]
      decoded.rd_idx          = reg_idx_from_instr;    // DDDD = Rp index
      decoded.rs_idx          = reg_idx_from_instr;    // also Rp (for rs2 path)
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;            // N flag (sign of 0 - Rp)
      decoded.wb_flag_mask    = '{n: 1'b1, c: 1'b0, z: 1'b0, v: 1'b0};
      decoded.needs_imm16     = 1'b1;            // fetch the mask word
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // MMFM Rp, register list  (Move Multiple registers From Memory — pop)
    //
    // Top11 match on 11'b00001001_101. Rd/Rs carry the Rp index; the
    // second instruction word (fetched via the imm16 path) is the same
    // 16-bit register-list mask MMTM uses. CORE_MEMORY iterates set bits
    // in the residual mask HIGHEST-order first, reads each register's
    // 32-bit value from mem[Rp] back into the regfile, and post-increments
    // the working Rp by 32 per read. Final Rp (= initial + 32*count) is
    // written back to the Rp slot at WRITEBACK.
    //
    // wb_flags_en = 0: MMFM leaves N/C/Z/V Unaffected per SPVU001A
    // page 12-109. rs2 path reads the initial Rp (= rf_rs2_data) used to
    // seed the iterator on entry to CORE_MEMORY.
    // -----------------------------------------------------------------------
    if (top11 == MMFM_TOP11) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_MMFM;
      decoded.rd_file         = reg_file_from_instr;   // R bit at instr[4]
      decoded.rd_idx          = reg_idx_from_instr;    // DDDD = Rp index
      decoded.rs_idx          = reg_idx_from_instr;    // also Rp (for rs2 path)
      decoded.wb_reg_en       = 1'b1;            // write final Rp
      decoded.wb_flags_en     = 1'b0;            // all flags Unaffected
      decoded.needs_imm16     = 1'b1;            // fetch the mask word
      decoded.needs_memory_op = 1'b1;
    end

    // -----------------------------------------------------------------------
    // ADD Rs, Rd  (reg-reg add; Rs and Rd in the same file)
    // -----------------------------------------------------------------------
    if (top7 == ADD_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_ADD_RR;
      decoded.rd_file         = reg_file_from_instr;   // single R bit governs both
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_ADD;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // ADDC Rs, Rd  (Rs + Rd + C → Rd; carry-in from ST.C)
    //
    // Spec source: SPVU001A page 12-37 (per-instruction page) and the
    // instruction-summary table — encoding `0100 001S SSSR DDDD`. Used
    // for extended-precision arithmetic chained with ADD/ADDI/ADDK.
    //
    // ADDC is commutative on its register operands so the default
    // operand routing (alu_a=Rs, alu_b=Rd) needs no swap. The ALU
    // already wires alu_cin from st_c, so this just selects ALU_OP_ADDC.
    // -----------------------------------------------------------------------
    if (top7 == ADDC_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_ADDC_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_ADDC;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // SUB Rs, Rd  (Rd - Rs → Rd; same-file constraint)
    // -----------------------------------------------------------------------
    if (top7 == SUB_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_SUB_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      // Operand routing: alu_a = Rd, alu_b = Rs so that SUB computes
      // (a - b) = (Rd - Rs) → Rd, matching the spec "Rd - Rs → Rd".
      // See the alu_b mux in tms34010_core.sv for the swap.
      decoded.alu_op          = ALU_OP_SUB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // SUBB Rs, Rd  (Rd - Rs - C → Rd; borrow-in from ST.C)
    //
    // Spec source: SPVU001A page 12-248 (per-instruction page) and the
    // instruction-summary table — encoding `0100 011S SSSR DDDD`. Used
    // for extended-precision arithmetic chained with SUB/SUBI/SUBK.
    //
    // Same operand-swap as SUB: alu_a = Rd, alu_b = Rs so the ALU
    // computes a - b - cin = Rd - Rs - C. SUBB just selects ALU_OP_SUBB.
    // SPVU001A page 12-248 provides authoritative test vectors; see
    // tb_addc_subb for the rows used.
    // -----------------------------------------------------------------------
    if (top7 == SUBB_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_SUBB_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_SUBB;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // Reg-reg logical ops (AND, ANDN, OR, XOR).
    // All share the same encoding shape with a different 7-bit prefix.
    // Operand routing: alu_a = Rs (default), alu_b = Rd. Operations are
    // commutative (or via ANDN's complement-of-b form) so no swap is
    // needed. Every logical form updates Z only; N, C, and V are
    // architecturally Unaffected.
    // -----------------------------------------------------------------------
    if (top7 == AND_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_AND_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_AND;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    if (top7 == ANDN_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_ANDN_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      // ANDN per spec: Rd = Rd & ~Rs. ALU_OP_ANDN computes a & ~b, so
      // we need alu_a = Rd, alu_b = Rs. Operand swap handled in core.
      decoded.alu_op          = ALU_OP_ANDN;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    if (top7 == OR_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_OR_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_OR;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    if (top7 == XOR_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_XOR_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_XOR;
      decoded.wb_reg_en       = 1'b1;
      decoded.wb_flags_en     = 1'b1;
      decoded.wb_flag_mask    = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // CMP Rs, Rd  (flags from Rd - Rs; Rd unchanged — first wb_reg_en=0)
    // -----------------------------------------------------------------------
    if (top7 == CMP_RR_TOP7) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_CMP_RR;
      decoded.rd_file         = reg_file_from_instr;
      decoded.rd_idx          = reg_idx_from_instr;
      decoded.rs_idx          = rs_idx_from_instr;
      decoded.alu_op          = ALU_OP_CMP;        // same arithmetic as SUB
      decoded.wb_reg_en       = 1'b0;              // *** key difference ***
      decoded.wb_flags_en     = 1'b1;
    end

    // -----------------------------------------------------------------------
    // JRcc short (Jump Relative Conditional, 8-bit signed displacement)
    //
    // Encoding: bits[15:12] = 1100, bits[11:8] = cc, bits[7:0] = disp.
    // Low-byte values 0x00 and 0x80 are reserved (long-relative and
    // absolute form markers); only short-form disps land here.
    //
    // Per A0017, only condition codes verified against SPVU001A Table
    // 12-8 are decoded:
    //   cc = 0000 → UC (unconditional)
    //   cc = 0100 → EQ (Z = 1)
    //   cc = 0111 → NE (Z = 0)
    // Other cc values fall through to ILLEGAL until the table is read
    // more carefully — better to trap on an unverified condition than
    // silently mis-branch.
    // -----------------------------------------------------------------------
    // Short form: `1100 cccc dddd dddd` with dddd_dddd != 0x00 and != 0x80.
    // Target = PC_post_fetch + sign_extend(disp8) × 16 (per A0016).
    if (instr[15:12] == JRCC_TOP4 &&
        instr[7:0] != 8'h00 && instr[7:0] != 8'h80 &&
        (instr[11:8] == CC_UC ||
         instr[11:8] == CC_LO ||
         instr[11:8] == CC_LS ||
         instr[11:8] == CC_HI ||
         instr[11:8] == CC_LT ||
         instr[11:8] == CC_LE ||
         instr[11:8] == CC_GT ||
         instr[11:8] == CC_GE ||
         instr[11:8] == CC_EQ ||
         instr[11:8] == CC_NE ||
         instr[11:8] == CC_HS ||
         instr[11:8] == CC_C  ||
         instr[11:8] == CC_V  ||
         instr[11:8] == CC_NV ||
         instr[11:8] == CC_N  ||
         instr[11:8] == CC_NN)) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_JRCC_SHORT;
      decoded.branch_cc   = instr[11:8];
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // -----------------------------------------------------------------------
    // JRcc long form (16-bit signed displacement)
    //
    // Encoding: `1100 cccc 0000 0000` followed by a 16-bit signed-word
    // displacement in the next instruction word. Per SPVU001A page 12-96:
    // "The assembler calculates the offset as (Address - PC')/16 and
    // inserts the resulting offset into the second instruction word of
    // the opcode. The range for this form of the JRcc instruction is
    // -32,768 to +32,767 words (excluding 0)."
    //
    // Target math (A0016 generalized): branch_target_long =
    // PC_after_both_fetches + sign_extend(disp16) × 16. By the time the
    // FSM is in CORE_WRITEBACK, PC has already been advanced twice
    // (once per FETCH ack), so `pc_value` already equals
    // PC_original + 32 — matching the spec's PC'.
    //
    // Only disp8 == 0x00 unlocks the long form here. The absolute marker
    // disp8 == 0x80 is decoded by the separate JAcc block below.
    // -----------------------------------------------------------------------
    if (instr[15:12] == JRCC_TOP4 &&
        instr[7:0] == 8'h00 &&
        (instr[11:8] == CC_UC ||
         instr[11:8] == CC_LO ||
         instr[11:8] == CC_LS ||
         instr[11:8] == CC_HI ||
         instr[11:8] == CC_LT ||
         instr[11:8] == CC_LE ||
         instr[11:8] == CC_GT ||
         instr[11:8] == CC_GE ||
         instr[11:8] == CC_EQ ||
         instr[11:8] == CC_NE ||
         instr[11:8] == CC_HS ||
         instr[11:8] == CC_C  ||
         instr[11:8] == CC_V  ||
         instr[11:8] == CC_NV ||
         instr[11:8] == CC_N  ||
         instr[11:8] == CC_NN)) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_JRCC_LONG;
      decoded.branch_cc   = instr[11:8];
      decoded.needs_imm16 = 1'b1;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // -----------------------------------------------------------------------
    // NOP — no operation. Single fixed encoding 0x0300 (A0021). Decoded as
    // valid; both writeback gates stay 0 so the FSM walks the full
    // FETCH→DECODE→EXECUTE→WRITEBACK loop with no datapath effect, leaving
    // PC advance (driven by FETCH ack) as the only architectural change.
    // -----------------------------------------------------------------------
    if (instr == NOP_OPCODE) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_NOP;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // EMU — pulse EMUA and sample RUN/EMU. The core owns the sampled
    // run-vs-halt behavior; decode has no architectural writeback.
    if (instr == EMU_OPCODE) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_EMU;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // FILL L — fill a DY×DX pixel array with COLOR1. Fixed opcode 0x0FC0; the
    // operands are the implied B-file registers DADDR(B2)/DPTCH(B3)/DYDX(B7)/
    // COLOR1(B9), read and looped over by the core's FILL engine. All flags
    // Unaffected; DADDR is updated by the engine (not the normal writeback).
    if (instr == 16'h0FC0) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_FILL_L;
      decoded.wb_reg_en       = 1'b0;   // DADDR write handled by the FILL engine
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;   // EXECUTE routes to the FILL states
    end

    // FILL XY — like FILL L, but DADDR (B2) holds an XY value that the FILL
    // engine converts to a linear start address (CONVDP + OFFSET(B4) + PSIZE)
    // at CORE_FILL_SETUP. Fixed opcode 0x0FE0. The core implements all four
    // window modes (Tasks 0105, 0107, and 0109).
    if (instr == 16'h0FE0) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_FILL_XY;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;
    end

    // PIXBLT L,L — transfer a DY×DX source array (SADDR=B0/SPTCH=B1) to a dest
    // array (DADDR=B2/DPTCH=B3), processing each pixel. Fixed opcode 0x0F00.
    // The core's PIXBLT engine reads the implied B-regs and loops; SADDR and
    // DADDR are updated by the engine. All flags Unaffected.
    if (instr == 16'h0F00) begin
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PIXBLT_LL;
      decoded.wb_reg_en       = 1'b0;   // SADDR/DADDR writes handled by the engine
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;   // EXECUTE routes to the PIXBLT states
    end

    // PIXBLT XY variants — same engine as L,L but the SADDR / DADDR implied
    // register holds an XY value converted to linear at PBLT_SETUP (source via
    // CONVSP, destination via CONVDP, + OFFSET(B4) + PSIZE).
    //   L,XY  (0x0F20): destination XY.   XY,L  (0x0F40): source XY.
    //   XY,XY (0x0F60): both.
    if (instr == 16'h0F20) begin          // PIXBLT L,XY
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PIXBLT_LL;
      decoded.blt_dst_xy      = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;
    end
    if (instr == 16'h0F40) begin          // PIXBLT XY,L
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PIXBLT_LL;
      decoded.blt_src_xy      = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;
    end
    if (instr == 16'h0F60) begin          // PIXBLT XY,XY
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PIXBLT_LL;
      decoded.blt_src_xy      = 1'b1;
      decoded.blt_dst_xy      = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;
    end

    // PIXBLT B variants — the source is a 1-bit-per-pixel bitmap; each bit
    // expands to COLOR1(B9) if 1, COLOR0(B8) if 0, before processing.
    //   B,L  (0x0F80): linear dest.   B,XY (0x0FA0): XY dest (CONVDP convert).
    if (instr == 16'h0F80) begin          // PIXBLT B,L
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PIXBLT_LL;
      decoded.blt_binary      = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;
    end
    if (instr == 16'h0FA0) begin          // PIXBLT B,XY
      decoded.illegal         = 1'b0;
      decoded.iclass          = INSTR_PIXBLT_LL;
      decoded.blt_binary      = 1'b1;
      decoded.blt_dst_xy      = 1'b1;
      decoded.wb_reg_en       = 1'b0;
      decoded.wb_flags_en     = 1'b0;
      decoded.needs_memory_op = 1'b0;
    end

    // -----------------------------------------------------------------------
    // JUMP Rs (register-indirect jump)
    //
    // Encoding (SPVU001A page 12-98 + summary table):
    //   bits[15:5] = 11'b00000001_011 (= 0x00B)
    //   bit[4]     = R (file: 0 = A, 1 = B)
    //   bits[3:0]  = Rs index
    //
    // Semantics: Rs → PC, with the bottom 4 bits of PC forced to 0.
    // No status-register effect. Single-word instruction (no immediate).
    //
    // We populate the source-register selectors so the regfile's rs1
    // port reads Rs; the core's PC-load mux then masks rf_rs1_data
    // with `~32'hF` to enforce word alignment, and asserts pc_load_en
    // in CORE_WRITEBACK.
    // -----------------------------------------------------------------------
    if (top11 == JUMP_RS_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_JUMP_RS;
      decoded.rd_file     = reg_file_from_instr;   // same file bit governs Rs
      decoded.rs_idx      = reg_idx_from_instr;    // Rs index lives in low 4 bits
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // -----------------------------------------------------------------------
    // DSJ / DSJEQ / DSJNE Rd, Address  (Decrement Rd and Skip Jump)
    //
    // All three share:
    //   - bits[4]    = R (file)
    //   - bits[3:0]  = Rd index
    //   - 16-bit signed offset in the next instruction word
    //   - alu computes Rd - 1 (alu_a = Rd via the swap group; alu_b = 1
    //     via the K-form arm with decoded.k5 = 5'd1)
    //   - wb_reg_en = 1 (write decremented Rd; gated by core based on
    //     the pre-condition for DSJEQ/DSJNE)
    //   - wb_flags_en = 0 (status bits unaffected per the spec)
    //   - needs_imm16 = 1 (fetch the offset word)
    //
    // The branch decision and the Rd-writeback gating both depend on
    // runtime state (ST.Z and post-decrement Rd nonzero). Those gates
    // live in the core, not the decoder. The decoder just identifies
    // the class.
    // -----------------------------------------------------------------------
    if (top11 == DSJ_TOP11 || top11 == DSJEQ_TOP11 || top11 == DSJNE_TOP11) begin
      decoded.illegal     = 1'b0;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = 5'd1;          // alu_b = 1 via K-form mux arm
      decoded.alu_op      = ALU_OP_SUB;    // alu computes Rd - 1
      decoded.needs_imm16 = 1'b1;
      decoded.imm_sign_extend = 1'b0;      // offset is consumed directly, not via imm32
      decoded.wb_reg_en   = 1'b1;          // gated further by core (DSJEQ/DSJNE)
      decoded.wb_flags_en = 1'b0;          // spec: N/C/Z/V unaffected
      unique case (top11)
        DSJ_TOP11:   decoded.iclass = INSTR_DSJ;
        DSJEQ_TOP11: decoded.iclass = INSTR_DSJEQ;
        DSJNE_TOP11: decoded.iclass = INSTR_DSJNE;
        default:     decoded.iclass = INSTR_ILLEGAL;
      endcase
    end

    // -----------------------------------------------------------------------
    // JAcc Address (absolute-form conditional jump)
    //
    // Encoding (SPVU001A page 12-91 + summary table):
    //   word 0:  1100 cccc 1000 0000     (low byte = 0x80 unlocks JAcc)
    //   word 1:  16 LSBs of absolute address
    //   word 2:  16 MSBs of absolute address
    //
    // Semantics: if condition true, PC ← address (with bottom 4 bits
    // forced to 0 per spec). N/C/Z/V unaffected by the instruction.
    //
    // Reuses the same 11-cc recognized set as JRcc (A0023). The 32-bit
    // address fetch uses the existing IMM_LO/IMM_HI path via
    // needs_imm32=1, just like MOVI IL — but the core extracts the
    // address from {imm_hi_q, imm_lo_q} for a PC load, not for the
    // register file.
    // -----------------------------------------------------------------------
    if (instr[15:12] == JRCC_TOP4 &&
        instr[7:0] == 8'h80 &&
        (instr[11:8] == CC_UC ||
         instr[11:8] == CC_LO ||
         instr[11:8] == CC_LS ||
         instr[11:8] == CC_HI ||
         instr[11:8] == CC_LT ||
         instr[11:8] == CC_LE ||
         instr[11:8] == CC_GT ||
         instr[11:8] == CC_GE ||
         instr[11:8] == CC_EQ ||
         instr[11:8] == CC_NE ||
         instr[11:8] == CC_HS ||
         instr[11:8] == CC_C  ||
         instr[11:8] == CC_V  ||
         instr[11:8] == CC_NV ||
         instr[11:8] == CC_N  ||
         instr[11:8] == CC_NN)) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_JACC;
      decoded.branch_cc   = instr[11:8];
      decoded.needs_imm32 = 1'b1;
      decoded.wb_reg_en   = 1'b0;
      decoded.wb_flags_en = 1'b0;
    end

    // -----------------------------------------------------------------------
    // DSJS Rd, Address (Decrement-and-Skip-Jump Short, single-word)
    //
    // Encoding (SPVU001A page 12-74 + summary table line 13844):
    //   bits[15:11] = 5'b00111
    //   bit[10]     = D (direction; 0 = forward, 1 = backward)
    //   bits[9:5]   = 5-bit unsigned offset (words from PC')
    //   bit[4]      = R (file)
    //   bits[3:0]   = Rd index
    //
    // Semantics: Rd ← Rd - 1; if Rd' != 0, branch (PC' +/- offset*16);
    // else fall through. Status bits N/C/Z/V unaffected.
    //
    // The decoder identifies the class and sets the SAME control as
    // DSJ (alu_op=SUB, k5=1, wb_reg_en=1, wb_flags_en=0). The
    // direction and offset are not captured in the decoded struct;
    // the core extracts them combinationally from instr_word_q[10]
    // and instr_word_q[9:5] for the branch-target computation.
    // -----------------------------------------------------------------------
    if (instr[15:11] == DSJS_TOP5) begin
      decoded.illegal     = 1'b0;
      decoded.iclass      = INSTR_DSJS;
      decoded.rd_file     = reg_file_from_instr;
      decoded.rd_idx      = reg_idx_from_instr;
      decoded.k5          = 5'd1;          // alu_b = 1 via K-form mux arm
      decoded.alu_op      = ALU_OP_SUB;
      decoded.wb_reg_en   = 1'b1;
      decoded.wb_flags_en = 1'b0;          // spec: N/C/Z/V unaffected
    end

    // -----------------------------------------------------------------------
    // BTST K, Rd  (Test Register Bit — Constant)
    //
    // Encoding: bits[15:10] = 6'b000111, bits[9:5] = K (5-bit unsigned
    // bit index), bit[4] = R, bits[3:0] = Rd index. Per SPVU001A
    // page 12-46 + summary table line 26942. Status: Z = !(bit_K_of_Rd);
    // N, C, V unaffected — gated via wb_flag_mask.
    //
    // The bit-mask `1 << K` is computed in the core (alu_b mux) and
    // ANDed with Rd. The Z flag from the AND result is what we want.
    // -----------------------------------------------------------------------
    if (top6 == BTST_K_TOP6) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_BTST_K;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.k5           = ~instr[9:5];  // K is stored as its 1s complement: bit = 31 - K (SPVU001A 12-46; MAME BTST_K)
      decoded.alu_op       = ALU_OP_AND;
      decoded.wb_reg_en    = 1'b0;          // Rd is NOT written
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // CLRC : ST.C ← 0. Single fixed encoding 0x0320. N, Z, V unaffected.
    // Implemented via the new per-flag mask: only C updates, and the
    // C value comes from a custom-flags mux in the core (set to 0).
    // -----------------------------------------------------------------------
    if (instr == CLRC_OPCODE) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_CLRC;
      decoded.wb_reg_en    = 1'b0;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b0, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // SETC : ST.C ← 1. Single fixed encoding 0x0DE0. Same mask as CLRC.
    // -----------------------------------------------------------------------
    if (instr == SETC_OPCODE) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_SETC;
      decoded.wb_reg_en    = 1'b0;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b1, z: 1'b0, v: 1'b0};
    end

    // -----------------------------------------------------------------------
    // GETST Rd : Rd ← ST. Per the summary table, top11 = 11'b00000001_100.
    // bit[4]=R, bits[3:0]=Rd. Status bits unaffected.
    //
    // The core's regfile-write-data mux gains an INSTR_GETST arm
    // routing `st_value` to `rf_wr_data`.
    // -----------------------------------------------------------------------
    if (top11 == GETST_TOP11) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_GETST;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b0;
    end

    // -----------------------------------------------------------------------
    // PUTST Rs : ST ← Rs (full 32-bit write). top11 = 11'b00000001_101.
    // bit[4]=R, bits[3:0]=Rs (the chart writes DODD but it's the
    // source-register field).
    //
    // We use the existing full ST-write path (st_write_en + st_write_data
    // in the core), driving st_write_data from rf_rs1_data (which is
    // already gated by decoded.rd_file/rs_idx in this kind of arm).
    //
    // Per spec summary table page A-15, PUTST DOES affect N C Z V —
    // because the bits being copied to ST happen to lie at the N/C/Z/V
    // positions. Our full-write path stores all 32 bits, so this is
    // automatically correct.
    // -----------------------------------------------------------------------
    if (top11 == PUTST_TOP11) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_PUTST;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rs_idx       = reg_idx_from_instr;   // Rs index for the rs1 read
      decoded.wb_reg_en    = 1'b0;
      decoded.wb_flags_en  = 1'b0;                  // full ST write covers all bits
    end

    // -----------------------------------------------------------------------
    // GETPC Rd : Rd ← PC. Per SPVU001A summary table A-16.
    // The current PC is bit-addressed; we deliver pc_value (the value
    // observed at CORE_WRITEBACK, which already reflects the opcode
    // fetch's PC advance).
    // -----------------------------------------------------------------------
    if (top11 == GETPC_TOP11) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_GETPC;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b0;
    end

    // -----------------------------------------------------------------------
    // EXGPC Rd : atomic swap next PC ↔ Rd. Page 12-79 explicitly requires
    // the new PC's low 4 bits to be forced to 0 for word alignment.
    //
    // We use the regfile's rs2 port (which reads decoded.rd_idx) to
    // get the OLD Rd value for the PC-load. The regfile write port
    // simultaneously stores pc_value into Rd. Because the regfile is
    // async-read + sync-write, rf_rs2_data sees the OLD value during
    // the same WRITEBACK cycle, so the swap is atomic.
    // -----------------------------------------------------------------------
    if (top11 == EXGPC_TOP11) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_EXGPC;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b0;
    end

    // -----------------------------------------------------------------------
    // REV Rd : Rd ← chip revision-number constant. Per the format and
    // example on page 12-233, the value is 32'h0000_0008. Status bits
    // are unaffected.
    // -----------------------------------------------------------------------
    if (top11 == REV_TOP11) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_REV;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.wb_reg_en    = 1'b1;
      decoded.wb_flags_en  = 1'b0;
    end

    // -----------------------------------------------------------------------
    // BTST Rs, Rd  (Test Register Bit — Register)
    //
    // Encoding: bits[15:9] = 7'b0100_101 (top7), bits[8:5] = Rs idx,
    // bit[4] = R, bits[3:0] = Rd idx. Per SPVU001A page 12-47 +
    // summary table line 26943. Bit index is Rs[4:0].
    // Same flag policy as BTST K (Z only).
    // -----------------------------------------------------------------------
    if (top7 == BTST_RR_TOP7) begin
      decoded.illegal      = 1'b0;
      decoded.iclass       = INSTR_BTST_RR;
      decoded.rd_file      = reg_file_from_instr;
      decoded.rd_idx       = reg_idx_from_instr;
      decoded.rs_idx       = rs_idx_from_instr;
      decoded.alu_op       = ALU_OP_AND;
      decoded.wb_reg_en    = 1'b0;
      decoded.wb_flags_en  = 1'b1;
      decoded.wb_flag_mask = '{n: 1'b0, c: 1'b0, z: 1'b1, v: 1'b0};
    end
    // Task 0180: pixel-array instructions become illegal when PIXEL_OPS=0.
    if (!PIXEL_OPS) begin
      unique case (decoded.iclass)
        INSTR_FILL_L, INSTR_FILL_XY, INSTR_PIXBLT_LL, INSTR_DRAV, INSTR_LINE: begin
          decoded.illegal      = 1'b1;
          decoded.illegal_trap = 1'b1;
        end
        default: ;
      endcase
    end

  end

endmodule : tms34010_decode
`default_nettype wire

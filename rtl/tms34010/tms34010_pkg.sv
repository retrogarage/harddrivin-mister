// -----------------------------------------------------------------------------
// tms34010_pkg.sv
//
// Architectural constants, widths, and shared typedefs for the TMS34010 core.
//
// This file is the single source of truth for project-wide widths and enum
// types. Other RTL files must not introduce parallel "magic number" widths.
// As more of the spec is implemented, this package grows with concrete
// register-bit layouts and instruction-class typedefs.
//
// Spec source: third_party/TMS34010_Info/docs/ti-official/
//              1988_TI_TMS34010_Users_Guide.pdf
//
// Current through Task 0156: architectural widths and register layouts,
// instruction/control types, I/O fields, interrupt vectors, the CPU/graphics
// FSM states, physical-memory word geometry, local-cycle kinds, and the
// original local-clock command/phase types are here.
// -----------------------------------------------------------------------------

`default_nettype none
package tms34010_pkg;

  // ---------------------------------------------------------------------------
  // Architectural widths
  // ---------------------------------------------------------------------------

  // TMS34010 is a 32-bit architecture with a bit-addressed memory model.
  // The external bus on the original device is 16 bits, but the RTL exposes
  // an internal-style request/valid interface at full architectural width;
  // an external glue module (Phase 6) handles the bus multiplexing.
  parameter int unsigned ADDR_WIDTH       = 32;
  parameter int unsigned DATA_WIDTH       = 32;

  // Field-size operands are 1..32 bits, encoded in 6 bits.
  // Spec: 1988 User's Guide, field-move/field-addressing chapter.
  parameter int unsigned FIELD_SIZE_WIDTH = 6;

  // Original local memory is transferred as aligned 16-bit words. A maximum
  // 32-bit field beginning at bit offset 15 occupies at most three words.
  // Spec: 1988 User's Guide §3.1 pages 3-2/3-3 and §4.1 pages 4-2 through 4-5.
  parameter int unsigned LOCAL_WORD_WIDTH     = 16;
  parameter int unsigned LOCAL_WORD_ADDR_LSB  = 4;
  parameter int unsigned FIELD_WINDOW_WIDTH   = 3 * LOCAL_WORD_WIDTH;
  parameter int unsigned FIELD_MAX_WORDS      = 3;
  typedef logic [LOCAL_WORD_WIDTH-1:0] local_word_t;

  // Abstract local-memory cycle selected by the fixed-priority arbiter.
  // A later physical-bus controller translates these cycle kinds into the
  // original LAD/RAS/CAS/TR/Q timing. Screen refresh and DRAM refresh carry
  // their dedicated payloads rather than pretending to be CPU word accesses.
  typedef enum logic [3:0] {
    LOCAL_CYCLE_WORD_READ      = 4'd0,
    LOCAL_CYCLE_WORD_WRITE     = 4'd1,
    LOCAL_CYCLE_SCREEN_REFRESH = 4'd2,
    LOCAL_CYCLE_DRAM_RAS       = 4'd3,
    LOCAL_CYCLE_DRAM_CBR       = 4'd4,
    LOCAL_CYCLE_IO_READ        = 4'd5,
    LOCAL_CYCLE_IO_WRITE       = 4'd6,
    LOCAL_CYCLE_PIXEL_MTR      = 4'd7,
    LOCAL_CYCLE_PIXEL_RTM      = 4'd8
  } local_cycle_kind_t;

  // Eight half-quarter subphases resolve the control transitions within each
  // Q1..Q4 local-clock period and the specified middle-of-Q4 read sample.
  // A future FPGA top supplies the 8x PLL clock consumed by the local-bus
  // controller; LCLK1/LCLK2 are output waveforms, not internal fabric clocks.
  typedef enum logic [2:0] {
    LOCAL_PHASE_Q1A = 3'd0,
    LOCAL_PHASE_Q1B = 3'd1,
    LOCAL_PHASE_Q2A = 3'd2,
    LOCAL_PHASE_Q2B = 3'd3,
    LOCAL_PHASE_Q3A = 3'd4,
    LOCAL_PHASE_Q3B = 3'd5,
    LOCAL_PHASE_Q4A = 3'd6,
    LOCAL_PHASE_Q4B = 3'd7
  } local_subphase_t;

  // Coherent command payload transferred from the core-clock memory fabric to
  // the dedicated 8x local-bus domain. The CDC bridge registers and holds this
  // entire bundle while only a request toggle crosses through a synchronizer.
  typedef struct packed {
    local_cycle_kind_t             kind;
    logic [ADDR_WIDTH-1:0]         addr;
    local_word_t                   wdata;
    local_word_t                   io_rdata;
    logic                          iaq;
    logic [13:0]                   srfaddr;
    logic [15:0]                   dpytap;
    logic                          screen_org;
    logic [7:0]                    dram_row;
  } local_cycle_cmd_t;

  // ---------------------------------------------------------------------------
  // Instruction-stream constants
  //
  // Spec: bibliography/hdl-reimplementation/01-architecture.md
  // ("Instructions are 16-bit-aligned half-words in memory. PC is a
  //   bit-address but increments by 16 per fetch.")
  // ---------------------------------------------------------------------------
  parameter logic [FIELD_SIZE_WIDTH-1:0] INSTR_WORD_BITS = 6'd16;

  // Width of the PC's per-advance amount (in bits). 8 bits covers 1- to
  // 15-word instructions with headroom; the longest documented form is
  // 3 words (48 bits) for instructions with a 32-bit immediate.
  parameter int unsigned PC_ADVANCE_WIDTH = 8;

  // Internal PC-register reset value. The architectural reset sequence
  // overwrites this value from RESET_VECTOR_ADDR before the first instruction
  // fetch; keeping the flop reset explicit makes its reset-time state defined.
  parameter logic [ADDR_WIDTH-1:0] RESET_PC = '0;

  // ---------------------------------------------------------------------------
  // Core top-level FSM states
  //
  // Added in Phase 3:
  //   CORE_FETCH_IMM_LO/HI — fetch the 16- or 32-bit immediate that follows
  //   long-immediate-form instructions (MOVI IW/IL, ADDI IW/IL, CMPI, etc.).
  // ---------------------------------------------------------------------------
  typedef enum logic [5:0] {
    CORE_RESET        = 6'd0,
    CORE_FETCH        = 6'd1,
    CORE_DECODE       = 6'd2,
    CORE_FETCH_IMM_LO = 6'd3,
    CORE_FETCH_IMM_HI = 6'd4,
    CORE_EXECUTE      = 6'd5,
    CORE_MEMORY       = 6'd6,
    CORE_WRITEBACK    = 6'd7,
    CORE_DIVIDE       = 6'd8,  // multi-cycle wait for the divider (DIVU/MODU/...)
    CORE_FILL_SETUP   = 6'd9,  // FILL: latch COLOR1 + init counters (operands B2/B3/B7
                               //       latched at EXECUTE); the pixel loop follows
    CORE_FILL         = 6'd10, // FILL: optional destination read plus pixel-write loop
    CORE_FILL_WB      = 6'd11, // FILL: write the final DADDR back to B2
    CORE_PBLT_SETUP   = 6'd12, // PIXBLT: latch SPTCH/DPTCH (SADDR/DADDR/DYDX at EXECUTE)
    CORE_PBLT         = 6'd13, // PIXBLT: per-pixel source / optional destination / write loop
    CORE_PBLT_WB      = 6'd14, // PIXBLT: write the final SADDR back to B0
    CORE_PBLT_WB2     = 6'd15, // PIXBLT: write the final DADDR back to B2
    CORE_PBLT_SETUP2  = 6'd16, // PIXBLT B (color expand): latch COLOR0/COLOR1
    CORE_INT_PUSH_ST  = 6'd17, // interrupt entry: push ST onto the stack
    CORE_INT_PUSH_PC  = 6'd18, // interrupt entry: push PC onto the stack
    CORE_INT_VECTOR   = 6'd19, // interrupt entry: read the vector -> PC
    CORE_INT_DONE     = 6'd20, // interrupt entry: write back SP, initialize ST
    CORE_FILL_SETUP_WIN = 6'd21, // FILL XY (W=2/3): read WSTART/WEND for window
    CORE_PBLT_SETUP_WIN = 6'd22, // PIXBLT XY (W=3): read WSTART/WEND for clipping
    CORE_FILL_WIN_MISS  = 6'd23, // FILL XY (W=2): array outside window — no draw, V=1, WVP
    CORE_PBLT_WIN_MISS  = 6'd24, // PIXBLT XY (W=2): array outside window — no draw, V=1, WVP
    CORE_FILL_WIN_HIT   = 6'd25, // FILL XY (W=1): hit detection — no draw, V/WVP on overlap
    CORE_PBLT_WIN_HIT   = 6'd26, // PIXBLT XY (W=1): hit detection — no draw, V/WVP on overlap
    CORE_DRAV           = 6'd27, // DRAV: optional destination read + pixel draw at Rd's XY
    CORE_DRAV_SETUP_WIN = 6'd28, // DRAV (W!=0): read WSTART/WEND, test Rd's XY against window
    CORE_LINE_SETUP1    = 6'd29, // LINE: read d(B0)/DYDX(B7)/COUNT(B10)
    CORE_LINE_SETUP2    = 6'd30, // LINE: read INC1(B11)/INC2(B12)/OFFSET(B4)
    CORE_LINE_SETUP3    = 6'd31, // LINE: read DADDR(B2)/COLOR1(B9)
    CORE_LINE_DRAW      = 6'd32, // LINE: pixel draw + Bresenham step (d/DADDR/COUNT)
    CORE_LINE_WB_D      = 6'd33, // LINE: write back d -> B0
    CORE_LINE_WB_DADDR  = 6'd34, // LINE: write back DADDR -> B2
    CORE_LINE_WB_COUNT  = 6'd35, // LINE: write back COUNT (=0) -> B10
    CORE_LINE_SETUP_WIN = 6'd36, // LINE (W=3): read WSTART/WEND for per-pixel clip
    CORE_PIXT_SETUP_WIN = 6'd37, // PIXT XY (W!=0): read WSTART/WEND before pixel access
    CORE_FETCH_IMM_EXT_LO = 6'd38, // fourth instruction word (second 32-bit operand low)
    CORE_FETCH_IMM_EXT_HI = 6'd39, // fifth instruction word (second 32-bit operand high)
    CORE_EMU_HALT         = 6'd40, // EMU sampled active: quiescent until RUN returns high
    CORE_RESET_HALT       = 6'd41, // HCS-high reset: wait for host before vector fetch
    CORE_HOST_HALT        = 6'd42, // HSTCTL.HLT: quiescent at an instruction boundary
    CORE_DECODE_WAIT      = 6'd43, // second decode-settle cycle before capture
    CORE_DISPATCH         = 6'd44, // registered decode selects operand/immediate path
    CORE_FILL_W1_DYDX     = 6'd45, // FILL XY W=1 hit: write common-rectangle DYDX
    CORE_PBLT_W1_DYDX     = 6'd46, // PIXBLT XY-dest W=1 hit: write common DYDX
    CORE_ARRAY_CKPT_B0    = 6'd47, // array checkpoint: next source address
    CORE_ARRAY_CKPT_B2    = 6'd48, // array checkpoint: next destination address
    CORE_ARRAY_CKPT_B10   = 6'd49, // array checkpoint: next {row,column} cursor
    CORE_ARRAY_CKPT_B11   = 6'd50, // array checkpoint: effective {height,width}
    CORE_ARRAY_CKPT_B12   = 6'd51, // array checkpoint: source/FILL result context
    CORE_ARRAY_CKPT_B13   = 6'd52, // array checkpoint: destination result context
    CORE_ARRAY_CKPT_B14   = 6'd53, // array checkpoint: effective raw destination XY
    CORE_ARRAY_RESUME1    = 6'd54, // PBX resume: read pitches/effective dimensions
    CORE_ARRAY_RESUME2    = 6'd55, // PBX resume: read result/raw-destination context
    CORE_ARRAY_RESUME3    = 6'd56, // PBX resume: read window/original dimensions
    CORE_ARRAY_RESUME4    = 6'd57, // PBX resume: read expansion/fill colors
    CORE_LINE_CKPT_D      = 6'd58, // LINE checkpoint: next error term -> B0
    CORE_LINE_CKPT_DADDR  = 6'd59, // LINE checkpoint: next destination -> B2
    CORE_LINE_CKPT_COUNT  = 6'd60, // LINE checkpoint: remaining count -> B10
    CORE_PBLT_WIN_EVAL    = 6'd61, // evaluate registered PIXBLT window geometry
    CORE_PBLT_WIN_OFFSETS = 6'd62, // register clipped PIXBLT address offsets
    CORE_PBLT_WIN_APPLY   = 6'd63  // apply clipped PIXBLT working geometry
  } core_state_t;

  // ---------------------------------------------------------------------------
  // Register file types
  //
  // Spec: bibliography/hdl-reimplementation/03-registers.md
  //   - Two 15-entry general-purpose register banks (A and B).
  //   - One shared stack pointer accessible from both banks as A15/B15.
  //   - All 32 bits.
  // ---------------------------------------------------------------------------
  typedef enum logic {
    REG_FILE_A = 1'b0,
    REG_FILE_B = 1'b1
  } reg_file_t;

  // 4-bit register index inside a file. Index 4'hF is the shared SP alias.
  typedef logic [3:0] reg_idx_t;

  parameter reg_idx_t REG_SP_IDX = 4'hF;

  // CPW's implied window operands are fixed B-file registers (SPVU001A
  // page 12-57): WSTART = B5 (inclusive lesser corner), WEND = B6.
  parameter reg_idx_t CPW_WSTART_IDX = 4'd5;
  parameter reg_idx_t CPW_WEND_IDX   = 4'd6;

  // Graphics B-file registers with fixed implied roles (SPVU001A §"Implied
  // Operands"). B2 = DADDR (destination address), B3 = DPTCH (destination
  // pitch), B4 = OFFSET (linear address of XY origin 0,0), B7 = DYDX (array
  // dimensions rows:cols), B9 = COLOR1 (fill / source color).
  parameter reg_idx_t B_SADDR_IDX  = 4'd0;
  parameter reg_idx_t B_SPTCH_IDX  = 4'd1;
  parameter reg_idx_t B_DADDR_IDX  = 4'd2;
  parameter reg_idx_t B_DPTCH_IDX  = 4'd3;
  parameter reg_idx_t B_OFFSET_IDX = 4'd4;
  parameter reg_idx_t B_DYDX_IDX   = 4'd7;
  parameter reg_idx_t B_COLOR0_IDX = 4'd8;
  parameter reg_idx_t B_COLOR1_IDX = 4'd9;
  // LINE (and other incremental-draw) implied B registers.
  parameter reg_idx_t B_COUNT_IDX  = 4'd10; // COUNT (loop count) / SADDR-d alias
  parameter reg_idx_t B_INC1_IDX   = 4'd11; // INC1 (minor/diagonal XY increment)
  parameter reg_idx_t B_INC2_IDX   = 4'd12; // INC2 (major/dominant XY increment)

  // ---------------------------------------------------------------------------
  // ALU operation enum
  //
  // Spec sources:
  //   - bibliography/hdl-reimplementation/02-instruction-set.md
  //     §"Arithmetic / Logical": "N, C, Z, V plus the field-size mode bits
  //     ... all live in the ST register".
  //   - bibliography/hdl-reimplementation/03-registers.md §"Status register".
  //
  // Concrete per-instruction flag semantics and write masks were reconciled
  // against every implemented instruction page in Task 0135; see
  // docs/status_audit.md. The ALU provides the standard arithmetic values:
  //   C = unsigned overflow (carry from MSB on ADD; "borrow" output on SUB,
  //       which here equals NOT carry-out-of-(a + ~b + 1)).
  //   V = signed overflow.
  //   N = result[31].
  //   Z = result == 0.
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    ALU_OP_ADD    = 4'd0,
    ALU_OP_ADDC   = 4'd1,
    ALU_OP_SUB    = 4'd2,
    ALU_OP_SUBB   = 4'd3,
    ALU_OP_CMP    = 4'd4,
    ALU_OP_AND    = 4'd5,
    ALU_OP_ANDN   = 4'd6,
    ALU_OP_OR     = 4'd7,
    ALU_OP_XOR    = 4'd8,
    ALU_OP_NOT    = 4'd9,
    ALU_OP_NEG    = 4'd10,
    ALU_OP_PASS_A = 4'd11,
    ALU_OP_PASS_B = 4'd12,
    ALU_OP_ABS    = 4'd13   // |a|; result mux'd between a and (0-a) based on
                            // sign bit. V on MIN_INT. See A0024.
  } alu_op_t;

  typedef struct packed {
    logic n;
    logic c;
    logic z;
    logic v;
  } alu_flags_t;

  // ---------------------------------------------------------------------------
  // Shifter operation enum
  //
  // Spec source: bibliography/hdl-reimplementation/01-architecture.md
  //   §"Top-level blocks" — "32-bit barrel shifter; the shifter is critical
  //   for field operations and pixel shifts"; 02-instruction-set.md lists
  //   SLA, SLL, SRA, SRL, RL as the shift/rotate primitives.
  //
  // SLA vs SLL is per-spec: the shifted output is the same, but SLA reports
  // overflow when the new sign or any shifted-out bit differs from the old
  // sign. Task 0130 resolved the complete shift-family status policy.
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    SHIFT_OP_SLL = 3'd0,  // shift left, logical (fill 0)
    SHIFT_OP_SLA = 3'd1,  // shift left, arithmetic (SLL result + SLA overflow)
    SHIFT_OP_SRL = 3'd2,  // shift right, logical (fill 0)
    SHIFT_OP_SRA = 3'd3,  // shift right, arithmetic (sign-extend)
    SHIFT_OP_RL  = 3'd4,  // rotate left
    SHIFT_OP_RR  = 3'd5   // rotate right
  } shift_op_t;

  parameter int unsigned SHIFT_AMOUNT_WIDTH = 5;  // 32-bit shifter, 0..31

  // ---------------------------------------------------------------------------
  // Status register layout — VERIFIED against SPVU001A §5.2 (Table 5-2,
  // page 5-18). The original A0010 placeholder for N/C/Z/V at bits
  // 31..28 happened to be correct; the field-size bits and IE/PBX are
  // now pinned to their authoritative positions.
  //
  //   Bits[4:0]   FS0     Field Size 0  (5-bit; 00000 → field-size 32)
  //   Bit[5]      FE0     Field Extend 0 (0 = zero-ext, 1 = sign-ext)
  //   Bits[10:6]  FS1     Field Size 1
  //   Bit[11]     FE1     Field Extend 1
  //   Bits[20:12] reserved (preserved as written; reset = 0)
  //   Bit[21]     IE      Interrupt Enable (DINT clears, EINT sets)
  //   Bits[24:22] reserved
  //   Bit[25]     PBX     PixBlt Executing
  //   Bits[27:26] reserved
  //   Bit[28]     V       Overflow
  //   Bit[29]     Z       Zero
  //   Bit[30]     C       Carry
  //   Bit[31]     N       Negative
  //
  // ST resets to 0x0000_0010 per spec page 5-18 → FS0 = 16 at reset.
  // ---------------------------------------------------------------------------
  parameter int unsigned ST_FS0_LO  = 0;
  parameter int unsigned ST_FS0_HI  = 4;
  parameter int unsigned ST_FE0_BIT = 5;
  parameter int unsigned ST_FS1_LO  = 6;
  parameter int unsigned ST_FS1_HI  = 10;
  parameter int unsigned ST_FE1_BIT = 11;
  parameter int unsigned ST_IE_BIT  = 21;
  parameter int unsigned ST_PBX_BIT = 25;
  parameter int unsigned ST_V_BIT   = 28;
  parameter int unsigned ST_Z_BIT   = 29;
  parameter int unsigned ST_C_BIT   = 30;
  parameter int unsigned ST_N_BIT   = 31;
  parameter logic [DATA_WIDTH-1:0] ST_RESET_VALUE = 32'h0000_0010;

  // ---------------------------------------------------------------------------
  // Misc architectural constants (kept here so RTL has no magic numbers).
  // ---------------------------------------------------------------------------
  // REV Rd result. The defined TMS34010 format and worked example on the
  // 1988 User's Guide page 12-233 produce 0x00000008 (A0025 resolved).
  parameter logic [DATA_WIDTH-1:0] REV_VALUE = 32'h0000_0008;
  // MOVK/ADDK/SUBK encode architectural constant 32 with a zero K field.
  parameter logic [DATA_WIDTH-1:0] K_ZERO_VALUE = DATA_WIDTH'(32);
  // Trap-vector table top. Vector for TRAP N is TRAP_VECTOR_BASE - N*32.
  // Per SPVU001A page 12-252; TRAP 0's vector lives at the base itself.
  parameter logic [DATA_WIDTH-1:0] TRAP_VECTOR_BASE = 32'hFFFF_FFE0;
  // Reset reads the same level-0 vector as TRAP 0 (1988 User's Guide
  // pages 8-10 and 8-12).
  parameter logic [ADDR_WIDTH-1:0] RESET_VECTOR_ADDR = TRAP_VECTOR_BASE;

  // One architectural word is DATA_WIDTH bits. Stack push/pop and the
  // field-move pointers (at field-size 32) step the bit-address SP /
  // pointer by one word per 32-bit transfer; the multi-push sequences
  // (TRAP, RETI) step by two words.
  parameter logic [ADDR_WIDTH-1:0] WORD_BIT_SIZE   = ADDR_WIDTH'(DATA_WIDTH);     // 32
  parameter logic [ADDR_WIDTH-1:0] WORD_BIT_SIZE_2 = ADDR_WIDTH'(2 * DATA_WIDTH); // 64
  // Memory-transfer size (in bits) of a full 32-bit word access, as it
  // appears on the mem_size port (FIELD_SIZE_WIDTH bits wide).
  parameter logic [FIELD_SIZE_WIDTH-1:0] MEM_SIZE_32 = FIELD_SIZE_WIDTH'(DATA_WIDTH); // 6'd32

  // ---------------------------------------------------------------------------
  // On-chip I/O register file (1988 User's Guide Figure 6-1, page 6-3).
  //
  // 32 memory-mapped 16-bit registers occupy the bit-address range
  // 0xC0000000-0xC00001FF. Each register sits at a 0x10-bit-aligned address
  // (16 bits apart). An address is in I/O space when its two MSBs are 11 and
  // bits[29:9] are 0; the register index is addr[8:4]. All registers reset to
  // 0 except HSTCTLH.HLT, whose reset value is selected by HCS (UG §6.1).
  // ---------------------------------------------------------------------------
  parameter logic [ADDR_WIDTH-1:0] IO_BASE_ADDR  = 32'hC000_0000;
  parameter int unsigned           IO_REG_COUNT  = 32;
  parameter int unsigned           IO_REG_IDX_W  = 5;     // addr[8:4]
  // Register indices (= address[8:4] = (addr - IO_BASE) >> 4).
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HESYNC  = 5'h00; // Horizontal End Sync
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HEBLNK  = 5'h01; // Horizontal End Blank
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HSBLNK  = 5'h02; // Horizontal Start Blank
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HTOTAL  = 5'h03; // Horizontal Total
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_VESYNC  = 5'h04; // Vertical End Sync
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_VEBLNK  = 5'h05; // Vertical End Blank
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_VSBLNK  = 5'h06; // Vertical Start Blank
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_VTOTAL  = 5'h07; // Vertical Total
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_DPYCTL  = 5'h08; // Display Control
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_DPYSTRT = 5'h09; // Display Start
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_DPYINT  = 5'h0A; // Display Interrupt
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_CONTROL = 5'h0B; // Control
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HSTDATA = 5'h0C; // Host Data
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HSTADRL = 5'h0D; // Host Address (LSBs)
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HSTADRH = 5'h0E; // Host Address (MSBs)
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HSTCTLL = 5'h0F; // Host Control (LSBs)
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HSTCTLH = 5'h10; // Host Control (MSBs)
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_INTENB  = 5'h11; // Interrupt Enable
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_INTPEND = 5'h12; // Interrupt Pending
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_CONVSP  = 5'h13; // Source Conversion Pitch
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_CONVDP  = 5'h14; // Destination Conversion Pitch
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_PSIZE   = 5'h15; // Pixel Size
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_PMASK   = 5'h16; // Plane Mask
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_RESERVED_17 = 5'h17;
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_RESERVED_18 = 5'h18;
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_RESERVED_19 = 5'h19;
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_RESERVED_1A = 5'h1A;
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_DPYTAP  = 5'h1B; // Display Tap Point
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_HCOUNT  = 5'h1C; // Horizontal Count
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_VCOUNT  = 5'h1D; // Vertical Count
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_DPYADR  = 5'h1E; // Display Address
  parameter logic [IO_REG_IDX_W-1:0] IO_IDX_REFCNT  = 5'h1F; // DRAM Refresh Count

  // Host-interface register selected by HFS1:HFS0 (User's Guide Table 10-1).
  typedef enum logic [1:0] {
    HOST_REG_HSTADRL = 2'b00,
    HOST_REG_HSTADRH = 2'b01,
    HOST_REG_HSTDATA = 2'b10,
    HOST_REG_HSTCTL  = 2'b11
  } host_reg_sel_t;

  // Display Control register fields used by the integrated timing generator
  // (1988 User's Guide DPYCTL pages 6-18 through 6-23).
  parameter int unsigned DPYCTL_HSD_BIT    = 0;  // external HSYNC direction
  parameter int unsigned DPYCTL_DUDATE_LO = 2;  // one-hot display update
  parameter int unsigned DPYCTL_DUDATE_HI = 9;
  parameter int unsigned DPYCTL_ORG_BIT    = 10; // 0=invert pins, 1=direct pins
  parameter int unsigned DPYCTL_SRT_BIT    = 11; // programmed pixel transfers
  parameter int unsigned DPYCTL_SRE_BIT    = 12; // automatic screen refresh
  parameter int unsigned DPYCTL_DXV_BIT    = 13; // 1=internal video timing
  parameter int unsigned DPYCTL_NIL_BIT    = 14; // 0=interlaced, 1=noninterlaced
  parameter int unsigned DPYCTL_ENV_BIT    = 15; // enable video/display IRQ
  parameter logic [15:0] DPYCTL_WRITABLE_MASK = 16'hFFFD;

  // DPYADR/DPYSTRT split into a two-bit line count and 14-bit screen address.
  parameter int unsigned DPY_LNCNT_LO  = 0;
  parameter int unsigned DPY_LNCNT_HI  = 1;
  parameter int unsigned DPY_SRFADR_LO = 2;
  parameter int unsigned DPY_SRFADR_HI = 15;
  parameter int unsigned DPYTAP_LO      = 0;
  parameter int unsigned DPYTAP_HI      = 13;
  parameter logic [15:0] DPYTAP_MASK    = 16'h3FFF;

  // Interrupt enable/pending (INTENB/INTPEND) bit positions and the trap vector
  // address of each maskable interrupt (1988 UG §8.3/8.4, Tables 8-2/8-3).
  // Priority (high→low): HI > DI > WV > INT1 > INT2 (internal before external).
  parameter int unsigned INT_X1_BIT = 1;   // external interrupt 1
  parameter int unsigned INT_X2_BIT = 2;   // external interrupt 2
  parameter int unsigned INT_HI_BIT = 9;   // host interrupt
  parameter int unsigned INT_DI_BIT = 10;  // display interrupt
  parameter int unsigned INT_WV_BIT = 11;  // window violation
  parameter logic [15:0] INT_SOURCE_MASK =
      (16'h0001 << INT_X1_BIT)
    | (16'h0001 << INT_X2_BIT)
    | (16'h0001 << INT_HI_BIT)
    | (16'h0001 << INT_DI_BIT)
    | (16'h0001 << INT_WV_BIT);
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_X1 = 32'hFFFF_FFC0; // trap 1
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_X2 = 32'hFFFF_FFA0; // trap 2
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_HI = 32'hFFFF_FEC0; // trap 9
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_DI = 32'hFFFF_FEA0; // trap 10
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_WV = 32'hFFFF_FE80; // trap 11
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_ILLOP = 32'hFFFF_FC20; // trap 30

  // Nonmaskable interrupt (NMI): host sets HSTCTLH.NMI; the device vectors
  // through trap 8 and (if NMIM=0) pushes PC+ST. 1988 UG §8 (Table 8-2,
  // page 8-? "NMI ... 8 ... FFFFFEE0h") and HSTCTLH (page 5507/4322).
  parameter logic [ADDR_WIDTH-1:0] INT_VEC_NMI   = 32'hFFFF_FEE0; // trap 8
  parameter int unsigned           HSTCTL_NMI_BIT  = 8;  // HSTCTLH: NMI request
  parameter int unsigned           HSTCTL_NMIM_BIT = 9;  // HSTCTLH: NMI mode (1=no push)
  parameter int unsigned           HSTCTL_INCW_BIT = 11; // increment pointer after host write
  parameter int unsigned           HSTCTL_INCR_BIT = 12; // increment pointer before host read
  parameter int unsigned           HSTCTL_LBL_BIT  = 13; // lower-byte-last host ordering
  parameter int unsigned           HSTCTL_CF_BIT   = 14; // instruction-cache flush/disable
  parameter int unsigned           HSTCTL_HLT_BIT  = 15; // halt processor at next boundary
  parameter int unsigned           HSTCTL_INTIN_BIT  = 3; // HSTCTLL: host-to-GSP IRQ
  parameter int unsigned           HSTCTL_INTOUT_BIT = 7; // HSTCTLL: GSP-to-host IRQ
  parameter logic [15:0] HSTCTLH_WRITABLE_MASK =
      (16'h0001 << HSTCTL_NMI_BIT)
    | (16'h0001 << HSTCTL_NMIM_BIT)
    | (16'h0001 << HSTCTL_INCW_BIT)
    | (16'h0001 << HSTCTL_INCR_BIT)
    | (16'h0001 << HSTCTL_LBL_BIT)
    | (16'h0001 << HSTCTL_CF_BIT)
    | (16'h0001 << HSTCTL_HLT_BIT);

  // CONTROL register bit fields (1988 UG page 4-? CONTROL register).
  parameter int unsigned CTRL_RM_BIT   = 2;   // DRAM refresh mode: 0=RAS-only
  parameter int unsigned CTRL_RR_LO    = 3;   // DRAM refresh rate
  parameter int unsigned CTRL_RR_HI    = 4;
  parameter int unsigned CTRL_T_BIT    = 5;   // pixel transparency enable
  parameter int unsigned CTRL_W_LO     = 6;   // window violation detection mode
  parameter int unsigned CTRL_W_HI     = 7;
  parameter int unsigned CTRL_PBH_BIT  = 8;   // PixBlt horizontal direction
  parameter int unsigned CTRL_PBV_BIT  = 9;   // PixBlt vertical direction
  parameter int unsigned CTRL_PPOP_LO  = 10;  // pixel-processing operation select
  parameter int unsigned CTRL_PPOP_HI  = 14;
  parameter logic [15:0] CONTROL_WRITABLE_MASK = 16'hFFFC;

  // ---------------------------------------------------------------------------
  // Instruction word + decoded-instruction control
  //
  // Spec: bibliography/hdl-reimplementation/02-instruction-set.md
  //   §"Encoding shape": "16-bit-aligned half-words ... The decode space is
  //   dense — there is no easy 'top-bits-give-class' partition. Use the
  //   SPVU004 opcode chart ... rather than hand-rolling the decoder."
  //
  // `decoded_instr_t` carries instruction class, operands, memory/extension
  // requirements, writeback enables, and the graphics-specific mode flags.
  // ---------------------------------------------------------------------------
  parameter int unsigned INSTR_WORD_WIDTH = 16;
  typedef logic [INSTR_WORD_WIDTH-1:0] instr_word_t;

  // Instruction class — used by the core control FSM to pick the
  // decode/execute/memory/writeback path. It has widened as classes were
  // added; seven bits currently cover classes through INSTR_LINE.
  typedef enum logic [6:0] {
    INSTR_ILLEGAL    = 7'd0,
    INSTR_MOVI_IW    = 7'd1,  // MOVI IW K, Rd    — 16-bit sign-extended immediate
    INSTR_MOVI_IL    = 7'd2,  // MOVI IL K, Rd    — 32-bit immediate
    INSTR_MOVK       = 7'd3,  // MOVK K, Rd       — 5-bit zero-extended constant
    INSTR_ADD_RR     = 7'd4,  // ADD Rs, Rd       — Rs + Rd → Rd; both same file
    INSTR_SUB_RR     = 7'd5,  // SUB Rs, Rd       — Rd - Rs → Rd; both same file
    INSTR_AND_RR     = 7'd6,  // AND Rs, Rd       — Rd & Rs  → Rd
    INSTR_ANDN_RR    = 7'd7,  // ANDN Rs, Rd      — Rd & ~Rs → Rd
    INSTR_OR_RR      = 7'd8,  // OR Rs, Rd        — Rd | Rs  → Rd
    INSTR_XOR_RR     = 7'd9,  // XOR Rs, Rd       — Rd ^ Rs  → Rd
    INSTR_CMP_RR     = 7'd10, // CMP Rs, Rd       — flags from (Rd - Rs); Rd unchanged
    INSTR_JRCC_SHORT = 7'd11, // JRcc short       — conditional relative jump
                              //                    (cc=0 = UC = unconditional)
    INSTR_ADDK       = 7'd12, // ADDK K, Rd       — K + Rd → Rd  (K = 5-bit, zext)
    INSTR_SUBK       = 7'd13, // SUBK K, Rd       — Rd - K → Rd  (K = 5-bit, zext)
    INSTR_NEG        = 7'd14, // NEG Rd           — 0 - Rd → Rd
    INSTR_NOT        = 7'd15, // NOT Rd           — ~Rd → Rd     (Z only)
    INSTR_ADDI_IW    = 7'd16, // ADDI IW K, Rd    — Rd + sext(K16) → Rd
    INSTR_SUBI_IW    = 7'd17, // SUBI IW K, Rd    — Rd - sext(K16) → Rd
    INSTR_CMPI_IW    = 7'd18, // CMPI IW K, Rd    — flags from Rd - sext(K16); Rd unchanged
    INSTR_SLA_K      = 7'd19, // SLA K, Rd        — Rd << K (arithmetic, may set V)
    INSTR_SLL_K      = 7'd20, // SLL K, Rd        — Rd << K (logical)
    INSTR_SRA_K      = 7'd21, // SRA K, Rd        — Rd >>> K (arithmetic / sign-extend)
    INSTR_SRL_K      = 7'd22, // SRL K, Rd        — Rd >> K  (logical, MSB ← 0)
    INSTR_RL_K       = 7'd23, // RL K, Rd         — Rd ROL K (rotate left)
    INSTR_ADDI_IL    = 7'd24, // ADDI IL K, Rd    — Rd + K32 → Rd
    INSTR_SUBI_IL    = 7'd25, // SUBI IL K, Rd    — Rd - K32 → Rd
    INSTR_CMPI_IL    = 7'd26, // CMPI IL K, Rd    — flags from Rd - K32; Rd unchanged
    INSTR_ANDI_IL    = 7'd27, // ANDI/ANDNI IL,Rd — Rd & ~extension → Rd
    INSTR_ORI_IL     = 7'd28, // ORI  IL K, Rd    — Rd | K32 → Rd
    INSTR_XORI_IL    = 7'd29, // XORI IL K, Rd    — Rd ^ K32 → Rd
    INSTR_MOVE_RR    = 7'd30, // MOVE Rs, Rd      — Rs → Rd (same-file reg-reg)
    INSTR_NOP        = 7'd31, // NOP              — no operation, PC advances only
    INSTR_ADDC_RR    = 7'd32, // ADDC Rs, Rd      — Rs + Rd + C → Rd (carry-in)
    INSTR_SUBB_RR    = 7'd33, // SUBB Rs, Rd      — Rd - Rs - C → Rd (borrow-in)
    INSTR_JRCC_LONG  = 7'd34, // JRcc Address     — long form: 16-bit signed disp
                              //                    in the following word; target =
                              //                    PC_after_both_fetches + disp*16
    INSTR_JUMP_RS    = 7'd35, // JUMP Rs          — Rs → PC (bottom 4 bits cleared)
    INSTR_DSJ        = 7'd36, // DSJ Rd, Address  — Rd-1→Rd; if Rd!=0 branch (long form)
    INSTR_DSJEQ      = 7'd37, // DSJEQ Rd, Address — if Z=1: DSJ semantics; else skip
    INSTR_DSJNE      = 7'd38, // DSJNE Rd, Address — if Z=0: DSJ semantics; else skip
    INSTR_JACC       = 7'd39, // JAcc Address      — absolute-form conditional jump
                              //                     (low byte = 0x80; 32-bit abs addr follows)
    INSTR_DSJS       = 7'd40, // DSJS Rd, Address  — short form: 5-bit offset + 1-bit
                              //                     direction; single-word instruction
    INSTR_ABS        = 7'd41, // ABS Rd            — |Rd| → Rd (V on MIN_INT)
    INSTR_NEGB       = 7'd42, // NEGB Rd           — (2s_comp Rd) - C → Rd (borrow-in)
    INSTR_BTST_K     = 7'd43, // BTST K, Rd        — test bit K of Rd; only Z updates
    INSTR_BTST_RR    = 7'd44, // BTST Rs, Rd       — test bit Rs[4:0] of Rd; Z only
    INSTR_CLRC       = 7'd45, // CLRC              — ST.C ← 0; N, Z, V unchanged
    INSTR_SETC       = 7'd46, // SETC              — ST.C ← 1; N, Z, V unchanged
    INSTR_GETST      = 7'd47, // GETST Rd          — Rd ← ST
    INSTR_PUTST      = 7'd48, // PUTST Rs          — ST ← Rs (full 32-bit write)
    INSTR_SLA_RR     = 7'd49, // SLA Rs, Rd        — shift left arithmetic by Rs[4:0]
    INSTR_SLL_RR     = 7'd50, // SLL Rs, Rd        — shift left logical by Rs[4:0]
    INSTR_SRA_RR     = 7'd51, // SRA Rs, Rd        — shift right arithmetic by 2sCmp(Rs[4:0])
    INSTR_SRL_RR     = 7'd52, // SRL Rs, Rd        — shift right logical by 2sCmp(Rs[4:0])
    INSTR_RL_RR      = 7'd53, // RL  Rs, Rd        — rotate left by Rs[4:0]
    INSTR_GETPC      = 7'd54, // GETPC Rd          — Rd ← PC (current bit-addressed PC)
    INSTR_EXGPC      = 7'd55, // EXGPC Rd          — swap PC ↔ Rd (PC's low 4 bits cleared)
    INSTR_REV        = 7'd56, // REV Rd            — Rd ← chip-revision number constant
    INSTR_LMO_RR     = 7'd57, // LMO Rs, Rd        — Rd ← 31 - bit_pos(leftmost-1 in Rs)
                              //                     in bottom 5 bits; upper 27 bits = 0;
                              //                     Z = (Rs == 0); N/C/V unaffected
    INSTR_SETF       = 7'd58, // SETF FS, FE, F    — set field-size params for FS<F>/FE<F>
                              //                     (F bit at instr[9]; FE at instr[5];
                              //                     FS at instr[4:0]). Status unaffected.
    INSTR_SEXT       = 7'd59, // SEXT Rd, F        — sign-extend low FS<F> bits to 32;
                              //                     flags N, Z (C, V unaffected via mask).
    INSTR_ZEXT       = 7'd60, // ZEXT Rd, F        — zero-extend low FS<F> bits to 32;
                              //                     flag Z only (N, C, V unaffected).
    INSTR_EXGF       = 7'd61, // EXGF Rd, F        — atomic swap Rd[5:0] ↔ FE<F>:FS<F>
                              //                     in ST; Rd[31:6] cleared. Status
                              //                     bits all "Unaffected".
    INSTR_DINT       = 7'd62, // DINT              — clear ST.IE  (disable interrupts)
    INSTR_EINT       = 7'd63, // EINT              — set ST.IE    (enable interrupts)
    INSTR_PUSHST     = 7'd64, // PUSHST            — SP -= 32; mem[SP] <- ST (32-bit write)
    INSTR_POPST      = 7'd65, // POPST             — ST <- mem[SP]; SP += 32 (32-bit read)
    INSTR_CALL_RS    = 7'd66, // CALL Rs           — SP -= 32; mem[SP] <- PC'; PC <- Rs
                              //                     (with bottom 4 bits forced to 0).
    INSTR_RETS       = 7'd67, // RETS [N]          — PC <- mem[SP]; SP += 32 + 16*N
                              //                     N at instr[4:0], range 0..31.
    INSTR_CALLA      = 7'd68, // CALLA Address     — SP -= 32; mem[SP] <- PC';
                              //                     PC <- absolute (32-bit) with bottom
                              //                     4 bits forced to 0.
    INSTR_CALLR      = 7'd69, // CALLR Address     — SP -= 32; mem[SP] <- PC';
                              //                     PC <- PC' + sign_ext(disp16)*16.
    INSTR_RETI       = 7'd70, // RETI              — ST <- mem[SP]; SP += 32;
                              //                     PC <- mem[SP]; SP += 32.
                              //                     Two-step pop. Status from popped ST.
    INSTR_TRAP       = 7'd71, // TRAP N            — SP -= 32; mem[SP] <- PC';
                              //                     SP -= 32; mem[SP] <- ST;
                              //                     ST <- 0x00000010; PC <- mem[vec].
                              //                     vec = 0xFFFFFFE0 - N*32. N at instr[4:0]
                              //                     (range 0..31). N>0 uses write/write/read;
                              //                     TRAP 0 performs only the vector read.
    INSTR_MMTM       = 7'd72, // MMTM Rp, list     — For each register Rn in the list
                              //                     (lowest-order first): Rp -= 32; mem[Rp] <- Rn.
                              //                     Second word = 16-bit mask (bit N = Rn). Up
                              //                     to 16 32-bit writes per instruction.
                              //                     N flag set to sign of (0 - initial Rp); see
                              //                     SPVU001A page 12-111 for edge cases (deferred).
    INSTR_MMFM       = 7'd73, // MMFM Rp, list     — For each register Rn in the list
                              //                     (highest-order first): Rn <- mem[Rp]; Rp += 32.
                              //                     Pop counterpart of MMTM; reverses its action.
                              //                     Second word = 16-bit mask (bit N = Rn). Up to
                              //                     16 32-bit reads. Final Rp = initial + 32*count.
                              //                     All flags Unaffected (SPVU001A page 12-109).
    INSTR_MOVE_FIELD_STORE = 7'd74, // MOVE Rs,*Rd  — field in Rs -> mem[*Rd]. Rd is a bit-address
                              //                     pointer (unchanged). Encoding 1000 00FS SSSR DDDD.
                              //                     All flags Unaffected. Task 0059 implements the
                              //                     field-size-32, word-aligned case only.
    INSTR_MOVE_FIELD_LOAD  = 7'd75, // MOVE *Rs,Rd  — field at mem[*Rs] -> Rd, sign/zero-extended.
                              //                     Rs is a bit-address pointer. Encoding
                              //                     1000 01FS SSSR DDDD. Implicit compare-to-0:
                              //                     N=sign, Z=zero, V=0, C Unaffected. Task 0059
                              //                     implements field-size-32, word-aligned only.
    INSTR_MOVE_FIELD_M2M   = 7'd76, // MOVE *Rs,*Rd — field at mem[*Rs] -> mem[*Rd]. Both Rs and
                              //                     Rd are bit-address pointers (unchanged for the
                              //                     plain form). Encoding 1000 10FS SSSR DDDD.
                              //                     Two memory transactions (read then write). All
                              //                     flags Unaffected. Task 0061: plain form only,
                              //                     field-size-32, word-aligned.
    INSTR_MOVE_ABS_STORE   = 7'd77, // MOVE Rs,@DAddr — field in Rs -> mem[DAddr]. DAddr is a
                              //                     32-bit absolute bit-address (2 words follow the
                              //                     opcode). Encoding 0000 01F1 100R SSSS. All flags
                              //                     Unaffected. Task 0063: field-size-32, word-aligned.
    INSTR_MOVE_ABS_LOAD    = 7'd78, // MOVE @SAddr,Rd — field at mem[SAddr] -> Rd, sign/zero-ext.
                              //                     SAddr is a 32-bit absolute bit-address. Encoding
                              //                     0000 01F1 101R DDDD. Implicit compare-to-0:
                              //                     N=sign, Z=zero, V=0, C Unaffected. Task 0063.
    INSTR_MOVE_OFF_STORE   = 7'd79, // MOVE Rs,*Rd(off) — field in Rs -> mem[Rd + sext(off16)].
                              //                     Rd pointer unchanged; off16 is the 2nd word.
                              //                     Encoding 1011 00FS SSSR DDDD. All flags
                              //                     Unaffected. Task 0064: field-size-32, aligned.
    INSTR_MOVE_OFF_LOAD    = 7'd80, // MOVE *Rs(off),Rd — field at mem[Rs + sext(off16)] -> Rd.
                              //                     Rs pointer unchanged. Encoding 1011 01FS SSSR
                              //                     DDDD. Implicit compare-to-0: N=sign, Z=zero,
                              //                     V=0, C Unaffected. Task 0064.
    INSTR_MOVX             = 7'd81, // MOVX Rs,Rd — RsX -> RdX (low 16 bits). Rd's Y half (high 16)
                              //                     unaffected. Encoding 1110 110S SSSR DDDD. All
                              //                     flags Unaffected. Task 0065.
    INSTR_MOVY             = 7'd82, // MOVY Rs,Rd — RsY -> RdY (high 16 bits). Rd's X half (low 16)
                              //                     unaffected. Encoding 1110 111S SSSR DDDD. All
                              //                     flags Unaffected. Task 0065.
    INSTR_ADDXY            = 7'd83, // ADDXY Rs,Rd — RdX += RsX, RdY += RsY (independent 16-bit, no
                              //                     carry between halves). Encoding 1110 000S SSSR
                              //                     DDDD. Status (graphics): N=(Xres==0), V=Xres[15],
                              //                     Z=(Yres==0), C=Yres[15]. Task 0066.
    INSTR_SUBXY            = 7'd84, // SUBXY Rs,Rd — RdX -= RsX, RdY -= RsY (independent 16-bit).
                              //                     Encoding 1110 001S SSSR DDDD. Status (compare):
                              //                     N=(RsX==RdX), V=(RsX>RdX), Z=(RsY==RdY),
                              //                     C=(RsY>RdY), signed 16-bit. Tasks 0066/0134.
    INSTR_CMPXY            = 7'd85, // CMPXY Rs,Rd — nondestructive XY compare (Rd unchanged). Sets
                              //                     status as if RdX-RsX / RdY-RsY: N=(Xres==0),
                              //                     V=Xres[15], Z=(Yres==0), C=Yres[15]. Encoding
                              //                     1110 010S SSSR DDDD. Task 0067.
    INSTR_CPW              = 7'd86, // CPW Rs,Rd — compare the XY point in Rs against the window
                              //                     corners WSTART (B5) / WEND (B6); load the 4-bit
                              //                     out-of-window code into Rd[8:5]. V=outside-window.
                              //                     Encoding 1110 011S SSSR DDDD. Task 0070.
    INSTR_MPYS             = 7'd87, // MPYS Rs,Rd — signed 32x32 -> 64-bit. Even Rd: {Rd=hi32,
                              //                     Rd+1=lo32}; odd Rd: lo32 in Rd. N/Z follow
                              //                     the stored 64- or 32-bit result respectively.
                              //                     Task 0071 implements FS1=32 (full 32-bit Rs).
    INSTR_MPYU             = 7'd88, // MPYU Rs,Rd — unsigned 32x32 -> 64-bit (same writeback as
                              //                     MPYS). Z follows stored result width; N U.
                              //                     0101 111S SSSR DDDD. Task 0071 (FS1=32).
    INSTR_DIVU             = 7'd89, // DIVU Rs,Rd — unsigned divide. Even Rd: {Rd:Rd+1}/Rs ->
                              //                     quotient in Rd, remainder in Rd+1. Odd Rd:
                              //                     Rd/Rs -> quotient in Rd. V=overflow (Rs=0 or
                              //                     quotient>32b; result unchanged), Z=quotient==0,
                              //                     N Unaffected. Encoding 0101 101S SSSR DDDD.
    INSTR_MODU             = 7'd90, // MODU Rs,Rd — unsigned 32-bit modulo: Rd mod Rs -> Rd
                              //                     (remainder). V=(Rs==0; Rd unchanged), Z=remainder
                              //                     ==0 (unaffected if Rs==0), N Unaffected. Encoding
                              //                     0110 111S SSSR DDDD. Reuses the divider.
    INSTR_DIVS             = 7'd91, // DIVS Rs,Rd — signed divide (even Rd: 64-bit {Rd:Rd+1}).
                              //                     Quotient->Rd (+remainder->Rd+1 even); N follows
                              //                     quotient/0x80000000 rules, Z=zero, V=overflow.
    INSTR_MODS             = 7'd92, // MODS Rs,Rd — signed 32-bit modulo: Rd mod Rs -> Rd
                              //                     (remainder has dividend's sign). N/C U;
                              //                     Z=remainder zero except Rs=0; V=quot overflow.
    INSTR_CVXYL            = 7'd93, // CVXYL Rs,Rd — convert the XY address in Rs to a linear
                              //                     address in Rd: ((Y<<(31-CONVDP)) | (X<<log2
                              //                     PSIZE)) + OFFSET(B4). Flags Unaffected.
                              //                     Encoding 1110 100S SSSR DDDD.
    INSTR_FILL_L           = 7'd94, // FILL L — fill a DY×DX pixel array with COLOR1 (B9).
                              //                     Implied B-regs DADDR(B2)/DPTCH(B3)/DYDX(B7);
                              //                     multi-cycle. DADDR updated. Encoding 0x0FC0.
    INSTR_FILL_XY          = 7'd95, // FILL XY — like FILL L but DADDR (B2) holds an XY value,
                              //                     converted to linear (CONVDP+OFFSET+PSIZE)
                              //                     before the fill. Encoding 0x0FE0.
    INSTR_PIXBLT_LL        = 7'd96, // PIXBLT L,L — transfer a DY×DX source array (SADDR=B0/
                              //                     SPTCH=B1) to a dest array (DADDR=B2/DPTCH=B3),
                              //                     processing each pixel. Encoding 0x0F00.
    INSTR_DRAV             = 7'd97, // DRAV Rs,Rd — draw COLOR1 at Rd's XY (PSIZE pixel, with
                              //                     PPOP/T/PMASK), then Rd += Rs as an XY add
                              //                     (no carry X->Y). Encoding 0xF600.
    INSTR_LINE             = 7'd98, // LINE Z — Bresenham inner loop: draw COUNT pixels of
                              //                     COLOR1 along a line. B-file: d/DADDR/DYDX
                              //                     (b:a)/COUNT/INC1/INC2. Enc 0xDF1A/0xDF9A.
    INSTR_MOVB_OFF_M2M     = 7'd99, // MOVB *Rs(SOff),*Rd(DOff): two signed offsets;
                              //                     8-bit memory-to-memory transfer.
    INSTR_MOVB_ABS_M2M     = 7'd100, // MOVB @SAddr,@DAddr: two 32-bit bit addresses;
                              //                     8-bit memory-to-memory transfer.
    INSTR_MOVE_OFF_M2M_PI  = 7'd101, // MOVE *Rs(off),*Rd+: signed source offset;
                              //                     destination postincrements by FS.
    INSTR_MOVE_ABS_M2M_PI  = 7'd102, // MOVE @SAddr,*Rd+: absolute source;
                              //                     destination postincrements by FS.
    INSTR_EMU              = 7'd103, // EMU: pulse EMUA; NOP in RUN or halt in EMU
    INSTR_MOVE_ABS_M2M     = 7'd104, // MOVE @SAddr,@DAddr[,F]: two 32-bit
                              //                     absolute bit addresses; FS-bit
                              //                     transfer, all status unaffected.
    INSTR_MOVE_OFF_M2M     = 7'd105  // MOVE *Rs(SOff),*Rd(DOff)[,F]:
                              //                     two signed 16-bit offsets; FS-bit
                              //                     transfer, pointers unaffected.
  } instr_class_t;

  // Condition codes used by JRcc / JAcc (and other conditional ops).
  // Source: SPVU001A Table 12-8 ("Condition Codes for JRcc and JAcc
  // Instructions"), as re-extracted cleanly with `pdftotext -layout`
  // from the long-form JRcc page (12-96). The earlier (A0017) hand-
  // guess of EQ=0100 and NE=0111 was WRONG — those codes are actually
  // LT and GT, respectively. Corrected in Task 0030; see A0023.
  parameter logic [3:0] CC_UC = 4'b0000;  // unconditional
  parameter logic [3:0] CC_LO = 4'b0001;  // lower-than       (unsigned; C = 1; alias "B")
  parameter logic [3:0] CC_LS = 4'b0010;  // lower-or-same    (unsigned; C | Z = 1)
  parameter logic [3:0] CC_HI = 4'b0011;  // higher-than      (unsigned; ~C & ~Z = 1)
  parameter logic [3:0] CC_LT = 4'b0100;  // less-than        (signed;   N ^ V = 1)
  parameter logic [3:0] CC_GE = 4'b0101;  // greater-or-equal (signed;   N ^ V = 0; alias "JRZ" in spec when comparing-to-zero? — see A0023)
  parameter logic [3:0] CC_LE = 4'b0110;  // less-or-equal    (signed;   (N ^ V) | Z = 1)
  parameter logic [3:0] CC_GT = 4'b0111;  // greater-than     (signed;   !(N ^ V) & !Z = 1)
  parameter logic [3:0] CC_HS = 4'b1001;  // higher-or-same   (unsigned; C = 0; alias "NC"/"NB")
  parameter logic [3:0] CC_EQ = 4'b1010;  // equal            (Z = 1; alias "JRZ")
  parameter logic [3:0] CC_NE = 4'b1011;  // not-equal        (Z = 0; alias "JRNZ")
  // General-arithmetic single-flag conditions (Table 12-8, page 12-31). These
  // test one ST flag directly; unambiguous single-bit codes from the table.
  parameter logic [3:0] CC_C  = 4'b1000;  // carry set        (C = 1; alias "B" borrow)
  parameter logic [3:0] CC_V  = 4'b1100;  // overflow         (V = 1; also window clip)
  parameter logic [3:0] CC_NV = 4'b1101;  // no overflow      (V = 0)
  parameter logic [3:0] CC_N  = 4'b1110;  // negative         (N = 1)
  parameter logic [3:0] CC_NN = 4'b1111;  // nonnegative      (N = 0)

  // Auto-update addressing mode for the indirect MOVE family. The pointer
  // register steps by the field size (32 bits for the implemented FS=32
  // case): POSTINC uses the pointer then adds; PREDEC subtracts first and
  // uses the new value. NONE leaves the pointer unchanged.
  typedef enum logic [1:0] {
    MV_ADDR_NONE    = 2'b00,
    MV_ADDR_POSTINC = 2'b01,
    MV_ADDR_PREDEC  = 2'b10
  } move_addr_mode_t;

  // What the control FSM needs from decode in order to execute. Fields are
  // populated only when the instruction class uses them; the rest hold safe
  // defaults (REG_FILE_A, idx 0, ALU_OP_PASS_A, etc.) so an undriven path
  // never mis-writes the register file.
  typedef struct packed {
    logic          illegal;      // 1 if the encoding is not recognized
    logic          illegal_trap; // 1 for a silicon-reserved §8.7 opcode
    instr_class_t  iclass;       // dispatch class for the control FSM
    reg_file_t     rd_file;     // destination register file (A or B); also
                                // governs Rs file for MOST reg-reg ops because
                                // they constrain Rs and Rd to the same file
                                // (single R bit in encoding).
    reg_idx_t      rd_idx;      // destination register index
    reg_idx_t      rs_idx;      // source register index (reg-reg ops)
    reg_file_t     rs_file;     // source register file. Equals rd_file for all
                                // same-file ops; the ONLY exception is
                                // MOVE Rs,Rd (the M-bit cross-file move), where
                                // Rs and Rd may be in different files. The core
                                // reads this for rf_rs1_file only on MOVE_RR.
    logic          needs_imm16; // fetch one extra 16-bit word for immediate
    logic          needs_imm32; // fetch two extra 16-bit words; LO first then HI
    logic          needs_imm64; // fetch four extra words; two LO/HI 32-bit operands
    logic          imm_sign_extend; // if 1, sign-extend imm16 to 32 bits
    alu_op_t       alu_op;      // ALU op to use in CORE_EXECUTE
    shift_op_t     shift_op;    // shifter op (used when result_source = SHIFTER)
    logic          use_shifter; // 1 ⇒ writeback from shifter, else from ALU
    logic [4:0]    k5;          // K-form 5-bit constant (MOVK, ADDK, SUBK, ...)
    logic [3:0]    branch_cc;   // condition code for JRcc / JAcc / DSJcc
    logic          wb_reg_en;   // 1 ⇒ regfile write in CORE_WRITEBACK
    logic          wb_flags_en; // 1 ⇒ ST flag update in CORE_WRITEBACK
    // Per-flag mask: which of {N,C,Z,V} actually take their new value
    // from `alu_flags` when wb_flags_en is set. All-ones = "full update"
    // (the default for arithmetic). BTST uses {n:0,c:0,z:1,v:0}; ABS
    // uses {n:1,c:0,z:1,v:1} to leave C "Unaffected" per spec.
    alu_flags_t    wb_flag_mask;
    // 1 ⇒ instruction includes a memory transaction (read or write)
    // in the CORE_MEMORY state between EXECUTE and WRITEBACK. PUSHST
    // is the first user; later: POPST, CALL, RETS, MMTM, MMFM, MOVE
    // indirect, MOVB, TRAP.
    logic          needs_memory_op;
    // Pointer auto-update mode for the indirect MOVE family (INSTR_MOVE_
    // FIELD_STORE / _LOAD). MV_ADDR_NONE for every other instruction.
    move_addr_mode_t move_mode;
    // 1 ⇒ MOVB (move byte): force the field size to 8 regardless of ST.FS,
    // and sign-extend on load (MOVB loads are always sign-extended). Reuses
    // the INSTR_MOVE_FIELD/OFF/ABS datapaths with FS forced to 8.
    logic          force_byte;
    // 1 ⇒ PIXT (pixel transfer, linear): force the field size to the PSIZE
    // I/O register value and ZERO-extend on load. Reuses the INSTR_MOVE_
    // FIELD_STORE/LOAD/M2M datapaths. PIXT load also reports V = (pixel != 0).
    logic          force_pixel;
    // 1 ⇒ the pointer register holds an XY address (not a linear one): the
    // core converts it to a linear address (via CONVDP for a destination /
    // CONVSP for a source, OFFSET=B4, PSIZE) before the field access. Used by
    // the XY-addressed PIXT forms (always with force_pixel).
    logic          xy_addr;
    // PIXBLT source/destination XY: the SADDR / DADDR implied register holds an
    // XY value (converted to linear at PBLT_SETUP via CONVSP / CONVDP).
    logic          blt_src_xy;
    logic          blt_dst_xy;
    // PIXBLT binary (B,L / B,XY): the source is a 1-bit-per-pixel bitmap; each
    // bit expands to COLOR1(B9) if 1, COLOR0(B8) if 0.
    logic          blt_binary;
  } decoded_instr_t;

endpackage : tms34010_pkg
`default_nettype wire

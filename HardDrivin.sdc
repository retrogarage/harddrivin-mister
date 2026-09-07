derive_pll_clocks
derive_clock_uncertainty

# The MiSTer framework constraint predates this core's PLL hierarchy and its
# wildcard does not match u_milestone/u_pll. The native core clocks cross into
# the stock OSD/video framework, LED plumbing, and HPS reset controls through
# deliberate asynchronous interfaces. Keep those framework clocks out of the
# core timing reports. The core clocks are split from one another below.
set_clock_groups -asynchronous \
  -group [get_clocks {*u_milestone*u_pll*u_pll*general[*]*divclk}] \
  -group [get_clocks {FPGA_CLK1_50}] \
  -group [get_clocks {FPGA_CLK2_50}] \
  -group [get_clocks {FPGA_CLK3_50}] \
  -group [get_clocks {*h2f_user0_clk}] \
  -group [get_clocks {*pll_audio*divclk}] \
  -group [get_clocks {*pll_hdmi*divclk}]

# The original Compact board uses independent 48 MHz GSP and 50 MHz MSP
# oscillators.  Preserve that domain boundary even though the FPGA derives
# both clocks from one PLL.  The 100 MHz memory and 6 MHz video domains are
# independent too; their crossings use explicit mailboxes/synchronizers.
set_clock_groups -asynchronous \
  -group [get_clocks {*u_milestone*u_pll*u_pll*general[0]*divclk}] \
  -group [get_clocks {*u_milestone*u_pll*u_pll*general[1]*divclk}] \
  -group [get_clocks {*u_milestone*u_pll*u_pll*general[2]*divclk}] \
  -group [get_clocks {*u_milestone*u_pll*u_pll*general[3]*divclk}]

# Carry the reusable TMS34010 core's documented timing contract into both
# GSP/MSP instances. Opcode decode and the listed state-driven register-file
# paths are architecturally separated by explicit quiet/acknowledge states;
# these are the same narrow exceptions used by the component's standalone
# Cyclone V sign-off, not blanket paths through the core.
set tms_opcode_sources [get_registers {*|u_core|instr_word_q*}]
set tms_decode_dests [get_registers {*|u_core|decoded*}]
set tms_regfile_dests [get_registers {*|u_regfile|*}]
set_multicycle_path -setup -end 2 \
  -from $tms_opcode_sources -to $tms_decode_dests
set_multicycle_path -hold -end 1 \
  -from $tms_opcode_sources -to $tms_decode_dests
set_multicycle_path -setup -end 2 \
  -from $tms_decode_dests -to $tms_regfile_dests
set_multicycle_path -hold -end 1 \
  -from $tms_decode_dests -to $tms_regfile_dests

# A decoded transaction cannot reach the memory-fabric ingress in the same
# cycle.  Non-immediate instructions cross CORE_DISPATCH and CORE_EXECUTE
# before CORE_MEMORY asserts mem_req; immediate fetches cross CORE_DISPATCH
# before their first request.  The ingress registers capture the complete
# held command, and the arbiter owner can select it only on a later edge.
# Constrain only those request-capture registers; memory responses and core
# consumers retain normal single-cycle analysis.  The raw two-word on-chip
# I/O view is captured with the same held request at mem_req, before the
# fabric can acknowledge it, and is aligned only from registered operands.
set tms_memory_request_dests [get_registers \
  {*|u_memory_fabric|cpu_request_* \
   *|u_memory_fabric|u_arbiter|state_q* \
   *|u_core|io_request_wdata_q* \
   *|u_core|io_rdata_words_q*}]
set_multicycle_path -setup -end 2 \
  -from $tms_decode_dests -to $tms_memory_request_dests
set_multicycle_path -hold -end 1 \
  -from $tms_decode_dests -to $tms_memory_request_dests

# FILL's private row base cannot sample decoded instruction fields until the
# edge after both CORE_DISPATCH and CORE_EXECUTE. `decoded` is written only
# when leaving CORE_DECODE_WAIT and remains held for the entire instruction,
# so this is an explicit two-cycle datapath in both reused instances.
set tms_fill_row_base_dests [get_registers {*|u_core|fill_row_base_q*}]
set_multicycle_path -setup -end 2 \
  -from $tms_decode_dests -to $tms_fill_row_base_dests
set_multicycle_path -hold -end 1 \
  -from $tms_decode_dests -to $tms_fill_row_base_dests

# A decoded branch/call/return cannot load the program counter until
# CORE_WRITEBACK, after complete CORE_DISPATCH and CORE_EXECUTE cycles.
# FETCH/FETCH_IMM increment the PC from fetch state and memory acknowledge,
# not from decoded instruction data.  Two cycles is therefore conservative.
set tms_pc_dests [get_registers {*|u_pc|pc_q*}]
set_multicycle_path -setup -end 2 \
  -from $tms_decode_dests -to $tms_pc_dests
set_multicycle_path -hold -end 1 \
  -from $tms_decode_dests -to $tms_pc_dests

set tms_graphics_read_only_states [get_registers \
  {*|u_core|state_q.CORE_FILL_SETUP* \
   *|u_core|state_q.CORE_PBLT_SETUP* \
   *|u_core|state_q.CORE_DRAV* \
   *|u_core|state_q.CORE_LINE_SETUP* \
   *|u_core|state_q.CORE_PIXT_SETUP_WIN*}]
set_false_path -from $tms_graphics_read_only_states -to $tms_regfile_dests

set tms_memory_states [get_registers {*|u_core|state_q.CORE_MEMORY*}]
set_multicycle_path -setup -end 2 \
  -from $tms_memory_states -to $tms_regfile_dests
set_multicycle_path -hold -end 1 \
  -from $tms_memory_states -to $tms_regfile_dests

set tms_mm_mask_sources [get_registers {*|u_core|mm_mask_q*}]
set_multicycle_path -setup -end 2 \
  -from $tms_mm_mask_sources -to $tms_regfile_dests
set_multicycle_path -hold -end 1 \
  -from $tms_mm_mask_sources -to $tms_regfile_dests

# MMTM/MMFM use the same mask to decide whether an acknowledged word returns
# to CORE_MEMORY or advances to CORE_WRITEBACK.  Neither state changes the PC;
# the earliest PC advance is a later CORE_FETCH acknowledgement.  Preserve
# that explicit quiet cycle when Quartus folds the state/data muxes together.
set_multicycle_path -setup -end 2 \
  -from $tms_mm_mask_sources -to $tms_pc_dests
set_multicycle_path -hold -end 1 \
  -from $tms_mm_mask_sources -to $tms_pc_dests

# b123 slow/hot STA: osd_de[2] -> osd_mux failed by 0.030 ns;
# 5.686 ns of its 6.034 ns data path was routing. Tighten this existing
# single-cycle path (HDMI period 6.732 ns) so placement shortens the wire.
# This changes neither pipeline latency nor clock/hold exceptions.
# Quartus automatically includes ~DUPLICATE registers for each exact RTL
# endpoint. Constrain every such copy and fail if either endpoint is absent.
set hd_osd_de2 [get_registers {hdmi_osd|osd_de[2]}]
set hd_osd_mux [get_registers {hdmi_osd|osd_mux}]
if {[get_collection_size $hd_osd_de2] < 1 || [get_collection_size $hd_osd_mux] < 1} {
  error "HardDrivin OSD route constraint must match both RTL endpoints, including fitter duplicates"
}
set_max_delay -from $hd_osd_de2 -to $hd_osd_mux 5.000

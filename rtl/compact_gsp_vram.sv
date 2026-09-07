`default_nettype none

// Compact MultiSync GSP video memory.
//
// The board uses sixteen 64Kx4 serial-access VRAMs: 256Kx16 (512 KiB)
// through the random port, with an independent serial scanout port.  Cyclone
// V true dual-port M10Ks are a close architectural match and remove raster
// reads from the shared external-memory budget.
module compact_gsp_vram #(
  parameter integer ADDR_WIDTH = 18
) (
  input  logic                  cpu_clk_i,
  input  logic                  cpu_rst_i,
  input  logic                  cpu_req_i,
  input  logic                  cpu_we_i,
  input  logic [ADDR_WIDTH-1:0] cpu_addr_i,
  input  logic [15:0]           cpu_wdata_i,
  input  logic [1:0]            cpu_be_i,
  output logic                  cpu_ack_o,
  output logic [15:0]           cpu_rdata_o,
  output logic [31:0]           cpu_words_o,

  input  logic                  scan_clk_i,
  input  logic                  scan_rst_i,
  input  logic                  scan_req_i,
  input  logic [ADDR_WIDTH-1:0] scan_addr_i,
  output logic                  scan_ack_o,
  output logic [15:0]           scan_data_o,
  output logic [31:0]           scan_words_o
);
  localparam integer WORDS = 1 << ADDR_WIDTH;

  logic [15:0] ram_cpu_q;
  logic [15:0] ram_scan_q;
  logic cpu_armed;

  assign cpu_rdata_o = ram_cpu_q;
  assign scan_data_o  = ram_scan_q;

`ifdef HD_QUARTUS
  // Explicitly use the same proven Intel primitive style as MiSTer's SD
  // buffer. The behavioral two-clock/byte-enable form is legal HDL, but
  // Quartus expands this unusually large instance during map before deciding
  // whether it can be represented by M10Ks. This fixes the implementation at
  // before technology mapping begins.
  altsyncram vram_m10k (
    .clock0(cpu_clk_i),
    .address_a(cpu_addr_i),
    .data_a(cpu_wdata_i),
    .wren_a(cpu_req_i && cpu_armed && cpu_we_i),
    .rden_a(cpu_req_i && cpu_armed),
    .byteena_a(cpu_be_i),
    .q_a(ram_cpu_q),

    .clock1(scan_clk_i),
    .address_b(scan_addr_i),
    .data_b(16'd0),
    .wren_b(1'b0),
    .rden_b(scan_req_i),
    .byteena_b(2'b11),
    .q_b(ram_scan_q),

    .aclr0(1'b0),
    .aclr1(1'b0),
    .addressstall_a(1'b0),
    .addressstall_b(1'b0),
    .clocken0(1'b1),
    .clocken1(1'b1),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .eccstatus()
  );
  defparam
    vram_m10k.address_reg_b = "CLOCK1",
    vram_m10k.clock_enable_input_a = "BYPASS",
    vram_m10k.clock_enable_input_b = "BYPASS",
    vram_m10k.clock_enable_output_a = "BYPASS",
    vram_m10k.clock_enable_output_b = "BYPASS",
    vram_m10k.indata_reg_b = "CLOCK1",
    vram_m10k.intended_device_family = "Cyclone V",
    vram_m10k.lpm_type = "altsyncram",
    vram_m10k.numwords_a = WORDS,
    vram_m10k.numwords_b = WORDS,
    vram_m10k.operation_mode = "BIDIR_DUAL_PORT",
    vram_m10k.outdata_aclr_a = "NONE",
    vram_m10k.outdata_aclr_b = "NONE",
    vram_m10k.outdata_reg_a = "UNREGISTERED",
    vram_m10k.outdata_reg_b = "UNREGISTERED",
    vram_m10k.power_up_uninitialized = "TRUE",
    vram_m10k.read_during_write_mode_mixed_ports = "DONT_CARE",
    vram_m10k.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
    vram_m10k.read_during_write_mode_port_b = "NEW_DATA_NO_NBE_READ",
    vram_m10k.widthad_a = ADDR_WIDTH,
    vram_m10k.widthad_b = ADDR_WIDTH,
    vram_m10k.width_a = 16,
    vram_m10k.width_b = 16,
    vram_m10k.width_byteena_a = 2,
    vram_m10k.width_byteena_b = 2,
    vram_m10k.wrcontrol_wraddress_reg_b = "CLOCK1";
`else
  logic [15:0] memory [0:WORDS-1];
  assign ram_cpu_q  = memory[cpu_addr_i];
  assign ram_scan_q = memory[scan_addr_i];
`endif

  always_ff @(posedge cpu_clk_i) begin
    if (cpu_rst_i) begin
      cpu_ack_o   <= 1'b0;
      cpu_words_o <= 32'd0;
      cpu_armed   <= 1'b1;
    end else begin
      cpu_ack_o <= 1'b0;
      if (!cpu_req_i) cpu_armed <= 1'b1;
      if (cpu_req_i && cpu_armed) begin
        cpu_armed   <= 1'b0;
        cpu_ack_o   <= 1'b1;
        cpu_words_o <= cpu_words_o + 32'd1;
`ifndef HD_QUARTUS
        if (cpu_we_i) begin
          if (cpu_be_i[0]) memory[cpu_addr_i][7:0]  <= cpu_wdata_i[7:0];
          if (cpu_be_i[1]) memory[cpu_addr_i][15:8] <= cpu_wdata_i[15:8];
        end
`endif
      end
    end
  end

  always_ff @(posedge scan_clk_i) begin
    if (scan_rst_i) begin
      scan_ack_o   <= 1'b0;
      scan_words_o <= 32'd0;
    end else begin
      scan_ack_o <= scan_req_i;
      if (scan_req_i) begin
        scan_words_o <= scan_words_o + 32'd1;
      end
    end
  end
endmodule

`default_nettype wire

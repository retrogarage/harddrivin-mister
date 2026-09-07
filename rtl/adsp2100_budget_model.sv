`default_nettype none

module adsp_budget_ram #(
  parameter integer WIDTH = 16,
  parameter integer ADDR_WIDTH = 11
) (
  input  logic                  clk_i,
  input  logic                  we_i,
  input  logic [ADDR_WIDTH-1:0] waddr_i,
  input  logic [WIDTH-1:0]      wdata_i,
  input  logic [ADDR_WIDTH-1:0] raddr_i,
  output logic [WIDTH-1:0]      rdata_o
);
  (* ramstyle = "M10K" *) logic [WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
  always_ff @(posedge clk_i) begin
    if (we_i) mem[waddr_i] <= wdata_i;
    rdata_o <= mem[raddr_i];
  end
endmodule

// This is deliberately not presented as an ADSP-2100 core. It exercises the
// principal resource classes (16x16 multiplier, 40-bit MAC/ALU, barrel shift,
// DAG state, and local program/data RAM) so the baseline build has a useful
// lower bound. RESERVE_BITS supports a separate conservative sensitivity fit.
module adsp2100_budget_model #(
  parameter integer RESERVE_BITS = 1
) (
  input  logic        clk_i,
  input  logic        rst_i,
  output logic [31:0] signature_o
);
  logic [13:0] pc;
  logic [23:0] instruction;
  logic [23:0] program_wdata;
  logic [15:0] data_rdata;
  logic [15:0] data_wdata;
  logic [10:0] data_raddr, data_waddr;
  logic [1:0] next_dag;
  logic signed [15:0] mx, my;
  logic signed [31:0] product;
  logic signed [39:0] accumulator;
  logic [15:0] ireg [0:3];
  logic [15:0] mreg [0:3];
  logic [2:0] dag_sel;
  logic [4:0] shift_amount;
  logic [39:0] shifted;

  assign program_wdata = {pc[7:0], accumulator[15:0]};
  assign data_wdata = accumulator[15:0];
  assign data_waddr = ireg[dag_sel[1:0]][10:0];
  assign next_dag = dag_sel[1:0] + 2'd1;
  assign data_raddr = ireg[next_dag][10:0];

  adsp_budget_ram #(.WIDTH(24), .ADDR_WIDTH(11)) u_program_ram (
    .clk_i(clk_i), .we_i(!rst_i), .waddr_i(pc[10:0]),
    .wdata_i(program_wdata), .raddr_i(pc[10:0] ^ 11'h155),
    .rdata_o(instruction)
  );
  adsp_budget_ram #(.WIDTH(16), .ADDR_WIDTH(11)) u_data_ram (
    .clk_i(clk_i), .we_i(!rst_i), .waddr_i(data_waddr),
    .wdata_i(data_wdata), .raddr_i(data_raddr), .rdata_o(data_rdata)
  );

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      pc          <= 14'd0;
      mx          <= 16'sh1234;
      my          <= 16'sh0101;
      accumulator <= 40'sd0;
      dag_sel     <= 3'd0;
      shift_amount <= 5'd0;
    end else begin
      mx <= data_rdata ^ instruction[15:0];
      my <= my + 16'sd17;
      accumulator <= accumulator + product + $signed({24'd0, shifted[15:0]});
      pc <= pc + 14'd1;
      ireg[dag_sel[1:0]] <= ireg[dag_sel[1:0]] + mreg[dag_sel[1:0]] + 16'd1;
      mreg[dag_sel[1:0]] <= mreg[dag_sel[1:0]] ^ instruction[15:0];
      dag_sel <= dag_sel + 3'd1;
      shift_amount <= instruction[20:16];
    end
  end

  assign product = mx * my;
  assign shifted = $unsigned(accumulator) >> shift_amount;

  generate
    if (RESERVE_BITS > 1) begin : g_reserve
      (* preserve, altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *)
      logic [RESERVE_BITS-1:0] reserve_state;
      always_ff @(posedge clk_i) begin
        if (rst_i) reserve_state <= {{(RESERVE_BITS-1){1'b0}}, 1'b1};
        else reserve_state <= {
          reserve_state[RESERVE_BITS-2:0],
          reserve_state[RESERVE_BITS-1] ^ reserve_state[RESERVE_BITS/2] ^
          reserve_state[RESERVE_BITS/3] ^ reserve_state[0]
        };
      end
      assign signature_o = accumulator[31:0] ^ ireg[0] ^
                           {31'd0, reserve_state[RESERVE_BITS-1]};
    end else begin : g_no_reserve
      assign signature_o = accumulator[31:0] ^ ireg[0];
    end
  endgenerate
endmodule

`default_nettype wire

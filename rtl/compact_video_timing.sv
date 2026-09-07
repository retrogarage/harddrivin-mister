`default_nettype none

// Compact GSP input clock / 4 = 12 MHz pixel rate.
// 12 MHz / (646 * 308) = 60.3112 Hz with a 512x288 active raster.
module compact_video_timing (
  input  logic        clk_i,
  input  logic        rst_i,
  input  logic [31:0] debug_i,
  input  logic [7:0]  diagnostic_i,
  output logic        ce_pix_o,
  output logic        hblank_o,
  output logic        vblank_o,
  output logic        hsync_o,
  output logic        vsync_o,
  output logic [23:0] rgb_o,
  output logic        frame_start_o
);
  logic [1:0]  divider;
  logic [9:0]  h;
  logic [8:0]  v;

  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      divider      <= 2'd0;
      h            <= 10'd0;
      v            <= 9'd0;
      ce_pix_o     <= 1'b0;
      frame_start_o <= 1'b0;
    end else begin
      divider      <= divider + 2'd1;
      ce_pix_o     <= 1'b0;
      frame_start_o <= 1'b0;
      if (divider == 2'd3) begin
        ce_pix_o <= 1'b1;
        if (h == 10'd645) begin
          h <= 10'd0;
          if (v == 9'd307) begin
            v <= 9'd0;
            frame_start_o <= 1'b1;
          end else begin
            v <= v + 9'd1;
          end
        end else begin
          h <= h + 10'd1;
        end
      end
    end
  end

  always_comb begin
    hblank_o = (h >= 10'd512);
    vblank_o = (v >= 9'd288);
    hsync_o  = (h >= 10'd536) && (h < 10'd600);
    vsync_o  = (v >= 9'd292) && (v < 9'd296);
    if (hblank_o || vblank_o) begin
      rgb_o = 24'h000000;
    end else if (v < 9'd24) begin
      // Eight fixed-width state lamps make hardware progress readable in a
      // capture frame. Green means the milestone was observed; red means it
      // has not happened yet. The remaining field retains the live signature.
      rgb_o = diagnostic_i[h[8:6]] ? 24'h00d040 : 24'hb01818;
    end else begin
      rgb_o = {
        h[7:0] ^ debug_i[7:0],
        v[7:0] ^ debug_i[15:8],
        h[8:1] ^ v[7:0] ^ debug_i[23:16]
      };
    end
  end
endmodule

`default_nettype wire

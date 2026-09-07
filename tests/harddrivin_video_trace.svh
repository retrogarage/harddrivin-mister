// Include inside harddrivin_milestone for simulation tracing. This observes
// completed transactions and serializer events; it never drives the core.
// Enable with +VIDEO_TRACE=path. HINT is the host interrupt, distinct from DI.
`ifndef SYNTHESIS
  integer video_trace_fd = 0;
  string video_trace_path;
  logic video_trace_hint_q;
  logic [1:0] video_trace_pal_q;
  logic [3:0] video_trace_fine_q;
  logic video_trace_bank_q;
  initial begin
    if ($value$plusargs("VIDEO_TRACE=%s", video_trace_path)) begin
      video_trace_fd = $fopen(video_trace_path, "w");
      if (!video_trace_fd) $fatal(1, "Cannot open VIDEO_TRACE output");
    end
  end
  always @(posedge clk_gsp) begin
    video_trace_hint_q <= gsp_hint_n;
    video_trace_pal_q <= gsp_palette_bank;
    video_trace_fine_q <= gsp_fine_scroll;
    video_trace_bank_q <= u_scanout.active_bank_q;
    if (video_trace_fd && !tms_gsp_rst) begin
      if (gsp_cycle_req && gsp_cycle_ack &&
          gsp_cycle_kind == tms34010_pkg::LOCAL_CYCLE_IO_WRITE &&
          (gsp_cycle_addr[8:4] == 5'h1b || gsp_cycle_addr[8:4] == 5'h1e ||
           gsp_cycle_addr[8:4] == 5'h09 || gsp_cycle_addr[8:4] == 5'h0a))
        $fdisplay(video_trace_fd,"REG t=%0t raster=%0d:%0d pc=%08x addr=%08x value=%04x",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,gsp_pc,
          gsp_cycle_addr,gsp_cycle_wdata);
      if (gsp_palette_bank != video_trace_pal_q || gsp_fine_scroll != video_trace_fine_q)
        $fdisplay(video_trace_fd,"SPLIT t=%0t raster=%0d:%0d pc=%08x tap=%04x pal=%0d fine=%0d",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,gsp_pc,
          gsp_dpytap_q,gsp_palette_bank,gsp_fine_scroll);
      if (gsp_hint_n != video_trace_hint_q)
        $fdisplay(video_trace_fd,"HINT t=%0t raster=%0d:%0d pc=%08x hint_n=%b",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,gsp_pc,gsp_hint_n);
      if (gsp_cycle_req && gsp_cycle_ack && !gsp_cycle_iaq &&
          gsp_cycle_addr == 32'hfffffea0)
        $fdisplay(video_trace_fd,"DI t=%0t raster=%0d:%0d pc=%08x",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,gsp_pc);
      if (gsp_screen_cycle && !u_scanout.screen_seen_q)
        $fdisplay(video_trace_fd,"REQUEST t=%0t raster=%0d:%0d srf=%04x tap=%04x live_tap=%04x pal=%0d fine=%0d lc=%0d",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,
          gsp_cycle_srfaddr,gsp_cycle_dpytap,gsp_dpytap_q,
          gsp_palette_bank,gsp_fine_scroll,gsp_dpystart_q[1:0]);
      if (u_scanout.publish_sync_q[1] != u_scanout.publish_seen_q)
        $fdisplay(video_trace_fd,"PUBLISH t=%0t raster=%0d:%0d srf=%04x col=%02x pal=%0d fine=%0d bank=%b active=%b pending=%b",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,
          u_scanout.published_srfaddr_q,u_scanout.published_col_q,
          u_scanout.published_palette_bank_q,u_scanout.published_fine_scroll_q,
          u_scanout.published_bank_q,u_scanout.active_bank_q,u_scanout.pending_row_q);
      if (u_scanout.active_bank_q != video_trace_bank_q)
        $fdisplay(video_trace_fd,"DISPLAY t=%0t raster=%0d:%0d col=%02x pal=%0d fine=%0d bank=%b",
          $time,u_scanout.raster_v_q,u_scanout.raster_h_q,
          u_scanout.active_col_q,u_scanout.display_palette_bank,
          u_scanout.display_fine_scroll,u_scanout.active_bank_q);
    end
  end
  always @(posedge clk_100) begin
    if (video_trace_fd && !traffic_reset && u_scanout.row_write_en &&
        u_scanout.row_valid_q && u_scanout.load_bank_q == u_scanout.active_bank_q &&
        u_scanout.raster_v_q < (COCKPIT ? 384 : 288) &&
        u_scanout.raster_h_q < (COCKPIT ? 508 : 512))
      $fdisplay(video_trace_fd,"ACTIVE_WRITE t=%0t raster=%0d:%0d bank=%b word=%03x",
        $time,u_scanout.raster_v_q,u_scanout.raster_h_q,
        u_scanout.load_bank_q,u_scanout.load_index_q);
  end
  final if (video_trace_fd) $fclose(video_trace_fd);
`endif

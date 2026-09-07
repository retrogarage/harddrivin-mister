// Passive per-frame row selection and first dashboard descriptor timing.
logic [255:0] scan_debug_q;
logic [7:0] sd_frame_q, sd_flags_q, sd_missed_q;
logic [8:0] sd_req_y_q,sd_accept_y_q,sd_publish_y_q,sd_promote_y_q,sd_lines_q;
logic [9:0] sd_req_x_q,sd_accept_x_q,sd_publish_x_q,sd_promote_x_q;
logic [13:0] sd_active_srf_q,sd_first_srf_q;
logic [29:0] sd_row300_q,sd_row360_q;
logic [15:0] sd_row0_q;
logic sd_consumed_prev_q;
always_ff @(posedge core_clk_i) begin
  if(core_rst_i) begin
    scan_debug_q<=0;sd_frame_q<=0;sd_flags_q<=0;sd_missed_q<=0;sd_lines_q<=0;
    sd_req_y_q<=0;sd_accept_y_q<=0;sd_publish_y_q<=0;sd_promote_y_q<=0;
    sd_req_x_q<=0;sd_accept_x_q<=0;sd_publish_x_q<=0;sd_promote_x_q<=0;
    sd_active_srf_q<=0;sd_first_srf_q<=0;sd_row300_q<=0;sd_row360_q<=0;sd_row0_q<=0;
    sd_consumed_prev_q<=0;
  end else begin
    sd_consumed_prev_q<=consumed_toggle_q;
    if(screen_req_i && !screen_seen_q && screen_srfaddr_i==14'h3e7f) begin
      sd_flags_q[0]<=1;sd_req_y_q<=raster_v_q;sd_req_x_q<=raster_h_q;
      sd_first_srf_q<=screen_srfaddr_i;
    end
    // Accepted mailbox payload stays stable until this request retires.
    if(accepted_sync_q[1]!=accepted_seen_q && command_srfaddr_q==14'h3e7f) begin
      sd_flags_q[1]<=1;sd_accept_y_q<=raster_v_q;sd_accept_x_q<=raster_h_q;
    end
    if(publish_sync_q[1]!=publish_seen_q && published_srfaddr_q==14'h3e7f) begin
      sd_flags_q[2]<=1;sd_publish_y_q<=raster_v_q;sd_publish_x_q<=raster_h_q;
    end
    if(consumed_toggle_q!=sd_consumed_prev_q) begin
      sd_active_srf_q<=published_srfaddr_q;
      if(published_srfaddr_q==14'h3e7f) begin
        sd_flags_q[3]<=1;sd_promote_y_q<=raster_v_q;sd_promote_x_q<=raster_h_q;
      end
    end
    if(programmed_hactive_start && !pending_row_q && row_valid_q && active_lcstrt_q==0 &&
       raster_v_q>=250 && raster_v_q<384 && !(&sd_missed_q)) sd_missed_q<=sd_missed_q+8'd1;
    if(ce_pix_o && captured_x_q==200 && !captured_blank_q) begin
      sd_lines_q<=sd_lines_q+9'd1;
      if(captured_line_q==0) sd_row0_q<={serial_line_q,sd_active_srf_q};
      if(captured_line_q==300) begin
        sd_flags_q[4]<=1;
        sd_row300_q<={display_fine_scroll,display_palette_bank,active_col_q,serial_line_q,sd_active_srf_q};
      end
      if(captured_line_q==360) begin
        sd_flags_q[5]<=1;
        sd_row360_q<={display_fine_scroll,display_palette_bank,active_col_q,serial_line_q,sd_active_srf_q};
      end
    end
    if(frame_start_o) begin
      sd_frame_q<=sd_frame_q+8'd1;sd_flags_q<=0;sd_missed_q<=0;sd_lines_q<=0;
      scan_debug_q[127:0]<={6'd0,sd_first_srf_q,
        sd_promote_x_q,sd_promote_y_q,sd_publish_x_q,sd_publish_y_q,
        sd_accept_x_q,sd_accept_y_q,sd_req_x_q,sd_req_y_q,sd_flags_q,sd_frame_q,16'hd4a7};
      scan_debug_q[255:128]<={3'd0,sd_missed_q,sd_lines_q,sd_row0_q,
        sd_row360_q,sd_row300_q,sd_flags_q,sd_frame_q,16'hd5a7};
    end
  end
end
logic video_debug_bit_q;
wire [511:0] video_debug_pages={scan_debug_q,video_debug_i};
always_ff @(posedge core_clk_i) begin
  if(core_rst_i) video_debug_bit_q<=0;
  else if(pixel_phase_q==0)
    video_debug_bit_q<=video_debug_pages[{raster_v_q[3:2],visible_x_q[8:2]}];
end

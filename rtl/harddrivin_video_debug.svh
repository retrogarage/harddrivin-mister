// Passive hardware telemetry. Snapshot the preceding frame for the top strip.
logic [255:0] video_debug_q;
logic [7:0] vd_frame_q, vd_flags_q;
logic [8:0] vd_dpy_y_q, vd_heb_y_q, vd_di_y_q, vd_split_y_q;
logic [9:0] vd_dpy_x_q, vd_heb_x_q, vd_di_x_q, vd_split_x_q;
logic [15:0] vd_dpy_data_q, vd_heb_data_q;
logic [23:0] vd_dpy_pc_q;
logic [15:0] vd_dash_start_q,vd_dash_tap_q,vd_hint_age_q,vd_hint_max_q;
logic [1:0] vd_pal_prev_q, vd_split_pal_q;
logic [3:0] vd_fine_prev_q, vd_split_fine_q;
always_ff @(posedge clk_gsp) begin
  if(tms_gsp_rst) begin
    video_debug_q<=0;vd_frame_q<=0;vd_flags_q<=0;
    vd_dpy_y_q<=0;vd_heb_y_q<=0;vd_di_y_q<=0;vd_split_y_q<=0;
    vd_dpy_x_q<=0;vd_heb_x_q<=0;vd_di_x_q<=0;vd_split_x_q<=0;
    vd_dash_start_q<=0;vd_dash_tap_q<=0;vd_hint_age_q<=0;vd_hint_max_q<=0;
    vd_dpy_data_q<=0;vd_heb_data_q<=0;vd_dpy_pc_q<=0;
    vd_pal_prev_q<=0;vd_fine_prev_q<=0;vd_split_pal_q<=0;vd_split_fine_q<=0;
  end else begin
    if(!gsp_hint_n) begin
      vd_flags_q[5]<=1;
      if(!(&vd_hint_age_q)) vd_hint_age_q<=vd_hint_age_q+16'd1;
    end else vd_hint_age_q<=0;
    if(vd_hint_age_q>vd_hint_max_q) vd_hint_max_q<=vd_hint_age_q;
    if(scan_line_dbg==300 && scan_x_dbg==200) begin
      vd_flags_q[4]<=1;vd_dash_start_q<=gsp_dpystart_q;vd_dash_tap_q<=gsp_dpytap_q;
    end
    vd_pal_prev_q<=gsp_palette_bank;vd_fine_prev_q<=gsp_fine_scroll;
    if(gsp_cycle_req && gsp_cycle_ack && scan_line_dbg>=128 && scan_line_dbg<384) begin
      if(gsp_cycle_kind==tms34010_pkg::LOCAL_CYCLE_IO_WRITE) begin
        if(gsp_cycle_addr[8:4]==5'h1e) begin
          vd_flags_q[6]<=gsp_video_hblank;vd_flags_q[7]<=gsp_video_vblank;
          vd_flags_q[0]<=1;vd_dpy_y_q<=scan_line_dbg;vd_dpy_x_q<=scan_x_dbg;
          vd_dpy_data_q<=gsp_cycle_wdata;vd_dpy_pc_q<=gsp_pc[23:0];
        end
        if(gsp_cycle_addr[8:4]==5'h01) begin
          vd_flags_q[1]<=1;vd_heb_y_q<=scan_line_dbg;vd_heb_x_q<=scan_x_dbg;
          vd_heb_data_q<=gsp_cycle_wdata;
        end
      end
      // Keep the split IRQ entry, not the ROM's following short IRQ at 286.
      if(!gsp_cycle_iaq && gsp_cycle_addr==32'hfffffea0 && !vd_flags_q[2]) begin
        vd_flags_q[2]<=1;vd_di_y_q<=scan_line_dbg;vd_di_x_q<=scan_x_dbg;
      end
    end
    if(scan_line_dbg>=128 && scan_line_dbg<384 &&
       (vd_pal_prev_q!=gsp_palette_bank || vd_fine_prev_q!=gsp_fine_scroll)) begin
      vd_flags_q[3]<=1;vd_split_y_q<=scan_line_dbg;vd_split_x_q<=scan_x_dbg;
      vd_split_pal_q<=gsp_palette_bank;vd_split_fine_q<=gsp_fine_scroll;
    end
    if(frame_start) begin
      vd_frame_q<=vd_frame_q+8'd1;vd_flags_q<=0;vd_hint_max_q<=0;
      // Every page's bit 127 is padding: cockpit emits 127 four-pixel cells.
      video_debug_q[127:0]<={2'd0,vd_heb_data_q,vd_heb_x_q,vd_heb_y_q,
        vd_dpy_pc_q,vd_dpy_data_q,vd_dpy_x_q,vd_dpy_y_q,vd_flags_q,vd_frame_q,16'hd2a7};
      video_debug_q[255:128]<={4'd0,vd_hint_max_q,vd_split_x_q,vd_split_y_q,
        vd_split_fine_q,vd_split_pal_q,vd_dash_start_q,vd_dash_tap_q,
        vd_di_x_q,vd_di_y_q,vd_flags_q,vd_frame_q,16'hd3a7};
    end
  end
end

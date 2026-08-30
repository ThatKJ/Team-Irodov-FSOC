# Telemetry Schema

Suggested CSV columns:

```text
sim_time_s
frame_index
wall_fps
target_x_m,target_y_m,target_z_m
target_visible
detection_found
centroid_u_px,centroid_v_px
error_u_px,error_v_px
error_az_rad,error_el_rad
camera_pan_rad,camera_tilt_rad
pan_rate_cmd_rad_s,tilt_rate_cmd_rad_s
pan_rate_applied_rad_s,tilt_rate_applied_rad_s
pan_saturated,tilt_saturated
```

Missing detections should be represented explicitly, not with a fake valid centroid.

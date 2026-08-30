# Coordinate Systems and Camera Math

## World frame
`+X forward`, `+Y right`, `+Z up`.

For pan `p` and tilt `t`:

```text
forward = [cos(t)cos(p), cos(t)sin(p), sin(t)]
right   = [-sin(p),       cos(p),       0]
up      = forward x right
```

For target delta `d = target - camera`:

```text
x_cam = d dot right
y_cam = d dot up
z_cam = d dot forward
```

Reject `z_cam <= 0`.

## Pinhole projection

```text
fx = (width/2)  / tan(hfov/2)
fy = (height/2) / tan(vfov/2)

u = cx + fx * x_cam/z_cam
v = cy - fy * y_cam/z_cam
```

The minus sign on `v` converts camera-up to image-down coordinates.

A point is visible only when horizontal/vertical bearing lies inside the configured half-FOV.

## Ideal pointing (diagnostics only)

```text
pan  = atan2(dy, dx)
tilt = atan2(dz, hypot(dx,dy))
```

Never use ideal pointing as operational controller feedback; it bypasses the camera/detector loop.

## Image coordinates and tracking error (Step 3 — frozen)

Image plane: origin top-left, `u` (= `x_px`) grows right, `v` (= `y_px`) grows down.
This follows directly from the projection equations above: `u = cx + fx·x_cam/z_cam`
and `v = cy - fy·y_cam/z_cam` (the minus maps camera-up to image-down).

Principal point / image centre, floating point, owned by the camera model:

```text
cx = width_px  / 2.0
cy = height_px / 2.0
```

Pixel tracking error (detected centroid minus image centre):

```text
error_x_px = detected_x_px - cx      error_x_px > 0  => beacon RIGHT of centre
error_y_px = detected_y_px - cy      error_y_px > 0  => beacon BELOW centre
                                     error_y_px < 0  => beacon ABOVE centre
```

Angular tracking error — reuse `PanTiltCamera::pixel_error_to_angles`, do not re-derive:

```text
pan_rad  =  atan(error_x_px / fx)     pan_rad  > 0  => beacon right => command pan right
tilt_rad = -atan(error_y_px / fy)     tilt_rad > 0  => beacon above => command tilt up
```

The `-` on `tilt_rad` undoes image-down vs. world/camera-up, so a positive `tilt_rad`
fed to `PanTiltCamera::step` as a tilt rate drives the optical axis upward toward the
beacon. A sign error here would create positive feedback once PID is added, so all four
quadrants are asserted in `fsoc_step3_tests` and machine-checked in `step3_observation_smoke`.

Worked check: centre `(320,240)`, beacon `(400,180)` → `error = (+80, -60)` px → RIGHT +
ABOVE → `pan_rad = atan(80/fx) > 0` (pan right), `tilt_rad = -atan(-60/fy) > 0` (tilt up).

Target-lost is represented only by an empty `std::optional` (`CameraObservation` with a
non-`Visible` status carries no image point). No `(-1,-1)`, NaN, or zero sentinels.

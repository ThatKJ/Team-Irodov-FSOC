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

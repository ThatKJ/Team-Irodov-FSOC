# Frontend Data Contract (Step 11)

**Status: documentation only.** Step 11 adds NO networking, NO WebSocket, NO HTTP,
NO JSON library. This file freezes the *shape* of the payload a future transport
layer will serialize, so the Next.js / Three.js mission-control frontend can be
built against a stable target.

## Boundary

```
C++ engine (fixed 50 Hz, radians)         <- authoritative
  -> DemoSnapshot            (fsoc/demo.hpp — a pure copy / view model)
  -> [future transport / WebSocket adapter — NOT in this repo yet]
  -> Next.js / Three.js frontend (degrees, camelCase JSON)   <- visualizes only
```

**C++ decides reality. The frontend visualizes reality.** The frontend must
**never recompute**: PID output, tracking error, visibility, the beacon centroid,
or camera dynamics. Those come from the engine, per frame, already computed.

## `DemoSnapshot` — the in-process view model

Built only from `SimulationStepResult` + `TelemetryRecord` + the `CameraConfig`
the runner used, by `fsoc::make_demo_snapshot(...)`. It never participates in the
control loop (proven by `test_non_interference`).

| DemoSnapshot field | type | unit | source | present |
|---|---|---|---|---|
| `simulation_time_s` | `double` | s | `result.simulation_time_s` | always |
| `frame_index` | `std::size_t` | — | `result.frame_index` | always |
| `state` | `DemoTrackingState` | — | `telemetry.tracking_state` (1:1) | always |
| `target.{x,y,z}_m` | `double` | m | `result.target_truth.position_m` | always |
| `target.{vx,vy,vz}_mps` | `double` | m/s | `result.target_truth.velocity_mps` | always |
| `camera.pan_rad` / `camera.tilt_rad` | `double` | rad | `result.camera_{pan,tilt}_rad` | always |
| `camera.pan_rate_rad_s` / `camera.tilt_rate_rad_s` | `double` | rad/s | `result.applied_rates` (post actuator clamp) | always |
| `camera.horizontal_fov_rad` / `camera.vertical_fov_rad` | `double` | rad | `CameraConfig.{h,v}fov_rad` | always |
| `detection.detected` | `bool` | — | `result.target_detected` | always |
| `detection.x_px` / `detection.y_px` | `std::optional<double>` | px | `telemetry.detected_{x,y}_px` | only while TRACKING |
| `tracking.error_x_px` / `tracking.error_y_px` | `std::optional<double>` | px | `telemetry.pixel_error_{x,y}_px` | only while TRACKING |
| `tracking.pan_error_rad` / `tracking.tilt_error_rad` | `std::optional<double>` | rad | `telemetry.angular_error_{pan,tilt}_rad` | only while TRACKING |
| `tracking.total_error_rad` | `std::optional<double>` | rad | `telemetry.angular_error_total_rad` = hypot(pan,tilt) | only while TRACKING |
| `control.command_pan_rate_rad_s` / `control.command_tilt_rate_rad_s` | `double` | rad/s | `result.command` (PID output, pre actuator clamp) | always ({0,0} on loss / open-loop) |
| `control.pan_saturated` / `control.tilt_saturated` | `bool` | — | `telemetry.{pan,tilt}_saturated` (axis at the actuator rate limit) | always |

**Absent = `std::nullopt`.** There is no `-1` / `NaN` / `"N/A"` sentinel. A lost
frame carries no centroid and no error — the four `tracking.*` optionals and the
two `detection.*` optionals are all empty, `state == TARGET_LOST`, and both
`control.command_*` are exactly `0.0`.

## Units — radians inside, degrees at the UI boundary

| layer | angles | rates | distance | pixels |
|---|---|---|---|---|
| C++ core / `DemoSnapshot` | **radians** | **rad/s** | metres | pixels |
| UI transport / JSON (future) | **degrees** | **deg/s** | metres | pixels |

The **only** place degrees appear in C++ is the helper `fsoc::to_degrees(const
DemoSnapshot&) -> DemoSnapshotAnglesDeg`, which converts just the angular
quantities (pan, tilt, their rates, the FOVs, the optional angular errors, and
the command rates). Pixels and metres are unit-invariant and are passed straight
through. The core physics units are never modified.

## Future JSON payload (one frame) — SHAPE ONLY, not implemented

```json
{
  "simulationTime": 4.25,
  "frame": 212,
  "trackingState": "TRACKING",

  "target": {
    "position": { "x": 100.0, "y": 8.2, "z": 4.1 },
    "velocity": { "x": 0.0, "y": 1.2, "z": 0.3 }
  },

  "camera": {
    "panDeg": 4.57,
    "tiltDeg": 2.28,
    "panRateDegS": 1.2,
    "tiltRateDegS": 0.4,
    "horizontalFovDeg": 20.0,
    "verticalFovDeg": 15.0
  },

  "detection": {
    "detected": true,
    "xPx": 321.2,
    "yPx": 239.7
  },

  "tracking": {
    "errorXPx": 1.2,
    "errorYPx": -0.3,
    "panErrorDeg": 0.012,
    "tiltErrorDeg": 0.004,
    "totalErrorDeg": 0.013
  },

  "control": {
    "panCommandDegS": 1.2,
    "tiltCommandDegS": 0.4,
    "panSaturated": false,
    "tiltSaturated": false
  }
}
```

Rules for whoever writes the transport adapter later:

- `trackingState` is `"TRACKING"` or `"TARGET_LOST"` — the exact strings from
  `fsoc::to_string(DemoTrackingState)`. No other values. `"PAUSED"` is **not** a
  tracking state (see below).
- Angular values are degrees (`*Deg` / `*DegS` suffix); positions/velocities stay
  in metres; pixel values stay in pixels.
- When `trackingState == "TARGET_LOST"`: omit `detection.xPx/yPx` and the whole
  `tracking` error block (or send them as `null`) — never fabricate a value.
- `camera.*Rate*` is the rate the actuator **applied** (after clamping);
  `control.*Command*` is the PID's **requested** rate (before clamping). They
  differ while an axis is saturated.
- The frontend renders at its own refresh rate (typically 60 FPS). The engine is
  a fixed 50 Hz. A transport/frontend MAY interpolate between snapshots for
  smooth motion, but the authoritative state is always the discrete 50 Hz
  snapshot. Interpolation is **not** implemented here.

## Tracking state vs demo run state

| enum | values | describes | owned by |
|---|---|---|---|
| `TrackingState` / `DemoTrackingState` | `TRACKING`, `TARGET_LOST` | control / observation reality | the engine (per frame) |
| `DemoRunState` | `Ready`, `Running`, `Paused`, `Finished` | demo/application lifecycle | `DemoSession` |

They are never mixed. `Paused` is an application concept — a paused
`DemoSession::step()` returns the last snapshot and does **not** advance
simulation time (no hidden simulation). It is unrelated to `TARGET_LOST`, which
means the detector found no beacon that frame.

## What the frontend must not do

Do not recompute in JS/TS: PID output, tracking error (pixel or angular),
visibility / FOV tests, the beacon centroid, or any camera dynamics. Consume the
snapshot fields as given. The C++ engine and its Step-10 acceptance suite are the
single source of truth.

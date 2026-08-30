# Interface Contracts

Planned contracts should preserve this information flow:

```cpp
TargetState trajectory.state_at(double sim_time_s);   // { position_m, velocity_mps }
std::optional<Projection> camera.project(Vec3 target_world_m);
CameraObservation observe_beacon(const PanTiltCamera&, Vec3 beacon_world_position_m);
Image renderer.render(const CameraObservation& observation, ...);
std::optional<BeaconDetection> detector.detect(const Image& frame);
std::optional<TrackingError> compute_tracking_error(
    const std::optional<BeaconDetection>&, const PanTiltCamera&);
ControlCommand pid.update(const TrackingError&, double dt_s);
AppliedRates camera.step(command.pan_rate_rad_s, command.tilt_rate_rad_s, dt_s);
telemetry.record(...);
```

The exact types may evolve, but dependencies must point in this direction. In particular:
- trajectory never owns/advances the simulation clock (the runner passes absolute `sim_time_s`) and knows nothing of camera/perception/control,
- detector never receives world truth,
- controller never receives target XYZ,
- camera never owns PID gains,
- telemetry never controls behavior.

`Trajectory::state_at` contract: `sim_time_s` must be finite and `>= 0`; violations throw
`std::invalid_argument`. It never returns NaN/Inf for valid input and is deterministic
(repeated calls with equal argument return bit-identical state).

## Observation / measurement / tracking-error contracts (Step 3, frozen)

Layers stay distinct — do not alias them:

| type | concept | knows about |
|---|---|---|
| `TargetState` | physical truth | world only |
| `Projection` / `CameraObservation` | exact camera projection | camera + a world point |
| `BeaconDetection` | estimated image measurement | image pixels only |
| `TrackingError` | controller-facing error | detection + camera intrinsics |

- **Image coordinate convention:** origin top-left, `+x_px` right, `+y_px` down. Centre is
  `cx = width_px / 2.0`, `cy = height_px / 2.0`, owned by `PanTiltCamera::cx_px()/cy_px()`.
  No other module may redefine the centre.
- **Visibility:** `ObservationStatus { Visible, OutsideFieldOfView, BehindCamera }`. The
  image point is present iff `Visible`. Depth test (`z_cam <= 0`) precedes the FOV test.
- **Lost target:** represented only by an empty `std::optional`. No `(-1,-1)`, no NaN, no
  zero sentinel. `compute_tracking_error(std::nullopt, …) == std::nullopt`.
- **Pixel error sign:** `x_px = detected_x - cx` (`>0` = beacon RIGHT), `y_px = detected_y - cy`
  (`>0` = beacon BELOW, because image `+y` is down).
- **Angular error sign:** `pan_rad = atan(x_px / fx)` (`>0` = beacon right → command pan
  right); `tilt_rad = -atan(y_px / fy)` (`>0` = beacon above → command tilt up). Produced by
  the single existing routine `PanTiltCamera::pixel_error_to_angles`; never re-derived.
- **Validation:** non-finite detection centroid → `std::invalid_argument`. Invalid image
  dimensions rejected by `CameraConfig::validate()`. `ScenarioConfig::validate()` requires
  finite `duration_s > 0`, finite `timestep_s > 0`, `timestep_s <= duration_s`.
- `TrackingError` is a pure image-plane quantity: it does not depend on camera pan/tilt pose.

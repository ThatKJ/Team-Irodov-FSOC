# Architecture Decision Log

## ADR-001 — Core language: C++20
**Status:** Accepted

The project has migrated from the original Python starter to modern C++20. Reasons: deterministic timing, direct mapping to embedded/robotics/GNC implementation, stronger type boundaries, and better long-term fit for real-time control work.

Consequence: Python packaging files and workflows are forbidden unless this decision is explicitly superseded.

## ADR-002 — Step-1 math has zero third-party dependencies
**Status:** Accepted

The foundation uses small auditable vector primitives. OpenCV is deferred until pixels; Eigen is deferred until matrix-heavy UKF/MPC work.

## ADR-003 — Fixed world/camera coordinate convention
**Status:** Accepted

World +X forward, +Y right, +Z up. Camera +x right, +y up, +z forward. Image v increases downward.

## ADR-004 — Ground truth never feeds operational controller
**Status:** Accepted

Ideal target angles exist only for diagnostics/tests. The baseline controller must operate from image-derived centroid error.

## ADR-005 — Trajectory engine is a pure time -> state map
**Status:** Accepted

The Step-2 target trajectory layer is a stateless mathematical function
`TargetState state_at(double time_s)`, not an `update(dt)` integrator. The simulation
runner owns the clock and passes absolute `time_s`; this keeps replay deterministic and
lets the runner sub-step or resample without the trajectory drifting.

`TargetState` now carries `position_m` and `velocity_mps` (SI, world frame) and lives in
its own header `fsoc/target_state.hpp` so the trajectory layer depends on target truth
without pulling in camera/projection headers. This is the only Step-1 change: the struct
moved out of `environment.hpp` and gained a zero-initialised `velocity_mps`; existing
aggregate initialisation and `Environment` behaviour are unchanged.

Input-validation policy (fail fast, never emit NaN):
- constructor parameters must be finite on every axis; `frequency_hz` and `amplitude_m`
  must be `>= 0` (sign is expressed through `phase_rad`);
- `state_at(time_s)` requires finite, non-negative time;
- all violations throw `std::invalid_argument`.

Sinusoidal frequency is specified in hertz and converted once as `omega = 2*pi*f`;
velocity is the exact analytic derivative `A*omega*cos(omega*t + phi)`.

## ADR-006 — Observation / measurement / tracking-error contracts
**Status:** Accepted

Four distinct types, never aliased into one: `TargetState` (truth), `Projection` /
`CameraObservation` (exact camera projection), `BeaconDetection` (image-pixel estimate),
`TrackingError` (controller-facing error). Dependencies point one way; the future detector
never receives `TargetState` and the future controller never receives world XYZ.

- **Image convention (frozen):** origin top-left, `+x_px` right, `+y_px` down — exactly the
  convention already in `PanTiltCamera::project()`. Image centre is `width_px/2.0`,
  `height_px/2.0`, owned solely by `PanTiltCamera::cx_px()/cy_px()`; no module redefines it.
- **Visibility is a 3-state enum** (`Visible`, `OutsideFieldOfView`, `BehindCamera`), not a
  bare optional, because telemetry and loss handling must tell the two failure modes apart.
  Depth test precedes FOV test.
- **Lost target = empty `std::optional` only.** No `(-1,-1)`, NaN, or zero sentinels. The
  `optional`-in / `optional`-out shape of `compute_tracking_error` makes computing an error
  from a lost target hard to do by accident.
- **Sign conventions (frozen, quadrant-tested):** pixel `error_x>0` = beacon right,
  `error_y>0` = beacon below; angular `pan_rad>0` = command pan right, `tilt_rad>0` =
  command tilt up. Angular conversion reuses `PanTiltCamera::pixel_error_to_angles`; the
  pinhole and pixel->angle equations are not duplicated anywhere.
- **Validation:** non-finite detection centroid throws `std::invalid_argument`; invalid
  image dimensions rejected by existing `CameraConfig::validate()`.
- `TrackingError` is pose-independent (pure image-plane quantity).

No Step 1 / Step 2 source changed. `ScenarioConfig` (trajectory mode, `duration_s`, fixed
`timestep_s`) is added as a pure data contract for the Step 7 runner; it holds no logic.

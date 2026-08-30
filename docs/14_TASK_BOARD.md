# Task Board

## Foundation
- [x] C++20/CMake scaffold
- [x] coordinate math
- [x] camera projection
- [x] actuator rate limits
- [x] native Step-1 tests

## Target motion truth
- [x] `TargetState` = position_m + velocity_mps (own header)
- [x] `Trajectory` abstract interface (time -> state, no owned clock)
- [x] stationary trajectory
- [x] linear constant-velocity trajectory
- [x] sinusoidal trajectory with analytic velocity
- [x] deterministic analytic tests + smoke

## Observation / measurement contracts
- [x] `ImagePoint` + frozen image convention (origin top-left, +x right, +y down)
- [x] `CameraObservation` / `ObservationStatus` (Visible / OutsideFieldOfView / BehindCamera)
- [x] `observe_beacon()` reuses `PanTiltCamera::project()` (no duplicated projection math)
- [x] `BeaconDetection` (centroid only; no fabricated confidence)
- [x] `PixelError` / `AngularError` / `TrackingError` with frozen sign conventions
- [x] `compute_tracking_error()` — `std::optional` in / out, non-finite rejected
- [x] `ScenarioConfig` data contract (mode, duration_s, fixed timestep_s)

## Perception baseline
- [x] OpenCV renderer — `fsoc_render` lib (isolated), `SyntheticCameraRenderer`
- [x] CV_8UC1 grayscale, background + sub-pixel Gaussian beacon (sigma in px)
- [x] edge-safe clipped raster window; non-Visible -> background-only
- [x] test-only weighted-centroid recovers requested sub-pixel location
- [x] threshold detector — `fsoc_perception` lib, `BeaconDetector` (pixels only, no renderer dep)
- [x] centroid — 8-connected components + intensity-weighted centroid, `weight=(pixel-threshold)+1`
- [x] strongest-integrated-signal component selection (deterministic tie-break)
- [x] loss state — `std::nullopt` on no bright pixels / no component >= min size
- [x] input validation — empty / non-CV_8UC1 rejected with `std::invalid_argument`

## Control
- [x] PID class — `fsoc_control` lib (OpenCV-free, `fsoc::core` only), `PIDController`
- [x] `ControlCommand` (pan/tilt rate in rad/s) + `zero_control_command()`
- [x] independent pan/tilt axes; `u = kp*e + ki*I + kd*D` on angular error
- [x] derivative forced to 0 on first update after construction/reset
- [x] anti-windup — integral hard clamp + conditional integration; output clamp
- [x] `reset()` clears integral / previous error / first-sample flags
- [x] validation — bad `dt_s` / config / non-finite `TrackingError` -> `std::invalid_argument`
- [ ] closed-loop runner

## Evidence
- [ ] telemetry CSV
- [ ] performance metrics
- [ ] scenario suite
- [ ] demo overlay
- [ ] freeze `v1_baseline`

## Post-baseline
- [ ] Eigen-backed UKF
- [ ] MPC
- [ ] vibration PSD model
- [ ] Zernike/phase disturbance
- [ ] advanced detection

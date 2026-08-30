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

## Closed-loop integration
- [x] `fsoc_simulation` lib — `SimulationRunner` wires trajectory -> observe -> render ->
      detect -> tracking error -> PID -> camera.step (the one integration layer)
- [x] deterministic fixed-step (dt = 0.02 s, 50 Hz); `step()` + `run_for()` + `reset()`
- [x] `SimulationStepResult` (truth fields labelled; control path never reads them)
- [x] pixel-only feedback verified — control follows detected centroid, not projection truth
- [x] target-loss policy — PID reset + zero command + camera holds; reappearance resumes
- [x] `SimulationMetrics` + `evaluate()` (detection %, RMS/max/final angular error, lost frames)
- [x] open- vs closed-loop comparison (`control_enabled`)
- [x] empirically-tuned MVP baseline gains (kp=12, ki=0, kd=0)

## Evidence
- [x] telemetry CSV — `fsoc_telemetry` lib; `TelemetryRecord` (27 cols) + `CsvTelemetryLogger`
- [x] observer-only, non-interference proven (with/without telemetry -> identical results)
- [x] performance metrics — `BenchmarkMetrics` (RMS/mean/max/final/P95 angular, pixel error,
      saturation fractions, rates); wall clock via `std::chrono`, separate from sim dt
- [x] scenario suite — static / linear / sinusoidal closed / sinusoidal open benchmarks
- [x] `step8_telemetry_smoke` writes generated/step8_*.csv + prints the comparison table
- [x] demo overlay — `fsoc_visualization` lib; `TrackingVisualizer` (CV_8UC1 -> annotated CV_8UC3)
- [x] observer-only, perception frame never touched; mandatory non-interference test passed
- [x] overlays: crosshair / detection marker / error vector / status / HUD (attitude, errors, rates, SAT)
- [x] headless PNG sequence (required) + optional best-effort MP4; output -> generated/step9/
- [x] `step9_visualization_smoke` — static acquisition + sinusoidal + target-lost frames
- [ ] freeze `v1_baseline`

## Post-baseline
- [ ] Eigen-backed UKF
- [ ] MPC
- [ ] vibration PSD model
- [ ] Zernike/phase disturbance
- [ ] advanced detection

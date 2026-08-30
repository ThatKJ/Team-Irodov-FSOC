# Strict 48-Hour Roadmap — C++ MVP

## Hours 0–4 — Step 1: Math + camera foundation [DONE]
- CMake/C++20 project
- coordinate convention
- world-to-camera transform
- FOV/pinhole projection
- pan/tilt rate limits
- native unit checks
- terminal smoke test

Gate: `ctest --preset debug` passes and the camera converges numerically without pixels.

## Hours 4–8 — Step 2: Trajectory engine [DONE]
- `Trajectory` abstract interface: `TargetState state_at(double time_s) const`
- stationary target
- linear constant-velocity target (signed components)
- sinusoidal target with analytic velocity
- deterministic analytic tests + terminal smoke test
- deliberate input validation (`std::invalid_argument`)

Gate: sampled positions/velocities agree with analytic expectations. PASSED —
`fsoc_step2_tests` green, `step2_trajectory_smoke` matches hand computation, Step 1
unchanged (final pointing error still ~0.001 deg).

## Hours 8–13 — Step 3: Observation contract [DONE]
- projection/measurement structs: `ImagePoint`, `CameraObservation` + `ObservationStatus`
  (Visible / OutsideFieldOfView / BehindCamera), `BeaconDetection`, `PixelError`,
  `AngularError`, `TrackingError`
- lost-target semantics: `std::optional` everywhere, no sentinel coordinates
- pixel/angular error sign conventions frozen (see docs/04) and quadrant-tested
- `compute_tracking_error(std::optional<BeaconDetection>, PanTiltCamera)` reuses the
  existing exact `PanTiltCamera::pixel_error_to_angles` — no duplicated math
- `ScenarioConfig` (trajectory mode, duration_s, fixed timestep_s) data contract only

Gate: no controller exists yet; interfaces compile and tests cover signs/units. PASSED —
`fsoc_step3_tests` green (15 checks, all four quadrants + pinhole match + lost-target +
non-finite/invalid rejection + Step 1/2 regression), `step3_observation_smoke` machine-
verifies the (320,240)/(400,180) critical scenario -> pan RIGHT + tilt UP. Steps 1 and 2
unchanged.

## Hours 13–18 — Step 4: Synthetic image renderer [DONE]
- OpenCV C++ introduced, isolated in a separate `fsoc_render` library
  (`fsoc_core` stays OpenCV-free); CMake auto-detects OpenCV, `brew install opencv`
- `SyntheticCameraRenderer` : `CameraObservation -> cv::Mat` (CV_8UC1)
- background (default 5 counts) + analytic 2-D Gaussian beacon, peak 255, sigma in px
- true sub-pixel beacon centre (evaluated at the fractional ImagePoint, never rounded)
- local clipped raster window -> safe at image edges, no overrun
- non-`Visible` observation -> background-only frame (no fake beacon)
- no random noise this step (deterministic pixels first)

Gate: images can be generated headlessly and target pixels match projection coordinates.
PASSED — `fsoc_step4_tests` green (11 checks: dims/type, background level, brightness,
peak+weighted centroid near projection, sub-pixel recovery incl. the (400.4,179.7) manual
check, exact-centre symmetry, four-edge clipping, invalid-config rejection, Visible-needs-
finite-point, determinism, Step 1-3 regression), `step4_renderer_smoke` writes
generated/*.png headlessly. Steps 1-3 unchanged.

## Hours 18–23 — Step 5: Baseline detector [DONE]
- new `fsoc_perception` library (`fsoc::core` + OpenCV core/imgproc); MUST NOT depend
  on `fsoc_render` — `BeaconDetector::detect(const cv::Mat&)` takes pixels only
- intensity threshold (`pixel >= threshold_intensity`, default 64)
- 8-connected components (OpenCV imgproc) -> reject area < `min_bright_pixels`
  -> select the component with the greatest integrated signal (ties: lowest label)
- intensity-weighted centroid over that component only:
  `weight = (pixel - threshold) + 1`, `centroid = sum(w*pos)/sum(w)`
- found -> `std::optional<BeaconDetection>` (Step-3 type); not found -> `std::nullopt`
  (no sentinel); no fabricated "confidence"
- input validation: empty / non-CV_8UC1 frame -> `std::invalid_argument`
- perception chain check: renderer -> detector -> `compute_tracking_error` reproduces
  RIGHT+ABOVE -> pan>0, tilt>0

Gate: centroid error is bounded on clean synthetic frames. PASSED — `fsoc_step5_tests`
green (12 checks); clean interior sub-pixel error ~0.02 px (gate 0.15); the (400.4,179.7)
manual case recovered (400.410, 179.687) from pixels alone; `step5_detector_smoke` PASS.
Steps 1-4 unchanged.

## Hours 23–28 — Step 6: PID control [DONE]
- new `fsoc_control` library — depends on `fsoc::core` ONLY (via `fsoc/tracking_error.hpp`);
  OpenCV-free, no image/perception dependency
- `PIDController::update(const TrackingError&, double dt_s) -> ControlCommand` (rad/s rates,
  never absolute angles); `reset()`; `zero_control_command()` helper for the Step-7 runner
- two independent axes: `u = kp*e + ki*I + kd*D` on `angular.pan_rad` / `angular.tilt_rad`
- derivative `= (e - e_prev)/dt`, forced 0 on the first update after construction/reset
- anti-windup: integral hard-clamped to +/- `integral_limit` every update, plus conditional
  integration (hold the integral while the command is saturated in the error's direction)
- output clamped to +/- `output_limit_rad_s`
- validation: non-finite / <=0 `dt_s` and non-finite `TrackingError` -> `std::invalid_argument`
  (state untouched on throw); invalid config -> `std::invalid_argument`
- sign preserved: `e > 0` (RIGHT / ABOVE) with `kp > 0` -> command `> 0` (PAN RIGHT / TILT UP)
- default gains are UNTUNED placeholders (to be tuned in Step 7)

Gate: controller unit tests pass without OpenCV. PASSED — `fsoc_step6_tests` green (15 checks:
sign in all 4 quadrants + zero, P magnitude, I accumulation + clamp, D step + first-sample,
axis independence, output saturation, anti-windup bounded + prompt recovery, reset, invalid
dt / config / error rejection, deterministic sequence, manual sign check). `step6_pid_smoke`
PASS (incl. toy scalar-plant sanity: 0.100 -> 0.007 rad). Steps 1-5 unchanged.

## Hours 28–34 — Step 7: Closed-loop integration [DONE]
- new `fsoc_simulation` library (links fsoc::core + render + perception + control) — the
  ONE intentional integration layer; owns clock / fixed step / call order / loss policy /
  camera stepping, owns NO domain math
- `SimulationRunner::step()` order: `trajectory.state_at(t)` -> `observe_beacon` ->
  `renderer.render` -> `detector.detect(cv::Mat)` -> `compute_tracking_error(detection, camera)`
  -> `pid.update` (or loss policy) -> `camera.step` -> record -> `t += dt`
- fixed dt = 0.02 s (50 Hz); NOT wall-clock; replayable/deterministic
- TRUTH vs MEASUREMENT: control feedback is ONLY `detector -> compute_tracking_error -> pid`;
  `TargetState` / `observation.image_point_px` used only for the diagnostic result fields and
  the truth-vs-measurement scoring (`detection_error_px`)
- target-loss policy: no detection -> `pid.reset()` + zero command + camera holds (no search)
- empirically-tuned MVP baseline PID: kp=12, ki=0, kd=0 (P-dominant on the integrator plant),
  output limit = camera actuator rate (deg_to_rad(30)); NOT claimed optimal
- open- vs closed-loop comparison built in (`control_enabled` flag)

Gate: moving target remains inside FOV for nominal scenario significantly better than
open-loop. PASSED — static acquisition: 4.13 deg -> 0.0000 deg in ~0.34 s, 100% detection,
final centroid at exact image centre. Sinusoidal (+/-12.4 deg swing, 20 s): 100% detection,
RMS 0.55 deg, max 0.80 deg. Open vs closed on the same trajectory: detection 57.4% -> 100%,
RMS 6.45 deg -> 0.55 deg (11.8x). `fsoc_step7_tests` green (12 checks incl. truth-shortcut
proof). Steps 1-6 unchanged.

## Hours 34–39 — Step 8: Telemetry + metrics
- CSV logger
- FPS
- pixel and angular errors
- pan/tilt + commanded/applied rates
- saturation and visibility
- RMS/95th percentile error

## Hours 39–43 — Step 9: Visualization
- OpenCV display
- center crosshair
- beacon centroid
- FOV/tracking state
- telemetry overlay

Visualization cannot change simulation math.

## Hours 43–46 — Step 10: Validation scenarios
- slow linear
- sinusoidal
- near-FOV-edge acquisition
- actuator saturation
- target loss/re-entry

## Hours 46–48 — Step 11: Demo freeze
- benchmark run
- README instructions
- clean build on teammate Mac
- record metrics/demo
- commit + create `v1_baseline`

Only after this: UKF -> motion prediction -> MPC -> vibration/turbulence -> advanced detection.

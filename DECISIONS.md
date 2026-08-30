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

## ADR-007 — OpenCV enters only through a separate `fsoc_render` library
**Status:** Accepted

Step 4 introduces OpenCV (first allowed by CLAUDE.md rule 4). It is confined to one
translation unit pair, `include/fsoc/renderer.hpp` + `src/renderer.cpp`, compiled into a
**separate `fsoc_render` library** that links `fsoc::core` + OpenCV. `fsoc_core` never
links or includes OpenCV; the pure-math headers stay OpenCV-free. `renderer.hpp` is the
first (and, for now, only) public header that `#include`s `<opencv2/core.hpp>`.

- **Build discovery:** `FSOC_ENABLE_OPENCV` is tri-state `AUTO` (default) | `ON` | `OFF`.
  `AUTO` runs `find_package(OpenCV QUIET COMPONENTS core imgcodecs)` and builds the Step 4
  targets only if found, otherwise prints `brew install opencv` and continues (Steps 1-3
  stay buildable/green without OpenCV). `ON` makes it `REQUIRED`. No Homebrew absolute
  paths anywhere — pure CMake package discovery.
- **Renderer ownership limits:** the renderer owns pixels only. It consumes a
  `CameraObservation` and nothing else — no `TargetState`, world coordinates, trajectory,
  camera pose, control, or estimation. OpenCV owns no simulation state.
- **Image format:** `CV_8UC1`; background 5 counts; beacon = analytic 2-D Gaussian, peak
  255, `sigma` in pixels, evaluated at the true sub-pixel `ImagePoint` (never pre-rounded);
  clipped raster window for edge safety; non-`Visible` observation renders background-only.
- **No sensor noise in Step 4** — deterministic pixels are validated before disturbances;
  any future noise must be optional and explicitly seeded.
- **Dimensions authority is the camera** via `renderer_config_for(const CameraConfig&)`.
- `cv::Mat` is used directly at the perception boundary; it is not wrapped.

Generated debug PNGs go to `generated/` (git-ignored); no binary images are committed.

## ADR-008 — Baseline detector: `fsoc_perception`, pixels-only, threshold + CC + weighted centroid
**Status:** Accepted

Step 5 adds the first perception path. Key decisions:

- **Separate `fsoc_perception` library** (`fsoc/detector.hpp` + `src/detector.cpp`), linking
  `fsoc::core` + OpenCV `core`/`imgproc`. It **must not** depend on `fsoc_render`; the
  header includes neither `renderer.hpp` nor `observation.hpp`. Dependency direction is
  `perception -> core + OpenCV` only. The detector consumes `const cv::Mat&` and nothing
  else, so it works on any valid `CV_8UC1` frame — the renderer is just one source.
- **Transparent baseline algorithm** (no CNN / feature / template / filter): threshold
  (`pixel >= threshold_intensity`) -> 8-connected components (OpenCV
  `connectedComponentsWithStats`) -> drop `area < min_bright_pixels` -> pick the component
  with the greatest integrated signal -> intensity-weighted centroid over that component.
- **Connected components (not a single global centroid)** because two bright regions would
  otherwise be averaged together. `connectedComponentsWithStats` also yields the per-
  component area for the size gate. Ties on integrated signal resolve to the lowest label
  (first in raster order) for determinism.
- **Weight `= (pixel − threshold) + 1`.** Subtracting the config threshold (not an assumed
  background — the detector must not encode the renderer's background of 5) removes the
  pedestal so the weighted centroid follows the beacon's true sub-pixel centre; `+1` keeps
  every in-component pixel contributing so `Σ weight ≥ area ≥ 1`. Measured sub-pixel error
  on clean interior frames ≈ 0.02 px (gate 0.15).
- **`BeaconDetection` stays centroid-only** — no fabricated confidence. Diagnostics
  (integrated signal, area) are computable but not added to the controller-facing type.
- **Lost target = `std::nullopt`** (Step-3 contract). Frame validation rejects empty and
  non-`CV_8UC1` inputs with `std::invalid_argument` rather than reinterpreting them.
- CMake `find_package(OpenCV)` components extended to `core imgproc imgcodecs` (no highgui).

## ADR-009 — Pan/tilt PID controller: `fsoc_control`, OpenCV-free, angular-error in / rate out
**Status:** Accepted

Step 6 adds the first control law. Key decisions:

- **Separate `fsoc_control` library** depending on **`fsoc::core` only** (via
  `fsoc/tracking_error.hpp`). OpenCV-free by construction — it links no OpenCV, `fsoc_render`,
  or `fsoc_perception`, and includes no `opencv2/*`. Dependency direction: `control -> core`.
  It builds even when OpenCV is absent (unlike Steps 4/5).
- **Input is `TrackingError.angular` (radians), never pixels.** Controlling on angle makes
  the loop independent of image resolution, focal length, and FOV. `update()` also gets
  `dt_s`; nothing from the truth/perception layers may bypass `TrackingError`.
- **Output is `ControlCommand` = pan/tilt RATE in rad/s**, never absolute angles. The PID
  never touches `PanTiltCamera`; actuation is Step 7. `zero_control_command()` is provided
  for the runner's target-loss path — the PID has no notion of "lost".
- **Standard discrete PID per independent axis:** `u = kp·e + ki·I + kd·D`,
  `I += e·dt`, `D = (e − e_prev)/dt`. **Derivative is forced to 0 on the first update after
  construction/`reset()`** so an undefined previous sample cannot produce a kick. No
  derivative low-pass filtering in the baseline (no demonstrated need yet).
- **Anti-windup = bounded integrator + conditional integration.** `I` is hard-clamped to
  `±integral_limit` every update; additionally, a sample that would only push an
  already-saturated command further into saturation is not accumulated. This keeps `I`
  bounded and lets it unwind immediately once the error reverses — no back-calculation
  needed. Output is clamped to `±output_limit_rad_s`.
- **Sign is preserved, not inverted:** `e > 0` (RIGHT / ABOVE) with `kp > 0` gives
  `command > 0` (PAN RIGHT / TILT UP), matching the frozen `TrackingError` convention and
  `PanTiltCamera::step`.
- **Validation throws before mutating state:** non-finite / `≤ 0` `dt_s` and any non-finite
  `TrackingError` component raise `std::invalid_argument` with the controller state
  untouched; invalid config raises at construction.
- **Default gains are explicitly untuned placeholders** (`kp=1.5, ki=0.2, kd=0.05`,
  `integral_limit=0.5`, `output_limit_rad_s=deg_to_rad(30)`), to be tuned in Step 7. The
  PID never reads `CameraConfig`; the Step-7 runner keeps controller/actuator limits aligned.
- `PIDController::Axis` is a private nested helper, not a public reusable contract.

## ADR-010 — Closed-loop integration: `fsoc_simulation`, pixel-only feedback, P-dominant baseline
**Status:** Accepted

Step 7 wires the tested modules into the first real closed loop. Key decisions:

- **One integration library `fsoc_simulation`** links all four lower libs (`core`, `render`,
  `perception`, `control`). Integration is its job; the lower libs remain unaware of each
  other. `SimulationRunner` owns the clock, the fixed timestep, subsystem call order, the
  target-loss policy, and camera stepping — and duplicates **no** domain math.
- **Frozen `step()` order:** trajectory → `observe_beacon` → `renderer.render` →
  `detector.detect` → `compute_tracking_error` → PID (or loss policy) → `camera.step` →
  record → advance time. The camera is never moved before the frame is computed.
- **Truth vs measurement is enforced, not just intended.** The only feedback path is
  `detector.detect(cv::Mat)` → `compute_tracking_error(detection, camera)` → `pid.update`.
  `TargetState`, `CameraObservation.image_point_px`, and the exact `Projection` populate
  only the diagnostic `SimulationStepResult` fields and the `detection_error_px` score.
  An executable test injects a frame whose detected blob is on the opposite side of centre
  from the true projection and shows the command follows the **detected** blob.
- **Fixed dt only** (0.02 s = 50 Hz), advanced by `sim_time += dt`. No wall-clock in the
  dynamics; the run is bit-for-bit replayable.
- **Target-loss policy:** no detection → `pid.reset()` + zero command + camera holds. No
  search / scan / reacquisition mode in this step. If the target drifts back into the FOV,
  the PID resumes from reset.
- **Trajectory injected by `const Trajectory&`** (caller owns lifetime) — the runner never
  hardcodes a concrete trajectory. The camera is owned by the runner; `PanTiltCamera`
  stays the sole authority on pan/tilt state.
- **`SimulationRunnerConfig::validate()` rejects PID output limit > camera actuator rate**
  and renderer dimensions ≠ camera dimensions, so the runner can never silently demand a
  rate the actuator cannot deliver. The baseline sets PID output limit = camera max rate.
- **Empirically-tuned baseline PID: kp = 12, ki = 0, kd = 0.** Method: the plant (camera
  angle = integral of rate command) already contains one integrator, so with P-only the
  stationary-target loop is first-order (`de/dt = −kp·e`) — non-oscillatory for any
  `kp·dt < 1`. Swept kp ∈ {3, 6, 9, 12, 16, 20}: static settling time 1.44 s → 0.22 s,
  sinusoidal RMS 2.10° → 0.33°, all stable, all 100 % detection. Chose kp = 12: exact
  zero static steady-state error, sinusoidal RMS 0.55°, `kp·dt = 0.24` (well-damped),
  detector-noise amplification ≈ 1.3e-4 rad/s (negligible). Ki = 0: no steady-state bias
  on the integrator plant, and Ki would form a marginally-stable double integrator needing
  Kd. Kd = 0: plant already well damped; derivative would only amplify the ~0.02 px
  detector noise. NOT claimed optimal.
- **`SimulationMetrics` + `evaluate()`** are simple scoring helpers in `fsoc_simulation`,
  not the Step-8 telemetry system (no CSV/JSON/stream/logger).

## ADR-011 — Telemetry is an observer-only sink (`fsoc_telemetry`)
**Status:** Accepted

Step 8 adds telemetry + benchmarking. Key decisions:

- **`fsoc_telemetry` is a SINK.** It links `fsoc::simulation` and consumes
  `SimulationStepResult` values; it never calls back into the runner, PID, camera,
  detector, renderer, or trajectory. A simulation run with telemetry produces a
  bit-identical `SimulationStepResult` sequence to one without it — proven by
  `test_telemetry_non_interference` (run A plain vs run B with a CSV logger interleaved).
- **Absent values are `std::optional`, never a sentinel.** In memory, unavailable
  measurements are `std::nullopt`. In CSV they are empty fields between commas. No `-1`,
  NaN, or `N/A` anywhere in the telemetry API or the documented CSV format.
- **`TrackingState` has two values** (`Tracking`, `TargetLost`). The runner defines no
  deterministic acquisition phase and "flat-out slew while tracking" is already carried by
  `pan_saturated` / `tilt_saturated`, so an `Acquiring` state would be decorative.
- **`*_saturated` = "axis at the actuator rate limit"** (`|command rate| >= max rate - 1e-9`),
  because the baseline sets the PID output limit equal to the actuator rate, which makes
  the literal `|command - applied|` formula always zero. `make_telemetry_record` therefore
  takes the actuator rate limits rather than the record carrying the whole camera config.
- **CSV logger is synchronous** — `std::ofstream`, one line per record flushed to disk, no
  threads / async queue / networking / external CSV library. `column_names()` is the single
  source of column order (and the stable JSON key list for a later frontend).
- **Metrics denominators (documented in `docs/08_TELEMETRY_SCHEMA.md`):** angular/pixel
  error over `tracking_frames`; detection-error over frames with `detection_error_px`;
  saturation and rate stats over all frames.
- **Percentile = nearest-rank**, `index = ceil(0.95 * N) - 1` clamped to `[0, N-1]`, on the
  sorted-ascending magnitudes. No statistics dependency.
- **Wall clock is separate from simulation time.** Physics stays on the fixed `dt = 0.02 s`.
  `processing_fps = frames / wall_execution_time_s` is measured with `std::chrono` around
  the step loop only and never influences the simulation. Reported as a distinct number
  (~4700 FPS / ~90x real time) from the 50 Hz simulation rate.
- `SimulationStepResult` / `SimulationRunner` (Step 7) were NOT modified.

## ADR-012 — Visualization is an observer-only overlay; no runner change (`fsoc_visualization`)
**Status:** Accepted

Step 9 adds the engineering camera-view visualization. Key decisions:

- **`fsoc_visualization` is a SINK.** It links `fsoc::simulation` + `fsoc::telemetry` +
  OpenCV core/imgproc/imgcodecs (videoio iff present). `TrackingVisualizer::annotate()`
  reads a `SimulationStepResult` + `TelemetryRecord` and never calls back into the runner,
  detector, PID, camera, or trajectory. A run with vs without visualization produces a
  bit-identical `SimulationStepResult` sequence (`test_visualization_non_interference`).
- **The perception frame is never contaminated.** `annotate()` takes the `CV_8UC1` frame by
  const reference and does not modify it (byte-for-byte identical after the call). It
  returns a fresh `CV_8UC3` BGR image (`cvtColor` into a new buffer, then draws). The
  detector always runs on the original grayscale frame; overlay pixels cannot reach it.
- **`SimulationRunner` / `SimulationStepResult` were NOT changed.** The base frame for a
  result is reconstructed with `SyntheticCameraRenderer{config.renderer}.render(result.observation)`.
  The renderer is deterministic (Step 4), so this is byte-identical to the frame the runner
  detected on — zero coupling into control, and no heavy `cv::Mat` in the step result or
  telemetry (both stay lightweight). This is the minimal-disturbance path among the options
  in the Step-9 brief.
- **Detection over truth.** The default view shows the detected centroid + error vector.
  The exact projection (`DETECT ERR`, `TRUTH` square marker) is off by default; when enabled
  it is a distinct labelled marker and is never fed into control.
- **Headless first.** PNG per selected frame (`write_png`) is the required path — no
  `cv::imshow` / `cv::waitKey`. MP4 (`try_write_mp4` via `cv::VideoWriter`) is optional and
  best-effort: `videoio` is linked only if the OpenCV build has it (`FSOC_HAS_VIDEOIO`), and
  a missing codec/backend returns `false` without throwing and leaves no partial file. Step
  9 is never RED because MP4 is unavailable.
- Colour semantics are fixed (green = tracking, red = lost, amber = saturation, grey =
  neutral, cyan = optional truth); HUD is degrees for humans while physics stays radians.
- No 3D scene / FOV cone / telemetry graphs here — those belong in the future frontend.

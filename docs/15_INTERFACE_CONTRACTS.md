# Interface Contracts

Planned contracts should preserve this information flow:

```cpp
TargetState trajectory.state_at(double sim_time_s);   // { position_m, velocity_mps }
std::optional<Projection> camera.project(Vec3 target_world_m);
CameraObservation observe_beacon(const PanTiltCamera&, Vec3 beacon_world_position_m);
cv::Mat SyntheticCameraRenderer::render(const CameraObservation& observation);  // CV_8UC1
std::optional<BeaconDetection> BeaconDetector::detect(const cv::Mat& frame);    // pixels only
std::optional<TrackingError> compute_tracking_error(
    const std::optional<BeaconDetection>&, const PanTiltCamera&);
ControlCommand PIDController::update(const TrackingError&, double dt_s);  // rad/s rates
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

## Synthetic image renderer contract (Step 4, frozen)

- **First OpenCV boundary.** OpenCV appears only in `fsoc/renderer.hpp` + `src/renderer.cpp`,
  built as a separate `fsoc_render` library. `fsoc_core` and every pure-math header
  (`geometry.hpp`, `camera.hpp`, `target_state.hpp`, `trajectory.hpp`, `observation.hpp`,
  `tracking_error.hpp`, `scenario.hpp`) stay OpenCV-free.
- **Input / output:** `SyntheticCameraRenderer::render(const CameraObservation&) -> cv::Mat`,
  always `CV_8UC1` of the configured size. The renderer sees only the observation — never
  `TargetState`, world coordinates, trajectory, or pan/tilt state.
- **Image model:** uniform background (`background_intensity`, default 5) plus an analytic
  2-D Gaussian beacon,
  `I(u,v) = background + peak·exp(-((u-u0)² + (v-v0)²)/(2σ²))`, clamped to `[0,255]` and
  rounded. `peak` default 255. **σ is in pixels.**
- **Pixel-sampling convention:** pixel `(u,v)` is sampled at continuous coordinate `(u,v)`
  (pixel centres at integer coords, consistent with `cx = W/2.0`).
- **Sub-pixel:** the beacon centre `(u0,v0)` is the fractional `ImagePoint` value; it is
  never rounded before evaluating the Gaussian, so a weighted centroid recovers the
  requested sub-pixel location.
- **Status behaviour:** `Visible` → beacon rendered; `OutsideFieldOfView` / `BehindCamera`
  → background-only frame (never a fabricated beacon).
- **Edge safety:** the Gaussian is evaluated in a raster window clipped to the image; a
  near- or off-edge beacon cannot index out of bounds.
- **Validation:** `RendererConfig::validate()` rejects `width_px<=0`, `height_px<=0`,
  non-finite/`<=0` `beacon_sigma_px`, `beacon_window_sigmas<1`. `render()` throws
  `std::invalid_argument` if `status==Visible` but the image point is missing or non-finite.
- **Dimensions authority:** the camera. Build `RendererConfig` via
  `renderer_config_for(const CameraConfig&, ...)` so renderer and camera cannot disagree.
- **No noise this step.** Deterministic: the same observation renders byte-identical frames.

## Baseline beacon detector contract (Step 5, frozen)

- **Module boundary.** The detector lives in `fsoc_perception` (`fsoc/detector.hpp` +
  `src/detector.cpp`), linking `fsoc::core` + OpenCV `core`/`imgproc`. It **must not**
  depend on `fsoc_render` and its headers include **neither** `renderer.hpp` **nor**
  `observation.hpp`. `detect()` takes only `const cv::Mat&` — never `TargetState`,
  trajectory, `CameraObservation`, the exact projected `ImagePoint`, `TrackingError`, or
  controller state. It works on any valid `CV_8UC1` frame, not only renderer output.
- **Input validation.** `detect()` throws `std::invalid_argument` for an empty `cv::Mat`
  or any type other than `CV_8UC1` (RGB / 16-bit / float frames are rejected, not
  reinterpreted). `BeaconDetectorConfig::validate()` requires
  `threshold_intensity ∈ [1, 254]` and `min_bright_pixels ≥ 1`.
- **Threshold rule.** A pixel is a candidate iff `value >= threshold_intensity`
  (default 64 — a generic bright cut, not tied to the renderer's background).
- **Component selection.** 8-connected components on the candidate mask; components with
  `area < min_bright_pixels` are dropped; the surviving component with the greatest
  **integrated signal** `Σ weight` wins. Ties resolve to the lowest label (first in raster
  order) — deterministic.
- **Centroid math.** Over the winning component's pixels only:
  `weight(p) = (pixel(p) − threshold_intensity) + 1` (always ≥ 1);
  `centroid = Σ weight·pos / Σ weight`. Subtracting the threshold (not an assumed
  background) removes the pedestal so the estimate tracks the true sub-pixel centre;
  restricting to one component stops a second bright region from pulling it.
- **Sub-pixel gate.** On clean interior synthetic frames, `|Δx|, |Δy| ≤ 0.15 px`
  (achieved ≈ 0.02 px). Edge-clipped beacons are found safely but carry a bias.
- **Lost target.** `std::nullopt` when no pixel passes the threshold or no component is
  large enough. No `(-1,-1)`, NaN, or zero sentinel — the Step-3 contract holds.
- **Deterministic.** The same frame yields a bit-identical centroid.
- `BeaconDetection` stays centroid-only: no fabricated confidence.

## Pan/tilt PID controller contract (Step 6, frozen)

- **Module boundary.** The controller lives in `fsoc_control` (`fsoc/pid_controller.hpp` +
  `src/pid_controller.cpp`), linking **`fsoc::core` only** (via `fsoc/tracking_error.hpp`
  for the `TrackingError` contract). It is **OpenCV-free** and must not link or include
  `fsoc_render` / `fsoc_perception` / OpenCV. `update()` consumes only `TrackingError`
  (radians) + `dt_s` — never `TargetState`, target XYZ, trajectory, `CameraObservation`,
  `cv::Mat`, `BeaconDetection`, a projected `ImagePoint`, or `PanTiltCamera`.
- **Output.** `ControlCommand { double pan_rate_rad_s; double tilt_rate_rad_s; }` —
  desired actuator angular **velocity**, rad/s. Never absolute angles; the PID never
  touches `PanTiltCamera` (actuation is Step 7). `zero_control_command()` is the explicit
  hold-still value the runner uses on target loss.
- **Law (per axis, discrete).** `e` = `angular.pan_rad` / `angular.tilt_rad`;
  `D = (e − e_prev)/dt_s`, forced to `0` on the first `update()` after construction/`reset()`;
  `I += e·dt_s`; `u = kp·e + ki·I + kd·D`; `command = clamp(u, ±output_limit_rad_s)`.
- **Anti-windup.** (1) `I` is hard-clamped to `±integral_limit` every update. (2) Conditional
  integration: if `u` is beyond `±output_limit_rad_s` **and** `e` has the same sign as that
  saturation, the sample is not accumulated (`I` held). Any other case commits the clamped
  accumulation, so `I` unwinds as soon as `e` reverses.
- **Sign (frozen).** `e > 0` (beacon RIGHT / ABOVE) with `kp > 0` ⇒ `command > 0` ⇒ pan
  right / tilt up. Neither axis is inverted. Positive error → positive rate, which drives
  the axis toward the beacon under `PanTiltCamera::step`.
- **Reset.** `reset()` clears both integrals, both previous errors, and both first-sample
  flags; the next `update()` behaves exactly like the first.
- **Validation.** `update()` throws `std::invalid_argument` if `dt_s` is non-finite or
  `≤ 0`, or if any `TrackingError` component is non-finite — **state is not mutated on
  throw**. `PIDControllerConfig::validate()` requires finite `kp, ki, kd, integral_limit ≥ 0`
  and finite `output_limit_rad_s > 0` on both axes.
- **Axes independent.** Pan and tilt each own their integral / previous-error / first-sample
  state; one axis saturating or winding up does not affect the other.
- **Deterministic, allocation-free.** The same input sequence from `reset()` yields a
  bit-identical command sequence; `update()` performs no heap allocation.
- **Gains.** `PIDControllerConfig{}` defaults are **untuned baseline placeholders** — to be
  tuned during the Step-7 closed-loop experiments. `output_limit_rad_s` defaults to
  `deg_to_rad(30)` (the default `CameraConfig` actuator rate); the Step-7 runner is
  responsible for keeping controller and actuator limits consistent.

## Closed-loop SimulationRunner contract (Step 7, frozen)

- **The one integration layer.** `fsoc_simulation` (`fsoc/simulation_runner.hpp` +
  `src/simulation_runner.cpp`) links `fsoc::core` + `fsoc::render` + `fsoc::perception` +
  `fsoc::control`. This is the **only** place those modules come together; the lower
  libraries stay independent of each other. The runner owns the clock, the fixed timestep,
  subsystem call order, the target-loss policy, and camera stepping — and **no domain
  math** (it duplicates no projection / trajectory / centroid / PID / pixel→angle code).
- **`step()` order (frozen).** `trajectory.state_at(sim_time)` → `observe_beacon(camera, pos)`
  → `renderer.render(observation)` → `detector.detect(frame)` →
  `compute_tracking_error(detection, camera)` → if error present `pid.update(error, dt)`
  else `pid.reset()` + `zero_control_command()` → `camera.step(cmd.pan, cmd.tilt, dt)` →
  record `SimulationStepResult` → `sim_time += dt`, `++frame_index`. The camera is **not**
  moved before the frame is computed.
- **Fixed timestep.** `timestep_s` (default 0.02 s = 50 Hz), advanced by `sim_time += dt`.
  **Never wall-clock.** `std::chrono` may measure execution speed but never the simulation
  dt. Fully replayable: same config + trajectory → bit-identical `SimulationStepResult`
  sequence.
- **TRUTH vs MEASUREMENT (frozen).** The control feedback path is exactly
  `detector.detect(cv::Mat)` → `compute_tracking_error(detection, camera)` → `pid.update`.
  `TargetState.position_m/velocity_mps`, `CameraObservation.image_point_px`, and the exact
  `Projection` are used **only** for the diagnostic fields of `SimulationStepResult`
  (labelled "truth") and for `detection_error_px` scoring. None ever reaches the controller.
  Verified structurally (the runner reads `observation.image_point_px` only in the scoring
  block) and executably (`test_control_follows_detected_not_truth`).
- **Target-loss policy.** No detection ⇒ `tracking_error == std::nullopt` ⇒ `pid.reset()`,
  `command == {0,0}`, `camera.step(0,0,dt)` (camera holds its orientation). No search /
  reacquisition. If target motion brings the beacon back into the FOV, the PID resumes from
  its reset state.
- **Trajectory injection.** `SimulationRunner(SimulationRunnerConfig, const Trajectory&)` —
  any `Trajectory` subclass; the runner holds it by reference (caller keeps it alive) and
  never hardcodes a concrete trajectory.
- **Camera authority.** The runner constructs/owns `PanTiltCamera` and never duplicates its
  pan/tilt state; `reset()` reconstructs it at the configured initial pose.
- **Controller/actuator consistency.** `SimulationRunnerConfig::validate()` rejects a
  configuration whose PID `output_limit_rad_s` exceeds the camera's `max_*_rate_rad_s`, and
  requires renderer dimensions to equal the camera's. The MVP baseline sets them equal.
- **Baseline gains (empirically tuned, NOT optimal).** `baseline_runner_config()` uses
  kp = 12, ki = 0, kd = 0 on both axes (P-dominant: the plant angle is the integral of the
  rate command, so P alone gives a first-order, non-oscillatory response with zero
  steady-state error for a stationary target). Output limit = `deg_to_rad(30)`.
- **Performance gates.** Static acquisition: 100 % detection after frame 0, final total
  angular error < 0.05° (achieved ≈ 0°), final centroid < 2 px from centre. Linear:
  ≥ 95 % detection, RMS angular error < 0.75° (achieved ≈ 0.39°). Sinusoidal (±12.4°
  swing): ≥ 95 % detection, RMS < 1.0° (achieved ≈ 0.55°), max < 1.5°. Closed-loop must
  beat open-loop on both detection fraction and RMS on the same trajectory (achieved
  57 % → 100 %, 6.45° → 0.55°).

## Telemetry + benchmarking contract (Step 8, frozen)

- **Observer only.** `fsoc_telemetry` (`fsoc/telemetry.hpp` + `src/telemetry.cpp`) depends
  on `fsoc::simulation`. It is a SINK: `make_telemetry_record(...)`,
  `compute_benchmark_metrics(...)`, `CsvTelemetryLogger`, and `run_and_record(...)` consume
  `SimulationStepResult` values and never call back into the runner / PID / camera /
  detector / renderer / trajectory. Running a simulation with or without telemetry yields a
  **bit-identical** `SimulationStepResult` sequence (`test_telemetry_non_interference`).
- **`TelemetryRecord`** — 27 flat, unit-suffixed, JSON-mappable fields (full table in
  `docs/08_TELEMETRY_SCHEMA.md`). Unavailable measurements are `std::optional<double>` /
  `std::nullopt` **in memory — never a `-1` / NaN / `N/A` sentinel** — and **empty fields**
  in CSV. `make_telemetry_record` takes the actuator rate limits so the `*_saturated` flags
  can be set without the record carrying the whole camera config.
- **`TrackingState`** — `{ Tracking, TargetLost }` only. Present in every record;
  `Tracking` iff the frame produced a `TrackingError`.
- **`*_saturated`** — true when `|command_*_rate| ≥ max_*_rate − 1e-9` (axis slewing at the
  actuator limit); in the baseline the PID output limit equals the actuator rate, so this
  is equivalent to "PID output saturated" / "camera clipped the command".
- **`CsvTelemetryLogger`** — synchronous `std::ofstream`, header from
  `column_names()` (the one source of column order), one line per record flushed to disk,
  no threads / async / external CSV library. Output goes to `generated/` (git-ignored);
  logs are never committed.
- **`BenchmarkMetrics`** — see `docs/08_TELEMETRY_SCHEMA.md` for the full list.
  **Denominators:** angular/pixel error metrics over frames with a `TrackingError`
  (`tracking_frames`); `mean_detection_error_px` over frames with a `detection_error_px`;
  saturation fractions and rate means/peaks over all frames; `detection_fraction =
  detected_frames / frames`. **95th percentile:** nearest-rank on the sorted-ascending
  magnitudes, `index = ceil(0.95·N) − 1` clamped to `[0, N-1]`; no statistics library.
- **Wall clock vs simulation clock.** Physics uses the fixed `dt = 0.02 s` (50 Hz) only.
  `wall_execution_time_s` / `processing_fps = frames / wall_execution_time_s` are measured
  with `std::chrono::steady_clock` around the step loop **only** (telemetry conversion and
  CSV I/O excluded) and never feed the simulation timestep. The two rates are reported
  separately (`50 Hz` simulation vs `~4700 FPS` processing / `~90×` real time).

## Engineering visualization contract (Step 9, frozen)

- **Observer only.** `fsoc_visualization` (`fsoc/visualization.hpp` + `src/visualization.cpp`)
  depends on `fsoc::simulation` + `fsoc::telemetry` + OpenCV core/imgproc/imgcodecs (and
  `videoio` iff present). `TrackingVisualizer::annotate()` reads a `SimulationStepResult` +
  `TelemetryRecord` and **never** calls back into the runner / detector / PID / camera /
  trajectory. Running a simulation with vs without visualization yields a bit-identical
  `SimulationStepResult` sequence (`test_visualization_non_interference`, 500 frames).
- **The perception frame is untouched.** `annotate(const cv::Mat& grayscale_frame, …)` takes
  the `CV_8UC1` frame by const reference and leaves it **byte-for-byte identical**; it
  returns a **new `CV_8UC3` BGR** image of the same size (`cv::cvtColor(GRAY2BGR)` into a
  fresh buffer, then draws). Overlay pixels can never reach the detector. Empty /
  non-`CV_8UC1` input → `std::invalid_argument`.
- **Base-frame reconstruction — no runner change.** `SimulationRunner` /
  `SimulationStepResult` are unchanged. The base frame for a result is
  `SyntheticCameraRenderer{config.renderer}.render(result.observation)` — byte-identical to
  the frame the runner detected on (the renderer is deterministic, Step 4). No `cv::Mat` is
  stored in `SimulationStepResult` / `TelemetryRecord`.
- **Overlays** (full table + colour semantics in `docs/09_VISUALIZATION.md`): centre
  crosshair from `(cols/2, rows/2)` (not hardcoded); detection marker at
  `telemetry.detected_*` (absent when `TargetLost` — no fake marker); centre→detected error
  vector (shown iff a `TrackingError` exists; shrinks to zero on convergence); `TRACKING` /
  `TARGET LOST` from `telemetry.tracking_state`; `VISIBLE` vs `DETECTED` (distinct); SIM /
  FRAME; PAN / TILT / ANG ERR / ERR PX; CMD rates with amber `RATE LIMIT` driven by the
  Step-8 `*_saturated` flags. HUD values are degrees for humans; physics stays radians.
- **Colours (BGR, fixed):** green = tracking, red = target lost, amber = saturation/limit,
  white-grey = crosshair/neutral data, cyan = optional TRUTH marker.
- **Truth vs detection.** The default demo view emphasises the **detected** centroid. The
  exact projection (`DETECT ERR` line and `TRUTH` square marker) is **off by default**; when
  enabled it is a visibly distinct labelled marker and is never fed into control.
- **Headless.** PNG per selected frame (`write_png`) is the required portable path (no
  `cv::imshow` / `cv::waitKey`). `try_write_mp4` is best-effort: returns `false` without
  throwing when there is no `videoio` / codec / backend, and leaves no partial file. Output
  → `generated/` (git-ignored); no images committed.
- `VisualizationConfig` gates every overlay independently; all off → `annotate()` returns a
  plain `cvtColor(GRAY2BGR)` of the input.

# Engineering Camera-View Visualization (Step 9)

`fsoc::TrackingVisualizer` draws a restrained mission-control debug overlay for the FSOC
closed loop. It is the engineering visualization layer, **not** the future web frontend and
**not** a 3D scene view (camera orientation / trajectory / FOV cone / telemetry graphs
belong in the frontend and are not duplicated here).

## Observer-only (frozen)

- `annotate()` takes the perception frame by `const cv::Mat&` and **never modifies it**
  (byte-for-byte identical after the call — `test_output_format_and_input_immutable`,
  `test_visualization_non_interference`).
- The control path keeps running on the **original unannotated `CV_8UC1`** frame. Overlay
  pixels can never reach the detector: `annotate()` builds a fresh `CV_8UC3` buffer via
  `cv::cvtColor(GRAY2BGR)` and draws on that.
- Nothing here feeds back into detection / tracking error / PID / camera / trajectory /
  simulation time / telemetry values. Running a simulation with or without visualization
  produces a bit-identical `SimulationStepResult` sequence (mandatory non-interference
  test).

## Base-frame reconstruction

`SimulationRunner` is unchanged (Steps 1–8 untouched). The base frame for a
`SimulationStepResult` is reconstructed by rendering `result.observation` through a
`SyntheticCameraRenderer` built from the same `RendererConfig`. The renderer is
deterministic (Step 4), so the reconstructed frame is byte-identical to the one the runner
detected on — with zero coupling into the control path and no heavy `cv::Mat` stored in
`SimulationStepResult` / `TelemetryRecord`.

## Display frame

- Input: non-empty `CV_8UC1` (the perception frame). Empty / non-`CV_8UC1` → `std::invalid_argument`.
- Output: a **new** `CV_8UC3` (BGR) image of the same size.

## Overlays

| overlay | source | meaning |
|---|---|---|
| centre crosshair | frame geometry `(cols/2, rows/2)` — not hardcoded | desired beacon position (optical axis) |
| detection marker | `telemetry.detected_{x,y}_px` | detector output (round reticle); absent when `TargetLost` |
| error vector | centre → detected point; shown iff a `TrackingError` exists | tracking error; shrinks to nothing as the loop converges |
| status | `telemetry.tracking_state` | `TRACKING` (green) / `TARGET LOST` (red) |
| visibility | `target_visible` vs `target_detected` | truth-in-FOV vs measurement-found (distinct concepts) |
| SIM / FRAME | `simulation_time_s`, `frame_index` | fixed-step sim clock (not wall clock) |
| PAN / TILT | `camera_{pan,tilt}_rad` → deg | camera attitude (HUD is degrees; physics stays radians) |
| ANG ERR | `angular_error_total_rad` → deg | total angular tracking error; `--` when lost |
| ERR PX | `pixel_error_{x,y}_px` | image-plane error; `--` when lost |
| CMD PAN / TILT | `command_{pan,tilt}_rate_rad_s` → deg/s | controller rate command; amber + `RATE LIMIT` line when the matching `*_saturated` telemetry flag is set |
| DETECT ERR (opt) | `detection_error_px` | truth-vs-measurement scoring — **off by default**, never confused with tracking error |
| TRUTH marker (opt) | `result.observation.image_point_px` | exact projection — **off by default**, drawn as a distinct SQUARE labelled `TRUTH`, never used for control |

Layout: title / state / visibility / sim-time top-left; PAN / TILT / ANG ERR top-right;
detected centroid + pixel error bottom-left; command rates + saturation bottom-right. The
central viewport is kept clear of text so the beacon is never covered. Each HUD block sits
on a subtle translucent dark panel.

## Colour semantics (BGR, fixed)

| colour | BGR | meaning |
|---|---|---|
| green | `(90, 220, 90)` | detection valid / error vector / `TRACKING` |
| red | `(60, 60, 235)` | `TARGET LOST` |
| amber | `(40, 190, 240)` | saturation / rate limit / warning |
| white-grey | `(200, 200, 200)` | crosshair / neutral engineering data |
| dim grey | `(140, 140, 140)` | labels / unavailable values (`--`) |
| cyan | `(240, 200, 70)` | optional `TRUTH` marker only |

## Headless export

- **Required, portable:** PNG per selected frame via `write_png(path, image)` (creates
  parent dirs). Selected frames only — never thousands.
- **Optional:** `try_write_mp4(path, frames, fps)` via `cv::VideoWriter` (`mp4v`). Returns
  `false` **without throwing** when the OpenCV build has no `videoio`, no codec/backend
  opens, the frame list is empty, or the frames are not a uniform `CV_8UC3` size; leaves no
  partial file. Step 9 is never RED because MP4 is unavailable.
- Output goes to `generated/` (git-ignored): `generated/step9/{static,sinusoidal}_####.png`,
  `generated/step9/lost_{lost,reacquired}.png`, and optionally
  `generated/step9_{static,sinusoidal}.mp4`.

## Config

`VisualizationConfig` — one `bool` per overlay (all default `true` except
`show_detection_error` and `show_truth_marker`), plus `VisualizationColours` and a `title`
string. All overlays off → `annotate()` returns a plain `cvtColor(GRAY2BGR)` of the input.

# 48-Hour MVP Checklist — C++20

- [x] CMake/C++20 skeleton
- [x] Frozen coordinate convention
- [x] Vec3 geometry primitives
- [x] Virtual pan/tilt camera
- [x] Narrow FOV + pinhole projection
- [x] Velocity limits + tilt stops
- [x] Step-1 unit checks
- [x] Terminal math smoke test
- [x] Trajectory interface + stationary + linear trajectory
- [x] Sinusoidal trajectory
- [x] Measurement/error data contracts (observation / detection / tracking-error, frozen sign conventions)
- [x] Synthetic grayscale beacon renderer (OpenCV C++) — CV_8UC1, sub-pixel Gaussian beacon
- [x] Threshold + centroid detector — connected-components + intensity-weighted centroid, ~0.02 px on clean frames
- [x] Independent pan/tilt PID controller — `fsoc_control` (OpenCV-free), angular error -> rate command, anti-windup, reset
- [x] Closed-loop SimulationRunner — `fsoc_simulation`, deterministic fixed-step, pixel-only feedback, static acq final error ~0 deg
- [x] Telemetry logger — `fsoc_telemetry`, observer-only `TelemetryRecord` + synchronous `CsvTelemetryLogger` (27 cols)
- [x] FPS and timing measurement — `std::chrono` wall clock, separate from the fixed sim dt (~4700 FPS, ~90x real time)
- [x] Target-loss behavior — no detection -> PID reset + zero command + camera holds (no search mode)
- [x] Rate saturation telemetry — `pan_saturated` / `tilt_saturated` + `command_saturation_fraction` benchmark metric
- [x] Parameterized benchmark scenarios — static / linear / sinusoidal closed / sinusoidal open, `BenchmarkMetrics` (RMS / P95 / max)
- [x] OpenCV visualization — `fsoc_visualization`, observer-only `TrackingVisualizer` (CV_8UC1 -> annotated CV_8UC3), headless PNG/MP4 export
- [x] Plot/export telemetry for judging — `fsoc_validation` suite writes per-scenario 27-col CSV + annotated PNG + `generated/step10/VALIDATION_REPORT.md` + judge-friendly summary table
- [x] Baseline acceptance metrics met — 7/7 scenarios PASS (Static / Slow-Linear / Sinusoidal / Near-FOV-Edge / Actuator-Saturation / Loss-and-Re-entry / Open-vs-Closed); gates frozen in `docs/16_BASELINE_ACCEPTANCE.md`; `step10_validation_smoke` prints `STEP 10 BASELINE ACCEPTANCE: PASS`
- [x] Freeze `v1_baseline` — **tag created and pushed to `origin`** (points at the merged Step‑10 baseline `20c028c`); the validated Step‑10 baseline is FROZEN. Step‑11 demo/frontend packaging is additive work layered on top and does not move the tag.
- [x] Demo scenario presets — `DemoScenario` (static / sinusoidal / loss / open / closed) + `parse_demo_scenario`, reusing the validated Step-10 parameters verbatim
- [x] Frontend snapshot contract — `DemoSnapshot` view model built only from `SimulationStepResult` + `TelemetryRecord` + `CameraConfig` (`fsoc_demo_support`); radians internal, `to_degrees()` at the UI boundary; documented in `docs/18_FRONTEND_DATA_CONTRACT.md`
- [x] Demo session runner — `DemoSession` (deterministic, owns trajectory + `SimulationRunner`; `DemoRunState` Ready/Running/Paused/Finished; paused `step()` does not advance sim time); non-interference proven vs a bare `SimulationRunner`
- [x] Demo CLI — `apps/fsoc_demo.cpp` (`static|sinusoidal|loss|open|closed`, `--help`, `--csv`, `--duration`, `--quiet`); reuses the Step-8 telemetry + benchmark code
- [x] Reproducible demo workflow — `make demo` / `scripts/run_baseline_demo.sh` (validation + static + sinusoidal demos + Step-9 visualization evidence)
- [x] Demo freeze doc + teammate Mac checklist — `docs/17_DEMO_FREEZE.md`
- [x] Step-11 tests — `fsoc_step11_tests` (23 checks: snapshot copy, radians, `to_degrees`, session convergence, open==closed trajectory, reset replay, pause-no-advance, non-interference, baseline PID unchanged, Step-10 still PASS)

## V2 — AI PERCEPTION (`feat/ai-perception`, post-`v1_baseline`)

- [x] AI design + integration decisions — `DECISIONS.md` ADR-015 (learned detector behind the frozen `BeaconDetection` contract; TinyBeaconNet, not YOLO; ONNX→OpenCV-DNN; synthetic-only), ADR-016 (additive `PerceptionMode` strategy seam, default = Classical, bit-identical regression), and ADR-018 (post-Stage-2 safety revision: AI is candidate perception only — Safe Hybrid policy frozen, `agreement_radius_px`=8.0px, AI-only/disagreement both reject unconditionally; documentation only, Stage 3 not started)
- [x] Synthetic dataset generator — `fsoc_ai_datagen` (`fsoc/ai_frame_synth.hpp` + `src/ai_frame_synth.cpp`): pure seeded domain randomization (beacon peak/σ/anisotropy/edge-clip, background gradient + vignette, Gaussian read + shot-like noise, hot/dead/salt pixels, Gaussian blur / defocus / motion blur, star clutter + bright distractor + cluster, negatives); frozen 8-stage order; `synthesize(seed)` byte-reproducible
- [x] Dataset CLI — `apps/generate_ai_dataset.cpp` → `generate_ai_dataset` (contiguous disjoint train/val/test blocks over one global index space, per-sample `sample_seed_for`, evenly-spaced negatives, JSONL label manifests + `dataset.json` reproducibility manifest); output git-ignored under `generated/ai_dataset/`
- [x] AI datagen tests — `fsoc_ai_datagen_tests` (config validation, seed determinism + 20k collision-free, byte-identical frames per seed, force-target override, positive-has-signal / negative-is-empty, negative-fraction respected, edge-clip both ways, difficulty bounds)
- [x] Python training toolchain scaffold — `tools/ai/` (`common.py` frozen preprocessing/heatmap/decode, `model.py` TinyBeaconNet **27,282 params**, `dataset.py`, `train_beacon_net.py`, `eval_beacon_net.py` val-threshold calibration, `export_onnx.py`, `make_parity_fixture.py`, `selfcheck.py`, `requirements.txt`, `README.md`)
- [x] AI docs — `docs/19_AI_PERCEPTION_ARCHITECTURE.md`, `docs/20_AI_DATASET_AND_TRAINING.md`
- [x] **Stage 2** — Train TinyBeaconNet + calibrate threshold + export ONNX + model card. ADR-017 (foreground-weighted heatmap loss + global-max presence pool — plain MSE stalls). Dataset regenerated + integrity-checked (8400/8400 unique hashes, labels visually verified ≈0.04 px). Best epoch 9; **frozen presence threshold 0.95** (val only). Test @ 0.95: presence precision 0.989 / recall 0.404 / FPR 0.013; committed-detection centroid **median 1.71 px, 97.5 % ≤ 10 px**. `models/tiny_beacon_net.onnx` (110 KiB, opset 12, `onnx.checker` OK), torch↔ORT parity max\|Δ\| ≤ 8e-6 / 2e-6 px. `models/MODEL_CARD.md`. Known limit: recall ≈ 40 % (presence-head ceiling, 10 variants tried) — recall deferred to Stage-3 hybrid + Phase-2 temporal. Step-10 still PASS.
- [ ] C++ AI detector — `fsoc_ai_perception` (`AiBeaconDetector`, OpenCV-DNN ONNX), model-file validation, Python↔C++ preprocessing + inference parity tests
- [ ] Hybrid detector + `PerceptionMode` / `PerceptionSource`; implement the **Safe Hybrid** policy already frozen in ADR-018 / `docs/19 §5` (classical takes control precedence on agreement; AI-only and classical/AI disagreement both reject to `std::nullopt` unconditionally — no confidence override; `agreement_radius_px` = 8.0 px)
- [ ] `SimulationRunner` perception seam (default Classical, bit-identical regression; Step-10 still PASS) + AI telemetry fields
- [ ] AI evaluation suite `fsoc_ai_validation` — scenario families A–K, CLASSICAL vs AI vs HYBRID table, C++ inference latency
- [ ] Frontend AI integration — perception toggle, AI confidence/source/latency panels, heatmap overlay, benchmark + architecture views
- [ ] `docs/21_AI_VALIDATION.md` + README/VALIDATION/FILES/TASK_BOARD/INTERFACE_CONTRACTS updates

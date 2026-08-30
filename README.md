# SIH26169 — FSOC Virtual Camera Tracking MVP (C++20)

Engineering starter kit for **AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile Free Space Optical Communication (FSOC) Terminals**.

This repository treats the challenge as a **closed-loop guidance, tracking, and control problem**:

`Environment -> Camera -> Beacon measurement -> Estimation -> Prediction -> Control -> Pan/Tilt actuation -> Observation`

## Language decision

The project baseline is now **modern C++20**. There is no Python package, virtual environment, pip install, `pyproject.toml`, NumPy, or PyVista dependency in the core project.

For the first 48-hour MVP:
- Core math/physics/control: C++20
- Build: CMake + Ninja
- Pixel simulation/tracking visualization: OpenCV C++ (introduced after the math gate)
- Step-1 vector math: dependency-free to keep the foundation auditable
- Later UKF/MPC phase: add Eigen when matrix-heavy estimation/control begins

## Step 1 already implemented

- 3D world convention
- pan/tilt camera basis
- pinhole projection
- finite camera FOV
- actuator velocity saturation
- tilt mechanical limits
- ideal pointing angles for diagnostics only
- terminal-only smoke test
- 12 unit checks using CTest, with no external test framework

## Step 2 implemented — target trajectory engine

- `TargetState` = world `position_m` + `velocity_mps` (SI, double precision)
- `Trajectory` abstract interface: pure `state_at(double time_s)`, no owned clock
- stationary, linear constant-velocity (signed), and sinusoidal trajectories
- sinusoidal velocity is the exact analytic derivative; frequency in Hz (`omega = 2*pi*f`)
- deliberate input validation via `std::invalid_argument` (non-finite / negative time,
  non-finite params, negative frequency/amplitude)
- `step2_trajectory_smoke` + deterministic analytic CTest suite (`fsoc_step2_tests`)
- no coupling to camera / perception / control

## Step 3 implemented — observation / measurement / tracking-error contracts

- strongly typed layers kept distinct: `TargetState` (truth) → `CameraObservation` /
  `Projection` (exact projection) → `BeaconDetection` (image estimate) → `TrackingError`
  (controller-facing)
- `ObservationStatus` = `Visible` / `OutsideFieldOfView` / `BehindCamera`;
  `observe_beacon()` reuses `PanTiltCamera::project()` — no duplicated projection math
- frozen image convention: origin top-left, `+x_px` right, `+y_px` down;
  centre `cx = W/2.0`, `cy = H/2.0` owned by the camera
- frozen sign convention: pixel `error_x>0` = RIGHT, `error_y>0` = BELOW;
  angular `pan_rad>0` = command pan right, `tilt_rad>0` = command tilt up
- `compute_tracking_error(std::optional<BeaconDetection>, PanTiltCamera)` — `optional`
  in / out, non-finite centroid rejected, reuses `pixel_error_to_angles`
- target-lost = empty `std::optional` only (no `(-1,-1)` / NaN / zero sentinels)
- `step3_observation_smoke` (machine-checks the critical (400,180) scenario) +
  `fsoc_step3_tests` (all four quadrants, pinhole match, regressions)
- no OpenCV, no detector algorithm, no controller

## Step 4 implemented — synthetic virtual-camera image renderer

- first OpenCV use, isolated in a separate `fsoc_render` library; `fsoc_core` and all
  pure-math headers stay OpenCV-free
- `SyntheticCameraRenderer::render(const CameraObservation&) -> cv::Mat` (`CV_8UC1`)
- uniform dark background (default 5 counts) + analytic 2-D Gaussian beacon
  (peak 255, `sigma` in **pixels**), clamped to `[0,255]`
- **true sub-pixel** beacon centre — the fractional `ImagePoint` is never rounded before
  the Gaussian is evaluated, so a weighted centroid recovers it
- edge-safe: the Gaussian is rasterised in a window clipped to the image
- `OutsideFieldOfView` / `BehindCamera` → background-only frame (no fake beacon)
- no sensor noise this step; the same observation renders byte-identical frames
- `step4_renderer_smoke` writes `generated/*.png` headlessly + `fsoc_step4_tests`
- CMake `find_package(OpenCV)` auto-detected (`FSOC_ENABLE_OPENCV=AUTO|ON|OFF`)

## macOS quick start

```bash
xcode-select --install      # only if Command Line Tools are missing
brew install cmake ninja
brew install opencv          # required from Step 4 onward

cmake --preset debug
cmake --build --preset debug
ctest --preset debug
./build/debug/step1_math_smoke
./build/debug/step2_trajectory_smoke
./build/debug/step3_observation_smoke
./build/debug/step4_renderer_smoke
```

Steps 1–3 build and pass without OpenCV; if `opencv` is missing, CMake prints a notice
and skips the Step 4 renderer target only. Install it with `brew install opencv` and
reconfigure — no Homebrew paths are hardcoded.

## Repository layout

```text
include/fsoc/       Public interfaces
src/                Core implementations
apps/               Executable simulation/demo programs
tests/              Mathematical/unit validation
cmake/              Build policies
.claude/skills/     Claude Code engineering skills
.claude/agents/     Specialist subagents
docs/               PRD/SRS/design/roadmap/test plans
prompts/             Reusable Vibe Coding prompts
```

Read `CLAUDE.md` before asking an AI coding agent to modify the project.

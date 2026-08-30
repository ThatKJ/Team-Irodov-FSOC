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

## macOS quick start

```bash
xcode-select --install      # only if Command Line Tools are missing
brew install cmake ninja

cmake --preset debug
cmake --build --preset debug
ctest --preset debug
./build/debug/step1_math_smoke
./build/debug/step2_trajectory_smoke
./build/debug/step3_observation_smoke
```

Do **not** install OpenCV until the roadmap reaches the synthetic image/tracking step. When needed:

```bash
brew install opencv
```

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

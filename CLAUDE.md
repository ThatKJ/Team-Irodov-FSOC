# CLAUDE.md — SIH26169 FSOC C++ Engineering Contract

You are contributing to an aerospace-style closed-loop virtual tracking system, not a generic computer-vision demo.

## Mission

Build a mathematically defensible C++20 simulation in which a moving optical beacon is observed through a narrow-FOV virtual camera and a pan/tilt controller keeps the beacon close to image center under actuator constraints.

## Hard language/build rules

1. Production code is **C++20**.
2. Build only with **CMake**; use Ninja on macOS when available.
3. Never add Python, pip, `.venv`, NumPy, PyVista, `pyproject.toml`, or Python glue unless the human explicitly reverses this decision.
4. OpenCV C++ is allowed only for image synthesis, detection, visualization, and image I/O. It must not own the simulation state or controller.
5. Keep the Step-1 math dependency-free. Add Eigen later only when UKF/MPC matrix algebra justifies it.

## Closed-loop architecture boundaries

Keep these responsibilities independently testable:

- `Environment`: world state only.
- `Trajectory`: target truth generation only.
- `PanTiltCamera`: geometry, projection, FOV, actuator kinematics only.
- `Observation/Renderer`: creates synthetic image measurements.
- `Detector`: extracts beacon centroid/confidence from an image.
- `Estimator`: later filter state; baseline may pass measurement directly.
- `Controller`: PID now; MPC later.
- `SimulationRunner`: owns timing and data flow; no domain math should be hidden here.
- `Telemetry`: records outputs without changing behavior.

Never create a god-class or a single loop containing physics, OpenCV detection, PID equations, rendering, and logging inline.

## Coordinate convention — frozen

World:
- +X = forward
- +Y = right
- +Z = up

Camera projection frame:
- +x = image right
- +y = image up
- +z = optical axis forward

Image:
- `u` grows right
- `v` grows down
- principal point `(cx, cy)`

Internal angular units are radians. Public configuration names must include unit suffixes where ambiguity exists (`_rad`, `_rad_s`, `_m`, `_px`, `_s`).

Do not silently change coordinate signs or units. If a change is unavoidable, update the math document and tests in the same commit.

## C++ coding standards

- Prefer RAII and value semantics.
- No owning raw pointers and no manual `new`/`delete`.
- Use `std::optional` for legitimate absence (e.g. target outside FOV).
- Use `[[nodiscard]]` on calculations whose result should not be ignored.
- Use `const` aggressively where it improves correctness.
- Avoid global mutable state.
- Avoid preprocessor macros in production logic.
- Prefer clear structs and narrow interfaces over inheritance-heavy frameworks.
- Do not optimize before measurement; correctness and traceability first.
- No unexplained magic numbers; put engineering limits in config.
- Use fixed simulation timestep for deterministic baseline experiments unless a requirement says otherwise.

## Build quality gates

Every implementation step must pass:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

When touching Step 1, also run:

```bash
./build/debug/step1_math_smoke
```

Before claiming completion:
- no compiler errors
- no new warnings from project code
- tests pass
- coordinate/unit assumptions remain documented
- control code is not coupled to rendering

## Math-before-pixels rule

Before creating an OpenCV window, prove transforms and actuator behavior numerically in the terminal. The target's ideal pan/tilt and the virtual camera angles must move continuously and converge under a simple diagnostic law.

Ground-truth helpers (`ideal_angles_to`) are allowed only in tests and diagnostics. They must never become a shortcut used by the real baseline controller.

## Baseline freeze rule

Once the centroid -> PID -> pan/tilt baseline meets the acceptance metrics, create and protect a `v1_baseline` branch/tag before starting UKF, MPC, Zernike turbulence, CNN detection, or other advanced work.

Do not replace the baseline while experimenting. Advanced algorithms must be swappable implementations behind stable interfaces.

## 48-hour scope guard

IN:
- mathematical environment
- deterministic target trajectories
- narrow-FOV pinhole camera
- simple synthetic beacon image
- threshold/centroid detector
- PID pan/tilt controller
- velocity limits (acceleration limits may follow if time permits)
- telemetry: FPS, pixel/angular error, pan/tilt, visibility, saturation
- repeatable scenario and demo

OUT until baseline passes:
- UKF
- MPC
- Zernike phase screens
- CNN target detector
- PyTorch/TensorFlow
- distributed services/web dashboards
- ROS unless explicitly requested

## Agent behavior

Before editing:
1. Read relevant docs and interfaces.
2. State which module owns the change.
3. Preserve boundaries.
4. Add/modify tests with math changes.
5. Build and test before declaring success.

If a request would violate these rules, flag it and implement the smallest architecture-safe alternative.

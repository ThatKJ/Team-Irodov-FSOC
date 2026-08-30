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

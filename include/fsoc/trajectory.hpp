#pragma once

#include "fsoc/geometry.hpp"
#include "fsoc/target_state.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// Target trajectory engine
// ---------------------------------------------------------------------------
//
// A Trajectory is a pure, deterministic map
//
//     simulation time [s]  ->  TargetState { position_m, velocity_mps }
//
// representing the physical motion of the remote FSOC terminal.
//
// Design rules enforced here:
//   * The mathematical trajectory does NOT own or advance simulation time.
//     The integration runner owns the clock and passes an absolute time_s.
//     Hence the API is state_at(t), never update(dt).
//   * A Trajectory knows nothing about the camera, pan/tilt, FOV, pixels,
//     detection, control, estimation, or telemetry. It only generates truth.
//   * All angular parameters are radians; all physical quantities are SI,
//     with units encoded in the parameter names.
//   * Double precision throughout.
//
// Contract for state_at(time_s):
//   Precondition: std::isfinite(time_s) && time_s >= 0.0
//   Simulation time is defined only for t >= 0; the trajectory never
//   extrapolates backwards. A violated precondition throws
//   std::invalid_argument. The function never returns NaN/Inf for valid input.
class Trajectory {
public:
    virtual ~Trajectory() = default;

    // Deterministic truth sample at absolute simulation time `time_s` (seconds).
    // Repeated calls with the same argument return identical values.
    [[nodiscard]] virtual TargetState state_at(double time_s) const = 0;

protected:
    Trajectory() = default;
    // Copy/move are protected + defaulted: concrete leaf trajectories stay usable
    // as value types, while slicing through a Trajectory& is prevented.
    Trajectory(const Trajectory&) = default;
    Trajectory(Trajectory&&) = default;
    Trajectory& operator=(const Trajectory&) = default;
    Trajectory& operator=(Trajectory&&) = default;

    // Shared query-time contract used by every concrete implementation.
    // Throws std::invalid_argument when time_s is non-finite or negative.
    static void require_valid_time_s(double time_s);
};

// ---------------------------------------------------------------------------
// Trajectory 1 — Stationary
// ---------------------------------------------------------------------------
//
//   position(t) = initial_position_m      for every valid t
//   velocity(t) = (0, 0, 0)
//
// Models a fixed remote terminal. Useful as the acquisition/convergence
// reference case for the tracker.
class StationaryTrajectory final : public Trajectory {
public:
    // Throws std::invalid_argument if any component of initial_position_m is non-finite.
    explicit StationaryTrajectory(Vec3 initial_position_m);

    [[nodiscard]] TargetState state_at(double time_s) const override;

    [[nodiscard]] const Vec3& initial_position_m() const noexcept { return initial_position_m_; }

private:
    Vec3 initial_position_m_;
};

// ---------------------------------------------------------------------------
// Trajectory 2 — Linear constant velocity
// ---------------------------------------------------------------------------
//
//   position(t) = initial_position_m + constant_velocity_mps * t
//   velocity(t) = constant_velocity_mps
//
// Positive and negative velocity components on any axis are supported.
class LinearTrajectory final : public Trajectory {
public:
    // Throws std::invalid_argument if any component of either argument is non-finite.
    LinearTrajectory(Vec3 initial_position_m, Vec3 constant_velocity_mps);

    [[nodiscard]] TargetState state_at(double time_s) const override;

    [[nodiscard]] const Vec3& initial_position_m() const noexcept { return initial_position_m_; }
    [[nodiscard]] const Vec3& constant_velocity_mps() const noexcept {
        return constant_velocity_mps_;
    }

private:
    Vec3 initial_position_m_;
    Vec3 constant_velocity_mps_;
};

// ---------------------------------------------------------------------------
// Trajectory 3 — Sinusoidal (per-axis, independent)
// ---------------------------------------------------------------------------
//
// For each world axis a in {x, y, z}:
//
//   omega_a = 2 * pi * frequency_hz_a            [rad/s]
//   pos_a(t) = center_a + amplitude_a * sin(omega_a * t + phase_a)
//   vel_a(t) = amplitude_a * omega_a * cos(omega_a * t + phase_a)
//
// The velocity is the exact analytical derivative of the position, not a
// finite difference.
//
// Parameter policy (deliberate, consistent input validation):
//   * every component of every Vec3 parameter must be finite,
//   * frequency_hz components must be >= 0 (a negative oscillation rate is
//     ambiguous; use phase_rad for sign control),
//   * amplitude_m components must be >= 0 (magnitude-like; use phase_rad for
//     sign control),
//   * zero amplitude on an axis pins that axis to center (velocity 0),
//   * zero frequency on an axis yields a constant offset
//     center_a + amplitude_a * sin(phase_a) with velocity 0.
// Any violation throws std::invalid_argument from the constructor.
class SinusoidalTrajectory final : public Trajectory {
public:
    struct Parameters {
        Vec3 center_position_m{};  // oscillation midpoint, metres
        Vec3 amplitude_m{};        // peak displacement per axis, metres (>= 0)
        Vec3 frequency_hz{};       // oscillation frequency per axis, hertz (>= 0)
        Vec3 phase_rad{};          // phase offset per axis, radians
    };

    // Throws std::invalid_argument if the parameter policy above is violated.
    explicit SinusoidalTrajectory(Parameters params);

    [[nodiscard]] TargetState state_at(double time_s) const override;

    [[nodiscard]] const Parameters& parameters() const noexcept { return params_; }

private:
    Parameters params_;
    Vec3 omega_rad_s_{};  // cached 2*pi*frequency_hz, computed once at construction
};

}  // namespace fsoc

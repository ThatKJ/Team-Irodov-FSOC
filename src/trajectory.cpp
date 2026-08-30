#include "fsoc/trajectory.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace fsoc {

namespace {

[[nodiscard]] bool all_finite(const Vec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

[[nodiscard]] bool all_non_negative(const Vec3& v) noexcept {
    return v.x >= 0.0 && v.y >= 0.0 && v.z >= 0.0;
}

// 2*pi*f per axis: converts frequency in hertz to angular frequency in rad/s.
[[nodiscard]] Vec3 angular_frequency_rad_s(const Vec3& frequency_hz) noexcept {
    constexpr double two_pi = 2.0 * std::numbers::pi_v<double>;
    return {two_pi * frequency_hz.x, two_pi * frequency_hz.y, two_pi * frequency_hz.z};
}

}  // namespace

void Trajectory::require_valid_time_s(const double time_s) {
    if (!std::isfinite(time_s)) {
        throw std::invalid_argument("Trajectory::state_at: time_s must be finite.");
    }
    if (time_s < 0.0) {
        throw std::invalid_argument(
            "Trajectory::state_at: time_s must be non-negative (simulation time starts at 0).");
    }
}

// --- StationaryTrajectory --------------------------------------------------

StationaryTrajectory::StationaryTrajectory(const Vec3 initial_position_m)
    : initial_position_m_(initial_position_m) {
    if (!all_finite(initial_position_m_)) {
        throw std::invalid_argument(
            "StationaryTrajectory: initial_position_m must be finite on every axis.");
    }
}

TargetState StationaryTrajectory::state_at(const double time_s) const {
    require_valid_time_s(time_s);
    // Position is time-invariant; velocity is exactly zero.
    return TargetState{.position_m = initial_position_m_, .velocity_mps = Vec3{}};
}

// --- LinearTrajectory ----------------------------------------------------

LinearTrajectory::LinearTrajectory(
    const Vec3 initial_position_m,
    const Vec3 constant_velocity_mps)
    : initial_position_m_(initial_position_m),
      constant_velocity_mps_(constant_velocity_mps) {
    if (!all_finite(initial_position_m_)) {
        throw std::invalid_argument(
            "LinearTrajectory: initial_position_m must be finite on every axis.");
    }
    if (!all_finite(constant_velocity_mps_)) {
        throw std::invalid_argument(
            "LinearTrajectory: constant_velocity_mps must be finite on every axis.");
    }
}

TargetState LinearTrajectory::state_at(const double time_s) const {
    require_valid_time_s(time_s);
    // position(t) = p0 + v * t ; velocity(t) = v (constant).
    return TargetState{
        .position_m = initial_position_m_ + constant_velocity_mps_ * time_s,
        .velocity_mps = constant_velocity_mps_,
    };
}

// --- SinusoidalTrajectory ----------------------------------------------

SinusoidalTrajectory::SinusoidalTrajectory(Parameters params)
    : params_(params), omega_rad_s_(angular_frequency_rad_s(params.frequency_hz)) {
    if (!all_finite(params_.center_position_m)) {
        throw std::invalid_argument(
            "SinusoidalTrajectory: center_position_m must be finite on every axis.");
    }
    if (!all_finite(params_.amplitude_m)) {
        throw std::invalid_argument(
            "SinusoidalTrajectory: amplitude_m must be finite on every axis.");
    }
    if (!all_finite(params_.frequency_hz)) {
        throw std::invalid_argument(
            "SinusoidalTrajectory: frequency_hz must be finite on every axis.");
    }
    if (!all_finite(params_.phase_rad)) {
        throw std::invalid_argument(
            "SinusoidalTrajectory: phase_rad must be finite on every axis.");
    }
    if (!all_non_negative(params_.amplitude_m)) {
        throw std::invalid_argument(
            "SinusoidalTrajectory: amplitude_m must be >= 0 on every axis "
            "(use phase_rad for sign control).");
    }
    if (!all_non_negative(params_.frequency_hz)) {
        throw std::invalid_argument(
            "SinusoidalTrajectory: frequency_hz must be >= 0 on every axis "
            "(use phase_rad for sign control).");
    }
}

TargetState SinusoidalTrajectory::state_at(const double time_s) const {
    require_valid_time_s(time_s);

    // Per-axis phase argument theta_a = omega_a * t + phase_a.
    const double theta_x = omega_rad_s_.x * time_s + params_.phase_rad.x;
    const double theta_y = omega_rad_s_.y * time_s + params_.phase_rad.y;
    const double theta_z = omega_rad_s_.z * time_s + params_.phase_rad.z;

    const Vec3 position_m{
        params_.center_position_m.x + params_.amplitude_m.x * std::sin(theta_x),
        params_.center_position_m.y + params_.amplitude_m.y * std::sin(theta_y),
        params_.center_position_m.z + params_.amplitude_m.z * std::sin(theta_z),
    };

    // Analytical derivative: d/dt [A sin(omega t + phi)] = A omega cos(omega t + phi).
    const Vec3 velocity_mps{
        params_.amplitude_m.x * omega_rad_s_.x * std::cos(theta_x),
        params_.amplitude_m.y * omega_rad_s_.y * std::cos(theta_y),
        params_.amplitude_m.z * omega_rad_s_.z * std::cos(theta_z),
    };

    return TargetState{.position_m = position_m, .velocity_mps = velocity_mps};
}

}  // namespace fsoc

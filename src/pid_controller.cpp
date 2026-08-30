#include "fsoc/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsoc {

namespace {

[[nodiscard]] bool finite_and_non_negative(const double value) noexcept {
    return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] bool all_finite(const TrackingError& error) noexcept {
    return std::isfinite(error.pixel.x_px) && std::isfinite(error.pixel.y_px) &&
           std::isfinite(error.angular.pan_rad) && std::isfinite(error.angular.tilt_rad);
}

}  // namespace

void PIDAxisConfig::validate() const {
    if (!finite_and_non_negative(kp)) {
        throw std::invalid_argument("PIDAxisConfig: kp must be finite and >= 0.");
    }
    if (!finite_and_non_negative(ki)) {
        throw std::invalid_argument("PIDAxisConfig: ki must be finite and >= 0.");
    }
    if (!finite_and_non_negative(kd)) {
        throw std::invalid_argument("PIDAxisConfig: kd must be finite and >= 0.");
    }
    if (!finite_and_non_negative(integral_limit)) {
        throw std::invalid_argument("PIDAxisConfig: integral_limit must be finite and >= 0.");
    }
    if (!std::isfinite(output_limit_rad_s) || output_limit_rad_s <= 0.0) {
        throw std::invalid_argument("PIDAxisConfig: output_limit_rad_s must be finite and > 0.");
    }
}

void PIDControllerConfig::validate() const {
    pan.validate();
    tilt.validate();
}

PIDController::PIDController(PIDControllerConfig config) : config_(config) {
    config_.validate();
}

void PIDController::Axis::reset() noexcept {
    integral = 0.0;
    previous_error = 0.0;
    have_previous = false;
}

double PIDController::Axis::step(
    const PIDAxisConfig& cfg,
    const double error,
    const double dt_s) noexcept {
    // Derivative: zero on the first sample after construction/reset so an
    // undefined previous error cannot produce a derivative kick.
    const double derivative = have_previous ? (error - previous_error) / dt_s : 0.0;

    // Candidate integral with this sample, hard-bounded to +/- integral_limit.
    const double candidate_integral = std::clamp(
        integral + error * dt_s, -cfg.integral_limit, cfg.integral_limit);

    const double unsaturated =
        cfg.kp * error + cfg.ki * candidate_integral + cfg.kd * derivative;
    const double command =
        std::clamp(unsaturated, -cfg.output_limit_rad_s, cfg.output_limit_rad_s);

    // Conditional integration: do not accumulate a sample that would only drive
    // further into an already-saturated command. Any other case commits the
    // clamped accumulation, so the integral unwinds the moment the error reverses.
    const bool pushing_into_high_saturation =
        unsaturated > cfg.output_limit_rad_s && error > 0.0;
    const bool pushing_into_low_saturation =
        unsaturated < -cfg.output_limit_rad_s && error < 0.0;
    if (!pushing_into_high_saturation && !pushing_into_low_saturation) {
        integral = candidate_integral;
    }

    previous_error = error;
    have_previous = true;
    return command;
}

ControlCommand PIDController::update(const TrackingError& error, const double dt_s) {
    // Validate everything before mutating any state.
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("PIDController::update: dt_s must be finite and > 0.");
    }
    if (!all_finite(error)) {
        throw std::invalid_argument("PIDController::update: TrackingError has a non-finite value.");
    }

    return ControlCommand{
        .pan_rate_rad_s = pan_.step(config_.pan, error.angular.pan_rad, dt_s),
        .tilt_rate_rad_s = tilt_.step(config_.tilt, error.angular.tilt_rad, dt_s),
    };
}

void PIDController::reset() noexcept {
    pan_.reset();
    tilt_.reset();
}

}  // namespace fsoc

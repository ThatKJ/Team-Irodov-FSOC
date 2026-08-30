#pragma once

#include "fsoc/tracking_error.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// ControlCommand — desired actuator angular velocity  (Step 6 output)
// ---------------------------------------------------------------------------
//
// Angular RATES only, radians per second. Never absolute pan/tilt angles.
// The PID class never touches PanTiltCamera; the Step-7 runner applies these
// via camera.step(pan_rate, tilt_rate, dt).
struct ControlCommand {
    double pan_rate_rad_s{};
    double tilt_rate_rad_s{};
};

// Explicit "hold still" command. The Step-7 runner emits this on target loss
// (and calls PIDController::reset()); the PID class itself has no notion of a
// lost target.
[[nodiscard]] constexpr ControlCommand zero_control_command() noexcept {
    return ControlCommand{.pan_rate_rad_s = 0.0, .tilt_rate_rad_s = 0.0};
}

// ---------------------------------------------------------------------------
// PID configuration
// ---------------------------------------------------------------------------
//
// Defaults are INITIAL BASELINE PLACEHOLDERS — not tuned. They are to be tuned
// during the Step-7 closed-loop experiments. output_limit_rad_s matches the
// default CameraConfig actuator rate (deg_to_rad(30)); the Step-7 runner is
// responsible for keeping controller and actuator limits consistent — the PID
// never reads CameraConfig.
struct PIDAxisConfig {
    double kp{1.5};                              // >= 0
    double ki{0.2};                              // >= 0
    double kd{0.05};                             // >= 0
    double integral_limit{0.5};                  // >= 0 ; hard bound on the integral STATE
    double output_limit_rad_s{deg_to_rad(30.0)};  // > 0  ; bound on the axis command

    // All finite; kp,ki,kd,integral_limit >= 0; output_limit_rad_s > 0.
    // Throws std::invalid_argument otherwise.
    void validate() const;
};

struct PIDControllerConfig {
    PIDAxisConfig pan{};
    PIDAxisConfig tilt{};

    void validate() const;  // validates both axes
};

// ---------------------------------------------------------------------------
// PIDController
// ---------------------------------------------------------------------------
//
// Two independent discrete PID loops, one per ANGULAR tracking-error axis.
// Consumes ONLY TrackingError.angular (radians) + dt_s. No pixels, no world
// truth, no camera, no OpenCV. Stateful — see reset().
//
//   e            = tracking error for the axis (rad):
//                    pan  <- TrackingError.angular.pan_rad
//                    tilt <- TrackingError.angular.tilt_rad
//   derivative D = (e - e_prev) / dt_s        (0 on the first update after
//                                              construction / reset)
//   integral   I : I += e * dt_s , then:
//                    - hard-clamped to [-integral_limit, +integral_limit] every update;
//                    - conditional integration: if the unsaturated command is beyond
//                      +/- output_limit_rad_s AND e has the same sign as that saturation,
//                      this sample is NOT accumulated (I is held). Otherwise the clamped
//                      accumulation is committed, so I unwinds as soon as e reverses.
//   u            = kp*e + ki*I + kd*D
//   command      = clamp(u, -output_limit_rad_s, +output_limit_rad_s)
//
// Sign convention (frozen, from TrackingError): e_pan > 0 => beacon RIGHT =>
// (kp > 0) pan_rate_rad_s > 0 => pan right. e_tilt > 0 => beacon ABOVE =>
// tilt_rate_rad_s > 0 => tilt up. Neither axis is inverted.
class PIDController {
public:
    explicit PIDController(PIDControllerConfig config);

    [[nodiscard]] const PIDControllerConfig& config() const noexcept { return config_; }

    // Precondition: dt_s finite and > 0; every component of `error` finite.
    // Violations throw std::invalid_argument and leave the controller state
    // unchanged. No heap allocation.
    [[nodiscard]] ControlCommand update(const TrackingError& error, double dt_s);

    // Return to the first-update state: integrals 0, previous errors 0, the
    // derivative term suppressed on the next update.
    void reset() noexcept;

private:
    struct Axis {
        double integral{0.0};
        double previous_error{0.0};
        bool have_previous{false};

        [[nodiscard]] double step(const PIDAxisConfig& cfg, double error, double dt_s) noexcept;
        void reset() noexcept;
    };

    PIDControllerConfig config_;
    Axis pan_{};
    Axis tilt_{};
};

}  // namespace fsoc

// Step 6 smoke test: pan/tilt PID controller.
//
// Terminal-only. No OpenCV, no camera motion, no closed loop. Converts angular
// TrackingError into ControlCommand rates and demonstrates sign, integral
// growth, saturation, reset, and a toy scalar-plant sanity check.

#include <cmath>
#include <iomanip>
#include <iostream>

#include "fsoc/pid_controller.hpp"
#include "fsoc/tracking_error.hpp"

namespace {

[[nodiscard]] fsoc::TrackingError angular_error(const double pan_rad, const double tilt_rad) {
    fsoc::TrackingError error{};
    error.angular.pan_rad = pan_rad;
    error.angular.tilt_rad = tilt_rad;
    return error;
}

void print_command(const char* label, const fsoc::TrackingError& error,
                   const fsoc::ControlCommand& command) {
    std::cout << "  " << label << '\n'
              << std::fixed << std::setprecision(5)
              << "    input  : pan_err=" << std::showpos << error.angular.pan_rad
              << " rad, tilt_err=" << error.angular.tilt_rad << " rad\n"
              << "    output : pan_rate=" << command.pan_rate_rad_s
              << " rad/s, tilt_rate=" << command.tilt_rate_rad_s << " rad/s\n"
              << std::noshowpos;
}

}  // namespace

int main() {
    using namespace fsoc;

    // Easy gains for readable arithmetic (NOT tuned).
    PIDControllerConfig config{};
    const PIDAxisConfig axis{.kp = 2.0,
                             .ki = 0.5,
                             .kd = 0.1,
                             .integral_limit = 0.5,
                             .output_limit_rad_s = deg_to_rad(30.0)};
    config.pan = axis;
    config.tilt = axis;

    PIDController pid{config};
    constexpr double dt_s = 0.1;

    std::cout << "Step 6: pan/tilt PID controller\n"
              << std::fixed << std::setprecision(4)
              << "gains: kp=" << axis.kp << " ki=" << axis.ki << " kd=" << axis.kd
              << "  integral_limit=" << axis.integral_limit
              << "  output_limit=" << axis.output_limit_rad_s << " rad/s  dt=" << dt_s << " s\n";

    bool ok = true;

    // Scenario 1 — target RIGHT + ABOVE  => pan right, tilt up.
    std::cout << "\n=== Scenario 1: positive right/above error ===\n";
    {
        const auto error = angular_error(0.05, 0.02);
        const auto command = pid.update(error, dt_s);
        print_command("first update", error, command);
        const bool pass = command.pan_rate_rad_s > 0.0 && command.tilt_rate_rad_s > 0.0;
        std::cout << "    expect pan>0 and tilt>0 -> " << (pass ? "PASS" : "FAIL") << '\n';
        ok = ok && pass;
    }

    // Scenario 2 — target LEFT + BELOW => pan left, tilt down.
    std::cout << "\n=== Scenario 2: negative left/below error ===\n";
    {
        pid.reset();
        const auto error = angular_error(-0.05, -0.02);
        const auto command = pid.update(error, dt_s);
        print_command("first update", error, command);
        const bool pass = command.pan_rate_rad_s < 0.0 && command.tilt_rate_rad_s < 0.0;
        std::cout << "    expect pan<0 and tilt<0 -> " << (pass ? "PASS" : "FAIL") << '\n';
        ok = ok && pass;
    }

    // Scenario 3 — constant error, watch the integral contribution develop.
    std::cout << "\n=== Scenario 3: constant error over several timesteps ===\n";
    {
        pid.reset();
        const auto error = angular_error(0.03, 0.0);
        std::cout << std::setprecision(6)
                  << "    t[s]   pan_err   pan_rate   (P term = kp*e = "
                  << axis.kp * error.angular.pan_rad << ")\n";
        double last_pan_rate = 0.0;
        bool grows = true;
        for (int i = 1; i <= 6; ++i) {
            const auto command = pid.update(error, dt_s);
            std::cout << "    " << std::setw(5) << std::setprecision(2)
                      << static_cast<double>(i) * dt_s << std::setprecision(6) << "  "
                      << error.angular.pan_rad << "  " << command.pan_rate_rad_s << '\n';
            if (i > 1 && !(command.pan_rate_rad_s > last_pan_rate)) {
                grows = false;  // integral should make it grow each frame
            }
            last_pan_rate = command.pan_rate_rad_s;
        }
        std::cout << "    pan_rate strictly increases (integral accumulating) -> "
                  << (grows ? "PASS" : "FAIL") << '\n';
        ok = ok && grows;
    }

    // Scenario 4 — very large error, command saturates at output_limit.
    std::cout << "\n=== Scenario 4: very large error -> saturation ===\n";
    {
        pid.reset();
        const auto error = angular_error(2.0, -2.0);
        const auto command = pid.update(error, dt_s);
        print_command("first update", error, command);
        const bool pass = std::abs(command.pan_rate_rad_s - axis.output_limit_rad_s) < 1e-12 &&
                          std::abs(command.tilt_rate_rad_s + axis.output_limit_rad_s) < 1e-12;
        std::cout << "    expect pan_rate=+limit, tilt_rate=-limit -> " << (pass ? "PASS" : "FAIL")
                  << '\n';
        ok = ok && pass;
    }

    // Scenario 5 — reset, then the next update behaves like a first update.
    std::cout << "\n=== Scenario 5: reset() clears controller state ===\n";
    {
        // Wind the controller up first.
        for (int i = 0; i < 30; ++i) {
            (void)pid.update(angular_error(1.5, 1.5), dt_s);
        }
        pid.reset();
        // Kd-only comparison: first update after reset must have zero derivative term.
        PIDControllerConfig kd_only{};
        const PIDAxisConfig kd_axis{
            .kp = 0.0, .ki = 0.0, .kd = 1.0, .integral_limit = 1.0,
            .output_limit_rad_s = deg_to_rad(30.0)};
        kd_only.pan = kd_axis;
        kd_only.tilt = kd_axis;
        PIDController after_reset{kd_only};
        for (int i = 0; i < 10; ++i) {
            (void)after_reset.update(angular_error(0.4, 0.4), dt_s);
        }
        after_reset.reset();
        const auto command = after_reset.update(angular_error(0.4, 0.4), dt_s);
        print_command("first update after reset (kd-only)", angular_error(0.4, 0.4), command);
        const bool pass = command.pan_rate_rad_s == 0.0 && command.tilt_rate_rad_s == 0.0;
        std::cout << "    derivative suppressed on first post-reset update -> "
                  << (pass ? "PASS" : "FAIL") << '\n';
        ok = ok && pass;
    }

    // Offline sanity: toy scalar plant  error_next = error - command*dt  (NOT the FSOC loop).
    std::cout << "\n=== Offline sanity: toy scalar plant reduces a positive error ===\n";
    {
        PIDController plant_pid{config};
        double pan_err = 0.1;
        const double initial = pan_err;
        for (int i = 0; i < 40; ++i) {
            const auto command = plant_pid.update(angular_error(pan_err, 0.0), dt_s);
            pan_err -= command.pan_rate_rad_s * dt_s;
        }
        std::cout << std::setprecision(6) << "    initial |pan_err| = " << initial
                  << "  ->  final |pan_err| = " << std::abs(pan_err) << '\n';
        const bool pass = std::abs(pan_err) < initial;
        std::cout << "    control law reduces the error magnitude -> " << (pass ? "PASS" : "FAIL")
                  << '\n';
        ok = ok && pass;
    }

    std::cout << "\nStep 6 smoke: " << (ok ? "PASS" : "FAIL") << '\n';
    return ok ? 0 : 1;
}

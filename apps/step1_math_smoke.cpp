#include <cmath>
#include <iomanip>
#include <iostream>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/environment.hpp"

int main() {
    using namespace fsoc;

    CameraConfig config{};
    PanTiltCamera camera{config};
    Environment environment{camera, TargetState{{100.0, 8.0, 4.0}}};

    constexpr double dt_s = 0.02;
    constexpr int steps = 200;
    constexpr double tracking_gain = 2.0;  // diagnostic P-like pointing law, not the baseline PID.

    std::cout << "Step 1: terminal-only geometry/camera smoke test\n";
    std::cout << "World frame: +X forward, +Y right, +Z up\n\n";
    std::cout << std::fixed << std::setprecision(3);

    for (int i = 0; i < steps; ++i) {
        const auto [desired_pan, desired_tilt] =
            environment.camera().ideal_angles_to(environment.target().position_m);

        const double pan_error = wrap_pi(desired_pan - environment.camera().pan_rad());
        const double tilt_error = desired_tilt - environment.camera().tilt_rad();

        [[maybe_unused]] const AppliedRates applied = environment.camera().step(
            tracking_gain * pan_error,
            tracking_gain * tilt_error,
            dt_s);

        if (i % 20 == 0 || i == steps - 1) {
            const auto projection = environment.observe_ideal();
            std::cout << "t=" << i * dt_s << " s  "
                      << "pan=" << rad_to_deg(environment.camera().pan_rad()) << " deg  "
                      << "tilt=" << rad_to_deg(environment.camera().tilt_rad()) << " deg  ";
            if (projection) {
                std::cout << "pixel=(" << projection->u_px << ", " << projection->v_px << ")";
            } else {
                std::cout << "pixel=OUT_OF_FOV";
            }
            std::cout << '\n';
        }
    }

    const auto [ideal_pan, ideal_tilt] =
        environment.camera().ideal_angles_to(environment.target().position_m);
    const double final_pan_error_deg = rad_to_deg(
        std::abs(wrap_pi(ideal_pan - environment.camera().pan_rad())));
    const double final_tilt_error_deg = rad_to_deg(
        std::abs(ideal_tilt - environment.camera().tilt_rad()));

    std::cout << "\nFinal pointing errors: pan=" << final_pan_error_deg
              << " deg, tilt=" << final_tilt_error_deg << " deg\n";
    return (final_pan_error_deg < 0.1 && final_tilt_error_deg < 0.1) ? 0 : 1;
}

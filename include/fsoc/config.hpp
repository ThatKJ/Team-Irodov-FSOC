#pragma once

#include <numbers>
#include <stdexcept>

namespace fsoc {

[[nodiscard]] constexpr double deg_to_rad(const double degrees) noexcept {
    return degrees * std::numbers::pi_v<double> / 180.0;
}

[[nodiscard]] constexpr double rad_to_deg(const double radians) noexcept {
    return radians * 180.0 / std::numbers::pi_v<double>;
}

struct CameraConfig {
    int width_px{640};
    int height_px{480};
    double hfov_rad{deg_to_rad(20.0)};
    double vfov_rad{deg_to_rad(15.0)};
    double max_pan_rate_rad_s{deg_to_rad(30.0)};
    double max_tilt_rate_rad_s{deg_to_rad(30.0)};
    double min_tilt_rad{deg_to_rad(-80.0)};
    double max_tilt_rad{deg_to_rad(80.0)};

    void validate() const {
        constexpr double pi = std::numbers::pi_v<double>;
        if (width_px <= 0 || height_px <= 0) {
            throw std::invalid_argument("Image dimensions must be positive.");
        }
        if (!(hfov_rad > 0.0 && hfov_rad < pi)) {
            throw std::invalid_argument("Horizontal FOV must be in (0, pi) radians.");
        }
        if (!(vfov_rad > 0.0 && vfov_rad < pi)) {
            throw std::invalid_argument("Vertical FOV must be in (0, pi) radians.");
        }
        if (max_pan_rate_rad_s <= 0.0 || max_tilt_rate_rad_s <= 0.0) {
            throw std::invalid_argument("Angular rate limits must be positive.");
        }
        if (!(min_tilt_rad >= -pi / 2.0 && min_tilt_rad < max_tilt_rad &&
              max_tilt_rad <= pi / 2.0)) {
            throw std::invalid_argument("Tilt limits must be ordered within [-pi/2, pi/2].");
        }
    }
};

}  // namespace fsoc

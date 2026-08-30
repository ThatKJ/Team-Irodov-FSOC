#include "fsoc/geometry.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace fsoc {

double norm(const Vec3& v) noexcept {
    return std::sqrt(dot(v, v));
}

double wrap_pi(const double angle_rad) noexcept {
    constexpr double pi = std::numbers::pi_v<double>;
    constexpr double two_pi = 2.0 * pi;
    double wrapped = std::fmod(angle_rad + pi, two_pi);
    if (wrapped < 0.0) {
        wrapped += two_pi;
    }
    return wrapped - pi;
}

CameraBasis camera_basis(const double pan_rad, const double tilt_rad) noexcept {
    const double cp = std::cos(pan_rad);
    const double sp = std::sin(pan_rad);
    const double ct = std::cos(tilt_rad);
    const double st = std::sin(tilt_rad);

    const Vec3 forward{ct * cp, ct * sp, st};
    const Vec3 right{-sp, cp, 0.0};
    // This cross-product order gives +Z as image-up at zero pan/tilt.
    const Vec3 up = cross(forward, right);
    return {.right = right, .up = up, .forward = forward};
}

std::pair<double, double> ideal_pan_tilt(
    const Vec3& camera_position_m,
    const Vec3& target_position_m) {
    const Vec3 delta = target_position_m - camera_position_m;
    const double horizontal_range = std::hypot(delta.x, delta.y);
    if (horizontal_range == 0.0 && delta.z == 0.0) {
        throw std::invalid_argument(
            "Camera and target are coincident; pointing direction is undefined.");
    }
    return {
        std::atan2(delta.y, delta.x),
        std::atan2(delta.z, horizontal_range),
    };
}

}  // namespace fsoc

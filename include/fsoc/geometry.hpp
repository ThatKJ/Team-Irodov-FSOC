#pragma once

#include <utility>

namespace fsoc {

// World frame convention: +X forward, +Y right, +Z up.
// Camera projection frame: x=image-right, y=image-up, z=optical-axis forward.
struct Vec3 {
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& rhs) const noexcept {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& rhs) const noexcept {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }
    [[nodiscard]] constexpr Vec3 operator*(const double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }
};

[[nodiscard]] constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

[[nodiscard]] double norm(const Vec3& v) noexcept;
[[nodiscard]] double wrap_pi(double angle_rad) noexcept;

struct CameraBasis {
    Vec3 right;
    Vec3 up;
    Vec3 forward;
};

[[nodiscard]] CameraBasis camera_basis(double pan_rad, double tilt_rad) noexcept;
[[nodiscard]] std::pair<double, double> ideal_pan_tilt(
    const Vec3& camera_position_m,
    const Vec3& target_position_m);

}  // namespace fsoc

#pragma once

#include <optional>
#include <utility>

#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"

namespace fsoc {

struct Projection {
    double u_px{};
    double v_px{};
    double x_cam_m{};
    double y_cam_m{};
    double z_cam_m{};
};

struct AppliedRates {
    double pan_rate_rad_s{};
    double tilt_rate_rad_s{};
};

class PanTiltCamera {
public:
    explicit PanTiltCamera(
        CameraConfig config,
        Vec3 position_m = {},
        double pan_rad = 0.0,
        double tilt_rad = 0.0);

    [[nodiscard]] const CameraConfig& config() const noexcept { return config_; }
    [[nodiscard]] Vec3 position_m() const noexcept { return position_m_; }
    [[nodiscard]] double pan_rad() const noexcept { return pan_rad_; }
    [[nodiscard]] double tilt_rad() const noexcept { return tilt_rad_; }

    [[nodiscard]] double cx_px() const noexcept;
    [[nodiscard]] double cy_px() const noexcept;
    [[nodiscard]] double fx_px() const noexcept;
    [[nodiscard]] double fy_px() const noexcept;

    void set_position(Vec3 position_m) noexcept { position_m_ = position_m; }

    // Kinematics only: applies rate saturation and angle limits. No tracking logic belongs here.
    [[nodiscard]] AppliedRates step(
        double pan_rate_cmd_rad_s,
        double tilt_rate_cmd_rad_s,
        double dt_s);

    [[nodiscard]] Vec3 world_to_camera(const Vec3& point_world_m) const noexcept;
    [[nodiscard]] std::optional<Projection> project(const Vec3& point_world_m) const noexcept;

    // Ground-truth helper for tests/diagnostics only. Never feed it to the closed-loop tracker.
    [[nodiscard]] std::pair<double, double> ideal_angles_to(const Vec3& point_world_m) const;

    // du is image-right. dv is image-down, hence the sign inversion for elevation.
    [[nodiscard]] std::pair<double, double> pixel_error_to_angles(
        double du_px,
        double dv_px) const noexcept;

private:
    CameraConfig config_;
    Vec3 position_m_{};
    double pan_rad_{};
    double tilt_rad_{};
};

}  // namespace fsoc

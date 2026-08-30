#include "fsoc/camera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsoc {

PanTiltCamera::PanTiltCamera(
    CameraConfig config,
    const Vec3 position_m,
    const double pan_rad,
    const double tilt_rad)
    : config_(config),
      position_m_(position_m),
      pan_rad_(wrap_pi(pan_rad)),
      tilt_rad_(std::clamp(tilt_rad, config.min_tilt_rad, config.max_tilt_rad)) {
    config_.validate();
}

double PanTiltCamera::cx_px() const noexcept {
    return static_cast<double>(config_.width_px) / 2.0;
}

double PanTiltCamera::cy_px() const noexcept {
    return static_cast<double>(config_.height_px) / 2.0;
}

double PanTiltCamera::fx_px() const noexcept {
    return cx_px() / std::tan(config_.hfov_rad / 2.0);
}

double PanTiltCamera::fy_px() const noexcept {
    return cy_px() / std::tan(config_.vfov_rad / 2.0);
}

AppliedRates PanTiltCamera::step(
    const double pan_rate_cmd_rad_s,
    const double tilt_rate_cmd_rad_s,
    const double dt_s) {
    if (dt_s <= 0.0) {
        throw std::invalid_argument("dt_s must be positive.");
    }

    const double pan_rate = std::clamp(
        pan_rate_cmd_rad_s,
        -config_.max_pan_rate_rad_s,
        config_.max_pan_rate_rad_s);
    const double tilt_rate = std::clamp(
        tilt_rate_cmd_rad_s,
        -config_.max_tilt_rate_rad_s,
        config_.max_tilt_rate_rad_s);

    pan_rad_ = wrap_pi(pan_rad_ + pan_rate * dt_s);
    tilt_rad_ = std::clamp(
        tilt_rad_ + tilt_rate * dt_s,
        config_.min_tilt_rad,
        config_.max_tilt_rad);

    return {.pan_rate_rad_s = pan_rate, .tilt_rate_rad_s = tilt_rate};
}

Vec3 PanTiltCamera::world_to_camera(const Vec3& point_world_m) const noexcept {
    const Vec3 delta = point_world_m - position_m_;
    const CameraBasis basis = camera_basis(pan_rad_, tilt_rad_);
    return {
        dot(delta, basis.right),
        dot(delta, basis.up),
        dot(delta, basis.forward),
    };
}

std::optional<Projection> PanTiltCamera::project(const Vec3& point_world_m) const noexcept {
    const Vec3 camera_point = world_to_camera(point_world_m);
    if (camera_point.z <= 0.0) {
        return std::nullopt;
    }

    const double horizontal_angle = std::atan2(camera_point.x, camera_point.z);
    const double vertical_angle = std::atan2(camera_point.y, camera_point.z);
    if (std::abs(horizontal_angle) > config_.hfov_rad / 2.0 ||
        std::abs(vertical_angle) > config_.vfov_rad / 2.0) {
        return std::nullopt;
    }

    const double u_px = cx_px() + fx_px() * camera_point.x / camera_point.z;
    const double v_px = cy_px() - fy_px() * camera_point.y / camera_point.z;
    return Projection{
        .u_px = u_px,
        .v_px = v_px,
        .x_cam_m = camera_point.x,
        .y_cam_m = camera_point.y,
        .z_cam_m = camera_point.z,
    };
}

std::pair<double, double> PanTiltCamera::ideal_angles_to(const Vec3& point_world_m) const {
    return ideal_pan_tilt(position_m_, point_world_m);
}

std::pair<double, double> PanTiltCamera::pixel_error_to_angles(
    const double du_px,
    const double dv_px) const noexcept {
    return {
        std::atan(du_px / fx_px()),
        -std::atan(dv_px / fy_px()),
    };
}

}  // namespace fsoc

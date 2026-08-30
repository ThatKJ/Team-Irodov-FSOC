#include "fsoc/tracking_error.hpp"

#include <stdexcept>

namespace fsoc {

std::optional<TrackingError> compute_tracking_error(
    const std::optional<BeaconDetection>& detection,
    const PanTiltCamera& camera) {
    // Legitimate absence: no detection this frame -> no error to act on.
    if (!detection.has_value()) {
        return std::nullopt;
    }

    // A non-finite centroid is a broken measurement, not "no measurement".
    // Reject it loudly so it can never turn into a NaN rate command.
    if (!is_finite(*detection)) {
        throw std::invalid_argument(
            "compute_tracking_error: detection centroid must be finite.");
    }

    // Image centre is owned by the camera model; do not recompute it here.
    const double x_error_px = detection->centroid_px.x_px - camera.cx_px();
    const double y_error_px = detection->centroid_px.y_px - camera.cy_px();

    // Exact pinhole pixel->angle conversion lives in exactly one place.
    const auto [pan_rad, tilt_rad] = camera.pixel_error_to_angles(x_error_px, y_error_px);

    return TrackingError{
        .pixel = PixelError{.x_px = x_error_px, .y_px = y_error_px},
        .angular = AngularError{.pan_rad = pan_rad, .tilt_rad = tilt_rad},
    };
}

}  // namespace fsoc

#pragma once

#include <optional>

#include "fsoc/camera.hpp"
#include "fsoc/measurement.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// Tracking-error layer  (controller-facing contract)
// ---------------------------------------------------------------------------
//
// The error between the detected beacon centroid and the image centre
// (principal point / optical axis). This is the ONLY quantity the future PID
// controller consumes. It is derived from image pixels + camera intrinsics
// only — never from world TargetState.
//
// ---- Pixel error sign convention (FROZEN) --------------------------------
//   x_px = detected_x_px - cx      cx = camera.cx_px() = width_px  / 2.0
//   y_px = detected_y_px - cy      cy = camera.cy_px() = height_px / 2.0
//
//   x_px > 0  -> beacon is RIGHT of centre
//   x_px < 0  -> beacon is LEFT  of centre
//   y_px > 0  -> beacon is BELOW centre   (image +y points DOWN)
//   y_px < 0  -> beacon is ABOVE centre
//
// ---- Angular error sign convention (FROZEN) ---------------------------
// Computed by the one existing exact routine, PanTiltCamera::pixel_error_to_angles:
//   pan_rad  =  atan(x_px / fx)
//   tilt_rad = -atan(y_px / fy)        the minus undoes image-down vs world-up
//
//   pan_rad  > 0  -> beacon to the RIGHT  -> command pan toward +pan  (right)
//   pan_rad  < 0  -> beacon to the LEFT   -> command pan toward -pan  (left)
//   tilt_rad > 0  -> beacon ABOVE centre  -> command tilt toward +tilt (up)
//   tilt_rad < 0  -> beacon BELOW centre  -> command tilt toward -tilt (down)
//
// These map straight onto PanTiltCamera::step(pan_rate, tilt_rate, dt): a
// positive error becomes a positive rate that drives the axis toward the beacon.

struct PixelError {
    double x_px{};
    double y_px{};
};

struct AngularError {
    double pan_rad{};
    double tilt_rad{};
};

// Distinct concepts kept distinct: pixel offset vs. the pan/tilt angles it implies.
struct TrackingError {
    PixelError pixel{};
    AngularError angular{};
};

// Controller-facing error, or std::nullopt when there is no detection to act on.
//
//   detection == std::nullopt  -> returns std::nullopt (no error is computed)
//   detection has non-finite centroid -> throws std::invalid_argument
//   otherwise -> TrackingError using camera.cx_px()/cy_px()/fx_px()/fy_px()
//               and camera.pixel_error_to_angles() (no duplicated math)
//
// The std::optional in / std::optional out shape makes "compute an error from a
// lost target" hard to do by accident.
[[nodiscard]] std::optional<TrackingError> compute_tracking_error(
    const std::optional<BeaconDetection>& detection,
    const PanTiltCamera& camera);

}  // namespace fsoc

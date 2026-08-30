#pragma once

#include <cmath>

namespace fsoc {

// ---------------------------------------------------------------------------
// Image coordinate convention  (FROZEN — authoritative for all pixel modules)
// ---------------------------------------------------------------------------
//
//   origin  = top-left corner of the image
//   +x_px   = right   (grows with camera-frame +x, i.e. image-right)
//   +y_px   = down    (grows opposite to camera-frame +y / world +Z-up)
//
// This is exactly the convention baked into PanTiltCamera::project():
//       u = cx + fx * x_cam / z_cam        (image-right)
//       v = cy - fy * y_cam / z_cam        (image-down; note the minus sign)
//
// Image centre (principal point) is defined with floating-point division and is
// owned by the camera model:  cx = width_px / 2.0 ,  cy = height_px / 2.0
// (see PanTiltCamera::cx_px() / cy_px()). No other module may redefine it.

// A single point in the image plane, in pixels. Not a measurement by itself —
// see BeaconDetection (measurement layer) and Projection (camera layer).
struct ImagePoint {
    double x_px{};
    double y_px{};
};

[[nodiscard]] inline bool is_finite(const ImagePoint& point) noexcept {
    return std::isfinite(point.x_px) && std::isfinite(point.y_px);
}

}  // namespace fsoc

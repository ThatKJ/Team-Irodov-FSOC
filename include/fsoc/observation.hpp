#pragma once

#include <optional>

#include "fsoc/camera.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/image_geometry.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// Observation layer
// ---------------------------------------------------------------------------
//
// Turns "where is the beacon in the world" into "what the perfect virtual
// camera would see". This is still exact mathematics, NOT a detector:
//   * it consumes a world position (truth),
//   * it produces an image point plus an explicit visibility classification.
//
// Downstream (renderer / detector / controller) consume Detection / TrackingError,
// never this and never the world position.

// Why the target is or is not on the sensor. Distinguishes the two failure
// modes the controller and telemetry care about, instead of collapsing them
// into a bare "no value".
enum class ObservationStatus {
    Visible,             // in front of the camera AND inside the configured half-FOV
    OutsideFieldOfView,  // in front of the camera but bearing exceeds half-FOV
    BehindCamera,        // at or behind the camera plane (z_cam <= 0): no image forms
};

// Result of projecting one world point through the camera.
// Invariant: image_point_px has a value if and only if status == Visible.
// The "lost" cases carry NO coordinates — never a (-1,-1) / NaN / 0 sentinel.
struct CameraObservation {
    ObservationStatus status{ObservationStatus::BehindCamera};
    std::optional<ImagePoint> image_point_px{};

    [[nodiscard]] bool visible() const noexcept {
        return status == ObservationStatus::Visible;
    }
};

// Project a world-frame beacon position through the camera and classify the result.
//
// Reuses PanTiltCamera::project() for the pinhole equations and
// PanTiltCamera::world_to_camera() for the depth test, so the projection math
// exists in exactly one place. Takes a bare position (not TargetState): the
// observation layer must not depend on target velocity or trajectory identity.
[[nodiscard]] CameraObservation observe_beacon(
    const PanTiltCamera& camera,
    const Vec3& beacon_world_position_m);

}  // namespace fsoc

#include "fsoc/observation.hpp"

namespace fsoc {

CameraObservation observe_beacon(
    const PanTiltCamera& camera,
    const Vec3& beacon_world_position_m) {
    // Depth test first: project() also rejects z_cam <= 0, but we must tell
    // "behind camera" apart from "outside FOV" for telemetry and loss handling.
    const Vec3 beacon_camera_frame_m = camera.world_to_camera(beacon_world_position_m);
    if (beacon_camera_frame_m.z <= 0.0) {
        return CameraObservation{.status = ObservationStatus::BehindCamera,
                                 .image_point_px = std::nullopt};
    }

    // In front of the camera: the single source of the pinhole equations decides
    // whether the bearing is inside the configured half-FOV.
    const std::optional<Projection> projection = camera.project(beacon_world_position_m);
    if (!projection.has_value()) {
        return CameraObservation{.status = ObservationStatus::OutsideFieldOfView,
                                 .image_point_px = std::nullopt};
    }

    return CameraObservation{
        .status = ObservationStatus::Visible,
        .image_point_px = ImagePoint{.x_px = projection->u_px, .y_px = projection->v_px},
    };
}

}  // namespace fsoc

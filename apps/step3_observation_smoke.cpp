// Step 3 smoke test: observation / measurement / tracking-error CONTRACTS.
//
// Terminal-only. No OpenCV, no rendering, no detector algorithm, no controller.
// It feeds known pixel coordinates through compute_tracking_error() and known
// world points through observe_beacon() to demonstrate the frozen sign
// conventions.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/measurement.hpp"
#include "fsoc/observation.hpp"
#include "fsoc/tracking_error.hpp"

namespace {

[[nodiscard]] std::string_view horizontal_word(const double x_px) {
    if (x_px > 0.0) return "RIGHT";
    if (x_px < 0.0) return "LEFT";
    return "centre";
}

[[nodiscard]] std::string_view vertical_word(const double y_px) {
    if (y_px < 0.0) return "ABOVE";
    if (y_px > 0.0) return "BELOW";
    return "centre";
}

[[nodiscard]] std::string_view status_word(const fsoc::ObservationStatus status) {
    switch (status) {
        case fsoc::ObservationStatus::Visible:
            return "Visible";
        case fsoc::ObservationStatus::OutsideFieldOfView:
            return "OutsideFieldOfView";
        case fsoc::ObservationStatus::BehindCamera:
            return "BehindCamera";
    }
    return "?";
}

void show_detection_case(
    const char* label,
    const fsoc::PanTiltCamera& camera,
    const std::optional<fsoc::BeaconDetection>& detection) {
    std::cout << "\n-- " << label << " --\n";
    if (detection.has_value()) {
        std::cout << "  centroid_px      = (" << detection->centroid_px.x_px << ", "
                  << detection->centroid_px.y_px << ")\n";
    } else {
        std::cout << "  centroid_px      = <no detection>\n";
    }

    const std::optional<fsoc::TrackingError> error =
        fsoc::compute_tracking_error(detection, camera);
    if (!error.has_value()) {
        std::cout << "  tracking error   = <none> (not computed for a lost target)\n";
        return;
    }

    const bool centred = error->pixel.x_px == 0.0 && error->pixel.y_px == 0.0;
    const auto pan_word = error->angular.pan_rad > 0.0    ? "pan RIGHT"
                          : error->angular.pan_rad < 0.0  ? "pan LEFT"
                                                          : "pan hold";
    const auto tilt_word = error->angular.tilt_rad > 0.0   ? "tilt UP"
                           : error->angular.tilt_rad < 0.0 ? "tilt DOWN"
                                                           : "tilt hold";

    std::cout << std::setprecision(3)
              << "  pixel error      = (" << error->pixel.x_px << ", " << error->pixel.y_px
              << ") px  -> "
              << (centred ? "centred on optical axis"
                          : (std::string(horizontal_word(error->pixel.x_px)) + " + " +
                             std::string(vertical_word(error->pixel.y_px))))
              << '\n'
              << std::setprecision(6)
              << "  angular error    = pan " << error->angular.pan_rad << " rad, tilt "
              << error->angular.tilt_rad << " rad\n"
              << std::setprecision(4)
              << "                     pan " << fsoc::rad_to_deg(error->angular.pan_rad)
              << " deg, tilt " << fsoc::rad_to_deg(error->angular.tilt_rad) << " deg\n"
              << "  command meaning  = " << pan_word << ", " << tilt_word << '\n'
              << std::setprecision(6);
}

void show_observation_case(
    const char* label,
    const fsoc::PanTiltCamera& camera,
    const fsoc::Vec3& world_point_m) {
    const fsoc::CameraObservation observation = fsoc::observe_beacon(camera, world_point_m);
    std::cout << "\n-- " << label << " --\n"
              << "  world_point_m    = (" << world_point_m.x << ", " << world_point_m.y << ", "
              << world_point_m.z << ")\n"
              << "  status           = " << status_word(observation.status) << '\n';
    if (observation.image_point_px.has_value()) {
        std::cout << "  image_point_px   = (" << observation.image_point_px->x_px << ", "
                  << observation.image_point_px->y_px << ")\n";
    } else {
        std::cout << "  image_point_px   = <none>\n";
    }
}

}  // namespace

int main() {
    using namespace fsoc;

    const CameraConfig config{};
    const PanTiltCamera camera{config};

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Step 3: observation / measurement / tracking-error contracts\n";
    std::cout << "Image convention: origin top-left, +x right, +y down\n\n";
    std::cout << "image size       = " << config.width_px << " x " << config.height_px << " px\n"
              << "image centre     = (" << camera.cx_px() << ", " << camera.cy_px() << ") px"
              << "   [cx = W/2.0, cy = H/2.0]\n"
              << std::setprecision(4) << "focal length     = fx " << camera.fx_px()
              << " px, fy " << camera.fy_px() << " px\n"
              << std::setprecision(6);

    // ----- tracking-error sign demonstrations --------------------------
    const double cx = camera.cx_px();
    const double cy = camera.cy_px();

    show_detection_case("centred target", camera,
                        BeaconDetection{.centroid_px = {cx, cy}});
    show_detection_case("CRITICAL REVIEW: centre (320,240), beacon (400,180)", camera,
                        BeaconDetection{.centroid_px = {400.0, 180.0}});
    show_detection_case("target to the LEFT", camera,
                        BeaconDetection{.centroid_px = {cx - 80.0, cy}});
    show_detection_case("target BELOW", camera,
                        BeaconDetection{.centroid_px = {cx, cy + 60.0}});
    show_detection_case("no detection (lost target)", camera, std::nullopt);

    // ----- observation classification demonstrations -----------------
    show_observation_case("beacon straight ahead (visible, centred)", camera, {100.0, 0.0, 0.0});
    show_observation_case("beacon slightly right+up (visible)", camera, {100.0, 3.0, 2.0});
    show_observation_case("beacon far to the right (outside FOV)", camera, {100.0, 60.0, 0.0});
    show_observation_case("beacon behind the camera", camera, {-10.0, 0.0, 0.0});

    // ----- machine check of the CRITICAL REVIEW scenario -------------
    const auto critical = compute_tracking_error(
        BeaconDetection{.centroid_px = {400.0, 180.0}}, camera);
    const bool ok = critical.has_value() && critical->pixel.x_px > 0.0 &&
                    critical->pixel.y_px < 0.0 && critical->angular.pan_rad > 0.0 &&
                    critical->angular.tilt_rad > 0.0;

    std::cout << "\nCRITICAL REVIEW check: beacon (400,180) vs centre (320,240)\n"
              << "  expect pixel (+,-)  -> RIGHT + ABOVE\n"
              << "  expect pan > 0 (right) and tilt > 0 (up)\n"
              << "  result: " << (ok ? "PASS" : "FAIL") << '\n';

    return ok ? 0 : 1;
}

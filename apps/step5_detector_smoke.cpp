// Step 5 smoke test: baseline beacon detector.
//
// Terminal-only, headless. The renderer GENERATES frames; the detector then
// receives ONLY the cv::Mat. Requested (truth) coordinates exist here purely to
// score the detector's output — they are never passed to BeaconDetector.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>

#include <opencv2/core.hpp>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/detector.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/measurement.hpp"
#include "fsoc/observation.hpp"
#include "fsoc/renderer.hpp"
#include "fsoc/tracking_error.hpp"

namespace {

// Sub-pixel accuracy gate for clean interior synthetic frames (per axis).
constexpr double kInteriorGatePx = 0.15;

struct Scenario {
    const char* label{};
    fsoc::ImagePoint requested_px{};  // smoke-only truth for scoring
    bool expect_detection{true};
    double gate_px{kInteriorGatePx};
};

[[nodiscard]] double detect_micros(
    const fsoc::BeaconDetector& detector,
    const cv::Mat& frame,
    std::optional<fsoc::BeaconDetection>& out) {
    const auto t0 = std::chrono::steady_clock::now();
    out = detector.detect(frame);
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

bool run_scenario(
    const Scenario& scenario,
    const fsoc::BeaconDetector& detector,
    const cv::Mat& frame) {
    std::optional<fsoc::BeaconDetection> detection;
    const double micros = detect_micros(detector, frame, detection);

    std::cout << "\n=== " << scenario.label << " ===\n"
              << std::fixed << std::setprecision(3)
              << "  frame            = " << frame.cols << " x " << frame.rows
              << " CV_8UC1\n"
              << "  detector time    = " << std::setprecision(1) << micros << " us\n"
              << std::setprecision(3);

    if (!scenario.expect_detection) {
        const bool ok = !detection.has_value();
        std::cout << "  requested        = <no target>\n"
                  << "  detected         = " << (detection ? "(unexpected detection!)" : "<none>")
                  << '\n'
                  << "  result           = " << (ok ? "PASS" : "FAIL") << '\n';
        return ok;
    }

    if (!detection.has_value()) {
        std::cout << "  requested        = (" << scenario.requested_px.x_px << ", "
                  << scenario.requested_px.y_px << ")\n"
                  << "  detected         = <none>  (expected a detection)\n"
                  << "  result           = FAIL\n";
        return false;
    }

    const double dx = detection->centroid_px.x_px - scenario.requested_px.x_px;
    const double dy = detection->centroid_px.y_px - scenario.requested_px.y_px;
    const double err_norm = std::hypot(dx, dy);
    const bool ok = std::abs(dx) <= scenario.gate_px && std::abs(dy) <= scenario.gate_px;

    std::cout << "  requested        = (" << scenario.requested_px.x_px << ", "
              << scenario.requested_px.y_px << ")   [smoke-only truth]\n"
              << "  detected         = (" << detection->centroid_px.x_px << ", "
              << detection->centroid_px.y_px << ")\n"
              << "  error_x          = " << std::showpos << dx << " px\n"
              << "  error_y          = " << dy << " px\n"
              << std::noshowpos
              << "  error_norm       = " << err_norm << " px   (gate |dx|,|dy| <= "
              << scenario.gate_px << ")\n"
              << "  result           = " << (ok ? "PASS" : "FAIL") << '\n';
    return ok;
}

}  // namespace

int main() {
    using namespace fsoc;

    const CameraConfig camera_config{};
    const PanTiltCamera camera{camera_config};
    const SyntheticCameraRenderer renderer{renderer_config_for(camera_config, 2.0)};
    const BeaconDetector detector{BeaconDetectorConfig{}};

    std::cout << "Step 5: baseline beacon detector\n"
              << "threshold >= " << static_cast<int>(detector.config().threshold_intensity)
              << ", min bright pixels " << detector.config().min_bright_pixels
              << ", weight = (pixel - threshold) + 1\n"
              << "renderer: CV_8UC1, background "
              << static_cast<int>(renderer.config().background_intensity) << ", peak "
              << static_cast<int>(renderer.config().beacon_peak_intensity) << ", sigma "
              << renderer.config().beacon_sigma_px << " px\n";

    bool all_ok = true;

    // Scenario 1 — centred beacon (truth via the real projection pipeline).
    const CameraObservation centred = observe_beacon(camera, {100.0, 0.0, 0.0});
    all_ok &= run_scenario(
        Scenario{.label = "Scenario 1: centred beacon",
                 .requested_px = centred.image_point_px.value_or(ImagePoint{})},
        detector, renderer.render(centred));

    // Scenario 2 — sub-pixel beacon (the manual engineering case).
    const CameraObservation subpixel{.status = ObservationStatus::Visible,
                                     .image_point_px = ImagePoint{.x_px = 400.4, .y_px = 179.7}};
    const cv::Mat subpixel_frame = renderer.render(subpixel);
    all_ok &= run_scenario(
        Scenario{.label = "Scenario 2: sub-pixel beacon (400.4, 179.7)",
                 .requested_px = ImagePoint{.x_px = 400.4, .y_px = 179.7}},
        detector, subpixel_frame);

    // Scenario 3 — no target (OutsideFieldOfView -> background-only frame).
    const CameraObservation outside = observe_beacon(camera, {100.0, 60.0, 0.0});
    all_ok &= run_scenario(
        Scenario{.label = "Scenario 3: no target (outside FOV)", .expect_detection = false},
        detector, renderer.render(outside));

    // Scenario 4 — near-edge beacon (clipped Gaussian window; looser gate).
    const CameraObservation near_edge{.status = ObservationStatus::Visible,
                                      .image_point_px = ImagePoint{.x_px = 6.0, .y_px = 240.0}};
    all_ok &= run_scenario(
        Scenario{.label = "Scenario 4: near-edge beacon (6.0, 240.0)",
                 .requested_px = ImagePoint{.x_px = 6.0, .y_px = 240.0},
                 .gate_px = 0.5},
        detector, renderer.render(near_edge));

    // --- Full perception measurement chain: renderer -> detector -> tracking error ---
    std::cout << "\n=== Perception chain check: renderer -> detector -> compute_tracking_error ===\n";
    const std::optional<BeaconDetection> chain_detection = detector.detect(subpixel_frame);
    const std::optional<TrackingError> chain_error =
        compute_tracking_error(chain_detection, camera);
    bool chain_ok = chain_detection.has_value() && chain_error.has_value();
    if (chain_ok) {
        const TrackingError& e = *chain_error;
        chain_ok = e.pixel.x_px > 0.0 && e.pixel.y_px < 0.0 && e.angular.pan_rad > 0.0 &&
                   e.angular.tilt_rad > 0.0;
        std::cout << std::fixed << std::setprecision(4)
                  << "  detected centroid = (" << chain_detection->centroid_px.x_px << ", "
                  << chain_detection->centroid_px.y_px << ")  vs image centre ("
                  << camera.cx_px() << ", " << camera.cy_px() << ")\n"
                  << "  pixel error       = (" << std::showpos << e.pixel.x_px << ", "
                  << e.pixel.y_px << ") px  -> "
                  << (e.pixel.x_px > 0.0 ? "RIGHT" : "LEFT") << " + "
                  << (e.pixel.y_px < 0.0 ? "ABOVE" : "BELOW") << '\n'
                  << "  angular error     = pan " << e.angular.pan_rad << " rad, tilt "
                  << e.angular.tilt_rad << " rad  -> "
                  << (e.angular.pan_rad > 0.0 ? "PAN RIGHT" : "PAN LEFT") << ", "
                  << (e.angular.tilt_rad > 0.0 ? "TILT UP" : "TILT DOWN") << '\n'
                  << std::noshowpos;
    } else {
        std::cout << "  chain produced no detection / no error\n";
    }
    std::cout << "  result            = " << (chain_ok ? "PASS" : "FAIL") << '\n';
    all_ok &= chain_ok;

    std::cout << "\nStep 5 smoke: " << (all_ok ? "PASS" : "FAIL") << '\n';
    return all_ok ? 0 : 1;
}

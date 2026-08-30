// Step 4 smoke test: synthetic virtual-camera image renderer.
//
// Terminal-only, headless (no cv::imshow / cv::waitKey). Renders a few
// CameraObservations to CV_8UC1 frames, prints numeric summaries, and writes a
// few example PNGs into ./generated/ for eyeballing.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <system_error>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/observation.hpp"
#include "fsoc/renderer.hpp"

namespace {

struct WeightedCentroid {
    double x_px{};
    double y_px{};
    double total_weight{};
};

// Validation-only tool (NOT the Step-5 detector): intensity-weighted centroid
// after subtracting the background floor.
[[nodiscard]] WeightedCentroid weighted_centroid(const cv::Mat& image, const int background) {
    double sum_w = 0.0;
    double sum_wx = 0.0;
    double sum_wy = 0.0;
    for (int y = 0; y < image.rows; ++y) {
        const std::uint8_t* row = image.ptr<std::uint8_t>(y);
        for (int x = 0; x < image.cols; ++x) {
            const double w = std::max(0.0, static_cast<double>(row[static_cast<std::size_t>(x)]) -
                                               static_cast<double>(background));
            sum_w += w;
            sum_wx += w * static_cast<double>(x);
            sum_wy += w * static_cast<double>(y);
        }
    }
    if (sum_w <= 0.0) {
        return WeightedCentroid{.x_px = std::nan(""), .y_px = std::nan(""), .total_weight = 0.0};
    }
    return WeightedCentroid{.x_px = sum_wx / sum_w, .y_px = sum_wy / sum_w, .total_weight = sum_w};
}

[[nodiscard]] std::string type_name(const cv::Mat& m) {
    return m.type() == CV_8UC1 ? "CV_8UC1" : "OTHER";
}

void report(
    const char* label,
    const std::optional<fsoc::ImagePoint>& requested_px,
    const cv::Mat& frame,
    const int background) {
    double min_val = 0.0;
    double max_val = 0.0;
    cv::Point min_loc;
    cv::Point max_loc;
    cv::minMaxLoc(frame, &min_val, &max_val, &min_loc, &max_loc);
    const WeightedCentroid wc = weighted_centroid(frame, background);

    std::cout << "\n=== " << label << " ===\n"
              << "  image size          = " << frame.cols << " x " << frame.rows << " px\n"
              << "  image type          = " << type_name(frame) << '\n';
    if (requested_px.has_value()) {
        std::cout << "  requested beacon    = (" << std::fixed << std::setprecision(3)
                  << requested_px->x_px << ", " << requested_px->y_px << ") px\n";
    } else {
        std::cout << "  requested beacon    = <none> (background-only expected)\n";
    }
    std::cout << "  peak pixel location = (" << max_loc.x << ", " << max_loc.y << ")\n"
              << "  weighted centroid   = ";
    if (wc.total_weight > 0.0) {
        std::cout << "(" << std::fixed << std::setprecision(3) << wc.x_px << ", " << wc.y_px
                  << ") px   [test-only tool]\n";
    } else {
        std::cout << "<undefined: no pixels above background>\n";
    }
    std::cout << "  intensity range     = [" << static_cast<int>(min_val) << ", "
              << static_cast<int>(max_val) << "]\n";
}

}  // namespace

int main() {
    using namespace fsoc;

    const CameraConfig camera_config{};
    const PanTiltCamera camera{camera_config};
    const RendererConfig renderer_config = renderer_config_for(camera_config, 2.0);
    const SyntheticCameraRenderer renderer{renderer_config};
    const int background = renderer_config.background_intensity;

    std::cout << "Step 4: synthetic virtual-camera image renderer\n"
              << "format CV_8UC1, background " << background << ", beacon peak "
              << static_cast<int>(renderer_config.beacon_peak_intensity) << ", sigma "
              << renderer_config.beacon_sigma_px << " px\n";

    bool ok = true;

    // ----- Scenario 1: beacon at image centre -----------------------
    const CameraObservation centred = observe_beacon(camera, {100.0, 0.0, 0.0});
    const cv::Mat frame_centre = renderer.render(centred);
    report("Scenario 1: beacon at centre", centred.image_point_px, frame_centre, background);
    {
        const bool has_point = centred.image_point_px.has_value();
        const WeightedCentroid wc = weighted_centroid(frame_centre, background);
        ok = ok && has_point && std::abs(wc.x_px - centred.image_point_px->x_px) < 0.15 &&
             std::abs(wc.y_px - centred.image_point_px->y_px) < 0.15;
    }

    // ----- Scenario 2: sub-pixel beacon, off centre (manual check) ----
    // Directly construct the observation so the requested location is exact.
    const CameraObservation subpixel{.status = ObservationStatus::Visible,
                                     .image_point_px = ImagePoint{.x_px = 400.4, .y_px = 179.7}};
    const cv::Mat frame_subpixel = renderer.render(subpixel);
    report("Scenario 2: sub-pixel beacon (400.4, 179.7)", subpixel.image_point_px, frame_subpixel,
           background);
    {
        const WeightedCentroid wc = weighted_centroid(frame_subpixel, background);
        std::cout << "  sub-pixel check     = recovered (" << std::fixed << std::setprecision(3)
                  << wc.x_px << ", " << wc.y_px << ") vs requested (400.400, 179.700); "
                  << "NOT (400, 180)\n";
        ok = ok && std::abs(wc.x_px - 400.4) < 0.15 && std::abs(wc.y_px - 179.7) < 0.15 &&
             std::abs(wc.x_px - 400.0) > 0.2 && std::abs(wc.y_px - 180.0) > 0.1;
    }

    // ----- Scenario 3: outside FOV -> background-only ---------------
    const CameraObservation outside = observe_beacon(camera, {100.0, 60.0, 0.0});
    const cv::Mat frame_none = renderer.render(outside);
    report("Scenario 3: outside FOV (background-only)", std::nullopt, frame_none, background);
    {
        double min_val = 0.0;
        double max_val = 0.0;
        cv::minMaxLoc(frame_none, &min_val, &max_val);
        std::cout << "  status              = "
                  << (outside.status == ObservationStatus::OutsideFieldOfView
                          ? "OutsideFieldOfView"
                          : "other")
                  << "  (no beacon rendered)\n";
        ok = ok && min_val == background && max_val == background;
    }

    // ----- write example PNGs -------------------------------------
    const std::filesystem::path out_dir{"generated"};
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    bool wrote = !ec;
    wrote = wrote && cv::imwrite((out_dir / "step4_center.png").string(), frame_centre);
    wrote = wrote && cv::imwrite((out_dir / "step4_subpixel.png").string(), frame_subpixel);
    wrote = wrote && cv::imwrite((out_dir / "step4_no_target.png").string(), frame_none);
    std::cout << "\nwrote generated/step4_center.png, step4_subpixel.png, step4_no_target.png: "
              << (wrote ? "OK" : "FAILED") << '\n';

    std::cout << "\nStep 4 smoke: " << (ok && wrote ? "PASS" : "FAIL") << '\n';
    return (ok && wrote) ? 0 : 1;
}

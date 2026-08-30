// Step 5 deterministic unit checks: baseline beacon detector.
//
// Same lightweight harness as tests/step1_tests.cpp .. tests/step4_tests.cpp.
// Tests MAY use SyntheticCameraRenderer to generate frames; the detector itself
// only ever receives a cv::Mat.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <opencv2/core.hpp>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/detector.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/measurement.hpp"
#include "fsoc/observation.hpp"
#include "fsoc/renderer.hpp"
#include "fsoc/tracking_error.hpp"
#include "fsoc/trajectory.hpp"

namespace {

using fsoc::BeaconDetection;
using fsoc::BeaconDetector;
using fsoc::BeaconDetectorConfig;
using fsoc::CameraConfig;
using fsoc::CameraObservation;
using fsoc::ImagePoint;
using fsoc::ObservationStatus;
using fsoc::PanTiltCamera;
using fsoc::SyntheticCameraRenderer;

int failures = 0;

void check(const bool condition, const std::string_view expression, const int line) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL line " << line << ": " << expression << '\n';
    }
}

void check_near(
    const double actual,
    const double expected,
    const double tolerance,
    const std::string_view expression,
    const int line) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        ++failures;
        std::cerr << "FAIL line " << line << ": " << expression << " actual=" << actual
                  << " expected=" << expected << " tol=" << tolerance << '\n';
    }
}

template <typename Fn>
void check_throws(Fn&& fn, const std::string_view expression, const int line) {
    bool threw_invalid_argument = false;
    try {
        std::forward<Fn>(fn)();
    } catch (const std::invalid_argument&) {
        threw_invalid_argument = true;
    } catch (...) {
        // Wrong exception type is still a failure.
    }
    check(threw_invalid_argument, expression, line);
}

#define CHECK(expr) check((expr), #expr, __LINE__)
#define CHECK_NEAR(actual, expected, tol) \
    check_near((actual), (expected), (tol), #actual " ~= " #expected, __LINE__)
#define CHECK_THROWS(expr) check_throws([&] { (void)(expr); }, #expr, __LINE__)

// Sub-pixel accuracy gate for clean interior synthetic frames (per axis).
constexpr double kInteriorGatePx = 0.15;
// Tighter bound we actually achieve on clean interior frames.
constexpr double kTightGatePx = 0.05;

// ---- helpers -----------------------------------------------------

[[nodiscard]] CameraObservation visible_at(const double x_px, const double y_px) {
    return CameraObservation{.status = ObservationStatus::Visible,
                             .image_point_px = ImagePoint{.x_px = x_px, .y_px = y_px}};
}

[[nodiscard]] SyntheticCameraRenderer default_renderer() {
    return SyntheticCameraRenderer{fsoc::renderer_config_for(CameraConfig{}, 2.0)};
}

[[nodiscard]] BeaconDetector default_detector() {
    return BeaconDetector{BeaconDetectorConfig{}};
}

[[nodiscard]] cv::Mat render_at(const SyntheticCameraRenderer& renderer, const double x_px,
                                const double y_px) {
    return renderer.render(visible_at(x_px, y_px));
}

// ---- 1 / 2. centred beacon detected at the image centre --------

void test_centred_beacon_detected() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();
    const PanTiltCamera camera{CameraConfig{}};

    const auto detection = detector.detect(render_at(renderer, camera.cx_px(), camera.cy_px()));
    CHECK(detection.has_value());
    if (detection.has_value()) {
        CHECK_NEAR(detection->centroid_px.x_px, camera.cx_px(), kTightGatePx);
        CHECK_NEAR(detection->centroid_px.y_px, camera.cy_px(), kTightGatePx);
    }
}

// ---- 3 / 4. sub-pixel recovery (incl. the manual engineering case) ----

void test_subpixel_recovery() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();

    const auto a = detector.detect(render_at(renderer, 320.4, 239.7));
    CHECK(a.has_value());
    if (a.has_value()) {
        CHECK_NEAR(a->centroid_px.x_px, 320.4, kTightGatePx);
        CHECK_NEAR(a->centroid_px.y_px, 239.7, kTightGatePx);
    }

    // Manual engineering case: renderer input (400.4, 179.7), detector sees pixels only.
    const auto b = detector.detect(render_at(renderer, 400.4, 179.7));
    CHECK(b.has_value());
    if (b.has_value()) {
        CHECK_NEAR(b->centroid_px.x_px, 400.4, kTightGatePx);
        CHECK_NEAR(b->centroid_px.y_px, 179.7, kTightGatePx);
        CHECK(std::abs(b->centroid_px.x_px - 400.0) > 0.25);  // genuinely sub-pixel
        CHECK(std::abs(b->centroid_px.y_px - 180.0) > 0.15);
    }
}

// ---- 5. detection error bounded per axis across the frame ------

void test_error_bounded_per_axis() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();

    for (const auto [x, y] : {std::pair{150.2, 111.8}, std::pair{300.5, 250.5},
                              std::pair{489.9, 333.1}, std::pair{200.25, 175.75},
                              std::pair{411.6, 300.4}}) {
        const auto detection = detector.detect(render_at(renderer, x, y));
        CHECK(detection.has_value());
        if (detection.has_value()) {
            CHECK(std::abs(detection->centroid_px.x_px - x) <= kInteriorGatePx);
            CHECK(std::abs(detection->centroid_px.y_px - y) <= kInteriorGatePx);
        }
    }
}

// ---- 6..9. quadrant sign of the detected centroid vs image centre ----

void test_quadrant_signs() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();
    const PanTiltCamera camera{CameraConfig{}};
    const double cx = camera.cx_px();
    const double cy = camera.cy_px();

    CHECK(detector.detect(render_at(renderer, cx + 80.0, cy))->centroid_px.x_px > cx);  // right
    CHECK(detector.detect(render_at(renderer, cx - 80.0, cy))->centroid_px.x_px < cx);  // left
    CHECK(detector.detect(render_at(renderer, cx, cy - 60.0))->centroid_px.y_px < cy);  // above
    CHECK(detector.detect(render_at(renderer, cx, cy + 60.0))->centroid_px.y_px > cy);  // below
}

// ---- 10..12. lost-target semantics -> std::nullopt --------------

void test_no_beacon_returns_nullopt() {
    const auto detector = default_detector();

    // Uniform dark frames.
    CHECK(!detector.detect(cv::Mat(480, 640, CV_8UC1, cv::Scalar(5.0))).has_value());
    CHECK(!detector.detect(cv::Mat(480, 640, CV_8UC1, cv::Scalar(0.0))).has_value());

    // A few scattered bright pixels, each below min_bright_pixels -> no valid component.
    cv::Mat speckled(480, 640, CV_8UC1, cv::Scalar(5.0));
    speckled.at<std::uint8_t>(100, 100) = 200;
    speckled.at<std::uint8_t>(300, 400) = 200;
    speckled.at<std::uint8_t>(50, 600) = 200;
    CHECK(!detector.detect(speckled).has_value());

    // Renderer background-only frames.
    const auto renderer = default_renderer();
    CHECK(!detector
               .detect(renderer.render(
                   CameraObservation{.status = ObservationStatus::OutsideFieldOfView}))
               .has_value());
    CHECK(!detector
               .detect(
                   renderer.render(CameraObservation{.status = ObservationStatus::BehindCamera}))
               .has_value());
}

// ---- 13. beacons near each edge are handled safely -------------

void test_edge_beacons_handled_safely() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();

    for (const auto [x, y] : {std::pair{2.0, 240.0}, std::pair{637.0, 240.0},
                              std::pair{320.0, 2.0}, std::pair{320.0, 477.0}}) {
        const auto detection = detector.detect(render_at(renderer, x, y));
        CHECK(detection.has_value());  // found, no crash
        if (detection.has_value()) {
            // Edge clipping biases the centroid; only require it stays near the beacon.
            CHECK(std::abs(detection->centroid_px.x_px - x) <= 1.0);
            CHECK(std::abs(detection->centroid_px.y_px - y) <= 1.0);
        }
    }
}

// ---- 14..16. input frame validation ---------------------------

void test_frame_validation() {
    const auto detector = default_detector();

    CHECK_THROWS(detector.detect(cv::Mat{}));                                    // empty
    CHECK_THROWS(detector.detect(cv::Mat(480, 640, CV_8UC3, cv::Scalar(200, 200, 200))));  // 3ch
    CHECK_THROWS(detector.detect(cv::Mat(480, 640, CV_32FC1, cv::Scalar(200.0))));         // float
    CHECK_THROWS(detector.detect(cv::Mat(480, 640, CV_16UC1, cv::Scalar(4000.0))));        // 16-bit
}

// ---- 17. invalid detector configuration is rejected ----------

void test_invalid_config_rejected() {
    BeaconDetectorConfig zero_threshold{};
    zero_threshold.threshold_intensity = 0;
    CHECK_THROWS(BeaconDetector{zero_threshold});

    BeaconDetectorConfig max_threshold{};
    max_threshold.threshold_intensity = 255;
    CHECK_THROWS(BeaconDetector{max_threshold});

    BeaconDetectorConfig zero_min_pixels{};
    zero_min_pixels.min_bright_pixels = 0;
    CHECK_THROWS(BeaconDetector{zero_min_pixels});

    BeaconDetectorConfig negative_min_pixels{};
    negative_min_pixels.min_bright_pixels = -3;
    CHECK_THROWS(BeaconDetector{negative_min_pixels});

    // Boundary-valid configs must NOT throw.
    bool ok = true;
    try {
        BeaconDetectorConfig lo{};
        lo.threshold_intensity = 1;
        lo.min_bright_pixels = 1;
        const BeaconDetector d_lo{lo};
        BeaconDetectorConfig hi{};
        hi.threshold_intensity = 254;
        const BeaconDetector d_hi{hi};
        const BeaconDetector d_default{BeaconDetectorConfig{}};
        (void)d_lo;
        (void)d_hi;
        (void)d_default;
    } catch (...) {
        ok = false;
    }
    CHECK(ok);
}

// ---- 18. determinism -----------------------------------------

void test_detection_is_deterministic() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();
    const cv::Mat frame = render_at(renderer, 333.3, 271.9);

    const auto a = detector.detect(frame);
    const auto b = detector.detect(frame);
    CHECK(a.has_value() && b.has_value());
    if (a.has_value() && b.has_value()) {
        CHECK(a->centroid_px.x_px == b->centroid_px.x_px);
        CHECK(a->centroid_px.y_px == b->centroid_px.y_px);
    }
}

// ---- 19. multiple bright regions -> strongest integrated signal wins ----

void test_strongest_component_policy() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();

    // Strong renderer beacon on the left; dim manual block on the right.
    {
        cv::Mat frame = render_at(renderer, 200.0, 240.0);
        frame(cv::Rect(447, 237, 6, 6)).setTo(cv::Scalar(100.0));  // 36 px, value 100 > threshold
        const auto detection = detector.detect(frame);
        CHECK(detection.has_value());
        if (detection.has_value()) {
            CHECK_NEAR(detection->centroid_px.x_px, 200.0, 1.0);  // the strong one, not a blend
            CHECK(detection->centroid_px.x_px < 300.0);
        }
    }
    // Mirror: strong beacon on the right, dim block on the left.
    {
        cv::Mat frame = render_at(renderer, 440.0, 240.0);
        frame(cv::Rect(187, 237, 6, 6)).setTo(cv::Scalar(100.0));
        const auto detection = detector.detect(frame);
        CHECK(detection.has_value());
        if (detection.has_value()) {
            CHECK_NEAR(detection->centroid_px.x_px, 440.0, 1.0);
            CHECK(detection->centroid_px.x_px > 300.0);
        }
    }
}

// ---- full perception chain: renderer -> detector -> tracking error ----

void test_perception_chain_to_tracking_error() {
    const auto renderer = default_renderer();
    const auto detector = default_detector();
    const PanTiltCamera camera{CameraConfig{}};  // centre (320, 240)

    // Beacon RIGHT + ABOVE of centre.
    const cv::Mat frame = render_at(renderer, 400.4, 179.7);
    const std::optional<BeaconDetection> detection = detector.detect(frame);
    CHECK(detection.has_value());

    const std::optional<fsoc::TrackingError> error =
        fsoc::compute_tracking_error(detection, camera);
    CHECK(error.has_value());
    if (error.has_value()) {
        CHECK(error->pixel.x_px > 0.0);      // RIGHT
        CHECK(error->pixel.y_px < 0.0);      // ABOVE
        CHECK(error->angular.pan_rad > 0.0);   // command PAN RIGHT
        CHECK(error->angular.tilt_rad > 0.0);  // command TILT UP
        // The measured error magnitude tracks the true offset within the gate.
        CHECK(std::abs(error->pixel.x_px - (400.4 - camera.cx_px())) <= kInteriorGatePx);
        CHECK(std::abs(error->pixel.y_px - (179.7 - camera.cy_px())) <= kInteriorGatePx);
    }
}

// ---- 20..23. prior-step behaviour still intact ---------------

void test_prior_steps_regression() {
    const PanTiltCamera camera{CameraConfig{}};

    // Step 1: projection.
    const auto centre = camera.project({100.0, 0.0, 0.0});
    CHECK(centre.has_value());
    CHECK_NEAR(centre->u_px, camera.cx_px(), 1e-9);
    CHECK_NEAR(centre->v_px, camera.cy_px(), 1e-9);

    // Step 2: trajectory.
    const fsoc::LinearTrajectory linear{{100.0, 0.0, 2.0}, {0.0, 1.0, 0.2}};
    const auto s = linear.state_at(2.0);
    CHECK_NEAR(s.position_m.y, 2.0, 1e-12);
    CHECK_NEAR(s.position_m.z, 2.4, 1e-12);

    // Step 3: observation classification.
    CHECK(fsoc::observe_beacon(camera, {-10.0, 0.0, 0.0}).status ==
          ObservationStatus::BehindCamera);

    // Step 4: renderer still produces a CV_8UC1 frame with a bright beacon.
    const auto renderer = default_renderer();
    const cv::Mat frame = render_at(renderer, 320.0, 240.0);
    CHECK(frame.type() == CV_8UC1 && frame.cols == 640 && frame.rows == 480);
    double max_val = 0.0;
    cv::minMaxLoc(frame, nullptr, &max_val);
    CHECK(max_val > renderer.config().background_intensity);
}

}  // namespace

int main() {
    test_centred_beacon_detected();
    test_subpixel_recovery();
    test_error_bounded_per_axis();
    test_quadrant_signs();
    test_no_beacon_returns_nullopt();
    test_edge_beacons_handled_safely();
    test_frame_validation();
    test_invalid_config_rejected();
    test_detection_is_deterministic();
    test_strongest_component_policy();
    test_perception_chain_to_tracking_error();
    test_prior_steps_regression();

    if (failures == 0) {
        std::cout << "PASS: 12 Step-5 detector checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

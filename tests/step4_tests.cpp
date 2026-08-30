// Step 4 deterministic unit checks: synthetic virtual-camera image renderer.
//
// Same lightweight harness as tests/step1_tests.cpp .. tests/step3_tests.cpp.
// Uses OpenCV core only (no imgproc / imgcodecs / highgui) and runs headless.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <opencv2/core.hpp>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/measurement.hpp"
#include "fsoc/observation.hpp"
#include "fsoc/renderer.hpp"
#include "fsoc/tracking_error.hpp"
#include "fsoc/trajectory.hpp"

namespace {

using fsoc::CameraConfig;
using fsoc::CameraObservation;
using fsoc::ImagePoint;
using fsoc::ObservationStatus;
using fsoc::RendererConfig;
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

// ---- shared helpers -----------------------------------------------

[[nodiscard]] CameraObservation visible_at(const double x_px, const double y_px) {
    return CameraObservation{.status = ObservationStatus::Visible,
                             .image_point_px = ImagePoint{.x_px = x_px, .y_px = y_px}};
}

[[nodiscard]] int px(const cv::Mat& image, const int row, const int col) {
    return image.ptr<std::uint8_t>(row)[static_cast<std::size_t>(col)];
}

struct WeightedCentroid {
    double x_px{};
    double y_px{};
    double total_weight{};
};

// Validation-only tool (NOT the Step-5 detector): background-subtracted
// intensity-weighted centroid over the whole image.
[[nodiscard]] WeightedCentroid weighted_centroid(const cv::Mat& image, const int background) {
    double sum_w = 0.0;
    double sum_wx = 0.0;
    double sum_wy = 0.0;
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            const double w =
                std::max(0.0, static_cast<double>(px(image, y, x)) - static_cast<double>(background));
            sum_w += w;
            sum_wx += w * static_cast<double>(x);
            sum_wy += w * static_cast<double>(y);
        }
    }
    return WeightedCentroid{.x_px = sum_wx / sum_w, .y_px = sum_wy / sum_w, .total_weight = sum_w};
}

[[nodiscard]] bool all_pixels_equal(const cv::Mat& image, const int value) {
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if (px(image, y, x) != value) {
                return false;
            }
        }
    }
    return true;
}

[[nodiscard]] int max_intensity(const cv::Mat& image) {
    double max_val = 0.0;
    cv::minMaxLoc(image, nullptr, &max_val);
    return static_cast<int>(max_val);
}

[[nodiscard]] cv::Point max_location(const cv::Mat& image) {
    cv::Point loc;
    cv::minMaxLoc(image, nullptr, nullptr, nullptr, &loc);
    return loc;
}

[[nodiscard]] SyntheticCameraRenderer default_renderer() {
    return SyntheticCameraRenderer{fsoc::renderer_config_for(CameraConfig{}, 2.0)};
}

constexpr int kBackground = 5;

// ---- 1. dimensions + type --------------------------------------

void test_frame_dimensions_and_type() {
    const auto renderer = default_renderer();
    const cv::Mat frame = renderer.render(visible_at(320.0, 240.0));
    CHECK(frame.cols == 640);
    CHECK(frame.rows == 480);
    CHECK(frame.type() == CV_8UC1);
    CHECK(frame.channels() == 1);
    CHECK(frame.depth() == CV_8U);
}

// ---- 2 / 11 / 12. background-only images -----------------------

void test_background_only_images() {
    const auto renderer = default_renderer();

    const cv::Mat outside =
        renderer.render(CameraObservation{.status = ObservationStatus::OutsideFieldOfView});
    CHECK(outside.cols == 640 && outside.rows == 480 && outside.type() == CV_8UC1);
    CHECK(all_pixels_equal(outside, kBackground));

    const cv::Mat behind =
        renderer.render(CameraObservation{.status = ObservationStatus::BehindCamera});
    CHECK(all_pixels_equal(behind, kBackground));
}

// ---- 3. visible beacon is brighter than background ------------

void test_visible_beacon_is_brighter() {
    const auto renderer = default_renderer();
    const cv::Mat frame = renderer.render(visible_at(320.0, 240.0));
    CHECK(max_intensity(frame) > kBackground);

    int brighter = 0;
    for (int y = 0; y < frame.rows; ++y) {
        for (int x = 0; x < frame.cols; ++x) {
            if (px(frame, y, x) > kBackground) {
                ++brighter;
            }
        }
    }
    CHECK(brighter > 50);  // a resolved spot, not a single hot pixel
}

// ---- 4. peak + weighted centroid near the projected point ----

void test_peak_and_centroid_near_projection() {
    const auto renderer = default_renderer();
    const cv::Mat frame = renderer.render(visible_at(320.0, 240.0));

    const cv::Point peak = max_location(frame);
    CHECK(std::abs(peak.x - 320) <= 1);
    CHECK(std::abs(peak.y - 240) <= 1);

    const WeightedCentroid wc = weighted_centroid(frame, kBackground);
    CHECK_NEAR(wc.x_px, 320.0, 0.1);
    CHECK_NEAR(wc.y_px, 240.0, 0.1);
}

// ---- 5. sub-pixel recovery (incl. the manual engineering check) ----

void test_subpixel_recovery() {
    const auto renderer = default_renderer();

    const WeightedCentroid a = weighted_centroid(renderer.render(visible_at(320.4, 239.7)), kBackground);
    CHECK_NEAR(a.x_px, 320.4, 0.1);
    CHECK_NEAR(a.y_px, 239.7, 0.1);

    // Manual engineering check: 640x480, observation (400.4, 179.7) must render
    // centred near (400.4, 179.7), NOT rounded to (400, 180).
    const WeightedCentroid b = weighted_centroid(renderer.render(visible_at(400.4, 179.7)), kBackground);
    CHECK_NEAR(b.x_px, 400.4, 0.1);
    CHECK_NEAR(b.y_px, 179.7, 0.1);
    CHECK(std::abs(b.x_px - 400.0) > 0.25);  // genuinely sub-pixel in x
    CHECK(std::abs(b.y_px - 180.0) > 0.15);  // genuinely sub-pixel in y
}

// ---- 6. exact-centre beacon renders symmetrically ------------

void test_exact_centre_symmetry() {
    const auto renderer = default_renderer();
    const cv::Mat frame = renderer.render(visible_at(320.0, 240.0));

    CHECK(max_location(frame) == cv::Point(320, 240));
    for (int k = 1; k <= 6; ++k) {
        CHECK(px(frame, 240, 320 - k) == px(frame, 240, 320 + k));  // left/right
        CHECK(px(frame, 240 - k, 320) == px(frame, 240 + k, 320));  // up/down
        CHECK(px(frame, 240 - k, 320 - k) == px(frame, 240 + k, 320 + k));  // diagonals
        CHECK(px(frame, 240 - k, 320 + k) == px(frame, 240 + k, 320 - k));
    }
}

// ---- 7..10. edge clipping is safe -----------------------------

void test_edge_clipping_is_safe() {
    const auto renderer = default_renderer();

    // Near each of the four edges: no overrun, valid frame, spot still present.
    for (const auto [x, y] : {std::pair{2.0, 240.0}, std::pair{637.0, 240.0},
                              std::pair{320.0, 2.0}, std::pair{320.0, 477.0}}) {
        const cv::Mat frame = renderer.render(visible_at(x, y));
        CHECK(frame.cols == 640 && frame.rows == 480 && frame.type() == CV_8UC1);
        CHECK(max_intensity(frame) > kBackground);
        const cv::Point peak = max_location(frame);
        CHECK(std::abs(peak.x - static_cast<int>(x)) <= 2);
        CHECK(std::abs(peak.y - static_cast<int>(y)) <= 2);
    }

    // Beacon exactly at the horizontal FOV edge projects to u = width (=640);
    // the last valid column is 639. Must clip, not overrun.
    const cv::Mat fov_edge = renderer.render(visible_at(640.0, 240.0));
    CHECK(fov_edge.cols == 640);
    CHECK(max_intensity(fov_edge) > kBackground);
    CHECK(max_location(fov_edge).x == 639);

    // Fully off-image: background-only, no crash (clipped window is empty).
    CHECK(all_pixels_equal(renderer.render(visible_at(-30.0, 240.0)), kBackground));
    CHECK(all_pixels_equal(renderer.render(visible_at(800.0, 240.0)), kBackground));
    CHECK(all_pixels_equal(renderer.render(visible_at(320.0, -50.0)), kBackground));
    CHECK(all_pixels_equal(renderer.render(visible_at(320.0, 999.0)), kBackground));
}

// ---- 13. invalid renderer configuration is rejected ---------

void test_invalid_config_is_rejected() {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();

    RendererConfig zero_width{};
    zero_width.width_px = 0;
    CHECK_THROWS(SyntheticCameraRenderer{zero_width});

    RendererConfig negative_width{};
    negative_width.width_px = -8;
    CHECK_THROWS(SyntheticCameraRenderer{negative_width});

    RendererConfig zero_height{};
    zero_height.height_px = 0;
    CHECK_THROWS(SyntheticCameraRenderer{zero_height});

    RendererConfig negative_height{};
    negative_height.height_px = -1;
    CHECK_THROWS(SyntheticCameraRenderer{negative_height});

    RendererConfig zero_sigma{};
    zero_sigma.beacon_sigma_px = 0.0;
    CHECK_THROWS(SyntheticCameraRenderer{zero_sigma});

    RendererConfig negative_sigma{};
    negative_sigma.beacon_sigma_px = -2.0;
    CHECK_THROWS(SyntheticCameraRenderer{negative_sigma});

    RendererConfig nan_sigma{};
    nan_sigma.beacon_sigma_px = nan_v;
    CHECK_THROWS(SyntheticCameraRenderer{nan_sigma});

    RendererConfig tiny_window{};
    tiny_window.beacon_window_sigmas = 0.5;
    CHECK_THROWS(SyntheticCameraRenderer{tiny_window});

    // renderer_config_for propagates an invalid camera config.
    CameraConfig bad_camera{};
    bad_camera.width_px = 0;
    CHECK_THROWS(fsoc::renderer_config_for(bad_camera));

    // Defaults are valid.
    bool ok = true;
    try {
        const auto renderer = default_renderer();
        (void)renderer;
    } catch (...) {
        ok = false;
    }
    CHECK(ok);
}

// ---- 14. Visible must carry a finite image point --------------

void test_visible_requires_finite_image_point() {
    const auto renderer = default_renderer();
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();

    CHECK_THROWS(renderer.render(CameraObservation{.status = ObservationStatus::Visible}));
    CHECK_THROWS(renderer.render(visible_at(nan_v, 240.0)));
    CHECK_THROWS(renderer.render(visible_at(320.0, inf_v)));

    // A non-visible status with no image point is fine (background-only).
    bool ok = true;
    try {
        const cv::Mat frame =
            renderer.render(CameraObservation{.status = ObservationStatus::BehindCamera});
        ok = all_pixels_equal(frame, kBackground);
    } catch (...) {
        ok = false;
    }
    CHECK(ok);
}

// ---- 15. rendering the same observation twice is identical --

void test_render_is_deterministic() {
    const auto renderer = default_renderer();
    const CameraObservation obs = visible_at(333.3, 271.9);

    const cv::Mat a = renderer.render(obs);
    const cv::Mat b = renderer.render(obs);
    CHECK(a.size() == b.size());
    CHECK(a.type() == b.type());

    cv::Mat diff;
    cv::absdiff(a, b, diff);
    CHECK(cv::countNonZero(diff) == 0);
}

// ---- 16..18. prior-step behaviour still intact --------------

void test_prior_steps_regression() {
    const fsoc::PanTiltCamera camera{CameraConfig{}};

    // Step 1: projection unchanged.
    const auto centre = camera.project({100.0, 0.0, 0.0});
    CHECK(centre.has_value());
    CHECK_NEAR(centre->u_px, camera.cx_px(), 1e-9);
    CHECK_NEAR(centre->v_px, camera.cy_px(), 1e-9);

    // Step 2: trajectory unchanged.
    const fsoc::LinearTrajectory linear{{100.0, 0.0, 2.0}, {0.0, 1.0, 0.2}};
    const auto s = linear.state_at(2.0);
    CHECK_NEAR(s.position_m.y, 2.0, 1e-12);
    CHECK_NEAR(s.position_m.z, 2.4, 1e-12);

    // Step 3: observation + tracking error unchanged.
    CHECK(fsoc::observe_beacon(camera, {-10.0, 0.0, 0.0}).status == ObservationStatus::BehindCamera);
    const auto err = fsoc::compute_tracking_error(
        fsoc::BeaconDetection{.centroid_px = {400.0, 180.0}}, camera);
    CHECK(err.has_value());
    CHECK(err->pixel.x_px == 80.0);
    CHECK(err->pixel.y_px == -60.0);
    CHECK(err->angular.pan_rad > 0.0);
    CHECK(err->angular.tilt_rad > 0.0);
}

}  // namespace

int main() {
    test_frame_dimensions_and_type();
    test_background_only_images();
    test_visible_beacon_is_brighter();
    test_peak_and_centroid_near_projection();
    test_subpixel_recovery();
    test_exact_centre_symmetry();
    test_edge_clipping_is_safe();
    test_invalid_config_is_rejected();
    test_visible_requires_finite_image_point();
    test_render_is_deterministic();
    test_prior_steps_regression();

    if (failures == 0) {
        std::cout << "PASS: 11 Step-4 renderer checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

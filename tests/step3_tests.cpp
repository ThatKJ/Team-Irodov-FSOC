// Step 3 deterministic unit checks: observation / measurement / tracking-error
// contracts and the frozen pixel + angular sign conventions.
//
// Same lightweight harness as tests/step1_tests.cpp and tests/step2_tests.cpp.

#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/measurement.hpp"
#include "fsoc/observation.hpp"
#include "fsoc/scenario.hpp"
#include "fsoc/tracking_error.hpp"
#include "fsoc/trajectory.hpp"

namespace {

using fsoc::BeaconDetection;
using fsoc::CameraConfig;
using fsoc::ObservationStatus;
using fsoc::PanTiltCamera;
using fsoc::TrackingError;
using fsoc::Vec3;

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

// Convenience: tracking error for a centroid at absolute pixel (x, y).
[[nodiscard]] TrackingError error_at(const PanTiltCamera& camera, const double x_px,
                                     const double y_px) {
    const auto result = fsoc::compute_tracking_error(
        BeaconDetection{.centroid_px = {x_px, y_px}}, camera);
    if (!result.has_value()) {
        throw std::logic_error("error_at: expected a value");
    }
    return *result;
}

// ---- 1. image centre -------------------------------------------------

void test_image_centre_is_half_dimensions() {
    const PanTiltCamera default_camera{CameraConfig{}};
    CHECK_NEAR(default_camera.cx_px(), 320.0, 1e-12);  // 640 / 2.0
    CHECK_NEAR(default_camera.cy_px(), 240.0, 1e-12);  // 480 / 2.0

    CameraConfig big{};
    big.width_px = 1024;
    big.height_px = 768;
    const PanTiltCamera big_camera{big};
    CHECK_NEAR(big_camera.cx_px(), 512.0, 1e-12);
    CHECK_NEAR(big_camera.cy_px(), 384.0, 1e-12);

    // Odd dimensions still use floating-point division (no integer truncation).
    CameraConfig odd{};
    odd.width_px = 641;
    odd.height_px = 481;
    const PanTiltCamera odd_camera{odd};
    CHECK_NEAR(odd_camera.cx_px(), 320.5, 1e-12);
    CHECK_NEAR(odd_camera.cy_px(), 240.5, 1e-12);
}

// ---- 2. centred detection -> zero error -----------------------------

void test_centred_detection_is_zero_error() {
    const PanTiltCamera camera{CameraConfig{}};
    const TrackingError e = error_at(camera, camera.cx_px(), camera.cy_px());
    CHECK(e.pixel.x_px == 0.0);
    CHECK(e.pixel.y_px == 0.0);
    CHECK_NEAR(e.angular.pan_rad, 0.0, 1e-12);
    CHECK_NEAR(e.angular.tilt_rad, 0.0, 1e-12);
}

// ---- 3/4. horizontal sign: RIGHT is +, LEFT is - -----------------

void test_horizontal_sign_convention() {
    const PanTiltCamera camera{CameraConfig{}};
    const double cx = camera.cx_px();
    const double cy = camera.cy_px();

    const TrackingError right = error_at(camera, cx + 50.0, cy);
    const TrackingError left = error_at(camera, cx - 50.0, cy);

    // (3) right of centre
    CHECK(right.pixel.x_px == 50.0);
    CHECK(right.angular.pan_rad > 0.0);
    CHECK_NEAR(right.angular.pan_rad, std::atan(50.0 / camera.fx_px()), 1e-12);

    // (4) left of centre: exact mirror
    CHECK(left.pixel.x_px == -50.0);
    CHECK(left.angular.pan_rad < 0.0);
    CHECK_NEAR(left.angular.pan_rad, -right.angular.pan_rad, 1e-12);

    // Horizontal error must not bleed into tilt.
    CHECK(right.pixel.y_px == 0.0);
    CHECK_NEAR(right.angular.tilt_rad, 0.0, 1e-12);
}

// ---- 5/6. vertical sign: ABOVE is -, BELOW is + ; tilt ABOVE is + --

void test_vertical_sign_convention() {
    const PanTiltCamera camera{CameraConfig{}};
    const double cx = camera.cx_px();
    const double cy = camera.cy_px();

    const TrackingError above = error_at(camera, cx, cy - 40.0);
    const TrackingError below = error_at(camera, cx, cy + 40.0);

    // (5) above centre: pixel y negative, tilt error positive (command up)
    CHECK(above.pixel.y_px == -40.0);
    CHECK(above.angular.tilt_rad > 0.0);
    CHECK_NEAR(above.angular.tilt_rad, std::atan(40.0 / camera.fy_px()), 1e-12);

    // (6) below centre: pixel y positive, tilt error negative (command down), exact mirror
    CHECK(below.pixel.y_px == 40.0);
    CHECK(below.angular.tilt_rad < 0.0);
    CHECK_NEAR(below.angular.tilt_rad, -above.angular.tilt_rad, 1e-12);

    // Vertical error must not bleed into pan.
    CHECK(above.pixel.x_px == 0.0);
    CHECK_NEAR(above.angular.pan_rad, 0.0, 1e-12);
}

// ---- 7. matches known pinhole geometry + reuses camera math -------

void test_matches_pinhole_geometry() {
    const PanTiltCamera camera{CameraConfig{}};
    const double cx = camera.cx_px();
    const double cy = camera.cy_px();
    const double fx = camera.fx_px();
    const double fy = camera.fy_px();

    for (const auto [dx, dy] : {std::pair{12.0, 7.0}, std::pair{-95.0, 30.0},
                                std::pair{0.0, -123.0}, std::pair{200.0, -160.0}}) {
        const TrackingError e = error_at(camera, cx + dx, cy + dy);
        CHECK(e.pixel.x_px == dx);
        CHECK(e.pixel.y_px == dy);
        // Exact pinhole relationship (docs/04): theta_x = atan(dx/fx), theta_y = -atan(dy/fy).
        CHECK_NEAR(e.angular.pan_rad, std::atan(dx / fx), 1e-12);
        CHECK_NEAR(e.angular.tilt_rad, -std::atan(dy / fy), 1e-12);
        // Must be exactly what the one existing camera routine returns.
        const auto [pan_ref, tilt_ref] = camera.pixel_error_to_angles(dx, dy);
        CHECK(e.angular.pan_rad == pan_ref);
        CHECK(e.angular.tilt_rad == tilt_ref);
    }

    // Round trip through the real projection: feed a projected pixel back in.
    const auto projection = camera.project({100.0, 4.0, -3.0});
    CHECK(projection.has_value());
    const TrackingError e = error_at(camera, projection->u_px, projection->v_px);
    CHECK_NEAR(e.pixel.x_px, projection->u_px - cx, 1e-9);
    CHECK_NEAR(e.pixel.y_px, projection->v_px - cy, 1e-9);
}

// ---- 8. no detection -> no tracking error --------------------------

void test_no_detection_yields_no_error() {
    const PanTiltCamera camera{CameraConfig{}};
    const std::optional<BeaconDetection> nothing{};
    const auto result = fsoc::compute_tracking_error(nothing, camera);
    CHECK(!result.has_value());
}

// ---- 9. non-finite detection coordinates are rejected -------------

void test_non_finite_detection_is_rejected() {
    const PanTiltCamera camera{CameraConfig{}};
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();

    const std::optional<BeaconDetection> nan_x{BeaconDetection{.centroid_px = {nan_v, 10.0}}};
    const std::optional<BeaconDetection> nan_y{BeaconDetection{.centroid_px = {10.0, nan_v}}};
    const std::optional<BeaconDetection> pos_inf{BeaconDetection{.centroid_px = {inf_v, 0.0}}};
    const std::optional<BeaconDetection> neg_inf{BeaconDetection{.centroid_px = {0.0, -inf_v}}};

    CHECK_THROWS(fsoc::compute_tracking_error(nan_x, camera));
    CHECK_THROWS(fsoc::compute_tracking_error(nan_y, camera));
    CHECK_THROWS(fsoc::compute_tracking_error(pos_inf, camera));
    CHECK_THROWS(fsoc::compute_tracking_error(neg_inf, camera));
}

// ---- 10. invalid image dimensions / scenario are rejected --------

void test_invalid_config_is_rejected() {
    CameraConfig zero_width{};
    zero_width.width_px = 0;
    CHECK_THROWS(PanTiltCamera{zero_width});

    CameraConfig negative_height{};
    negative_height.height_px = -5;
    CHECK_THROWS(PanTiltCamera{negative_height});

    fsoc::ScenarioConfig bad_duration{};
    bad_duration.duration_s = -1.0;
    CHECK_THROWS(bad_duration.validate());

    fsoc::ScenarioConfig bad_step{};
    bad_step.timestep_s = 0.0;
    CHECK_THROWS(bad_step.validate());

    fsoc::ScenarioConfig step_exceeds_duration{};
    step_exceeds_duration.duration_s = 1.0;
    step_exceeds_duration.timestep_s = 2.0;
    CHECK_THROWS(step_exceeds_duration.validate());

    fsoc::ScenarioConfig good{};
    bool ok = true;
    try {
        good.validate();
    } catch (...) {
        ok = false;
    }
    CHECK(ok);
}

// ---- 11. determinism ----------------------------------------------

void test_repeated_calls_are_bit_identical() {
    const PanTiltCamera camera{CameraConfig{}};
    const std::optional<BeaconDetection> detection{
        BeaconDetection{.centroid_px = {372.5, 191.25}}};

    const auto a = fsoc::compute_tracking_error(detection, camera);
    const auto b = fsoc::compute_tracking_error(detection, camera);
    CHECK(a.has_value() && b.has_value());
    CHECK(a->pixel.x_px == b->pixel.x_px);
    CHECK(a->pixel.y_px == b->pixel.y_px);
    CHECK(a->angular.pan_rad == b->angular.pan_rad);
    CHECK(a->angular.tilt_rad == b->angular.tilt_rad);
}

// ---- 12. finite outputs -----------------------------------------

void test_returned_values_are_finite() {
    const PanTiltCamera camera{CameraConfig{}};
    for (const auto [x, y] : {std::pair{0.0, 0.0}, std::pair{639.0, 479.0},
                              std::pair{-5000.0, 8000.0}, std::pair{1.0e6, -1.0e6}}) {
        const TrackingError e = error_at(camera, x, y);
        CHECK(std::isfinite(e.pixel.x_px));
        CHECK(std::isfinite(e.pixel.y_px));
        CHECK(std::isfinite(e.angular.pan_rad));
        CHECK(std::isfinite(e.angular.tilt_rad));
    }
}

// ---- tracking error is a pure image-plane quantity (pose independent) ----

void test_tracking_error_is_pose_independent() {
    const PanTiltCamera level{CameraConfig{}, {}, 0.0, 0.0};
    const PanTiltCamera slewed{CameraConfig{}, {}, 0.3, 0.2};
    const std::optional<BeaconDetection> detection{
        BeaconDetection{.centroid_px = {410.0, 205.0}}};

    const auto a = fsoc::compute_tracking_error(detection, level);
    const auto b = fsoc::compute_tracking_error(detection, slewed);
    CHECK(a.has_value() && b.has_value());
    CHECK(a->pixel.x_px == b->pixel.x_px);
    CHECK(a->pixel.y_px == b->pixel.y_px);
    CHECK(a->angular.pan_rad == b->angular.pan_rad);
    CHECK(a->angular.tilt_rad == b->angular.tilt_rad);
}

// ---- CRITICAL REVIEW scenario: (320,240) centre, (400,180) beacon -------

void test_critical_review_scenario() {
    const PanTiltCamera camera{CameraConfig{}};  // centre (320, 240)
    const TrackingError e = error_at(camera, 400.0, 180.0);

    CHECK(e.pixel.x_px == 80.0);   // RIGHT
    CHECK(e.pixel.y_px == -60.0);  // ABOVE
    CHECK(e.angular.pan_rad > 0.0);   // command pan toward the right
    CHECK(e.angular.tilt_rad > 0.0);  // command tilt upward
    CHECK_NEAR(e.angular.pan_rad, std::atan(80.0 / camera.fx_px()), 1e-12);
    CHECK_NEAR(e.angular.tilt_rad, std::atan(60.0 / camera.fy_px()), 1e-12);
}

// ---- observation classification --------------------------------------

void test_observe_beacon_classification() {
    const PanTiltCamera camera{CameraConfig{}};  // at origin, pan/tilt 0, forward +X

    const auto centred = fsoc::observe_beacon(camera, {100.0, 0.0, 0.0});
    CHECK(centred.status == ObservationStatus::Visible);
    CHECK(centred.visible());
    CHECK(centred.image_point_px.has_value());
    CHECK_NEAR(centred.image_point_px->x_px, camera.cx_px(), 1e-9);
    CHECK_NEAR(centred.image_point_px->y_px, camera.cy_px(), 1e-9);

    const auto off_axis = fsoc::observe_beacon(camera, {100.0, 3.0, 2.0});
    CHECK(off_axis.status == ObservationStatus::Visible);
    const auto projection = camera.project({100.0, 3.0, 2.0});
    CHECK(projection.has_value());
    CHECK(off_axis.image_point_px->x_px == projection->u_px);
    CHECK(off_axis.image_point_px->y_px == projection->v_px);
    CHECK(off_axis.image_point_px->x_px > camera.cx_px());  // +Y world -> image right
    CHECK(off_axis.image_point_px->y_px < camera.cy_px());  // +Z world -> image up

    const auto outside = fsoc::observe_beacon(camera, {100.0, 60.0, 0.0});
    CHECK(outside.status == ObservationStatus::OutsideFieldOfView);
    CHECK(!outside.visible());
    CHECK(!outside.image_point_px.has_value());

    const auto behind = fsoc::observe_beacon(camera, {-10.0, 0.0, 0.0});
    CHECK(behind.status == ObservationStatus::BehindCamera);
    CHECK(!behind.image_point_px.has_value());

    // Depth test wins over the FOV test: behind AND far off-axis -> BehindCamera.
    const auto behind_and_off = fsoc::observe_beacon(camera, {-10.0, 50.0, 0.0});
    CHECK(behind_and_off.status == ObservationStatus::BehindCamera);
}

// ---- 13. Step 1 projection behaviour unchanged --------------------

void test_step1_projection_regression() {
    const PanTiltCamera camera{CameraConfig{}};
    CHECK_NEAR(camera.cx_px(), 320.0, 1e-12);
    CHECK_NEAR(camera.cy_px(), 240.0, 1e-12);

    const auto centre = camera.project({100.0, 0.0, 0.0});
    CHECK(centre.has_value());
    CHECK_NEAR(centre->u_px, camera.cx_px(), 1e-10);
    CHECK_NEAR(centre->v_px, camera.cy_px(), 1e-10);

    CHECK(camera.project({100.0, 5.0, 0.0})->u_px > camera.cx_px());
    CHECK(camera.project({100.0, 0.0, 5.0})->v_px < camera.cy_px());
    CHECK(!camera.project({-10.0, 0.0, 0.0}).has_value());
    CHECK(!camera.project({100.0, 50.0, 0.0}).has_value());

    const auto [az, el] = camera.pixel_error_to_angles(50.0, -25.0);
    CHECK(az > 0.0);
    CHECK(el > 0.0);
}

// ---- 14. Step 2 trajectory behaviour unchanged -------------------

void test_step2_trajectory_regression() {
    const fsoc::LinearTrajectory linear{{100.0, 0.0, 2.0}, {0.0, 1.0, 0.2}};
    const auto s = linear.state_at(2.0);
    CHECK_NEAR(s.position_m.x, 100.0, 1e-12);
    CHECK_NEAR(s.position_m.y, 2.0, 1e-12);
    CHECK_NEAR(s.position_m.z, 2.4, 1e-12);
    CHECK(s.velocity_mps.y == 1.0);
    CHECK(s.velocity_mps.z == 0.2);

    const fsoc::StationaryTrajectory stat{{7.0, 8.0, 9.0}};
    const auto ss = stat.state_at(12.0);
    CHECK(ss.position_m.x == 7.0);
    CHECK(ss.velocity_mps.x == 0.0);
    CHECK(ss.velocity_mps.y == 0.0);
    CHECK(ss.velocity_mps.z == 0.0);

    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {0.0, 0.0, 0.0};
    p.amplitude_m = {0.0, 10.0, 0.0};
    p.frequency_hz = {0.0, 0.25, 0.0};  // omega_y = pi/2 rad/s
    const fsoc::SinusoidalTrajectory wave{p};
    CHECK_NEAR(wave.state_at(1.0).position_m.y, 10.0, 1e-9);  // 10*sin(pi/2)
    CHECK_NEAR(wave.state_at(1.0).velocity_mps.y, 0.0, 1e-9);
}

}  // namespace

int main() {
    test_image_centre_is_half_dimensions();
    test_centred_detection_is_zero_error();
    test_horizontal_sign_convention();
    test_vertical_sign_convention();
    test_matches_pinhole_geometry();
    test_no_detection_yields_no_error();
    test_non_finite_detection_is_rejected();
    test_invalid_config_is_rejected();
    test_repeated_calls_are_bit_identical();
    test_returned_values_are_finite();
    test_tracking_error_is_pose_independent();
    test_critical_review_scenario();
    test_observe_beacon_classification();
    test_step1_projection_regression();
    test_step2_trajectory_regression();

    if (failures == 0) {
        std::cout << "PASS: 15 Step-3 observation/tracking-error checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

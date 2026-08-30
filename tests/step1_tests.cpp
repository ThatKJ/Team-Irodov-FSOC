#include <cmath>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string_view>

#include "fsoc/camera.hpp"
#include "fsoc/config.hpp"
#include "fsoc/environment.hpp"
#include "fsoc/geometry.hpp"

namespace {

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
    if (std::abs(actual - expected) > tolerance) {
        ++failures;
        std::cerr << "FAIL line " << line << ": " << expression
                  << " actual=" << actual << " expected=" << expected
                  << " tol=" << tolerance << '\n';
    }
}

#define CHECK(expr) check((expr), #expr, __LINE__)
#define CHECK_NEAR(actual, expected, tol) check_near((actual), (expected), (tol), #actual " ~= " #expected, __LINE__)

void test_wrap_pi() {
    using fsoc::wrap_pi;
    constexpr double pi = std::numbers::pi_v<double>;
    CHECK_NEAR(wrap_pi(0.0), 0.0, 1e-12);
    CHECK_NEAR(wrap_pi(2.0 * pi), 0.0, 1e-12);
    CHECK_NEAR(wrap_pi(pi), -pi, 1e-12);
}

void test_camera_basis_zero() {
    const auto basis = fsoc::camera_basis(0.0, 0.0);
    CHECK_NEAR(basis.forward.x, 1.0, 1e-12);
    CHECK_NEAR(basis.forward.y, 0.0, 1e-12);
    CHECK_NEAR(basis.forward.z, 0.0, 1e-12);
    CHECK_NEAR(basis.right.x, 0.0, 1e-12);
    CHECK_NEAR(basis.right.y, 1.0, 1e-12);
    CHECK_NEAR(basis.up.z, 1.0, 1e-12);
}

void test_ideal_angles() {
    const auto [pan, tilt] = fsoc::ideal_pan_tilt({0.0, 0.0, 0.0}, {10.0, 10.0, 0.0});
    CHECK_NEAR(fsoc::rad_to_deg(pan), 45.0, 1e-10);
    CHECK_NEAR(tilt, 0.0, 1e-12);
}

void test_center_projection() {
    fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    const auto projection = camera.project({100.0, 0.0, 0.0});
    CHECK(projection.has_value());
    CHECK_NEAR(projection->u_px, camera.cx_px(), 1e-10);
    CHECK_NEAR(projection->v_px, camera.cy_px(), 1e-10);
}

void test_right_and_up_projection_signs() {
    fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    const auto right = camera.project({100.0, 5.0, 0.0});
    const auto up = camera.project({100.0, 0.0, 5.0});
    CHECK(right.has_value());
    CHECK(up.has_value());
    CHECK(right->u_px > camera.cx_px());
    CHECK(up->v_px < camera.cy_px());
}

void test_behind_camera_rejected() {
    fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    CHECK(!camera.project({-10.0, 0.0, 0.0}).has_value());
}

void test_outside_fov_rejected() {
    fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    CHECK(!camera.project({100.0, 50.0, 0.0}).has_value());
}

void test_rate_saturation() {
    fsoc::CameraConfig config{};
    fsoc::PanTiltCamera camera{config};
    const auto applied = camera.step(
        fsoc::deg_to_rad(1000.0),
        fsoc::deg_to_rad(-1000.0),
        0.1);
    CHECK_NEAR(applied.pan_rate_rad_s, config.max_pan_rate_rad_s, 1e-12);
    CHECK_NEAR(applied.tilt_rate_rad_s, -config.max_tilt_rate_rad_s, 1e-12);
}

void test_tilt_limit() {
    fsoc::CameraConfig config{};
    fsoc::PanTiltCamera camera{config, {}, 0.0, fsoc::deg_to_rad(79.0)};
    [[maybe_unused]] const auto applied = camera.step(0.0, fsoc::deg_to_rad(30.0), 1.0);
    CHECK_NEAR(camera.tilt_rad(), config.max_tilt_rad, 1e-12);
}

void test_invalid_dt_throws() {
    fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    bool threw = false;
    try {
        [[maybe_unused]] const auto ignored = camera.step(0.0, 0.0, 0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

void test_pixel_angle_round_trip_sign() {
    fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    const auto [az_error, el_error] = camera.pixel_error_to_angles(50.0, -25.0);
    CHECK(az_error > 0.0);
    CHECK(el_error > 0.0);
}

void test_environment_observation() {
    fsoc::Environment env{
        fsoc::PanTiltCamera{fsoc::CameraConfig{}},
        fsoc::TargetState{{100.0, 0.0, 0.0}}};
    const auto observation = env.observe_ideal();
    CHECK(observation.has_value());
    CHECK_NEAR(observation->u_px, env.camera().cx_px(), 1e-10);
    env.set_target_position({-1.0, 0.0, 0.0});
    CHECK(!env.observe_ideal().has_value());
}

}  // namespace

int main() {
    test_wrap_pi();
    test_camera_basis_zero();
    test_ideal_angles();
    test_center_projection();
    test_right_and_up_projection_signs();
    test_behind_camera_rejected();
    test_outside_fov_rejected();
    test_rate_saturation();
    test_tilt_limit();
    test_invalid_dt_throws();
    test_pixel_angle_round_trip_sign();
    test_environment_observation();

    if (failures == 0) {
        std::cout << "PASS: 12 Step-1 checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

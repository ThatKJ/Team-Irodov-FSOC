// Step 6 deterministic unit checks: pan/tilt PID controller.
//
// Same lightweight harness as tests/step1_tests.cpp .. tests/step5_tests.cpp.
// OpenCV-free: fsoc_step6_tests links fsoc::control only.

#include <cmath>
#include <iostream>
#include <limits>
#include <string_view>
#include <utility>
#include <vector>

#include "fsoc/pid_controller.hpp"
#include "fsoc/tracking_error.hpp"

namespace {

using fsoc::ControlCommand;
using fsoc::PIDAxisConfig;
using fsoc::PIDController;
using fsoc::PIDControllerConfig;
using fsoc::TrackingError;

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

// ---- helpers -----------------------------------------------------

[[nodiscard]] TrackingError angular_error(const double pan_rad, const double tilt_rad) {
    TrackingError error{};
    error.angular.pan_rad = pan_rad;
    error.angular.tilt_rad = tilt_rad;
    return error;
}

// One axis config; the returned PIDControllerConfig uses it for both axes.
[[nodiscard]] PIDControllerConfig config_both(const PIDAxisConfig& axis) {
    return PIDControllerConfig{.pan = axis, .tilt = axis};
}

constexpr double kDt = 0.1;

// ---- 1..5. proportional sign + zero -----------------------------

void test_proportional_signs() {
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 2.0, .ki = 0.0, .kd = 0.0, .integral_limit = 1.0, .output_limit_rad_s = 10.0})};

    // (1)(2) positive error -> positive command
    const auto pos = pid.update(angular_error(0.05, 0.02), kDt);
    CHECK(pos.pan_rate_rad_s > 0.0);
    CHECK(pos.tilt_rate_rad_s > 0.0);

    // (3)(4) negative error -> negative command
    pid.reset();
    const auto neg = pid.update(angular_error(-0.05, -0.02), kDt);
    CHECK(neg.pan_rate_rad_s < 0.0);
    CHECK(neg.tilt_rate_rad_s < 0.0);

    // (5) zero error -> zero proportional output
    pid.reset();
    const auto zero = pid.update(angular_error(0.0, 0.0), kDt);
    CHECK(zero.pan_rate_rad_s == 0.0);
    CHECK(zero.tilt_rate_rad_s == 0.0);
}

// ---- 6. proportional magnitude = kp * error -----------------

void test_proportional_magnitude() {
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 2.0, .ki = 0.0, .kd = 0.0, .integral_limit = 1.0, .output_limit_rad_s = 10.0})};
    const auto command = pid.update(angular_error(0.03, 0.017), kDt);
    CHECK_NEAR(command.pan_rate_rad_s, 2.0 * 0.03, 1e-15);
    CHECK_NEAR(command.tilt_rate_rad_s, 2.0 * 0.017, 1e-15);
}

// ---- 7. integral accumulation is predictable ------------------

void test_integral_accumulation() {
    // ki only: output == ki * (e * dt * n) while below the integral clamp.
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 0.0, .ki = 3.0, .kd = 0.0, .integral_limit = 100.0, .output_limit_rad_s = 100.0})};
    constexpr double e = 0.02;
    for (int n = 1; n <= 5; ++n) {
        const auto command = pid.update(angular_error(e, 0.0), kDt);
        CHECK_NEAR(command.pan_rate_rad_s, 3.0 * (e * kDt * static_cast<double>(n)), 1e-12);
    }
}

// ---- 8. integral state cannot exceed the configured bound ----

void test_integral_clamp() {
    // ki only, tiny output headroom above ki*integral_limit. A long constant
    // error must plateau at ki*integral_limit and never exceed it.
    constexpr double ki = 2.0;
    constexpr double integral_limit = 0.3;
    PIDController pid{config_both(PIDAxisConfig{.kp = 0.0,
                                               .ki = ki,
                                               .kd = 0.0,
                                               .integral_limit = integral_limit,
                                               .output_limit_rad_s = 100.0})};
    const double plateau = ki * integral_limit;
    double last = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const auto command = pid.update(angular_error(0.5, -0.5), kDt);
        CHECK(command.pan_rate_rad_s <= plateau + 1e-12);
        CHECK(command.tilt_rate_rad_s >= -plateau - 1e-12);
        last = command.pan_rate_rad_s;
    }
    CHECK_NEAR(last, plateau, 1e-9);  // reached, not exceeded
}

// ---- 9 / 10. derivative behaviour -------------------------------

void test_derivative_behaviour() {
    // kd only.
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 0.0, .ki = 0.0, .kd = 4.0, .integral_limit = 1.0, .output_limit_rad_s = 1e9})};

    // (10) first sample: derivative contribution is zero even for a non-zero error.
    const auto first = pid.update(angular_error(0.5, -0.3), kDt);
    CHECK(first.pan_rate_rad_s == 0.0);
    CHECK(first.tilt_rate_rad_s == 0.0);

    // (9) an error step of +0.2 over dt -> derivative = 0.2/dt -> kd * 0.2/dt.
    const auto stepped = pid.update(angular_error(0.7, -0.3), kDt);
    CHECK_NEAR(stepped.pan_rate_rad_s, 4.0 * (0.2 / kDt), 1e-12);
    CHECK_NEAR(stepped.tilt_rate_rad_s, 0.0, 1e-12);  // tilt error unchanged -> derivative 0

    // steady error -> derivative back to zero.
    const auto steady = pid.update(angular_error(0.7, -0.3), kDt);
    CHECK_NEAR(steady.pan_rate_rad_s, 0.0, 1e-12);
}

// ---- 11. pan and tilt axes are independent -------------------

void test_axis_independence() {
    const PIDAxisConfig axis{
        .kp = 1.0, .ki = 0.7, .kd = 0.3, .integral_limit = 5.0, .output_limit_rad_s = 1e9};
    PIDController with_pan{config_both(axis)};
    PIDController without_pan{config_both(axis)};

    // Identical tilt error every frame; pan error differs. Tilt outputs must match.
    for (int i = 0; i < 25; ++i) {
        const auto a = with_pan.update(angular_error(0.1, 0.02), kDt);
        const auto b = without_pan.update(angular_error(0.0, 0.02), kDt);
        CHECK(a.tilt_rate_rad_s == b.tilt_rate_rad_s);
    }
    // ... and the pan channel of `with_pan` actually did something different.
    CHECK(with_pan.update(angular_error(0.1, 0.02), kDt).pan_rate_rad_s !=
          without_pan.update(angular_error(0.0, 0.02), kDt).pan_rate_rad_s);
}

// ---- 12..14. output saturation ------------------------------

void test_output_saturation() {
    constexpr double limit = 0.4;
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 5.0, .ki = 2.0, .kd = 1.0, .integral_limit = 10.0, .output_limit_rad_s = limit})};

    for (const auto [pan, tilt] : {std::pair{0.001, -0.001}, std::pair{0.05, 0.05},
                                   std::pair{50.0, -50.0}, std::pair{-1000.0, 1000.0},
                                   std::pair{0.0, 0.0}}) {
        const auto command = pid.update(angular_error(pan, tilt), kDt);
        CHECK(std::abs(command.pan_rate_rad_s) <= limit);
        CHECK(std::abs(command.tilt_rate_rad_s) <= limit);
    }

    // (13) large positive error saturates positive; (14) large negative saturates negative.
    pid.reset();
    const auto big_pos = pid.update(angular_error(100.0, 100.0), kDt);
    CHECK(big_pos.pan_rate_rad_s == limit);
    CHECK(big_pos.tilt_rate_rad_s == limit);
    pid.reset();
    const auto big_neg = pid.update(angular_error(-100.0, -100.0), kDt);
    CHECK(big_neg.pan_rate_rad_s == -limit);
    CHECK(big_neg.tilt_rate_rad_s == -limit);
}

// ---- anti-windup: bounded integral + prompt recovery ----------

void test_anti_windup() {
    // kp alone saturates, so conditional integration must block the integrator
    // from the very first sample -> instant recovery once the error clears.
    constexpr double limit = 0.2;
    PIDController pid{config_both(PIDAxisConfig{.kp = 4.0,
                                               .ki = 3.0,
                                               .kd = 0.0,
                                               .integral_limit = 5.0,
                                               .output_limit_rad_s = limit})};

    for (int i = 0; i < 500; ++i) {
        const auto command = pid.update(angular_error(0.5, -0.5), kDt);
        CHECK(std::abs(command.pan_rate_rad_s) <= limit);  // never exceeds
        CHECK(command.pan_rate_rad_s == limit);            // pinned at the limit
    }
    // Error clears: the command must come off saturation within a couple of frames.
    const auto r0 = pid.update(angular_error(0.0, 0.0), kDt);
    CHECK(std::abs(r0.pan_rate_rad_s) < limit);
    CHECK(r0.pan_rate_rad_s == 0.0);  // kp*0 + ki*0(held) + kd*0

    // Toy-plant recovery: a saturating initial error is driven down, not stuck.
    PIDController plant_pid{config_both(PIDAxisConfig{.kp = 4.0,
                                                     .ki = 3.0,
                                                     .kd = 0.1,
                                                     .integral_limit = 5.0,
                                                     .output_limit_rad_s = limit})};
    double e = 1.0;
    const double initial = e;
    bool came_off_saturation = false;
    for (int i = 0; i < 400; ++i) {
        const auto command = plant_pid.update(angular_error(e, 0.0), kDt);
        e -= command.pan_rate_rad_s * kDt;
        if (std::abs(command.pan_rate_rad_s) < limit - 1e-9) {
            came_off_saturation = true;
        }
    }
    CHECK(std::abs(e) < initial);       // error reduced
    CHECK(std::abs(e) < 0.1);           // substantially reduced
    CHECK(came_off_saturation);         // not stuck saturated forever
}

// ---- 15 / 16. reset clears integral and derivative state -----

void test_reset_semantics() {
    const PIDAxisConfig axis{
        .kp = 1.0, .ki = 1.0, .kd = 1.0, .integral_limit = 10.0, .output_limit_rad_s = 1e9};
    PIDController pid{config_both(axis)};

    // Wind up integral and set a previous error.
    for (int i = 0; i < 20; ++i) {
        (void)pid.update(angular_error(0.3, -0.4), kDt);
    }
    pid.reset();

    // A fresh controller and the reset one must produce identical first outputs.
    PIDController fresh{config_both(axis)};
    const auto a = pid.update(angular_error(0.1, 0.05), kDt);
    const auto b = fresh.update(angular_error(0.1, 0.05), kDt);
    CHECK(a.pan_rate_rad_s == b.pan_rate_rad_s);
    CHECK(a.tilt_rate_rad_s == b.tilt_rate_rad_s);

    // (16) derivative specifically: first post-reset update with kd-only is zero.
    PIDController kd_pid{config_both(PIDAxisConfig{
        .kp = 0.0, .ki = 0.0, .kd = 2.0, .integral_limit = 1.0, .output_limit_rad_s = 1e9})};
    for (int i = 0; i < 5; ++i) {
        (void)kd_pid.update(angular_error(0.5, 0.5), kDt);
    }
    kd_pid.reset();
    const auto post = kd_pid.update(angular_error(0.9, 0.9), kDt);
    CHECK(post.pan_rate_rad_s == 0.0);
    CHECK(post.tilt_rate_rad_s == 0.0);
}

// ---- 17. invalid dt is rejected ---------------------------

void test_invalid_dt_rejected() {
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 1.0, .ki = 0.0, .kd = 0.0, .integral_limit = 1.0, .output_limit_rad_s = 1.0})};
    const auto error = angular_error(0.1, 0.1);
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();

    CHECK_THROWS(pid.update(error, 0.0));
    CHECK_THROWS(pid.update(error, -0.1));
    CHECK_THROWS(pid.update(error, nan_v));
    CHECK_THROWS(pid.update(error, inf_v));

    // State was not mutated by the throwing calls: the next valid update is a
    // first update (kd-only -> zero derivative term).
    PIDController kd_pid{config_both(PIDAxisConfig{
        .kp = 0.0, .ki = 0.0, .kd = 5.0, .integral_limit = 1.0, .output_limit_rad_s = 1e9})};
    CHECK_THROWS(kd_pid.update(angular_error(0.4, 0.4), -1.0));
    const auto first = kd_pid.update(angular_error(0.4, 0.4), kDt);
    CHECK(first.pan_rate_rad_s == 0.0);
    CHECK(first.tilt_rate_rad_s == 0.0);
}

// ---- 18. invalid config is rejected ----------------------

void test_invalid_config_rejected() {
    const PIDAxisConfig good{
        .kp = 1.0, .ki = 0.5, .kd = 0.1, .integral_limit = 0.5, .output_limit_rad_s = 0.5};
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();

    auto with_pan = [&](const PIDAxisConfig& pan) {
        return PIDControllerConfig{.pan = pan, .tilt = good};
    };

    PIDAxisConfig neg_kp = good;  neg_kp.kp = -0.1;
    PIDAxisConfig neg_ki = good;  neg_ki.ki = -0.1;
    PIDAxisConfig neg_kd = good;  neg_kd.kd = -0.1;
    PIDAxisConfig neg_il = good;  neg_il.integral_limit = -0.1;
    PIDAxisConfig zero_ol = good; zero_ol.output_limit_rad_s = 0.0;
    PIDAxisConfig neg_ol = good;  neg_ol.output_limit_rad_s = -1.0;
    PIDAxisConfig nan_kp = good;  nan_kp.kp = nan_v;
    PIDAxisConfig inf_ol = good;  inf_ol.output_limit_rad_s = inf_v;

    CHECK_THROWS(PIDController{with_pan(neg_kp)});
    CHECK_THROWS(PIDController{with_pan(neg_ki)});
    CHECK_THROWS(PIDController{with_pan(neg_kd)});
    CHECK_THROWS(PIDController{with_pan(neg_il)});
    CHECK_THROWS(PIDController{with_pan(zero_ol)});
    CHECK_THROWS(PIDController{with_pan(neg_ol)});
    CHECK_THROWS(PIDController{with_pan(nan_kp)});
    CHECK_THROWS(PIDController{with_pan(inf_ol)});

    // A bad tilt axis is caught too.
    const PIDControllerConfig bad_tilt{.pan = good, .tilt = neg_kd};
    CHECK_THROWS(PIDController{bad_tilt});

    // Defaults are valid.
    bool ok = true;
    try {
        const PIDController d{PIDControllerConfig{}};
        (void)d;
    } catch (...) {
        ok = false;
    }
    CHECK(ok);
}

// ---- 19. non-finite TrackingError is rejected ------------

void test_non_finite_error_rejected() {
    PIDController pid{config_both(PIDAxisConfig{
        .kp = 1.0, .ki = 0.0, .kd = 0.0, .integral_limit = 1.0, .output_limit_rad_s = 1.0})};
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();

    CHECK_THROWS(pid.update(angular_error(nan_v, 0.0), kDt));
    CHECK_THROWS(pid.update(angular_error(0.0, inf_v), kDt));

    TrackingError bad_pixel{};
    bad_pixel.pixel.x_px = nan_v;
    CHECK_THROWS(pid.update(bad_pixel, kDt));
}

// ---- 20. deterministic command sequence -----------------

void test_deterministic_sequence() {
    const PIDAxisConfig axis{
        .kp = 1.7, .ki = 0.6, .kd = 0.2, .integral_limit = 0.4, .output_limit_rad_s = 0.5};
    const std::vector<std::pair<double, double>> inputs{
        {0.08, -0.03}, {0.06, -0.02}, {0.03, 0.00}, {0.10, -0.05},
        {0.02, 0.04},  {-0.05, 0.06}, {-0.20, 0.20}, {0.00, 0.00}};

    auto run = [&]() {
        PIDController pid{config_both(axis)};
        std::vector<ControlCommand> out;
        for (const auto& [pan, tilt] : inputs) {
            out.push_back(pid.update(angular_error(pan, tilt), kDt));
        }
        return out;
    };

    const auto a = run();
    const auto b = run();
    CHECK(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].pan_rate_rad_s == b[i].pan_rate_rad_s);
        CHECK(a[i].tilt_rate_rad_s == b[i].tilt_rate_rad_s);
    }
}

// ---- control sign sanity (mandatory manual check) ------------

void test_manual_sign_check() {
    PIDController pid{PIDControllerConfig{}};  // baseline placeholder gains (kp,ki,kd > 0)

    const auto up_right = pid.update(angular_error(0.05, 0.02), kDt);
    CHECK(up_right.pan_rate_rad_s > 0.0);   // target RIGHT -> PAN RIGHT
    CHECK(up_right.tilt_rate_rad_s > 0.0);  // target ABOVE -> TILT UP

    pid.reset();
    const auto down_left = pid.update(angular_error(-0.05, -0.02), kDt);
    CHECK(down_left.pan_rate_rad_s < 0.0);
    CHECK(down_left.tilt_rate_rad_s < 0.0);
}

// ---- 21..25. prior steps remain green (behavioural spot checks) ---

void test_zero_control_command_helper() {
    constexpr auto zero = fsoc::zero_control_command();
    CHECK(zero.pan_rate_rad_s == 0.0);
    CHECK(zero.tilt_rate_rad_s == 0.0);
}

}  // namespace

int main() {
    test_proportional_signs();
    test_proportional_magnitude();
    test_integral_accumulation();
    test_integral_clamp();
    test_derivative_behaviour();
    test_axis_independence();
    test_output_saturation();
    test_anti_windup();
    test_reset_semantics();
    test_invalid_dt_rejected();
    test_invalid_config_rejected();
    test_non_finite_error_rejected();
    test_deterministic_sequence();
    test_manual_sign_check();
    test_zero_control_command_helper();

    if (failures == 0) {
        std::cout << "PASS: 15 Step-6 PID controller checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

// Step 2 deterministic unit checks for the target trajectory engine.
//
// Same lightweight harness as tests/step1_tests.cpp (no external framework).
// All expected values are analytic; sinusoidal phases are chosen so sin/cos
// land on 0 or +/-1.

#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "fsoc/trajectory.hpp"

namespace {

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

constexpr double kPi = std::numbers::pi_v<double>;

// ---- Stationary --------------------------------------------------------

void test_stationary_initial_and_hold() {
    const Vec3 p0{10.0, -5.0, 3.0};
    const fsoc::StationaryTrajectory traj{p0};

    // (1) at t=0 the state equals the initial position.
    const auto s0 = traj.state_at(0.0);
    CHECK_NEAR(s0.position_m.x, p0.x, 1e-12);
    CHECK_NEAR(s0.position_m.y, p0.y, 1e-12);
    CHECK_NEAR(s0.position_m.z, p0.z, 1e-12);

    // (2) position is unchanged at later times, (3) velocity is exactly zero.
    for (const double t : {0.5, 10.0, 1000.0, 86400.0}) {
        const auto s = traj.state_at(t);
        CHECK(s.position_m.x == p0.x);
        CHECK(s.position_m.y == p0.y);
        CHECK(s.position_m.z == p0.z);
        CHECK(s.velocity_mps.x == 0.0);
        CHECK(s.velocity_mps.y == 0.0);
        CHECK(s.velocity_mps.z == 0.0);
    }
}

// ---- Linear constant velocity -------------------------------------

void test_linear_follows_p0_plus_vt() {
    const Vec3 p0{100.0, 0.0, 2.0};
    const Vec3 v{0.0, 1.0, 0.2};
    const fsoc::LinearTrajectory traj{p0, v};

    // (4) at t=0 equals the initial position.
    const auto s0 = traj.state_at(0.0);
    CHECK_NEAR(s0.position_m.x, 100.0, 1e-12);
    CHECK_NEAR(s0.position_m.y, 0.0, 1e-12);
    CHECK_NEAR(s0.position_m.z, 2.0, 1e-12);

    // (5) position follows p0 + v*t, (6) velocity stays constant.
    for (const double t : {0.0, 1.0, 3.0, 7.5, 123.4}) {
        const auto s = traj.state_at(t);
        CHECK_NEAR(s.position_m.x, p0.x + v.x * t, 1e-9);
        CHECK_NEAR(s.position_m.y, p0.y + v.y * t, 1e-9);
        CHECK_NEAR(s.position_m.z, p0.z + v.z * t, 1e-9);
        CHECK(s.velocity_mps.x == v.x);
        CHECK(s.velocity_mps.y == v.y);
        CHECK(s.velocity_mps.z == v.z);
    }

    // Explicit hand value at t = 2 s: (100, 2, 2.4).
    const auto s2 = traj.state_at(2.0);
    CHECK_NEAR(s2.position_m.x, 100.0, 1e-12);
    CHECK_NEAR(s2.position_m.y, 2.0, 1e-12);
    CHECK_NEAR(s2.position_m.z, 2.4, 1e-12);
}

void test_linear_negative_velocity_components() {
    // (7) negative X/Y/Z velocity components behave correctly.
    const Vec3 p0{0.0, 0.0, 50.0};
    const Vec3 v{-2.0, -0.5, -3.0};
    const fsoc::LinearTrajectory traj{p0, v};

    const auto s = traj.state_at(10.0);
    CHECK_NEAR(s.position_m.x, -20.0, 1e-9);
    CHECK_NEAR(s.position_m.y, -5.0, 1e-9);
    CHECK_NEAR(s.position_m.z, 20.0, 1e-9);
    CHECK(s.velocity_mps.x == -2.0);
    CHECK(s.velocity_mps.y == -0.5);
    CHECK(s.velocity_mps.z == -3.0);

    // Position decreases monotonically on each negative axis.
    CHECK(traj.state_at(11.0).position_m.x < traj.state_at(10.0).position_m.x);
    CHECK(traj.state_at(11.0).position_m.y < traj.state_at(10.0).position_m.y);
    CHECK(traj.state_at(11.0).position_m.z < traj.state_at(10.0).position_m.z);
}

// ---- Sinusoidal --------------------------------------------------------

void test_sinusoidal_known_positions_and_velocities() {
    // center (10,20,30), amplitude (2,3,4), f_hz (0.25,0.5,1.0) -> omega (pi/2, pi, 2pi),
    // phase (0, pi/2, pi).
    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {10.0, 20.0, 30.0};
    p.amplitude_m = {2.0, 3.0, 4.0};
    p.frequency_hz = {0.25, 0.5, 1.0};
    p.phase_rad = {0.0, kPi / 2.0, kPi};
    const fsoc::SinusoidalTrajectory traj{p};

    // (8) positions at known phases, (9) analytical velocities at known phases.
    {  // t = 0
        const auto s = traj.state_at(0.0);
        CHECK_NEAR(s.position_m.x, 10.0, 1e-9);          // 10 + 2*sin(0)
        CHECK_NEAR(s.position_m.y, 23.0, 1e-9);          // 20 + 3*sin(pi/2)
        CHECK_NEAR(s.position_m.z, 30.0, 1e-9);          // 30 + 4*sin(pi)
        CHECK_NEAR(s.velocity_mps.x, kPi, 1e-9);         // 2*(pi/2)*cos(0)
        CHECK_NEAR(s.velocity_mps.y, 0.0, 1e-9);         // 3*pi*cos(pi/2)
        CHECK_NEAR(s.velocity_mps.z, -8.0 * kPi, 1e-9);  // 4*(2pi)*cos(pi)
    }
    {  // t = 1
        const auto s = traj.state_at(1.0);
        CHECK_NEAR(s.position_m.x, 12.0, 1e-9);          // 10 + 2*sin(pi/2)
        CHECK_NEAR(s.position_m.y, 17.0, 1e-9);          // 20 + 3*sin(3pi/2)
        CHECK_NEAR(s.position_m.z, 30.0, 1e-9);          // 30 + 4*sin(3pi)
        CHECK_NEAR(s.velocity_mps.x, 0.0, 1e-9);         // 2*(pi/2)*cos(pi/2)
        CHECK_NEAR(s.velocity_mps.y, 0.0, 1e-9);         // 3*pi*cos(3pi/2)
        CHECK_NEAR(s.velocity_mps.z, -8.0 * kPi, 1e-9);  // 4*(2pi)*cos(3pi)
    }
    {  // t = 2
        const auto s = traj.state_at(2.0);
        CHECK_NEAR(s.position_m.x, 10.0, 1e-9);          // 10 + 2*sin(pi)
        CHECK_NEAR(s.position_m.y, 23.0, 1e-9);          // 20 + 3*sin(5pi/2)
        CHECK_NEAR(s.position_m.z, 30.0, 1e-9);          // 30 + 4*sin(5pi)
        CHECK_NEAR(s.velocity_mps.x, -kPi, 1e-9);        // 2*(pi/2)*cos(pi)
        CHECK_NEAR(s.velocity_mps.y, 0.0, 1e-9);         // 3*pi*cos(5pi/2)
        CHECK_NEAR(s.velocity_mps.z, -8.0 * kPi, 1e-9);  // 4*(2pi)*cos(5pi)
    }
}

void test_sinusoidal_velocity_matches_central_difference() {
    // Independent cross-check that the analytic velocity is the true derivative
    // for arbitrary (non-special) phase arguments.
    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {5.0, -4.0, 12.0};
    p.amplitude_m = {1.5, 2.5, 0.75};
    p.frequency_hz = {0.3, 0.7, 1.1};
    p.phase_rad = {0.4, -1.1, 2.2};
    const fsoc::SinusoidalTrajectory traj{p};

    constexpr double h = 1e-4;
    for (const double t : {0.5, 1.3, 4.0, 9.9}) {
        const auto lo = traj.state_at(t - h);
        const auto hi = traj.state_at(t + h);
        const auto mid = traj.state_at(t);
        CHECK_NEAR(mid.velocity_mps.x, (hi.position_m.x - lo.position_m.x) / (2.0 * h), 1e-5);
        CHECK_NEAR(mid.velocity_mps.y, (hi.position_m.y - lo.position_m.y) / (2.0 * h), 1e-5);
        CHECK_NEAR(mid.velocity_mps.z, (hi.position_m.z - lo.position_m.z) / (2.0 * h), 1e-5);
    }
}

void test_sinusoidal_zero_amplitude_axis_is_fixed() {
    // (10) zero-amplitude axes stay pinned to center with zero velocity.
    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {7.0, 8.0, 9.0};
    p.amplitude_m = {0.0, 5.0, 0.0};
    p.frequency_hz = {2.0, 0.5, 3.0};
    p.phase_rad = {1.0, 0.0, -2.0};
    const fsoc::SinusoidalTrajectory traj{p};

    for (const double t : {0.0, 0.25, 1.0, 5.0, 50.0}) {
        const auto s = traj.state_at(t);
        CHECK(s.position_m.x == 7.0);
        CHECK(s.position_m.z == 9.0);
        CHECK(s.velocity_mps.x == 0.0);
        CHECK(s.velocity_mps.z == 0.0);
    }
    // The active (non-zero amplitude) axis actually moves: omega_y = pi rad/s, so
    // t = 0.5 s advances its phase by pi/2 -> y = 8 + 5*sin(pi/2) = 13 m.
    CHECK_NEAR(traj.state_at(0.0).position_m.y, 8.0, 1e-9);
    CHECK_NEAR(traj.state_at(0.5).position_m.y, 13.0, 1e-9);
}

void test_sinusoidal_zero_frequency_axis_is_constant() {
    // (11) zero-frequency axes yield a constant offset center + A*sin(phase), velocity 0.
    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {0.0, 0.0, 0.0};
    p.amplitude_m = {4.0, 3.0, 2.0};
    p.frequency_hz = {0.0, 0.0, 0.5};
    p.phase_rad = {kPi / 2.0, 0.0, 0.0};
    const fsoc::SinusoidalTrajectory traj{p};

    for (const double t : {0.0, 1.0, 10.0, 100.0}) {
        const auto s = traj.state_at(t);
        CHECK_NEAR(s.position_m.x, 4.0, 1e-9);  // 0 + 4*sin(pi/2)
        CHECK_NEAR(s.position_m.y, 0.0, 1e-9);  // 0 + 3*sin(0)
        CHECK(s.velocity_mps.x == 0.0);
        CHECK(s.velocity_mps.y == 0.0);
    }
}

// ---- Determinism, finiteness, and the time contract ------------------

void test_repeated_calls_are_bit_identical() {
    // (12) repeated state_at(t) calls return identical values.
    const fsoc::LinearTrajectory line{{1.0, 2.0, 3.0}, {0.1, -0.2, 0.3}};

    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {1.0, 2.0, 3.0};
    p.amplitude_m = {1.0, 2.0, 3.0};
    p.frequency_hz = {0.1, 0.2, 0.3};
    p.phase_rad = {0.5, 1.0, 1.5};
    const fsoc::SinusoidalTrajectory wave{p};

    for (const double t : {0.0, 0.333, 2.5, 7.7, 42.0}) {
        const auto a = wave.state_at(t);
        const auto b = wave.state_at(t);
        CHECK(a.position_m.x == b.position_m.x);
        CHECK(a.position_m.y == b.position_m.y);
        CHECK(a.position_m.z == b.position_m.z);
        CHECK(a.velocity_mps.x == b.velocity_mps.x);
        CHECK(a.velocity_mps.y == b.velocity_mps.y);
        CHECK(a.velocity_mps.z == b.velocity_mps.z);

        const auto c = line.state_at(t);
        const auto d = line.state_at(t);
        CHECK(c.position_m.x == d.position_m.x);
        CHECK(c.position_m.y == d.position_m.y);
        CHECK(c.position_m.z == d.position_m.z);
    }
}

void test_returned_values_are_finite() {
    // (13) normal returned values are finite for every trajectory type.
    const fsoc::StationaryTrajectory stat{{3.0, 4.0, 5.0}};
    const fsoc::LinearTrajectory line{{0.0, 0.0, 0.0}, {1.0, -2.0, 3.5}};

    fsoc::SinusoidalTrajectory::Parameters p{};
    p.center_position_m = {10.0, 10.0, 10.0};
    p.amplitude_m = {5.0, 4.0, 3.0};
    p.frequency_hz = {0.2, 0.5, 1.3};
    p.phase_rad = {0.1, 0.2, 0.3};
    const fsoc::SinusoidalTrajectory wave{p};

    for (const double t : {0.0, 0.01, 1.0, 13.37, 250.0, 3600.0}) {
        for (const fsoc::Trajectory* traj :
             std::initializer_list<const fsoc::Trajectory*>{&stat, &line, &wave}) {
            const auto s = traj->state_at(t);
            CHECK(std::isfinite(s.position_m.x));
            CHECK(std::isfinite(s.position_m.y));
            CHECK(std::isfinite(s.position_m.z));
            CHECK(std::isfinite(s.velocity_mps.x));
            CHECK(std::isfinite(s.velocity_mps.y));
            CHECK(std::isfinite(s.velocity_mps.z));
        }
    }
}

void test_negative_query_time_is_rejected() {
    // (14) negative simulation time violates the contract -> std::invalid_argument.
    const fsoc::StationaryTrajectory stat{{1.0, 1.0, 1.0}};
    const fsoc::LinearTrajectory line{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};

    fsoc::SinusoidalTrajectory::Parameters p{};
    p.amplitude_m = {1.0, 1.0, 1.0};
    p.frequency_hz = {1.0, 1.0, 1.0};
    const fsoc::SinusoidalTrajectory wave{p};

    CHECK_THROWS(stat.state_at(-1.0));
    CHECK_THROWS(line.state_at(-0.001));
    CHECK_THROWS(wave.state_at(-1e9));
}

void test_non_finite_query_time_is_rejected() {
    // (15) NaN / +Inf / -Inf simulation time -> std::invalid_argument (never NaN output).
    const fsoc::LinearTrajectory line{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
    const double nan_time = std::numeric_limits<double>::quiet_NaN();
    const double pos_inf = std::numeric_limits<double>::infinity();

    CHECK_THROWS(line.state_at(nan_time));
    CHECK_THROWS(line.state_at(pos_inf));
    CHECK_THROWS(line.state_at(-pos_inf));
}

void test_constructor_input_validation() {
    // Deliberate rejection of invalid constructor parameters (see trajectory.hpp policy).
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();

    CHECK_THROWS((fsoc::StationaryTrajectory{{nan_v, 0.0, 0.0}}));
    CHECK_THROWS((fsoc::LinearTrajectory{{0.0, inf_v, 0.0}, {0.0, 0.0, 0.0}}));
    CHECK_THROWS((fsoc::LinearTrajectory{{0.0, 0.0, 0.0}, {0.0, 0.0, nan_v}}));

    {  // non-finite sinusoidal field
        fsoc::SinusoidalTrajectory::Parameters p{};
        p.amplitude_m = {1.0, 1.0, inf_v};
        CHECK_THROWS((fsoc::SinusoidalTrajectory{p}));
    }
    {  // negative frequency
        fsoc::SinusoidalTrajectory::Parameters p{};
        p.frequency_hz = {1.0, -0.5, 1.0};
        CHECK_THROWS((fsoc::SinusoidalTrajectory{p}));
    }
    {  // negative amplitude
        fsoc::SinusoidalTrajectory::Parameters p{};
        p.amplitude_m = {-1.0, 0.0, 0.0};
        CHECK_THROWS((fsoc::SinusoidalTrajectory{p}));
    }
    {  // all-zero parameters are degenerate but well-defined (no throw)
        fsoc::SinusoidalTrajectory::Parameters p{};
        bool ok = true;
        try {
            const fsoc::SinusoidalTrajectory traj{p};
            const auto s = traj.state_at(5.0);
            ok = (s.position_m.x == 0.0 && s.velocity_mps.x == 0.0);
        } catch (...) {
            ok = false;
        }
        CHECK(ok);
    }
}

}  // namespace

int main() {
    test_stationary_initial_and_hold();
    test_linear_follows_p0_plus_vt();
    test_linear_negative_velocity_components();
    test_sinusoidal_known_positions_and_velocities();
    test_sinusoidal_velocity_matches_central_difference();
    test_sinusoidal_zero_amplitude_axis_is_fixed();
    test_sinusoidal_zero_frequency_axis_is_constant();
    test_repeated_calls_are_bit_identical();
    test_returned_values_are_finite();
    test_negative_query_time_is_rejected();
    test_non_finite_query_time_is_rejected();
    test_constructor_input_validation();

    if (failures == 0) {
        std::cout << "PASS: 12 Step-2 trajectory checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

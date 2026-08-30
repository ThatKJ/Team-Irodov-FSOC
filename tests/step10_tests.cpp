// Step 10 deterministic unit checks: baseline acceptance / validation suite.
//
// Same lightweight harness as tests/step1_tests.cpp .. tests/step9_tests.cpp.
// Links fsoc::validation (transitively simulation/telemetry/visualization + OpenCV).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/simulation_runner.hpp"
#include "fsoc/telemetry.hpp"
#include "fsoc/trajectory.hpp"
#include "fsoc/validation.hpp"

namespace {

using fsoc::AcceptanceCheck;
using fsoc::evaluate_passed;
using fsoc::ValidationResult;
using fsoc::ValidationScenarioId;
using fsoc::ValidationSuite;
using fsoc::ValidationSuiteResult;

int failures = 0;

void check(const bool condition, const std::string_view expression, const int line) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL line " << line << ": " << expression << '\n';
    }
}

#define CHECK(expr) check((expr), #expr, __LINE__)

[[nodiscard]] const AcceptanceCheck* find_check(const ValidationResult& r,
                                                const std::string_view needle) {
    for (const AcceptanceCheck& c : r.checks) {
        if (c.name.find(needle) != std::string::npos) {
            return &c;
        }
    }
    return nullptr;
}

[[nodiscard]] bool check_passed(const ValidationResult& r, const std::string_view needle) {
    const AcceptanceCheck* c = find_check(r, needle);
    return c != nullptr && c->passed;
}

[[nodiscard]] const ValidationResult& scenario(const ValidationSuiteResult& s,
                                               const ValidationScenarioId id) {
    for (const ValidationResult& r : s.scenarios) {
        if (r.id == id) {
            return r;
        }
    }
    static const ValidationResult kEmpty{};
    return kEmpty;
}

// The whole suite is run ONCE (no filesystem output) and shared by every test.
const ValidationSuiteResult& suite_result() {
    static const ValidationSuiteResult result = [] {
        ValidationSuite suite{{}};  // empty evidence dir -> no CSV/PNG/report I/O
        return suite.run_all();
    }();
    return result;
}

// ---- 1..3. nominal scenarios pass every gate -------------------

void test_nominal_scenarios_pass() {
    const auto& s = suite_result();
    CHECK(scenario(s, ValidationScenarioId::StaticAcquisition).passed);       // (1)
    CHECK(scenario(s, ValidationScenarioId::SlowLinearTracking).passed);      // (2)
    CHECK(scenario(s, ValidationScenarioId::SinusoidalTracking).passed);      // (3)
}

// ---- 4 / 5. near-FOV-edge starts inside FOV and converges -----

void test_fov_edge() {
    const ValidationResult& r = scenario(suite_result(), ValidationScenarioId::NearFovEdgeAcquisition);
    CHECK(r.passed);
    CHECK(check_passed(r, "initially visible"));   // (4) starts inside FOV
    CHECK(check_passed(r, "initially detected"));
    CHECK(check_passed(r, "command points toward target"));
    CHECK(check_passed(r, "no immediate target loss"));
    CHECK(check_passed(r, "converges"));           // (5)
    CHECK(check_passed(r, "final angular error"));
}

// ---- 6 / 7 / 8. actuator saturation --------------------------

void test_actuator_saturation() {
    const ValidationResult& r = scenario(suite_result(), ValidationScenarioId::ActuatorSaturation);
    CHECK(r.passed);
    const AcceptanceCheck* sat = find_check(r, "saturated frames occur");
    CHECK(sat != nullptr && sat->passed && sat->actual > 2.0);   // (6) actually saturates
    CHECK(r.metrics.command_saturation_fraction > 0.0);
    CHECK(check_passed(r, "max applied rate"));                  // (7) actuator rate respected
    CHECK(check_passed(r, "max command rate"));                  //     PID limit respected
    CHECK(check_passed(r, "comes off the limit"));               // (8) saturation clears
    CHECK(check_passed(r, "tracking error reduced"));
    CHECK(check_passed(r, "final angular error"));
}

// ---- 9..12. target loss / re-entry --------------------------

void test_loss_and_reentry() {
    const ValidationResult& r = scenario(suite_result(), ValidationScenarioId::TargetLossAndReentry);
    CHECK(r.passed);
    CHECK(r.metrics.lost_frames > 0);
    CHECK(check_passed(r, "TRACKING -> TARGET LOST transition"));      // (9)
    CHECK(check_passed(r, "TARGET LOST -> TRACKING transition"));      // (10)
    CHECK(check_passed(r, "command exactly zero while lost"));         // (11)
    CHECK(check_passed(r, "camera pose held across consecutive lost frames"));  // (12)
    CHECK(check_passed(r, "control resumes"));
}

// ---- 13 / 14 / 15. closed loop beats open loop -------------

void test_closed_beats_open() {
    const ValidationResult& r = scenario(suite_result(), ValidationScenarioId::OpenLoopComparison);
    CHECK(r.passed);
    CHECK(r.has_open_loop_comparison);
    CHECK(check_passed(r, "closed detection > open detection"));   // (13)
    CHECK(check_passed(r, "closed RMS < open RMS"));

    const AcceptanceCheck* rms = find_check(r, "RMS improvement factor");
    CHECK(rms != nullptr && rms->passed && rms->actual >= 3.0);    // (14)

    const AcceptanceCheck* det = find_check(r, "detection improvement");
    CHECK(det != nullptr && det->passed && det->actual >= 20.0);   // (15)

    // sanity: the known baseline improvement is large (~11.8x), not marginal.
    CHECK(r.open_loop_metrics.rms_angular_error_rad >
          5.0 * r.metrics.rms_angular_error_rad);
}

// ---- 16. finite-value validation catches nothing bad --------

void test_no_nan_inf_anywhere() {
    for (const ValidationResult& r : suite_result().scenarios) {
        CHECK(check_passed(r, "all telemetry values finite"));
        CHECK(check_passed(r, "simulation timestamps monotonic"));
        CHECK(check_passed(r, "fixed-dt deviation"));
        CHECK(check_passed(r, "max command rate"));
        CHECK(check_passed(r, "max applied rate"));
        CHECK(check_passed(r, "target-loss semantics"));
    }
}

// ---- 17. deterministic replay ------------------------------

void test_determinism() {
    for (const ValidationResult& r : suite_result().scenarios) {
        CHECK(r.deterministic);
        CHECK(check_passed(r, "deterministic replay"));
    }

    // Direct: two independent runs of the same scenario -> identical metrics.
    ValidationSuite a{{}};
    ValidationSuite b{{}};
    const ValidationResult ra = a.run_static_acquisition();
    const ValidationResult rb = b.run_static_acquisition();
    CHECK(ra.metrics.frames == rb.metrics.frames);
    CHECK(ra.metrics.detected_frames == rb.metrics.detected_frames);
    CHECK(ra.metrics.rms_angular_error_rad == rb.metrics.rms_angular_error_rad);
    CHECK(ra.metrics.max_angular_error_rad == rb.metrics.max_angular_error_rad);
    CHECK(ra.metrics.p95_angular_error_rad == rb.metrics.p95_angular_error_rad);
    CHECK(ra.metrics.final_angular_error_rad == rb.metrics.final_angular_error_rad);
}

// ---- 18. the checker can actually FAIL --------------------

void test_failure_check_is_real() {
    ValidationSuite suite{{}};
    ValidationResult r = suite.run_static_acquisition();
    CHECK(r.passed);
    CHECK(evaluate_passed(r));

    // (a) inject an impossible mandatory check -> scenario must fail.
    AcceptanceCheck impossible{};
    impossible.name = "impossible gate (test)";
    impossible.actual = 1.0;
    impossible.limit = 0.0;
    impossible.unit = "";
    impossible.comparator = "<";
    impossible.passed = impossible.actual < impossible.limit;  // false
    CHECK(!impossible.passed);
    r.checks.push_back(impossible);
    CHECK(!evaluate_passed(r));

    // (b) tightening a real threshold past the actual value flips its result.
    ValidationResult r2 = suite.run_static_acquisition();
    for (AcceptanceCheck& c : r2.checks) {
        if (c.name.find("initial total angular error") != std::string::npos) {
            c.limit = 100.0;                    // require > 100 deg (impossible)
            c.passed = c.actual > c.limit;      // now false
        }
    }
    CHECK(!evaluate_passed(r2));
    r2.passed = evaluate_passed(r2);  // the suite recomputes .passed the same way
    CHECK(!r2.passed);

    // (c) a suite with one failing scenario has overall_passed == false.
    ValidationSuiteResult fake{};
    fake.scenarios.push_back(suite.run_static_acquisition());  // passes
    fake.scenarios.push_back(r2);                              // fails
    fake.overall_passed = std::all_of(fake.scenarios.begin(), fake.scenarios.end(),
                                      [](const ValidationResult& x) { return x.passed; });
    CHECK(!fake.overall_passed);
    CHECK(fake.scenarios.front().passed);  // the untouched one still passes
}

// ---- 19 / 20. suite shape + overall verdict --------------

void test_suite_shape_and_overall() {
    const auto& s = suite_result();
    CHECK(s.scenarios.size() == 7);                         // (19)
    CHECK(s.overall_passed);                                // (20) real system passes
    // overall is PASS iff every scenario passed.
    const bool all = std::all_of(s.scenarios.begin(), s.scenarios.end(),
                                 [](const ValidationResult& r) { return r.passed; });
    CHECK(s.overall_passed == all);
}

// ---- 21..29. prior steps remain green (spot checks) -----

void test_prior_steps_regression() {
    const fsoc::PanTiltCamera camera{fsoc::CameraConfig{}};
    const auto centre = camera.project({100.0, 0.0, 0.0});
    CHECK(centre.has_value());
    CHECK(std::abs(centre->u_px - camera.cx_px()) < 1e-9);

    const fsoc::LinearTrajectory lin{{100.0, 0.0, 2.0}, {0.0, 1.0, 0.2}};
    CHECK(std::abs(lin.state_at(2.0).position_m.z - 2.4) < 1e-12);

    CHECK(fsoc::observe_beacon(camera, {-10.0, 0.0, 0.0}).status ==
          fsoc::ObservationStatus::BehindCamera);

    fsoc::PIDController pid{fsoc::PIDControllerConfig{}};
    fsoc::TrackingError e{};
    e.angular.pan_rad = 0.05;
    e.angular.tilt_rad = 0.02;
    const auto cmd = pid.update(e, 0.02);
    CHECK(cmd.pan_rate_rad_s > 0.0 && cmd.tilt_rate_rad_s > 0.0);

    // Step 7/8 end to end.
    const fsoc::StationaryTrajectory st{fsoc::Vec3{100.0, 6.0, 4.0}};
    fsoc::SimulationRunner runner{fsoc::baseline_runner_config(), st};
    const fsoc::RecordedRun run = fsoc::run_and_record(runner, 3.0);
    const fsoc::BenchmarkMetrics bm =
        fsoc::compute_benchmark_metrics(run.telemetry, run.wall_execution_time_s);
    CHECK(bm.frames == 150);
    CHECK(fsoc::rad_to_deg(bm.final_angular_error_rad) < 0.05);

    // The baseline PID gains are untouched by Step 10.
    const fsoc::SimulationRunnerConfig base = fsoc::baseline_runner_config();
    CHECK(base.controller.pan.kp == 12.0);
    CHECK(base.controller.pan.ki == 0.0);
    CHECK(base.controller.pan.kd == 0.0);
}

}  // namespace

int main() {
    test_nominal_scenarios_pass();
    test_fov_edge();
    test_actuator_saturation();
    test_loss_and_reentry();
    test_closed_beats_open();
    test_no_nan_inf_anywhere();
    test_determinism();
    test_failure_check_is_real();
    test_suite_shape_and_overall();
    test_prior_steps_regression();

    if (failures == 0) {
        std::cout << "PASS: 10 Step-10 validation-suite checks passed.\n";
        return 0;
    }
    std::cerr << "FAILED: " << failures << " check(s).\n";
    return 1;
}

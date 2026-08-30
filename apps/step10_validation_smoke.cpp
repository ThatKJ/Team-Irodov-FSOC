// Step 10 smoke: baseline acceptance / validation suite.
//
// Runs the EXISTING v1 system across the named deterministic scenarios, checks
// the documented acceptance gates, writes CSV + PNG evidence and
// generated/step10/VALIDATION_REPORT.md, and prints a judge-friendly table.
// Ends with "STEP 10 BASELINE ACCEPTANCE: PASS" only if every mandatory
// scenario passes.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "fsoc/config.hpp"
#include "fsoc/validation.hpp"

namespace {

using fsoc::rad_to_deg;
using fsoc::ValidationResult;
using fsoc::ValidationScenarioId;

[[nodiscard]] std::string pct(const double fraction) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(1) << 100.0 * fraction << "%";
    return os.str();
}

[[nodiscard]] std::string deg4(const double rad) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(4) << rad_to_deg(rad);
    return os.str();
}

void print_row(const ValidationResult& r) {
    std::cout << "  " << std::left << std::setw(28) << r.scenario_name << std::right
              << std::setw(8) << pct(r.metrics.detection_fraction) << "  " << std::setw(8)
              << deg4(r.metrics.rms_angular_error_rad) << "  " << std::setw(8)
              << deg4(r.metrics.p95_angular_error_rad) << "  " << std::setw(8)
              << deg4(r.metrics.max_angular_error_rad) << "  " << std::setw(6)
              << r.metrics.lost_frames << "   " << (r.passed ? "PASS" : "FAIL") << '\n';
}

}  // namespace

int main() {
    using namespace fsoc;

    std::cout << "SIH26169 BASELINE VALIDATION  (v1_baseline candidate)\n";
    const SimulationRunnerConfig base = baseline_runner_config();
    std::cout << "  simulation " << std::fixed << std::setprecision(1) << 1.0 / base.timestep_s
              << " Hz (fixed dt = " << std::setprecision(3) << base.timestep_s << " s)  |  PID kp="
              << std::setprecision(1) << base.controller.pan.kp
              << " ki=" << base.controller.pan.ki << " kd=" << std::setprecision(2)
              << base.controller.pan.kd << "  |  FOV "
              << std::setprecision(1) << rad_to_deg(base.camera.hfov_rad) << " x "
              << rad_to_deg(base.camera.vfov_rad) << " deg\n\n";

    ValidationSuite suite{"generated/step10"};
    const auto start = std::chrono::steady_clock::now();
    const ValidationSuiteResult result = suite.run_all();
    const auto end = std::chrono::steady_clock::now();
    const double wall_s = std::chrono::duration<double>(end - start).count();

    std::cout << "  " << std::left << std::setw(28) << "SCENARIO" << std::right << std::setw(8)
              << "DETECT" << std::setw(10) << "RMS deg" << std::setw(10) << "P95 deg"
              << std::setw(10) << "MAX deg" << std::setw(7) << "LOST" << "   RESULT\n";
    for (const ValidationResult& r : result.scenarios) {
        print_row(r);
    }

    // Open vs closed detail.
    for (const ValidationResult& r : result.scenarios) {
        if (!r.has_open_loop_comparison) {
            continue;
        }
        const auto& o = r.open_loop_metrics;
        const auto& c = r.metrics;
        const double rms_x = c.rms_angular_error_rad > 0.0
                                 ? o.rms_angular_error_rad / c.rms_angular_error_rad
                                 : 0.0;
        std::cout << "\n  OPEN vs CLOSED (same trajectory):\n"
                  << "    detection : " << pct(o.detection_fraction) << " -> "
                  << pct(c.detection_fraction) << "   (+"
                  << std::fixed << std::setprecision(1)
                  << 100.0 * (c.detection_fraction - o.detection_fraction) << " pts)\n"
                  << "    RMS error : " << deg4(o.rms_angular_error_rad) << " deg -> "
                  << deg4(c.rms_angular_error_rad) << " deg   (x" << std::setprecision(1) << rms_x
                  << " better)\n"
                  << "    lost      : " << o.lost_frames << " -> " << c.lost_frames << " frames\n";
    }

    // Failed-check detail, if any.
    for (const ValidationResult& r : result.scenarios) {
        if (r.passed) {
            continue;
        }
        std::cout << "\n  FAILED CHECKS in \"" << r.scenario_name << "\":\n";
        for (const auto& c : r.checks) {
            if (!c.passed) {
                std::cout << "    - " << c.name << " : actual " << c.actual << " " << c.comparator
                          << " " << c.limit << " " << c.unit << "\n";
            }
        }
    }

    const bool report_ok = suite.write_report(result);
    std::cout << "\n  evidence dir : generated/step10/\n"
              << "  report       : generated/step10/VALIDATION_REPORT.md "
              << (report_ok ? "(written)" : "(FAILED)") << "\n"
              << "  suite wall time : " << std::setprecision(2) << wall_s
              << " s (informational; simulation is fixed 50 Hz regardless)\n";

    const std::size_t passed = static_cast<std::size_t>(std::count_if(
        result.scenarios.begin(), result.scenarios.end(),
        [](const ValidationResult& r) { return r.passed; }));
    std::cout << "\n  " << passed << " / " << result.scenarios.size() << " scenarios passed\n";

    const bool ok = result.overall_passed && report_ok;
    std::cout << "\nSTEP 10 BASELINE ACCEPTANCE: " << (ok ? "PASS" : "FAIL") << '\n';
    return ok ? 0 : 1;
}

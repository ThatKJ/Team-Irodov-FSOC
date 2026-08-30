#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "fsoc/simulation_runner.hpp"
#include "fsoc/telemetry.hpp"

namespace fsoc {

// ===========================================================================
// Baseline acceptance / validation suite  (Step 10 — an EVALUATION layer)
// ===========================================================================
//
// Runs the EXISTING v1 system across a fixed set of named deterministic
// scenarios and checks documented acceptance gates. It reads outputs
// (SimulationStepResult / TelemetryRecord / BenchmarkMetrics); it does not
// control the loop and never modifies the trajectory / detector / PID /
// renderer / camera / telemetry semantics. Gates are defined up front in
// docs/16_BASELINE_ACCEPTANCE.md, not derived from the run being scored.

enum class ValidationScenarioId {
    StaticAcquisition,
    SlowLinearTracking,
    SinusoidalTracking,
    NearFovEdgeAcquisition,
    ActuatorSaturation,
    TargetLossAndReentry,
    OpenLoopComparison,
};

[[nodiscard]] std::string_view to_string(ValidationScenarioId id) noexcept;

// One acceptance gate: `actual <comparator> limit`.
struct AcceptanceCheck {
    std::string name;
    bool passed{};
    double actual{};
    double limit{};
    std::string unit;
    std::string comparator;  // "<", "<=", ">", ">=", "==", "bool"
};

struct ValidationResult {
    ValidationScenarioId id{};
    std::string scenario_name;
    std::string description;

    BenchmarkMetrics metrics{};                        // closed-loop run for this scenario
    std::vector<AcceptanceCheck> checks;               // scenario + global gates
    bool passed{};                                     // == every check passed
    bool deterministic{};                              // ran twice: metrics + results identical

    std::filesystem::path csv_path;
    std::vector<std::filesystem::path> evidence_images;

    // Present only for the open-vs-closed scenario.
    BenchmarkMetrics open_loop_metrics{};
    bool has_open_loop_comparison{};
};

struct ValidationSuiteResult {
    std::vector<ValidationResult> scenarios;
    bool overall_passed{};
};

// True iff every check in `result.checks` passed. `run_*` sets `result.passed`
// with this; tests use it to prove the checker can FAIL.
[[nodiscard]] bool evaluate_passed(const ValidationResult& result);

// Global gates applied to every scenario (finite values, monotonic + fixed dt,
// command <= PID limit, applied <= actuator limit, target-loss semantics).
[[nodiscard]] std::vector<AcceptanceCheck> global_acceptance_checks(
    const std::vector<SimulationStepResult>& results,
    const std::vector<TelemetryRecord>& telemetry,
    const SimulationRunnerConfig& config);

class ValidationSuite {
public:
    // `evidence_dir` receives the CSVs / PNG evidence / VALIDATION_REPORT.md.
    // Pass an empty path to run with no filesystem output (used by tests).
    explicit ValidationSuite(std::filesystem::path evidence_dir = "generated/step10");

    [[nodiscard]] const std::filesystem::path& evidence_dir() const noexcept {
        return evidence_dir_;
    }

    [[nodiscard]] ValidationSuiteResult run_all();

    [[nodiscard]] ValidationResult run_static_acquisition();
    [[nodiscard]] ValidationResult run_slow_linear();
    [[nodiscard]] ValidationResult run_sinusoidal();
    [[nodiscard]] ValidationResult run_near_fov_edge();
    [[nodiscard]] ValidationResult run_actuator_saturation();
    [[nodiscard]] ValidationResult run_loss_and_reentry();
    [[nodiscard]] ValidationResult run_open_vs_closed();

    // Writes `<evidence_dir>/VALIDATION_REPORT.md` from an actual suite result.
    // No-op returning false when evidence_dir is empty.
    [[nodiscard]] bool write_report(const ValidationSuiteResult& result) const;

private:
    std::filesystem::path evidence_dir_;
};

}  // namespace fsoc

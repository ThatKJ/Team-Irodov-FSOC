// Step 8 smoke test: telemetry + benchmarking.
//
// Runs the Step-7 benchmark scenarios, exports one CSV per scenario into
// generated/, prints a comparison table, and reports wall-clock processing
// throughput (distinct from the fixed 50 Hz simulation rate). Headless, no GUI.

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/simulation_runner.hpp"
#include "fsoc/telemetry.hpp"
#include "fsoc/trajectory.hpp"

namespace {

using fsoc::BenchmarkMetrics;
using fsoc::CsvTelemetryLogger;
using fsoc::rad_to_deg;
using fsoc::RecordedRun;
using fsoc::SimulationRunner;
using fsoc::SimulationRunnerConfig;
using fsoc::Vec3;

[[nodiscard]] BenchmarkMetrics write_csv_and_measure(
    const std::string& csv_path,
    const RecordedRun& run) {
    CsvTelemetryLogger logger{std::filesystem::path{csv_path}};
    logger.write_header();
    for (const auto& record : run.telemetry) {
        logger.record(record);
    }
    const BenchmarkMetrics metrics =
        fsoc::compute_benchmark_metrics(run.telemetry, run.wall_execution_time_s);

    const auto& columns = CsvTelemetryLogger::column_names();
    std::cout << "  wrote " << logger.path().string() << "\n"
              << "    records = " << logger.records_written()
              << " , columns = " << columns.size() << "\n"
              << "    header  = " << columns.front() << ",...," << columns.back() << "\n";
    return metrics;
}

void print_table_row(const char* name, const BenchmarkMetrics& m) {
    std::cout << std::left << std::setw(22) << name << std::right << std::fixed
              << std::setprecision(1) << std::setw(8) << 100.0 * m.detection_fraction << "%  "
              << std::setprecision(4) << std::setw(9) << rad_to_deg(m.rms_angular_error_rad)
              << "  " << std::setw(9) << rad_to_deg(m.p95_angular_error_rad) << "  "
              << std::setw(9) << rad_to_deg(m.max_angular_error_rad) << '\n';
}

}  // namespace

int main() {
    using namespace fsoc;

    std::cout << "Step 8: telemetry + benchmarking (headless)\n";
    std::filesystem::create_directories("generated");
    const SimulationRunnerConfig base = baseline_runner_config();

    // ---- A. static acquisition ----
    std::cout << "\n[A] static acquisition\n";
    const StationaryTrajectory static_target{Vec3{100.0, 6.0, 4.0}};
    SimulationRunner static_runner{base, static_target};
    const RecordedRun static_run = run_and_record(static_runner, 5.0);
    const BenchmarkMetrics static_m =
        write_csv_and_measure("generated/step8_static.csv", static_run);

    // ---- B. linear tracking ----
    std::cout << "\n[B] linear tracking\n";
    const LinearTrajectory linear_target{Vec3{100.0, -8.0, -3.0}, Vec3{0.0, 2.0, 0.8}};
    SimulationRunner linear_runner{base, linear_target};
    const RecordedRun linear_run = run_and_record(linear_runner, 10.0);
    const BenchmarkMetrics linear_m =
        write_csv_and_measure("generated/step8_linear.csv", linear_run);

    // ---- sinusoidal trajectory shared by C / D / E ----
    SinusoidalTrajectory::Parameters sin_params{};
    sin_params.center_position_m = Vec3{100.0, 0.0, 3.0};
    sin_params.amplitude_m = Vec3{0.0, 22.0, 4.0};
    sin_params.frequency_hz = Vec3{0.0, 0.12, 0.09};
    const SinusoidalTrajectory sin_target{sin_params};

    SimulationRunnerConfig sin_cfg = base;
    sin_cfg.initial_tilt_rad = std::atan2(3.0, 100.0);

    // ---- E. sinusoidal closed loop ----
    std::cout << "\n[E] sinusoidal closed loop\n";
    SimulationRunner sin_closed_runner{sin_cfg, sin_target};
    const RecordedRun sin_closed_run = run_and_record(sin_closed_runner, 20.0);
    const BenchmarkMetrics sin_closed_m =
        write_csv_and_measure("generated/step8_sinusoidal.csv", sin_closed_run);

    // ---- D. sinusoidal open loop ----
    std::cout << "\n[D] sinusoidal open loop\n";
    SimulationRunnerConfig sin_open_cfg = sin_cfg;
    sin_open_cfg.control_enabled = false;
    SimulationRunner sin_open_runner{sin_open_cfg, sin_target};
    const RecordedRun sin_open_run = run_and_record(sin_open_runner, 20.0);
    const BenchmarkMetrics sin_open_m =
        write_csv_and_measure("generated/step8_open_loop.csv", sin_open_run);

    // ---- benchmark table ----
    std::cout << "\n=== BENCHMARK COMPARISON (angular error in deg) ===\n"
              << std::left << std::setw(22) << "SCENARIO" << std::right << std::setw(9)
              << "DETECTED" << std::setw(11) << "RMS ERR" << std::setw(11) << "P95 ERR"
              << std::setw(11) << "MAX ERR" << '\n';
    print_table_row("Static", static_m);
    print_table_row("Linear", linear_m);
    print_table_row("Sinusoidal Closed", sin_closed_m);
    print_table_row("Sinusoidal Open", sin_open_m);

    // ---- open vs closed ----
    const double det_improvement =
        sin_closed_m.detection_fraction / std::max(sin_open_m.detection_fraction, 1e-9);
    const double rms_improvement = (sin_closed_m.rms_angular_error_rad > 0.0)
                                       ? sin_open_m.rms_angular_error_rad /
                                             sin_closed_m.rms_angular_error_rad
                                       : std::nan("");
    std::cout << "\n=== OPEN vs CLOSED (same trajectory) ===\n"
              << std::fixed << std::setprecision(1)
              << "  Detection : " << 100.0 * sin_open_m.detection_fraction << "% -> "
              << 100.0 * sin_closed_m.detection_fraction << "%  (x" << std::setprecision(2)
              << det_improvement << ")\n"
              << std::setprecision(4)
              << "  RMS error : " << rad_to_deg(sin_open_m.rms_angular_error_rad) << " deg -> "
              << rad_to_deg(sin_closed_m.rms_angular_error_rad) << " deg  (x"
              << std::setprecision(1) << rms_improvement << " better)\n";

    // ---- processing throughput vs simulation rate ----
    const std::size_t total_frames = static_run.frames() + linear_run.frames() +
                                     sin_closed_run.frames() + sin_open_run.frames();
    const double total_wall = static_run.wall_execution_time_s + linear_run.wall_execution_time_s +
                              sin_closed_run.wall_execution_time_s +
                              sin_open_run.wall_execution_time_s;
    const double processing_fps = static_cast<double>(total_frames) / total_wall;
    std::cout << "\n=== PERFORMANCE (wall clock; NOT the simulation timestep) ===\n"
              << std::setprecision(2)
              << "  Simulation frequency : " << 1.0 / base.timestep_s << " Hz (fixed dt = "
              << std::setprecision(3) << base.timestep_s << " s)\n"
              << std::setprecision(0)
              << "  Processing throughput: " << processing_fps << " FPS  ("
              << std::setprecision(1) << processing_fps * base.timestep_s
              << "x real time over " << total_frames << " frames in " << std::setprecision(3)
              << total_wall << " s)\n"
              << "  Per-frame processing : " << std::setprecision(4)
              << 1e6 * total_wall / static_cast<double>(total_frames) << " us\n";

    // ---- gates ----
    const double static_final_deg = rad_to_deg(static_m.final_angular_error_rad);
    const bool ok = static_m.detected_frames >= static_m.frames - 1 && static_final_deg < 0.05 &&
                    linear_m.detection_fraction >= 0.95 &&
                    sin_closed_m.detection_fraction >= 0.95 &&
                    sin_closed_m.detection_fraction > sin_open_m.detection_fraction &&
                    sin_closed_m.rms_angular_error_rad < sin_open_m.rms_angular_error_rad;
    std::cout << "\nStep 8 smoke: " << (ok ? "PASS" : "FAIL") << '\n';
    return ok ? 0 : 1;
}

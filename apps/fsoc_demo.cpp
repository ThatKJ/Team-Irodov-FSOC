// fsoc_demo — human-friendly entry point for the frozen v1_baseline engine.
//
//   ./fsoc_demo static
//   ./fsoc_demo sinusoidal --csv generated/demo/sinusoidal.csv
//   ./fsoc_demo loss --duration 6
//   ./fsoc_demo open        ./fsoc_demo closed        ./fsoc_demo --help
//
// It only packages the validated system: DemoSession owns a SimulationRunner and
// the Step-8 telemetry conversion; this file adds no physics, no control, no
// networking. Core values are radians; the CLI prints degrees for humans.

#include <chrono>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "fsoc/config.hpp"
#include "fsoc/demo.hpp"
#include "fsoc/telemetry.hpp"

namespace {

using namespace fsoc;

int usage_error(const std::string& message) {
    std::cerr << "fsoc_demo: " << message << "\n\n" << demo_help_text() << '\n';
    return 2;
}

// Fixed-width degree string for the running status lines.
std::string deg(const double radians, const int precision, const int width) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << std::setw(width) << rad_to_deg(radians);
    return os.str();
}

void print_status_line(const DemoSnapshot& s) {
    std::cout << "t=" << std::fixed << std::setprecision(2) << std::setw(6) << s.simulation_time_s
              << "s  " << std::left << std::setw(11) << to_string(s.state) << std::right
              << "  tgt=(" << std::setprecision(1) << std::setw(7) << s.target.x_m << ","
              << std::setw(7) << s.target.y_m << "," << std::setw(6) << s.target.z_m << ")"
              << "  cam p/t=" << deg(s.camera.pan_rad, 2, 7) << "/" << deg(s.camera.tilt_rad, 2, 6)
              << " deg";

    if (s.detection.detected && s.detection.x_px.has_value() && s.detection.y_px.has_value()) {
        std::cout << "  det=(" << std::setprecision(1) << std::setw(6) << *s.detection.x_px << ","
                  << std::setw(6) << *s.detection.y_px << ")";
    } else {
        std::cout << "  det=(   --  ,   --  )";
    }

    if (s.tracking.total_error_rad.has_value()) {
        std::cout << "  angErr=" << deg(*s.tracking.total_error_rad, 4, 8) << " deg";
    } else {
        std::cout << "  angErr=      -- ";
    }

    std::cout << "  cmd p/t=" << deg(s.control.command_pan_rate_rad_s, 2, 7) << "/"
              << deg(s.control.command_tilt_rate_rad_s, 2, 6) << " deg/s";
    if (s.control.pan_saturated || s.control.tilt_saturated) {
        std::cout << "  [RATE LIMIT]";
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + (argc > 0 ? 1 : 0), argv + argc);

    for (const std::string& arg : args) {
        if (arg == "--help" || arg == "-h") {
            std::cout << demo_help_text() << '\n';
            return 0;
        }
    }

    std::optional<DemoScenario> scenario;
    std::optional<double> duration_override;
    std::string csv_path;
    bool quiet = false;

    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--quiet") {
            quiet = true;
        } else if (arg == "--duration") {
            if (i + 1 >= args.size()) {
                return usage_error("--duration needs a value");
            }
            const std::string& value = args[++i];
            try {
                std::size_t consumed = 0;
                const double seconds = std::stod(value, &consumed);
                if (consumed != value.size() || !(seconds > 0.0)) {
                    return usage_error("--duration must be a positive number of seconds");
                }
                duration_override = seconds;
            } catch (const std::exception&) {
                return usage_error("--duration must be a number");
            }
        } else if (arg == "--csv") {
            if (i + 1 >= args.size()) {
                return usage_error("--csv needs a path");
            }
            csv_path = args[++i];
        } else if (!arg.empty() && arg.front() == '-') {
            return usage_error("unknown option: " + arg);
        } else if (!scenario.has_value()) {
            scenario = parse_demo_scenario(arg);
            if (!scenario.has_value()) {
                return usage_error("unknown scenario: '" + arg + "'");
            }
        } else {
            return usage_error("unexpected extra argument: '" + arg + "'");
        }
    }

    if (!scenario.has_value()) {
        return usage_error("no scenario given");
    }

    DemoSession session{*scenario,
                        duration_override.value_or(demo_scenario_duration_s(*scenario))};

    std::optional<CsvTelemetryLogger> logger;
    if (!csv_path.empty()) {
        logger.emplace(csv_path);
        logger->write_header();
    }

    std::cout << "scenario : " << to_string(session.scenario()) << "  ("
              << demo_scenario_token(session.scenario()) << ")\n"
              << "detail   : " << demo_scenario_description(session.scenario()) << "\n"
              << "duration : " << std::fixed << std::setprecision(2) << session.duration_s()
              << " s  (" << session.total_frames() << " frames @ 50 Hz, dt = 0.02 s)\n"
              << "control  : "
              << (session.runner_config().control_enabled ? "ENABLED (closed loop)"
                                                          : "DISABLED (open loop)")
              << "\n\n";

    std::vector<TelemetryRecord> records;
    records.reserve(session.total_frames());

    constexpr std::size_t kPrintEvery = 25;  // 0.5 s at 50 Hz
    const auto wall_start = std::chrono::steady_clock::now();

    while (!session.finished()) {
        const DemoSnapshot snapshot = session.step();
        records.push_back(session.last_telemetry());
        if (logger.has_value()) {
            logger->record(session.last_telemetry());
        }
        if (!quiet && (snapshot.frame_index % kPrintEvery == 0 || session.finished())) {
            print_status_line(snapshot);
        }
    }

    const auto wall_end = std::chrono::steady_clock::now();
    const double wall_s = std::chrono::duration<double>(wall_end - wall_start).count();

    const BenchmarkMetrics metrics = compute_benchmark_metrics(records, wall_s);

    std::cout << "\n--- summary : " << to_string(session.scenario()) << " ---\n"
              << "frames             : " << metrics.frames << "\n"
              << "detection          : " << std::fixed << std::setprecision(1)
              << 100.0 * metrics.detection_fraction << " %\n"
              << "RMS angular error  : " << std::setprecision(4)
              << rad_to_deg(metrics.rms_angular_error_rad) << " deg\n"
              << "P95 angular error  : " << rad_to_deg(metrics.p95_angular_error_rad) << " deg\n"
              << "max angular error  : " << rad_to_deg(metrics.max_angular_error_rad) << " deg\n"
              << "final angular error: " << rad_to_deg(metrics.final_angular_error_rad) << " deg\n"
              << "lost frames        : " << metrics.lost_frames << " / " << metrics.frames << "\n";
    if (logger.has_value()) {
        std::cout << "csv                : " << csv_path << "  (" << logger->records_written()
                  << " rows)\n";
    }
    std::cout << "clocks             : simulation 50 Hz (authoritative)  |  processing "
              << std::setprecision(0) << metrics.processing_fps << " FPS (wall, informational)\n";

    return 0;
}

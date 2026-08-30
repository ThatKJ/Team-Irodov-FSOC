// Step 7 smoke test: the first real closed-loop tracking simulation.
//
// Terminal-only, headless (no cv::imshow, no GUI). Simulation dynamics use a
// fixed timestep only — never wall-clock time.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fsoc/config.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/simulation_runner.hpp"
#include "fsoc/trajectory.hpp"

namespace {

using fsoc::deg_to_rad;
using fsoc::rad_to_deg;

[[nodiscard]] double err_deg(const fsoc::SimulationStepResult& r) {
    return r.tracking_error.has_value()
               ? rad_to_deg(fsoc::total_angular_error_rad(*r.tracking_error))
               : std::nan("");
}

void print_metrics(const char* label, const fsoc::SimulationMetrics& m) {
    std::cout << std::fixed << std::setprecision(4) << "  " << label << ":\n"
              << "    frames             = " << m.frame_count << '\n'
              << "    detected           = " << m.detected_frames << " ("
              << std::setprecision(1) << 100.0 * m.detection_fraction << " %)\n"
              << std::setprecision(4)
              << "    target-lost frames = " << m.lost_frames << '\n'
              << "    RMS angular error  = " << rad_to_deg(m.rms_angular_error_rad) << " deg\n"
              << "    max angular error  = " << rad_to_deg(m.max_angular_error_rad) << " deg\n"
              << "    final angular error= " << rad_to_deg(m.final_angular_error_rad) << " deg\n"
              << "    mean detect error  = " << m.mean_detection_error_px << " px (truth vs meas)\n";
}

}  // namespace

int main() {
    using namespace fsoc;

    bool all_ok = true;
    std::cout << "Step 7: closed-loop pixel-feedback tracking (headless, fixed dt)\n";
    const SimulationRunnerConfig base = baseline_runner_config();
    std::cout << std::fixed << std::setprecision(4)
              << "baseline: dt=" << base.timestep_s << " s (" << std::setprecision(0)
              << 1.0 / base.timestep_s << " Hz), PID kp=" << std::setprecision(2)
              << base.controller.pan.kp << " ki=" << base.controller.pan.ki
              << " kd=" << base.controller.pan.kd
              << ", camera max rate=" << rad_to_deg(base.camera.max_pan_rate_rad_s) << " deg/s\n";

    // ================= SCENARIO 1 — STATIC ACQUISITION =================
    std::cout << "\n=== SCENARIO 1: static acquisition ===\n";
    std::cout << "target (100, 6, 4) m ; camera starts at pan=0 tilt=0 (beacon off-centre)\n\n";
    {
        const StationaryTrajectory target{Vec3{100.0, 6.0, 4.0}};
        SimulationRunner runner{base, target};
        const auto results = runner.run_for(5.0);

        std::cout << std::fixed << std::setprecision(3)
                  << "   t[s]   centroid(px)        pix_err(px)      ang_err(deg)  "
                     "camera(pan,tilt deg)   cmd(pan,tilt deg/s)\n";
        for (const auto& r : results) {
            const double t = r.simulation_time_s;
            if (std::fmod(t + 1e-9, 0.5) > base.timestep_s && t > 1e-9) {
                continue;  // sample every 0.5 s
            }
            std::cout << "  " << std::setw(5) << std::setprecision(2) << t << "  ";
            if (r.detection.has_value()) {
                std::cout << std::setprecision(2) << "(" << std::setw(7)
                          << r.detection->centroid_px.x_px << "," << std::setw(7)
                          << r.detection->centroid_px.y_px << ")  ";
            } else {
                std::cout << "   <no detection>    ";
            }
            if (r.tracking_error.has_value()) {
                std::cout << std::setprecision(2) << "(" << std::setw(6)
                          << r.tracking_error->pixel.x_px << "," << std::setw(6)
                          << r.tracking_error->pixel.y_px << ")   " << std::setprecision(4)
                          << std::setw(9) << err_deg(r) << "  ";
            } else {
                std::cout << "      -              -      ";
            }
            std::cout << std::setprecision(3) << "(" << std::setw(6)
                      << rad_to_deg(r.camera_pan_rad) << "," << std::setw(6)
                      << rad_to_deg(r.camera_tilt_rad) << ")     (" << std::setw(6)
                      << rad_to_deg(r.command.pan_rate_rad_s) << "," << std::setw(6)
                      << rad_to_deg(r.command.tilt_rate_rad_s) << ")\n";
        }

        const auto m = evaluate(results);
        const auto& last = results.back();
        const double final_err_deg = rad_to_deg(m.final_angular_error_rad);
        const double final_centroid_offset_px =
            last.tracking_error.has_value()
                ? std::hypot(last.tracking_error->pixel.x_px, last.tracking_error->pixel.y_px)
                : std::nan("");
        std::cout << "\n  first-frame angular error = " << std::setprecision(4)
                  << err_deg(results.front()) << " deg\n"
                  << "  final  angular error      = " << final_err_deg << " deg  (gate < 0.05)\n"
                  << "  final  centroid offset    = " << std::setprecision(3)
                  << final_centroid_offset_px << " px  (gate < 2.0)\n"
                  << "  detection after frame 0   = "
                  << (m.detected_frames >= m.frame_count - 1 ? "100%" : "INCOMPLETE") << '\n';
        const bool ok = m.detected_frames >= m.frame_count - 1 && final_err_deg < 0.05 &&
                        final_centroid_offset_px < 2.0;
        std::cout << "  SCENARIO 1 -> " << (ok ? "PASS" : "FAIL") << '\n';
        all_ok = all_ok && ok;
    }

    // ================= SCENARIO 2 — SINUSOIDAL TRACKING =================
    std::cout << "\n=== SCENARIO 2: sinusoidal moving target ===\n";
    {
        SinusoidalTrajectory::Parameters p{};
        p.center_position_m = Vec3{100.0, 0.0, 3.0};
        p.amplitude_m = Vec3{0.0, 22.0, 4.0};
        p.frequency_hz = Vec3{0.0, 0.12, 0.09};
        const SinusoidalTrajectory target{p};

        SimulationRunnerConfig cfg = base;
        cfg.initial_tilt_rad = std::atan2(3.0, 100.0);  // point at the oscillation centre
        SimulationRunner runner{cfg, target};
        const auto results = runner.run_for(20.0);
        const auto m = evaluate(results);
        std::cout << "  Y +/-22 m @ 0.12 Hz (+/-12.4 deg), Z +/-4 m @ 0.09 Hz ; 20 s\n";
        print_metrics("closed loop", m);
        const bool ok = m.detection_fraction >= 0.95 &&
                        rad_to_deg(m.rms_angular_error_rad) < 1.0 &&
                        rad_to_deg(m.max_angular_error_rad) < 1.5;
        std::cout << "  gates: detected >= 95%, RMS < 1.0 deg, max < 1.5 deg -> "
                  << (ok ? "PASS" : "FAIL") << '\n';
        all_ok = all_ok && ok;
    }

    // ================= SCENARIO 3 — OPEN LOOP vs CLOSED LOOP =================
    std::cout << "\n=== SCENARIO 3: open-loop vs closed-loop (identical trajectory) ===\n";
    {
        SinusoidalTrajectory::Parameters p{};
        p.center_position_m = Vec3{100.0, 0.0, 3.0};
        p.amplitude_m = Vec3{0.0, 22.0, 4.0};
        p.frequency_hz = Vec3{0.0, 0.12, 0.09};
        const SinusoidalTrajectory target{p};

        SimulationRunnerConfig open_cfg = base;
        open_cfg.initial_tilt_rad = std::atan2(3.0, 100.0);
        open_cfg.control_enabled = false;
        SimulationRunner open_runner{open_cfg, target};
        const auto open_results = open_runner.run_for(20.0);
        const auto open_m = evaluate(open_results);

        SimulationRunnerConfig closed_cfg = open_cfg;
        closed_cfg.control_enabled = true;
        SimulationRunner closed_runner{closed_cfg, target};
        const auto closed_results = closed_runner.run_for(20.0);
        const auto closed_m = evaluate(closed_results);

        print_metrics("OPEN loop  (camera fixed)", open_m);
        print_metrics("CLOSED loop (PID tracking)", closed_m);

        const double det_gain = closed_m.detection_fraction /
                                std::max(open_m.detection_fraction, 1e-9);
        const double rms_ratio = (closed_m.rms_angular_error_rad > 0.0)
                                     ? open_m.rms_angular_error_rad / closed_m.rms_angular_error_rad
                                     : std::nan("");
        std::cout << std::setprecision(2)
                  << "  detection fraction: " << open_m.detection_fraction << " -> "
                  << closed_m.detection_fraction << "  (x" << det_gain << ")\n"
                  << "  RMS angular error : " << std::setprecision(4)
                  << rad_to_deg(open_m.rms_angular_error_rad) << " deg -> "
                  << rad_to_deg(closed_m.rms_angular_error_rad) << " deg  (x"
                  << std::setprecision(1) << rms_ratio << " better)\n";
        const bool ok = closed_m.detection_fraction > open_m.detection_fraction + 0.10 &&
                        closed_m.rms_angular_error_rad < open_m.rms_angular_error_rad;
        std::cout << "  closed-loop outperforms open-loop -> " << (ok ? "PASS" : "FAIL") << '\n';
        all_ok = all_ok && ok;
    }

    std::cout << "\nStep 7 smoke: " << (all_ok ? "PASS" : "FAIL") << '\n';
    return all_ok ? 0 : 1;
}

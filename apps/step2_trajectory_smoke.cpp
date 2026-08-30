// Step 2 smoke test: demonstrates the target trajectory engine ONLY.
//
// No camera, projection, FOV, detection, control, or telemetry appears here.
// It samples each trajectory type at deterministic timestamps and prints the
// resulting truth state (position + velocity) in SI units.

#include <array>
#include <iomanip>
#include <iostream>
#include <numbers>

#include "fsoc/trajectory.hpp"

namespace {

void print_state(const double time_s, const fsoc::TargetState& state) {
    std::cout << "t=" << time_s << " s\n"
              << "  position=(" << state.position_m.x << ", " << state.position_m.y << ", "
              << state.position_m.z << ") m\n"
              << "  velocity=(" << state.velocity_mps.x << ", " << state.velocity_mps.y << ", "
              << state.velocity_mps.z << ") m/s\n";
}

void sample(const char* title, const fsoc::Trajectory& trajectory,
            const std::array<double, 4>& times_s) {
    std::cout << "\n=== " << title << " ===\n";
    for (const double t : times_s) {
        print_state(t, trajectory.state_at(t));
    }
}

}  // namespace

int main() {
    using namespace fsoc;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Step 2: target trajectory engine smoke test\n";
    std::cout << "World frame: +X forward, +Y right, +Z up | position [m], velocity [m/s]\n";

    constexpr std::array<double, 4> times_s{0.0, 0.5, 1.0, 2.0};

    // 1. Stationary: fixed remote terminal.
    const StationaryTrajectory stationary{Vec3{120.0, -3.0, 6.0}};
    sample("Stationary Trajectory", stationary, times_s);

    // 2. Linear constant velocity: drifts +Y and +Z, no forward motion.
    const LinearTrajectory linear{Vec3{100.0, 0.0, 2.0}, Vec3{0.0, 1.0, 0.2}};
    sample("Linear Trajectory", linear, times_s);

    // 3. Sinusoidal: X fixed, Y sways +/-10 m at 0.25 Hz, Z bobs +/-2 m at 0.5 Hz.
    //    omega_y = 2*pi*0.25 = pi/2 rad/s, omega_z = 2*pi*0.5 = pi rad/s.
    SinusoidalTrajectory::Parameters params{};
    params.center_position_m = Vec3{100.0, 0.0, 5.0};
    params.amplitude_m = Vec3{0.0, 10.0, 2.0};
    params.frequency_hz = Vec3{0.0, 0.25, 0.5};
    params.phase_rad = Vec3{0.0, 0.0, 0.0};
    const SinusoidalTrajectory sinusoidal{params};
    sample("Sinusoidal Trajectory", sinusoidal, times_s);

    // Cross-check one analytic value the reader can verify by hand:
    // at t = 1 s, omega_y*t = pi/2 -> y = 10*sin(pi/2) = 10 m, vy = 10*(pi/2)*cos(pi/2) = 0 m/s.
    const TargetState checkpoint = sinusoidal.state_at(1.0);
    constexpr double pi = std::numbers::pi_v<double>;
    std::cout << "\nhand-check @ t=1.000 s: y=" << checkpoint.position_m.y
              << " m (expect 10.000), vy=" << checkpoint.velocity_mps.y
              << " m/s (expect 0.000), vz=" << checkpoint.velocity_mps.z
              << " m/s (expect " << -2.0 * pi << ")\n";

    std::cout << "\nStep 2 smoke: OK\n";
    return 0;
}

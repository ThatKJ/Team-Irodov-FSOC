#pragma once

#include <cmath>
#include <stdexcept>

namespace fsoc {

// ---------------------------------------------------------------------------
// Scenario configuration  (pure data contract — no behaviour here)
// ---------------------------------------------------------------------------
//
// Names the deterministic experiment the future SimulationRunner (Step 7) will
// execute: which truth trajectory, how long, and the FIXED integration step.
// It does NOT own trajectory parameters, camera config, or any loop logic; it
// is only the small typed handle that keeps runs repeatable and comparable.

enum class TrajectoryMode {
    Stationary,
    Linear,
    Sinusoidal,
};

struct ScenarioConfig {
    TrajectoryMode trajectory_mode{TrajectoryMode::Linear};
    double duration_s{20.0};
    double timestep_s{0.02};  // fixed step; technical design calls for 10-20 ms

    void validate() const {
        if (!std::isfinite(duration_s) || duration_s <= 0.0) {
            throw std::invalid_argument("ScenarioConfig: duration_s must be finite and > 0.");
        }
        if (!std::isfinite(timestep_s) || timestep_s <= 0.0) {
            throw std::invalid_argument("ScenarioConfig: timestep_s must be finite and > 0.");
        }
        if (timestep_s > duration_s) {
            throw std::invalid_argument("ScenarioConfig: timestep_s must not exceed duration_s.");
        }
    }
};

}  // namespace fsoc

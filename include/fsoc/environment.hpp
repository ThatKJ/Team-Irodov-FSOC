#pragma once

#include <optional>

#include "fsoc/camera.hpp"
#include "fsoc/geometry.hpp"
#include "fsoc/target_state.hpp"

namespace fsoc {

// TargetState is defined in fsoc/target_state.hpp so the trajectory layer can depend on
// target truth without pulling in camera/projection headers.

// Mathematical world container only. Trajectory generation and control policy stay separate.
class Environment {
public:
    Environment(PanTiltCamera camera, TargetState target)
        : camera_(std::move(camera)), target_(target) {}

    [[nodiscard]] PanTiltCamera& camera() noexcept { return camera_; }
    [[nodiscard]] const PanTiltCamera& camera() const noexcept { return camera_; }
    [[nodiscard]] TargetState& target() noexcept { return target_; }
    [[nodiscard]] const TargetState& target() const noexcept { return target_; }

    void set_target_position(Vec3 position_m) noexcept { target_.position_m = position_m; }
    [[nodiscard]] std::optional<Projection> observe_ideal() const noexcept;

private:
    PanTiltCamera camera_;
    TargetState target_;
};

}  // namespace fsoc

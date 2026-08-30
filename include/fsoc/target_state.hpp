#pragma once

#include "fsoc/geometry.hpp"

namespace fsoc {

// Instantaneous truth state of the remote optical terminal, expressed in the frozen
// world frame (+X forward, +Y right, +Z up).
//
// Produced by the Trajectory layer and consumed by the Environment / observation layer.
// It deliberately carries no sensing, control, or timing state: the runner owns the clock,
// the camera owns geometry, and the controller owns commands.
//
// Units are SI and encoded in the member names:
//   position_m    world position, metres
//   velocity_mps  world velocity, metres per second
struct TargetState {
    Vec3 position_m{};
    Vec3 velocity_mps{};
};

}  // namespace fsoc

#pragma once

#include "fsoc/image_geometry.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// Measurement layer  (detector-facing contract)
// ---------------------------------------------------------------------------
//
// What the future threshold/centroid detector will emit for one frame.
// It is an ESTIMATE from image pixels only. The detector never sees the world
// TargetState or the exact Projection.
//
// The baseline detector produces exactly one meaningful quantity: the beacon
// centroid in pixels. No "confidence" field: a threshold/centroid detector has
// no principled probability to report, and a fabricated number would mislead
// the controller and the judges. Integrated intensity / pixel area can be added
// later IF a downstream consumer genuinely needs them.
struct BeaconDetection {
    ImagePoint centroid_px{};
};

// "No detection this frame" is represented by std::optional<BeaconDetection>
// being empty at the call site. There is no in-band sentinel.

[[nodiscard]] inline bool is_finite(const BeaconDetection& detection) noexcept {
    return is_finite(detection.centroid_px);
}

}  // namespace fsoc

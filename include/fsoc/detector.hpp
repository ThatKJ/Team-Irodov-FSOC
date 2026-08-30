#pragma once

#include <cstdint>
#include <optional>

#include <opencv2/core.hpp>

#include "fsoc/measurement.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// Baseline beacon detector  (Step 5 — first true perception path)
// ---------------------------------------------------------------------------
//
// Pure image-space: detect() consumes ONLY a CV_8UC1 frame and returns an
// estimated beacon centroid, or std::nullopt. The detector never sees
// TargetState, trajectory, CameraObservation, the exact projected ImagePoint,
// tracking error, or any controller state.
//
// This header intentionally does NOT include fsoc/renderer.hpp or
// fsoc/observation.hpp. The detector works on any valid CV_8UC1 image, not
// only frames from SyntheticCameraRenderer, and fsoc_perception must not
// depend on fsoc_render.
//
// Pipeline (transparent, deterministic — no CNN / feature / template / filter):
//   1. threshold            candidate pixel  <=>  value >= threshold_intensity
//   2. connected components 8-connectivity on the candidate mask (OpenCV imgproc)
//   3. reject components with pixel area < min_bright_pixels
//   4. select the component with the greatest INTEGRATED SIGNAL
//      (sum of pixel weights); ties resolve to the lowest label, i.e. the one
//      encountered first in raster order -> fully deterministic
//   5. intensity-weighted centroid over that component's pixels only:
//
//        weight(p) = (pixel(p) - threshold_intensity) + 1        // >= 1
//        centroid  = sum(weight(p) * pos(p)) / sum(weight(p))
//
// Why connected components: with two bright regions a single global thresholded
// centroid would average them. Restricting the centroid to the strongest
// component keeps a stray bright blob from pulling the estimate. The component
// area also gives the min_bright_pixels gate.
//
// Why "pixel - threshold + 1": subtracting the threshold (NOT an assumed
// background) removes the pedestal so the weighted centroid follows the
// beacon's true sub-pixel centre; the +1 keeps every in-component pixel
// contributing so the denominator can never be zero.

struct BeaconDetectorConfig {
    // A pixel is a beacon candidate when its value is >= this threshold. It must
    // be a clearly-bright cut: well above sensor dark level and below saturation.
    // Not tied to any particular renderer background value.
    std::uint8_t threshold_intensity{64};

    // Minimum pixel count for a connected component to count as a beacon.
    // Rejects single hot pixels and specks. (OpenCV component areas are int.)
    int min_bright_pixels{5};

    // threshold_intensity in [1, 254]; min_bright_pixels >= 1.
    // Throws std::invalid_argument otherwise.
    void validate() const;
};

class BeaconDetector {
public:
    explicit BeaconDetector(BeaconDetectorConfig config);

    [[nodiscard]] const BeaconDetectorConfig& config() const noexcept { return config_; }

    // Estimated beacon centroid for `frame`, or std::nullopt when no valid
    // beacon is present (no pixel above threshold, or no component large
    // enough). Never returns a sentinel coordinate.
    //
    // Throws std::invalid_argument if `frame` is empty or not CV_8UC1
    // (single-channel 8-bit). RGB and floating-point images are rejected, not
    // silently reinterpreted.
    [[nodiscard]] std::optional<BeaconDetection> detect(const cv::Mat& frame) const;

private:
    BeaconDetectorConfig config_;
};

}  // namespace fsoc

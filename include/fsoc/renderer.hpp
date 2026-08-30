#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

#include "fsoc/config.hpp"
#include "fsoc/observation.hpp"

namespace fsoc {

// ---------------------------------------------------------------------------
// Synthetic virtual-camera image renderer  (first OpenCV-dependent module)
// ---------------------------------------------------------------------------
//
// Consumes a Step-3 CameraObservation and produces the grayscale frame the
// virtual FSOC tracking camera would see. It is NOT a detector and NOT a
// controller. It never sees TargetState, world coordinates, trajectory, or
// pan/tilt state — only the image-plane information carried by the observation.
//
// Image format: 8-bit single channel, cv::CV_8UC1.
//   background : uniform dark level (default 5 counts)
//   beacon     : 2-D Gaussian spot centred on the observation's ImagePoint,
//                peak = beacon_peak_intensity (default 255 counts):
//
//        I(u, v) = background + peak * exp( -((u-u0)^2 + (v-v0)^2) / (2 sigma^2) )
//
//                clamped to [0, 255] and rounded to the nearest integer count.
//   sigma      : Gaussian standard deviation, in PIXELS.
//
// Pixel-sampling convention: pixel (u, v) is sampled at continuous image
// coordinate (u, v) — i.e. pixel centres coincide with integer coordinates,
// consistent with the camera's principal point cx = width_px / 2.0. The beacon
// centre (u0, v0) is kept in double precision and the Gaussian is evaluated at
// its true sub-pixel location, never rounded to the nearest pixel first, so the
// future detector can recover a sub-pixel centroid.

struct RendererConfig {
    // Image dimensions. The CAMERA is the authority on these; prefer building
    // this struct with renderer_config_for(camera_config, ...) so the renderer
    // and the camera cannot silently disagree.
    int width_px{640};
    int height_px{480};

    std::uint8_t background_intensity{5};
    std::uint8_t beacon_peak_intensity{255};

    double beacon_sigma_px{2.0};

    // Half-side of the square raster window evaluated around the beacon centre,
    // measured in sigmas. 4 sigma covers the whole footprint that survives 8-bit
    // quantisation for a 255-count peak, while touching only a tiny sub-window.
    double beacon_window_sigmas{4.0};

    // width_px > 0, height_px > 0, beacon_sigma_px finite and > 0,
    // beacon_window_sigmas finite and >= 1. Throws std::invalid_argument otherwise.
    void validate() const;
};

// Renderer configuration whose image size is copied from the camera (single
// source of truth for dimensions); beacon appearance keeps defaults unless
// overridden by editing the returned struct.
[[nodiscard]] RendererConfig renderer_config_for(
    const CameraConfig& camera_config,
    double beacon_sigma_px = 2.0);

class SyntheticCameraRenderer {
public:
    explicit SyntheticCameraRenderer(RendererConfig config);

    [[nodiscard]] const RendererConfig& config() const noexcept { return config_; }

    // Returns a CV_8UC1 frame of the configured size.
    //   status == Visible          -> background + Gaussian beacon at image_point_px
    //   OutsideFieldOfView         -> background-only frame (no fake beacon)
    //   BehindCamera               -> background-only frame (no fake beacon)
    // Throws std::invalid_argument if status == Visible but image_point_px is
    // absent or non-finite (a Step-3 contract violation).
    [[nodiscard]] cv::Mat render(const CameraObservation& observation) const;

private:
    [[nodiscard]] cv::Mat background_frame() const;

    RendererConfig config_;
};

}  // namespace fsoc

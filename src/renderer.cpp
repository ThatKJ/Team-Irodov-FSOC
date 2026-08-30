#include "fsoc/renderer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsoc {

void RendererConfig::validate() const {
    if (width_px <= 0 || height_px <= 0) {
        throw std::invalid_argument("RendererConfig: width_px and height_px must be > 0.");
    }
    if (!std::isfinite(beacon_sigma_px) || beacon_sigma_px <= 0.0) {
        throw std::invalid_argument("RendererConfig: beacon_sigma_px must be finite and > 0.");
    }
    if (!std::isfinite(beacon_window_sigmas) || beacon_window_sigmas < 1.0) {
        throw std::invalid_argument(
            "RendererConfig: beacon_window_sigmas must be finite and >= 1.");
    }
}

RendererConfig renderer_config_for(const CameraConfig& camera_config, const double beacon_sigma_px) {
    camera_config.validate();  // dimensions come from a validated camera
    RendererConfig config{};
    config.width_px = camera_config.width_px;
    config.height_px = camera_config.height_px;
    config.beacon_sigma_px = beacon_sigma_px;
    return config;
}

SyntheticCameraRenderer::SyntheticCameraRenderer(RendererConfig config) : config_(config) {
    config_.validate();
}

cv::Mat SyntheticCameraRenderer::background_frame() const {
    return cv::Mat(config_.height_px, config_.width_px, CV_8UC1,
                   cv::Scalar(static_cast<double>(config_.background_intensity)));
}

cv::Mat SyntheticCameraRenderer::render(const CameraObservation& observation) const {
    cv::Mat frame = background_frame();

    if (observation.status != ObservationStatus::Visible) {
        // OutsideFieldOfView / BehindCamera -> valid background-only image.
        return frame;
    }

    // Step-3 invariant: a Visible observation must carry a finite image point.
    if (!observation.image_point_px.has_value()) {
        throw std::invalid_argument(
            "SyntheticCameraRenderer::render: Visible observation has no image point.");
    }
    const ImagePoint& centre_px = *observation.image_point_px;
    if (!is_finite(centre_px)) {
        throw std::invalid_argument(
            "SyntheticCameraRenderer::render: Visible image point is non-finite.");
    }

    const double u0 = centre_px.x_px;
    const double v0 = centre_px.y_px;
    const double two_sigma_sq = 2.0 * config_.beacon_sigma_px * config_.beacon_sigma_px;
    const double peak = static_cast<double>(config_.beacon_peak_intensity);
    const double background = static_cast<double>(config_.background_intensity);

    // Evaluate the Gaussian only in a small window around the sub-pixel centre,
    // clipped to the image so a near-edge beacon can never index out of bounds.
    const int radius_px =
        static_cast<int>(std::ceil(config_.beacon_window_sigmas * config_.beacon_sigma_px));
    const int u_min = std::max(0, static_cast<int>(std::floor(u0)) - radius_px);
    const int u_max =
        std::min(config_.width_px - 1, static_cast<int>(std::ceil(u0)) + radius_px);
    const int v_min = std::max(0, static_cast<int>(std::floor(v0)) - radius_px);
    const int v_max =
        std::min(config_.height_px - 1, static_cast<int>(std::ceil(v0)) + radius_px);

    // If the clipped window is empty the beacon lies fully off-image: background only.
    for (int v = v_min; v <= v_max; ++v) {
        std::uint8_t* row = frame.ptr<std::uint8_t>(v);
        const double dv = static_cast<double>(v) - v0;
        for (int u = u_min; u <= u_max; ++u) {
            const double du = static_cast<double>(u) - u0;
            const double gaussian = peak * std::exp(-(du * du + dv * dv) / two_sigma_sq);
            const double intensity = std::clamp(background + gaussian, 0.0, 255.0);
            row[static_cast<std::size_t>(u)] = static_cast<std::uint8_t>(std::lround(intensity));
        }
    }

    return frame;
}

}  // namespace fsoc

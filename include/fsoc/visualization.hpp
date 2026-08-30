#pragma once

#include <filesystem>
#include <vector>

#include <opencv2/core.hpp>

#include "fsoc/simulation_runner.hpp"
#include "fsoc/telemetry.hpp"

namespace fsoc {

// ===========================================================================
// Engineering camera-view visualization  (Step 9 — an OBSERVER of the loop)
// ===========================================================================
//
// Draws a mission-control debug overlay on top of a COPY of the perception
// frame. It is an observer:
//   * the ORIGINAL CV_8UC1 grayscale frame the detector ran on is never
//     modified (annotate() takes it by const& and does not touch it);
//   * annotate() returns a NEW CV_8UC3 (BGR) display frame;
//   * nothing here feeds back into detection / tracking error / PID / camera /
//     trajectory / simulation time / telemetry values.
//
// Base-frame note: SimulationRunner is unchanged. The base frame for a given
// SimulationStepResult is reconstructed by rendering `result.observation`
// through a SyntheticCameraRenderer built from the same RendererConfig. The
// renderer is deterministic (Step 4), so this is byte-identical to the frame
// the runner detected on — with zero coupling into the control path.

// BGR colours with fixed engineering meaning.
struct VisualizationColours {
    cv::Scalar crosshair{200.0, 200.0, 200.0};   // neutral white/grey — desired position
    cv::Scalar tracking{90.0, 220.0, 90.0};      // green  — detection / error vector / TRACKING
    cv::Scalar lost{60.0, 60.0, 235.0};          // red    — TARGET LOST
    cv::Scalar warning{40.0, 190.0, 240.0};      // amber  — saturation / limit
    cv::Scalar hud{205.0, 205.0, 205.0};         // light grey — neutral engineering data
    cv::Scalar hud_dim{140.0, 140.0, 140.0};     // dim grey — labels
    cv::Scalar truth{240.0, 200.0, 70.0};        // cyan   — optional TRUTH marker only
};

struct VisualizationConfig {
    bool show_title{true};
    bool show_crosshair{true};
    bool show_detection_marker{true};
    bool show_error_vector{true};
    bool show_status{true};            // TRACKING / TARGET LOST + VISIBLE / DETECTED
    bool show_sim_time{true};          // SIM t / FRAME n
    bool show_camera_attitude{true};   // PAN / TILT (deg)
    bool show_angular_error{true};     // ANG ERR (deg)
    bool show_pixel_error{true};       // ERR PX X / Y
    bool show_command_rates{true};     // CMD PAN / TILT (deg/s) + SAT
    bool show_detection_error{false};  // truth-vs-measurement diagnostic (off by default)
    bool show_truth_marker{false};     // exact projection marker (off by default)

    VisualizationColours colours{};
    std::string title{"SIH26169 | FSOC TRACKING"};
};

class TrackingVisualizer {
public:
    explicit TrackingVisualizer(VisualizationConfig config = {});

    [[nodiscard]] const VisualizationConfig& config() const noexcept { return config_; }

    // Returns a NEW CV_8UC3 BGR frame of the same size as `grayscale_frame`.
    // `grayscale_frame` must be a non-empty CV_8UC1 image and is left byte-for-byte
    // unchanged. `result` and `telemetry` are read only (the optional TRUTH marker
    // uses result.observation.image_point_px; every HUD value comes from telemetry).
    // Throws std::invalid_argument for an empty or non-CV_8UC1 input.
    [[nodiscard]] cv::Mat annotate(
        const cv::Mat& grayscale_frame,
        const SimulationStepResult& result,
        const TelemetryRecord& telemetry) const;

private:
    VisualizationConfig config_;
};

// ---------------------------------------------------------------------------
// Headless export helpers (PNG is the required portable path).
// ---------------------------------------------------------------------------

// Writes `image` to `path` as PNG, creating parent directories. Returns success.
[[nodiscard]] bool write_png(const std::filesystem::path& path, const cv::Mat& image);

// Best-effort MP4 export via cv::VideoWriter. Returns false (no throw) when the
// build has no videoio, no codec/backend can open the file, `frames` is empty,
// or the frames are not a uniform CV_8UC3 size. All-or-nothing: on failure no
// partial file is left behind.
[[nodiscard]] bool try_write_mp4(
    const std::filesystem::path& path,
    const std::vector<cv::Mat>& bgr_frames,
    double fps);

}  // namespace fsoc

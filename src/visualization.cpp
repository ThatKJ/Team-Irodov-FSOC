#include "fsoc/visualization.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef FSOC_HAS_VIDEOIO
#include <opencv2/videoio.hpp>
#endif

#include "fsoc/config.hpp"  // rad_to_deg

namespace fsoc {

namespace {

constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
constexpr double kFontScale = 0.42;
constexpr int kFontThickness = 1;
constexpr int kLineHeight = 16;

enum class HAlign { Left, Right };
enum class VAlign { Top, Bottom };

[[nodiscard]] std::string fnum(const double value, const int precision) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << value;
    return os.str();
}

[[nodiscard]] std::string fsigned(double value, const int precision) {
    if (value == 0.0) {
        value = 0.0;  // normalise -0.0 so the sign prefix is not doubled
    }
    std::ostringstream os;
    os << (value >= 0.0 ? "+" : "") << std::fixed << std::setprecision(precision) << value;
    return os.str();
}

[[nodiscard]] cv::Point round_point(const double x, const double y) {
    return cv::Point{static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y))};
}

void draw_crosshair(cv::Mat& display, const cv::Point& centre, const cv::Scalar& colour) {
    constexpr int gap = 5;   // keep the exact centre clear so a centred beacon shows through
    constexpr int arm = 16;
    cv::line(display, {centre.x - arm, centre.y}, {centre.x - gap, centre.y}, colour, 1,
             cv::LINE_AA);
    cv::line(display, {centre.x + gap, centre.y}, {centre.x + arm, centre.y}, colour, 1,
             cv::LINE_AA);
    cv::line(display, {centre.x, centre.y - arm}, {centre.x, centre.y - gap}, colour, 1,
             cv::LINE_AA);
    cv::line(display, {centre.x, centre.y + gap}, {centre.x, centre.y + arm}, colour, 1,
             cv::LINE_AA);
    cv::circle(display, centre, 10, colour, 1, cv::LINE_AA);
}

void draw_detection_marker(cv::Mat& display, const cv::Point& point, const cv::Scalar& colour) {
    cv::circle(display, point, 7, colour, 1, cv::LINE_AA);
    cv::drawMarker(display, point, colour, cv::MARKER_CROSS, 6, 1, cv::LINE_AA);
}

void draw_truth_marker(cv::Mat& display, const cv::Point& point, const cv::Scalar& colour) {
    // Deliberately a SQUARE so it can never be mistaken for the round detection marker.
    cv::rectangle(display, {point.x - 6, point.y - 6}, {point.x + 6, point.y + 6}, colour, 1,
                  cv::LINE_AA);
    cv::putText(display, "TRUTH", {point.x + 9, point.y - 7}, kFont, 0.35, colour, 1,
                cv::LINE_AA);
}

void draw_hud_block(
    cv::Mat& display,
    const cv::Point& anchor,
    const HAlign halign,
    const VAlign valign,
    const std::vector<std::pair<std::string, cv::Scalar>>& lines) {
    if (lines.empty()) {
        return;
    }
    int max_width = 0;
    for (const auto& [text, colour] : lines) {
        int baseline = 0;
        const cv::Size size = cv::getTextSize(text, kFont, kFontScale, kFontThickness, &baseline);
        max_width = std::max(max_width, size.width);
    }
    const int block_w = max_width + 12;
    const int block_h = static_cast<int>(lines.size()) * kLineHeight + 8;
    const int x0 = halign == HAlign::Left ? anchor.x : anchor.x - block_w;
    const int y0 = valign == VAlign::Top ? anchor.y : anchor.y - block_h;

    const cv::Rect image_rect{0, 0, display.cols, display.rows};
    const cv::Rect panel = cv::Rect{x0, y0, block_w, block_h} & image_rect;
    if (panel.area() > 0) {
        cv::Mat roi = display(panel);
        cv::Mat shade{roi.size(), roi.type(), cv::Scalar{18.0, 18.0, 18.0}};
        cv::addWeighted(shade, 0.5, roi, 0.5, 0.0, roi);
    }

    for (std::size_t i = 0; i < lines.size(); ++i) {
        const cv::Point org{x0 + 6, y0 + 13 + static_cast<int>(i) * kLineHeight};
        cv::putText(display, lines[i].first, org, kFont, kFontScale, lines[i].second,
                    kFontThickness, cv::LINE_AA);
    }
}

}  // namespace

TrackingVisualizer::TrackingVisualizer(VisualizationConfig config) : config_(std::move(config)) {}

cv::Mat TrackingVisualizer::annotate(
    const cv::Mat& grayscale_frame,
    const SimulationStepResult& result,
    const TelemetryRecord& telemetry) const {
    if (grayscale_frame.empty()) {
        throw std::invalid_argument("TrackingVisualizer::annotate: frame is empty.");
    }
    if (grayscale_frame.type() != CV_8UC1) {
        throw std::invalid_argument(
            "TrackingVisualizer::annotate: frame must be CV_8UC1 (the perception frame).");
    }

    // Fresh BGR display buffer — the input grayscale frame is never touched.
    cv::Mat display;
    cv::cvtColor(grayscale_frame, display, cv::COLOR_GRAY2BGR);

    const VisualizationColours& c = config_.colours;
    const bool tracking = telemetry.tracking_state == TrackingState::Tracking;

    // Image centre from the frame geometry (matches PanTiltCamera::cx_px/cy_px = W/2, H/2).
    const cv::Point centre = round_point(static_cast<double>(display.cols) / 2.0,
                                         static_cast<double>(display.rows) / 2.0);

    // --- viewport overlays (kept clear of the beacon) ---
    if (config_.show_crosshair) {
        draw_crosshair(display, centre, c.crosshair);
    }

    if (config_.show_truth_marker && result.observation.image_point_px.has_value()) {
        draw_truth_marker(
            display,
            round_point(result.observation.image_point_px->x_px,
                        result.observation.image_point_px->y_px),
            c.truth);
    }

    cv::Point detected_point{};
    const bool have_detection =
        telemetry.detected_x_px.has_value() && telemetry.detected_y_px.has_value();
    if (have_detection) {
        detected_point = round_point(*telemetry.detected_x_px, *telemetry.detected_y_px);
    }

    if (config_.show_error_vector && tracking && have_detection) {
        cv::arrowedLine(display, centre, detected_point, c.tracking, 1, cv::LINE_AA, 0, 0.18);
    }
    if (config_.show_detection_marker && have_detection) {
        draw_detection_marker(display, detected_point, c.tracking);
    }

    // --- TOP-LEFT: title / state / visibility / sim time ---
    {
        std::vector<std::pair<std::string, cv::Scalar>> lines;
        if (config_.show_title) {
            lines.emplace_back(config_.title, c.hud);
        }
        if (config_.show_status) {
            lines.emplace_back(tracking ? "TRACKING" : "TARGET LOST",
                               tracking ? c.tracking : c.lost);
            lines.emplace_back(
                std::string{"VISIBLE  "} + (telemetry.target_visible ? "YES" : "NO"),
                telemetry.target_visible ? c.hud : c.hud_dim);
            lines.emplace_back(
                std::string{"DETECTED "} + (telemetry.target_detected ? "YES" : "NO"),
                telemetry.target_detected ? c.hud : c.hud_dim);
        }
        if (config_.show_sim_time) {
            lines.emplace_back("SIM   " + fnum(telemetry.simulation_time_s, 3) + " s", c.hud);
            lines.emplace_back("FRAME " + std::to_string(telemetry.frame_index), c.hud);
        }
        draw_hud_block(display, {10, 10}, HAlign::Left, VAlign::Top, lines);
    }

    // --- TOP-RIGHT: camera attitude / angular error ---
    {
        std::vector<std::pair<std::string, cv::Scalar>> lines;
        if (config_.show_camera_attitude) {
            lines.emplace_back(
                "PAN  " + fsigned(rad_to_deg(telemetry.camera_pan_rad), 3) + " deg", c.hud);
            lines.emplace_back(
                "TILT " + fsigned(rad_to_deg(telemetry.camera_tilt_rad), 3) + " deg", c.hud);
        }
        if (config_.show_angular_error) {
            if (telemetry.angular_error_total_rad.has_value()) {
                lines.emplace_back(
                    "ANG ERR " + fnum(rad_to_deg(*telemetry.angular_error_total_rad), 4) + " deg",
                    c.tracking);
            } else {
                lines.emplace_back("ANG ERR   --", c.hud_dim);
            }
        }
        draw_hud_block(display, {display.cols - 10, 10}, HAlign::Right, VAlign::Top, lines);
    }

    // --- BOTTOM-LEFT: detected centroid / pixel error / (optional) detection error ---
    {
        std::vector<std::pair<std::string, cv::Scalar>> lines;
        if (config_.show_pixel_error) {
            if (have_detection) {
                lines.emplace_back("DET (" + fnum(*telemetry.detected_x_px, 1) + ", " +
                                       fnum(*telemetry.detected_y_px, 1) + ")",
                                   c.hud);
            } else {
                lines.emplace_back("DET  --", c.hud_dim);
            }
            if (telemetry.pixel_error_x_px.has_value() &&
                telemetry.pixel_error_y_px.has_value()) {
                lines.emplace_back("ERR PX X " + fsigned(*telemetry.pixel_error_x_px, 1) +
                                       "  Y " + fsigned(*telemetry.pixel_error_y_px, 1),
                                   c.tracking);
            } else {
                lines.emplace_back("ERR PX   --", c.hud_dim);
            }
        }
        if (config_.show_detection_error && telemetry.detection_error_px.has_value()) {
            lines.emplace_back(
                "DETECT ERR " + fnum(*telemetry.detection_error_px, 3) + " px (truth vs meas)",
                c.hud_dim);
        }
        draw_hud_block(display, {10, display.rows - 10}, HAlign::Left, VAlign::Bottom, lines);
    }

    // --- BOTTOM-RIGHT: command rates / saturation ---
    {
        std::vector<std::pair<std::string, cv::Scalar>> lines;
        if (config_.show_command_rates) {
            lines.emplace_back(
                "CMD PAN  " + fsigned(rad_to_deg(telemetry.command_pan_rate_rad_s), 1) + " deg/s",
                telemetry.pan_saturated ? c.warning : c.hud);
            lines.emplace_back(
                "CMD TILT " + fsigned(rad_to_deg(telemetry.command_tilt_rate_rad_s), 1) +
                    " deg/s",
                telemetry.tilt_saturated ? c.warning : c.hud);
            if (telemetry.pan_saturated || telemetry.tilt_saturated) {
                std::string sat = "RATE LIMIT";
                if (telemetry.pan_saturated && !telemetry.tilt_saturated) {
                    sat = "PAN RATE LIMIT";
                } else if (!telemetry.pan_saturated && telemetry.tilt_saturated) {
                    sat = "TILT RATE LIMIT";
                }
                lines.emplace_back(sat, c.warning);
            }
        }
        draw_hud_block(display, {display.cols - 10, display.rows - 10}, HAlign::Right,
                       VAlign::Bottom, lines);
    }

    return display;
}

// ---------------------------------------------------------------------------

bool write_png(const std::filesystem::path& path, const cv::Mat& image) {
    if (image.empty()) {
        return false;
    }
    if (const std::filesystem::path parent = path.parent_path(); !parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
    }
    return cv::imwrite(path.string(), image);
}

bool try_write_mp4(
    const std::filesystem::path& path,
    const std::vector<cv::Mat>& bgr_frames,
    const double fps) {
#ifdef FSOC_HAS_VIDEOIO
    if (bgr_frames.empty() || !std::isfinite(fps) || fps <= 0.0) {
        return false;
    }
    const cv::Size size = bgr_frames.front().size();
    for (const cv::Mat& frame : bgr_frames) {
        if (frame.empty() || frame.size() != size || frame.type() != CV_8UC3) {
            return false;
        }
    }
    if (const std::filesystem::path parent = path.parent_path(); !parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
    }
    cv::VideoWriter writer;
    const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    if (!writer.open(path.string(), fourcc, fps, size, /*isColor=*/true)) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
        return false;
    }
    for (const cv::Mat& frame : bgr_frames) {
        writer.write(frame);
    }
    writer.release();
    std::error_code ec;
    return std::filesystem::exists(path, ec);
#else
    (void)path;
    (void)bgr_frames;
    (void)fps;
    return false;
#endif
}

}  // namespace fsoc

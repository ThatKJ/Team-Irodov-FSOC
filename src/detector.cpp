#include "fsoc/detector.hpp"

#include <cstddef>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace fsoc {

void BeaconDetectorConfig::validate() const {
    // 0 would accept the dark background; 255 would accept only saturated pixels.
    if (threshold_intensity < 1 || threshold_intensity > 254) {
        throw std::invalid_argument(
            "BeaconDetectorConfig: threshold_intensity must be in [1, 254].");
    }
    if (min_bright_pixels < 1) {
        throw std::invalid_argument("BeaconDetectorConfig: min_bright_pixels must be >= 1.");
    }
}

BeaconDetector::BeaconDetector(BeaconDetectorConfig config) : config_(config) {
    config_.validate();
}

namespace {

// Weight for one in-component pixel: (value - threshold) + 1, always >= 1.
[[nodiscard]] double pixel_weight(const int value, const int threshold) noexcept {
    return static_cast<double>(value - threshold) + 1.0;
}

}  // namespace

std::optional<BeaconDetection> BeaconDetector::detect(const cv::Mat& frame) const {
    if (frame.empty()) {
        throw std::invalid_argument("BeaconDetector::detect: frame is empty.");
    }
    if (frame.type() != CV_8UC1) {
        throw std::invalid_argument(
            "BeaconDetector::detect: frame must be CV_8UC1 (single-channel 8-bit); "
            "RGB / floating-point frames are not accepted.");
    }

    const int threshold = static_cast<int>(config_.threshold_intensity);

    // Candidate mask: pixel >= threshold. THRESH_BINARY keeps pixel > thresh,
    // so pass (threshold - 1) to get the inclusive comparison for integer data.
    cv::Mat mask;
    cv::threshold(frame, mask, static_cast<double>(threshold) - 1.0, 255.0, cv::THRESH_BINARY);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;  // unweighted binary centroids — unused; we weight by intensity below
    const int label_count =
        cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

    // Labels 1 .. label_count-1 are real components (0 is background).
    // Pick the one with the greatest integrated signal among those big enough.
    int best_label = -1;
    double best_signal = 0.0;
    for (int label = 1; label < label_count; ++label) {
        if (stats.at<int>(label, cv::CC_STAT_AREA) < config_.min_bright_pixels) {
            continue;
        }
        const int x0 = stats.at<int>(label, cv::CC_STAT_LEFT);
        const int y0 = stats.at<int>(label, cv::CC_STAT_TOP);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);

        double signal = 0.0;
        for (int y = y0; y < y0 + height; ++y) {
            const std::uint8_t* frame_row = frame.ptr<std::uint8_t>(y);
            const int* label_row = labels.ptr<int>(y);
            for (int x = x0; x < x0 + width; ++x) {
                const auto col = static_cast<std::size_t>(x);
                if (label_row[col] == label) {
                    signal += pixel_weight(frame_row[col], threshold);
                }
            }
        }
        if (signal > best_signal) {  // strict '>' => first (lowest label) wins ties
            best_signal = signal;
            best_label = label;
        }
    }

    if (best_label < 0) {
        return std::nullopt;  // no component passed the min_bright_pixels gate
    }

    // Intensity-weighted centroid over the winning component's pixels only.
    const int x0 = stats.at<int>(best_label, cv::CC_STAT_LEFT);
    const int y0 = stats.at<int>(best_label, cv::CC_STAT_TOP);
    const int width = stats.at<int>(best_label, cv::CC_STAT_WIDTH);
    const int height = stats.at<int>(best_label, cv::CC_STAT_HEIGHT);

    double sum_w = 0.0;
    double sum_wx = 0.0;
    double sum_wy = 0.0;
    for (int y = y0; y < y0 + height; ++y) {
        const std::uint8_t* frame_row = frame.ptr<std::uint8_t>(y);
        const int* label_row = labels.ptr<int>(y);
        for (int x = x0; x < x0 + width; ++x) {
            const auto col = static_cast<std::size_t>(x);
            if (label_row[col] != best_label) {
                continue;
            }
            const double weight = pixel_weight(frame_row[col], threshold);
            sum_w += weight;
            sum_wx += weight * static_cast<double>(x);
            sum_wy += weight * static_cast<double>(y);
        }
    }

    // sum_w >= area >= min_bright_pixels >= 1: the division is always safe.
    return BeaconDetection{
        .centroid_px = {.x_px = sum_wx / sum_w, .y_px = sum_wy / sum_w},
    };
}

}  // namespace fsoc

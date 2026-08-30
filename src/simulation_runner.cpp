#include "fsoc/simulation_runner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>

namespace fsoc {

void SimulationRunnerConfig::validate() const {
    camera.validate();
    renderer.validate();
    detector.validate();
    controller.validate();

    if (renderer.width_px != camera.width_px || renderer.height_px != camera.height_px) {
        throw std::invalid_argument(
            "SimulationRunnerConfig: renderer dimensions must match the camera "
            "(build the renderer config with renderer_config_for(camera)).");
    }
    if (!std::isfinite(timestep_s) || timestep_s <= 0.0) {
        throw std::invalid_argument("SimulationRunnerConfig: timestep_s must be finite and > 0.");
    }
    // The PID must not silently demand a rate the actuator cannot deliver.
    if (controller.pan.output_limit_rad_s > camera.max_pan_rate_rad_s) {
        throw std::invalid_argument(
            "SimulationRunnerConfig: PID pan output limit exceeds camera max pan rate.");
    }
    if (controller.tilt.output_limit_rad_s > camera.max_tilt_rate_rad_s) {
        throw std::invalid_argument(
            "SimulationRunnerConfig: PID tilt output limit exceeds camera max tilt rate.");
    }
}

SimulationRunnerConfig baseline_runner_config() {
    SimulationRunnerConfig config{};
    config.camera = CameraConfig{};
    config.renderer = renderer_config_for(config.camera, 2.0);
    config.detector = BeaconDetectorConfig{};
    config.timestep_s = 0.02;  // 50 Hz fixed step
    config.camera_position_m = Vec3{0.0, 0.0, 0.0};
    config.initial_pan_rad = 0.0;
    config.initial_tilt_rad = 0.0;
    config.control_enabled = true;

    // Empirically-tuned MVP baseline (see ADR-010 / roadmap Step 7). The plant
    // (camera angle = integral of the rate command) already contains one
    // integrator, so a P-dominant law converges without oscillation:
    //   Ki = 0  -> no steady-state bias for a stationary target and no risk of
    //             integrator windup on the double-integrator that Ki would form;
    //   Kd = 0  -> the plant is already well damped; derivative would only
    //             amplify the ~0.02 px detector noise.
    // Output limit matches the camera actuator rate (deg_to_rad(30)).
    PIDAxisConfig axis{};
    axis.kp = 12.0;
    axis.ki = 0.0;
    axis.kd = 0.0;
    axis.integral_limit = 0.0;
    axis.output_limit_rad_s = config.camera.max_pan_rate_rad_s;  // == max_tilt_rate here
    config.controller.pan = axis;
    config.controller.tilt = axis;

    return config;
}

SimulationRunner::SimulationRunner(SimulationRunnerConfig config, const Trajectory& trajectory)
    : config_((config.validate(), std::move(config))),
      trajectory_(&trajectory),
      camera_(config_.camera, config_.camera_position_m, config_.initial_pan_rad,
              config_.initial_tilt_rad),
      renderer_(config_.renderer),
      detector_(config_.detector),
      controller_(config_.controller) {}

SimulationStepResult SimulationRunner::step() {
    SimulationStepResult result{};
    result.simulation_time_s = simulation_time_s_;
    result.frame_index = frame_index_;
    // Camera orientation that produces THIS frame's observation (before we step it).
    result.camera_pan_rad = camera_.pan_rad();
    result.camera_tilt_rad = camera_.tilt_rad();

    // 1. trajectory truth at the current sim time
    const TargetState target = trajectory_->state_at(simulation_time_s_);
    result.target_truth = target;

    // 2. observe the truth position through the current camera pose (exact projection)
    const CameraObservation observation = observe_beacon(camera_, target.position_m);
    result.observation = observation;
    result.target_visible = observation.visible();

    // 3. render the synthetic frame — the renderer sees ONLY the observation
    const cv::Mat frame = renderer_.render(observation);

    // 4. detect — MEASUREMENT path, pixels only
    const std::optional<BeaconDetection> detection = detector_.detect(frame);
    result.detection = detection;
    result.target_detected = detection.has_value();

    // 5. tracking error from the DETECTED centroid (never observation.image_point_px)
    const std::optional<TrackingError> tracking_error =
        compute_tracking_error(detection, camera_);
    result.tracking_error = tracking_error;

    // 6. control, or the target-loss / open-loop policy
    ControlCommand command = zero_control_command();
    if (!config_.control_enabled) {
        controller_.reset();  // keep PID state clean while open-loop
    } else if (tracking_error.has_value()) {
        command = controller_.update(*tracking_error, config_.timestep_s);
    } else {
        // Target lost: reset the PID, command zero, camera holds its orientation.
        controller_.reset();
    }
    result.command = command;

    // 7. step the camera (the sole authority on pan/tilt state)
    result.applied_rates =
        camera_.step(command.pan_rate_rad_s, command.tilt_rate_rad_s, config_.timestep_s);

    // Diagnostic scoring: detected centroid vs exact projection (truth used ONLY here).
    if (detection.has_value() && observation.image_point_px.has_value()) {
        const double dx = detection->centroid_px.x_px - observation.image_point_px->x_px;
        const double dy = detection->centroid_px.y_px - observation.image_point_px->y_px;
        result.detection_error_px = std::hypot(dx, dy);
    }

    // 8/9. advance the clock
    simulation_time_s_ += config_.timestep_s;
    ++frame_index_;
    return result;
}

std::vector<SimulationStepResult> SimulationRunner::run_for(const double duration_s) {
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
        throw std::invalid_argument("SimulationRunner::run_for: duration_s must be finite and > 0.");
    }
    const auto step_count =
        static_cast<std::size_t>(std::ceil(duration_s / config_.timestep_s));
    std::vector<SimulationStepResult> results;
    results.reserve(step_count);
    for (std::size_t i = 0; i < step_count; ++i) {
        results.push_back(step());
    }
    return results;
}

void SimulationRunner::reset() {
    camera_ = PanTiltCamera{config_.camera, config_.camera_position_m, config_.initial_pan_rad,
                            config_.initial_tilt_rad};
    controller_.reset();
    simulation_time_s_ = 0.0;
    frame_index_ = 0;
}

double total_angular_error_rad(const TrackingError& error) noexcept {
    return std::hypot(error.angular.pan_rad, error.angular.tilt_rad);
}

SimulationMetrics evaluate(const std::vector<SimulationStepResult>& results) {
    SimulationMetrics metrics{};
    metrics.frame_count = results.size();
    if (results.empty()) {
        return metrics;
    }

    double sum_sq_angular = 0.0;
    double sum_detection_error_px = 0.0;
    std::size_t detection_error_samples = 0;

    for (const SimulationStepResult& r : results) {
        if (r.target_visible) {
            ++metrics.visible_frames;
        }
        if (r.detection.has_value()) {
            ++metrics.detected_frames;
        } else {
            ++metrics.lost_frames;
        }
        if (r.tracking_error.has_value()) {
            const double e = total_angular_error_rad(*r.tracking_error);
            sum_sq_angular += e * e;
            metrics.max_angular_error_rad = std::max(metrics.max_angular_error_rad, e);
            metrics.final_angular_error_rad = e;  // last detected frame wins
        }
        if (r.detection_error_px.has_value()) {
            sum_detection_error_px += *r.detection_error_px;
            ++detection_error_samples;
        }
    }

    metrics.detection_fraction =
        static_cast<double>(metrics.detected_frames) / static_cast<double>(metrics.frame_count);
    if (metrics.detected_frames > 0) {
        metrics.rms_angular_error_rad =
            std::sqrt(sum_sq_angular / static_cast<double>(metrics.detected_frames));
    }
    if (detection_error_samples > 0) {
        metrics.mean_detection_error_px =
            sum_detection_error_px / static_cast<double>(detection_error_samples);
    }
    return metrics;
}

}  // namespace fsoc

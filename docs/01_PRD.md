# Product Requirements Document — 48-Hour Baseline

## Objective
Demonstrate that a virtual mobile FSOC terminal can keep a remote optical beacon near the center of a narrow-FOV camera using closed-loop image feedback.

## Product proof
A judge should be able to see:
1. a moving target that would drift out of view without steering,
2. a camera with explicit pan/tilt state and actuator limits,
3. an image-derived beacon centroid,
4. a PID command produced from centroid error,
5. reduced tracking error over time,
6. telemetry quantifying performance.

## Functional requirements
- 3D target and camera coordinates.
- At least linear and sinusoidal truth trajectories.
- Perspective projection through configurable horizontal/vertical FOV.
- Grayscale synthetic beacon observation.
- Threshold + centroid baseline detector.
- Independent pan and tilt PID axes.
- Saturated actuator velocity.
- Target visible/lost state.
- Fixed-step simulation.
- Real-time or faster-than-real-time executable.
- Telemetry: sim time, FPS, centroid, pixel error, angular error, pan/tilt, commanded/applied rate, visibility.

## Non-functional requirements
- macOS local development.
- C++20.
- CMake/Ninja build.
- Core physics and controller independently testable.
- Deterministic baseline scenario.
- No UKF/MPC/turbulence/CNN until baseline is frozen.

## Baseline acceptance target
For an agreed nominal trajectory after acquisition:
- target remains visible for >= 95% of evaluated frames,
- RMS image-center error <= 10% of image half-diagonal,
- no actuator command exceeds configured limit after saturation,
- no NaN/Inf telemetry,
- deterministic replay produces materially identical metrics.

These are hackathon engineering gates, not claimed ISRO flight qualification limits.

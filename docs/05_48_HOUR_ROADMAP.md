# Strict 48-Hour Roadmap — C++ MVP

## Hours 0–4 — Step 1: Math + camera foundation [DONE]
- CMake/C++20 project
- coordinate convention
- world-to-camera transform
- FOV/pinhole projection
- pan/tilt rate limits
- native unit checks
- terminal smoke test

Gate: `ctest --preset debug` passes and the camera converges numerically without pixels.

## Hours 4–8 — Step 2: Trajectory engine [DONE]
- `Trajectory` abstract interface: `TargetState state_at(double time_s) const`
- stationary target
- linear constant-velocity target (signed components)
- sinusoidal target with analytic velocity
- deterministic analytic tests + terminal smoke test
- deliberate input validation (`std::invalid_argument`)

Gate: sampled positions/velocities agree with analytic expectations. PASSED —
`fsoc_step2_tests` green, `step2_trajectory_smoke` matches hand computation, Step 1
unchanged (final pointing error still ~0.001 deg).

## Hours 8–13 — Step 3: Observation contract
- define projection/measurement structs
- lost-target semantics
- pixel/angular error conventions
- scenario configuration

Gate: no controller exists yet; interfaces compile and tests cover signs/units.

## Hours 13–18 — Step 4: Synthetic image renderer
- add OpenCV C++
- grayscale image
- sub/resolved bright beacon approximation
- background + optional simple Gaussian sensor noise

Gate: images can be generated headlessly and target pixels match projection coordinates.

## Hours 18–23 — Step 5: Baseline detector
- intensity threshold
- connected component/moments or weighted centroid
- found/not-found state
- confidence/brightness telemetry

Gate: centroid error is bounded on clean synthetic frames.

## Hours 23–28 — Step 6: PID control
- separate PID class
- independent pan/tilt loops
- anti-windup/output clamp
- reset on scenario restart/loss policy

Gate: controller unit tests pass without OpenCV.

## Hours 28–34 — Step 7: Closed-loop integration
- fixed-step runner
- trajectory -> camera -> renderer -> detector -> PID -> actuator
- target loss behavior

Gate: moving target remains inside FOV for nominal scenario significantly better than open-loop.

## Hours 34–39 — Step 8: Telemetry + metrics
- CSV logger
- FPS
- pixel and angular errors
- pan/tilt + commanded/applied rates
- saturation and visibility
- RMS/95th percentile error

## Hours 39–43 — Step 9: Visualization
- OpenCV display
- center crosshair
- beacon centroid
- FOV/tracking state
- telemetry overlay

Visualization cannot change simulation math.

## Hours 43–46 — Step 10: Validation scenarios
- slow linear
- sinusoidal
- near-FOV-edge acquisition
- actuator saturation
- target loss/re-entry

## Hours 46–48 — Step 11: Demo freeze
- benchmark run
- README instructions
- clean build on teammate Mac
- record metrics/demo
- commit + create `v1_baseline`

Only after this: UKF -> motion prediction -> MPC -> vibration/turbulence -> advanced detection.

# Test and Validation Plan

## Unit tests
- angle wrapping boundaries
- camera basis orthogonality/signs
- ideal pan/tilt geometry
- center/right/up projection signs
- behind-camera rejection
- FOV boundary rejection
- actuator rate saturation
- tilt mechanical clamp
- invalid timestep rejection
- pixel-to-angle sign convention
- trajectory analytic samples
- PID P/I/D terms and clamps
- lost-target behavior

## Integration tests
- static target converges to center
- linear target tracking beats open-loop
- sinusoidal trajectory remains stable
- saturation scenario reports constrained rates
- re-entry scenario does not generate invalid centroid commands

## Metrics
- visibility ratio
- RMS pixel error
- 95th percentile pixel error
- RMS angular error
- max commanded and applied rate
- percentage of saturated frames
- mean/median wall-clock FPS

Unit tests establish math correctness. Metrics establish control performance. A pretty OpenCV window establishes neither by itself.

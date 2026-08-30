# Task Board

## Foundation
- [x] C++20/CMake scaffold
- [x] coordinate math
- [x] camera projection
- [x] actuator rate limits
- [x] native Step-1 tests

## Target motion truth
- [x] `TargetState` = position_m + velocity_mps (own header)
- [x] `Trajectory` abstract interface (time -> state, no owned clock)
- [x] stationary trajectory
- [x] linear constant-velocity trajectory
- [x] sinusoidal trajectory with analytic velocity
- [x] deterministic analytic tests + smoke

## Perception baseline
- [ ] OpenCV renderer
- [ ] threshold detector
- [ ] centroid
- [ ] loss state

## Control
- [ ] PID class
- [ ] anti-windup
- [ ] closed-loop runner

## Evidence
- [ ] telemetry CSV
- [ ] performance metrics
- [ ] scenario suite
- [ ] demo overlay
- [ ] freeze `v1_baseline`

## Post-baseline
- [ ] Eigen-backed UKF
- [ ] MPC
- [ ] vibration PSD model
- [ ] Zernike/phase disturbance
- [ ] advanced detection

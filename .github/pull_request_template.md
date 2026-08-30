## What changed

## Owning module
- [ ] geometry/camera
- [ ] trajectory
- [ ] perception
- [ ] control
- [ ] integration
- [ ] telemetry/testing
- [ ] docs/build

## Engineering assumptions / units

## Validation
- [ ] `cmake --preset debug`
- [ ] `cmake --build --preset debug`
- [ ] `ctest --preset debug`
- [ ] relevant demo/smoke executable run

## Architecture checks
- [ ] no world-truth leakage into controller
- [ ] no rendering/control coupling
- [ ] no coordinate/unit convention change without tests/docs
- [ ] no pre-baseline UKF/MPC scope creep

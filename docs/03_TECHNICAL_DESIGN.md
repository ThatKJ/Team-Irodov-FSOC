# Technical Design

## Layering

```text
apps/sim_mvp
   |
SimulationRunner
   |---- Trajectory -> Environment truth
   |---- PanTiltCamera -> projection/FOV/actuator
   |---- BeaconRenderer (OpenCV) -> image
   |---- CentroidDetector (OpenCV) -> measurement
   |---- PIDController -> rate commands
   `---- TelemetryLogger -> CSV/console
```

`fsoc_core` starts as dependency-free C++ math/physics. Later libraries should be split by responsibility if build boundaries become useful (`fsoc_perception`, `fsoc_control`, etc.).

## Timing
Use a deterministic fixed simulation step such as 10–20 ms. Wall-clock FPS is telemetry; it must not silently alter the simulated physics timestep in baseline experiments.

## C++ data philosophy
Prefer small structs with units in names, `std::optional` for missing measurements, RAII ownership, and explicit interfaces. Do not introduce a framework hierarchy before it is needed.

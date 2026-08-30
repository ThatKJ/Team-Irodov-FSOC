# Interface Contracts

Planned contracts should preserve this information flow:

```cpp
TargetState trajectory.sample(double sim_time_s);
std::optional<Projection> camera.project(Vec3 target_world_m);
Image renderer.render(std::optional<Projection> projection, ...);
Detection detector.detect(const Image& frame);
TrackingError error_from_detection(const Detection&, const CameraModel&);
ControlCommand pid.update(const TrackingError&, double dt_s);
AppliedRates camera.step(command.pan_rate_rad_s, command.tilt_rate_rad_s, dt_s);
telemetry.record(...);
```

The exact types may evolve, but dependencies must point in this direction. In particular:
- detector never receives world truth,
- controller never receives target XYZ,
- camera never owns PID gains,
- telemetry never controls behavior.

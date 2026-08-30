# Experiment Protocol

For every comparative run record:
- git commit
- scenario name
- fixed timestep
- camera FOV/resolution
- actuator limits
- trajectory parameters
- PID gains
- random seed if noise is enabled
- run duration
- visibility ratio
- RMS/95th percentile error
- saturation ratio
- FPS

Never tune parameters and report the old configuration. Save configs/telemetry for the judge-facing benchmark.

# Software Requirements Specification

## Components

### Core geometry
Transforms world target position to camera coordinates and pinhole image coordinates.

### Environment
Owns world truth state. It must not decide camera commands.

### Trajectory engine
Given simulation time, returns target truth state. Deterministic by default.

### Camera
Owns pan/tilt state, projection parameters, and actuator constraints.

### Observation renderer
Converts a valid projection into a synthetic grayscale camera frame. OpenCV C++ belongs here, not in the controller.

### Detector
Consumes an image and produces `Detection{found, centroid, confidence}`. Baseline uses thresholding + moments/centroid.

### Controller
Consumes measured image/angular error and produces rate commands. Baseline PID must support reset and output clamping.

### Simulation runner
Sequences modules at fixed `dt`, timestamps data, and does not contain domain equations that belong elsewhere.

### Telemetry
Records observations/commands/metrics without mutating behavior.

## Reliability rules
- invalid `dt <= 0` is rejected,
- out-of-FOV projection is explicit absence,
- target loss is not represented by fake `(0,0)` centroid,
- all angular quantities are radians internally,
- saturation happens in the actuator/camera layer even if controller output is also limited.

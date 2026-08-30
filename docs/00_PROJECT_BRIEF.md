# Project Brief

Build a local macOS **C++20** simulation proving coarse alignment of a mobile FSOC terminal. A virtual pan/tilt camera must acquire and maintain a moving beacon within a narrow field of view using image-derived feedback.

This is a control-system MVP, not a YOLO demo.

Closed loop:

`Environment -> Camera observation -> Detection -> State/measurement -> Control -> Pan/Tilt -> next observation`

48-hour baseline: deterministic target, pinhole camera, synthetic beacon image, threshold/centroid detection, PID steering, telemetry, benchmark scenarios.

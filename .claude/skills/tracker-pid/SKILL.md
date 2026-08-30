---
name: tracker-pid
description: Implement and validate the baseline C++ image-error to PID pan/tilt controller.
---
Consume only image-derived measurement/error types. Keep PID free of OpenCV and world truth. Validate dt, clamp outputs, support reset and anti-windup, and unit-test each axis. Do not implement MPC or UKF.

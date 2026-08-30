# SIH Demo and Judging Story

Lead with the engineering problem: FSOC links are highly directional, so coarse alignment must keep the remote beacon inside a narrow optical field of view despite relative motion and platform disturbance.

Demo sequence:
1. show open-loop target drift/loss,
2. show camera FOV and measured centroid,
3. enable closed loop,
4. show PID commands and pan/tilt actuation,
5. show error curve/telemetry improvement,
6. trigger a harder trajectory or saturation case,
7. explain that UKF/MPC/turbulence are deliberate next layers built on the validated plant/perception/control interfaces.

Avoid presenting a generic object detector as the innovation. The strength is the end-to-end closed-loop system model and measurable alignment performance.

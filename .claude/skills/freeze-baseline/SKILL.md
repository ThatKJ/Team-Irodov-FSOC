---
name: freeze-baseline
description: Verify and freeze the validated C++ centroid+PID baseline before advanced UKF/MPC/turbulence work.
---
Confirm acceptance metrics and reproducible build/tests. Ensure the controller uses detector measurements, preserve telemetry/configs, then prepare the `v1_baseline` branch/tag workflow described in docs/13_GIT_WORKFLOW.md. Do not freeze a visually impressive but unmeasured system.

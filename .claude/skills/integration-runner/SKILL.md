---
name: integration-runner
description: Wire the tested C++ FSOC modules into the deterministic fixed-step closed-loop runner.
---
Orchestrate in strict order: trajectory -> environment -> camera projection -> renderer -> detector -> error -> controller -> actuator -> telemetry. Do not move domain equations into the loop. Keep visualization optional and behavior-neutral.

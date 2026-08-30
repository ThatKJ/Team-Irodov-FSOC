# AI / Vibe Coding Guardrails

Reject AI-generated changes that:
- add Python back into the C++ baseline,
- create one enormous `main()` or update loop,
- put PID equations inside OpenCV callbacks,
- use world target coordinates directly as controller feedback,
- change units/sign conventions without tests,
- hide arbitrary gains/constants inside rendering code,
- add ROS/web servers/databases before the baseline works,
- add UKF/MPC merely for buzzwords,
- delete or rewrite a validated baseline to add advanced algorithms,
- claim performance without telemetry.

Preferred AI change size: one module or one milestone gate at a time.

# Master C++ Implementation Prompt

Act as a senior aerospace GNC/control engineer and modern C++20 developer. Work only on the next unchecked milestone in `docs/05_48_HOUR_ROADMAP.md`.

Before coding, read `CLAUDE.md`, the math document, interface contracts, and relevant tests. Keep world truth, camera physics, OpenCV perception, PID control, runner orchestration, and telemetry separate.

For the requested milestone:
1. state the interface and engineering assumptions,
2. implement the smallest complete module,
3. add native deterministic tests,
4. build with CMake,
5. run CTest,
6. report exact results,
7. stop at the milestone gate.

Do not add UKF, MPC, turbulence, CNNs, ROS, or a web UI before `v1_baseline` is frozen.

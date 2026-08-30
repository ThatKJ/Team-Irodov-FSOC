# Debugging Prompt

Debug this as a control/geometry defect, not by random parameter tweaking.

1. Reproduce with a deterministic scenario.
2. Identify the owning layer: truth, coordinate transform, projection, detector, PID, actuator, runner, or telemetry.
3. Inspect units and sign conventions first.
4. Add a failing minimal native test.
5. Fix the owning module only.
6. Run CMake build + CTest + relevant executable.
7. Explain root cause and why the new test prevents regression.

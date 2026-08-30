# Claude Code Usage

Start Claude from repository root so `CLAUDE.md` and `.claude/` policies are in scope.

Recommended first request:

```text
/environment-camera Audit the existing Step-1 C++ implementation against docs/04_COORDINATES_AND_MATH.md. Run the CMake debug build, CTest, and step1_math_smoke. Do not implement trajectory code until the Step-1 gate is green.
```

Then use milestones in order:

```text
/target-trajectory Implement Step 2 only.
/tracker-pid Implement the PID module only after the perception measurement contract exists.
/integration-runner Wire already-tested components together without moving domain logic into the runner.
/telemetry-testing Validate nominal and stress scenarios and compute baseline metrics.
/freeze-baseline Freeze the validated C++ centroid+PID baseline before UKF/MPC work.
```

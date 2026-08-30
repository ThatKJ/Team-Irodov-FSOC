# AGENTS.md

All coding agents in this repository must follow `CLAUDE.md`.

## Repository-wide invariants

- C++20 only for production code.
- CMake is the source of build truth.
- Physics, perception, control, and telemetry are separate modules.
- Radians internally; SI units for physical quantities.
- A renderer/detector must never mutate target truth.
- A controller must receive measurements/errors, not world-truth coordinates.
- Deterministic tests precede visual polish.

## Required response pattern from coding agents

When implementing a task, report:
1. files changed
2. engineering assumption(s)
3. tests added/updated
4. exact build/test commands run
5. observed result
6. next milestone gate

Never say something is working without compiling/running the relevant test when execution access exists.

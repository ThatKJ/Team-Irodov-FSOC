# Architecture Decision Log

## ADR-001 — Core language: C++20
**Status:** Accepted

The project has migrated from the original Python starter to modern C++20. Reasons: deterministic timing, direct mapping to embedded/robotics/GNC implementation, stronger type boundaries, and better long-term fit for real-time control work.

Consequence: Python packaging files and workflows are forbidden unless this decision is explicitly superseded.

## ADR-002 — Step-1 math has zero third-party dependencies
**Status:** Accepted

The foundation uses small auditable vector primitives. OpenCV is deferred until pixels; Eigen is deferred until matrix-heavy UKF/MPC work.

## ADR-003 — Fixed world/camera coordinate convention
**Status:** Accepted

World +X forward, +Y right, +Z up. Camera +x right, +y up, +z forward. Image v increases downward.

## ADR-004 — Ground truth never feeds operational controller
**Status:** Accepted

Ideal target angles exist only for diagnostics/tests. The baseline controller must operate from image-derived centroid error.

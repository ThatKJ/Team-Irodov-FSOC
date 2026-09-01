# Git Workflow

Recommended branches:
- `main` — stable demonstration
- short-lived feature branches: `feat/trajectory`, `feat/renderer`, `feat/pid`, etc.

Baseline tag:
- **`v1_baseline` — created and pushed to `origin`.** It points at the merged Step‑10
  baseline (`20c028c`), the validated centroid + PID FSOC baseline. It is a frozen
  reference and is **never moved or recreated**; later work (Step 11 demo/frontend
  packaging, then UKF/MPC/turbulence) is layered on top.

Before merge:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Develop UKF/MPC on new branches off the stable integration line; never erase the reference implementation. To inspect the frozen baseline: `git checkout v1_baseline` (detached HEAD).

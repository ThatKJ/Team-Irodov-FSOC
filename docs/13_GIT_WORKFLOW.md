# Git Workflow

Recommended branches:
- `main` — stable demonstration
- short-lived feature branches: `feat/trajectory`, `feat/renderer`, `feat/pid`, etc.
- `v1_baseline` — frozen once baseline acceptance passes

Before merge:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

After baseline success:

```bash
git checkout -b v1_baseline
git push -u origin v1_baseline
git tag -a v1-baseline -m "Validated centroid + PID FSOC baseline"
git push origin v1-baseline
```

Develop UKF/MPC on new branches off the stable integration line; never erase the reference implementation.

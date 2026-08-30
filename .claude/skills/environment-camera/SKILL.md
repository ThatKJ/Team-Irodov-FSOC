---
name: environment-camera
description: Build or audit the C++ mathematical environment, coordinate transforms, virtual pinhole camera, FOV, and pan/tilt actuator kinematics.
---
Read CLAUDE.md and docs/04_COORDINATES_AND_MATH.md. Work in include/fsoc and src only as required. Keep controller/perception out. Add or update tests. Run `cmake --preset debug`, `cmake --build --preset debug`, `ctest --preset debug`, and `./build/debug/step1_math_smoke`. Stop when the Step-1 gate is green.

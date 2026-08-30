# Risk Register

| Risk | Effect | Mitigation |
|---|---|---|
| Sign/frame mistake | controller moves target away from center | frozen coordinate doc + unit tests |
| God-loop architecture | impossible to upgrade to UKF/MPC cleanly | strict module ownership |
| Ground-truth shortcut | fake closed-loop demo | forbid ideal-angle feedback in controller |
| OpenCV UI coupled to physics | nondeterminism/headless failures | rendering adapter only |
| Aggressive PID gains | oscillation/saturation | tune from slow target upward; log saturation |
| Target leaves FOV | no centroid | explicit lost state + acquisition/re-entry policy |
| Scope creep | baseline unfinished | hard post-baseline gate for advanced features |
| C++ build friction on teammate Mac | demo failure | CMake presets + clean Mac validation |

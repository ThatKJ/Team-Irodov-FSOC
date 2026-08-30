# Future Architecture After `v1_baseline`

Upgrade in controlled layers:

1. **State estimation:** add Eigen and implement EKF/UKF behind an estimator interface.
2. **Motion prediction:** future LOS/centroid prediction with uncertainty.
3. **Control:** add constrained MPC alongside PID, not as a destructive rewrite.
4. **Disturbance model:** vibration from PSD-shaped stochastic processes.
5. **Atmospheric optics:** Zernike/phase-screen approximations at the appropriate observation layer.
6. **Detection:** spatio-temporal anomaly detector for unresolved targets; lightweight CNN only for resolved targets where justified.
7. **Actuator realism:** acceleration, latency, backlash, quantization.
8. **Monte Carlo validation:** seeded scenario sweeps and comparative plots.

Keep PID as the interpretable reference baseline throughout judging.

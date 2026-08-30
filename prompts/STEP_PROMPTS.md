# Milestone Prompts

## Step 1 audit
`Audit camera geometry and actuator kinematics. Build/test only; fix defects if found. No OpenCV.`

## Step 2 trajectory
`Implement deterministic C++ target trajectory module with linear and sinusoidal trajectories, analytic tests, and no rendering/control dependencies.`

## Step 4 renderer
`Add OpenCV C++ synthetic grayscale beacon renderer behind a perception-layer interface. It receives projection data; it cannot read or modify world truth.`

## Step 5 detector
`Implement threshold + centroid detector from image pixels only. Return explicit detection absence.`

## Step 6 PID
`Implement standalone pan/tilt PID controllers with timestep validation, output clamps, reset behavior, and anti-windup. Test without OpenCV.`

## Step 7 integration
`Create a fixed-step SimulationRunner that wires tested modules in closed-loop order. No equations that belong to domain modules may be embedded in the runner.`

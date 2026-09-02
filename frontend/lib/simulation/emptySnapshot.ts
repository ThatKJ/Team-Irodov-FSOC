import type { DemoSnapshot } from "@/lib/telemetry/types";
import { CAMERA } from "@/lib/baseline/constants";

/** neutral frame shown before telemetry loads — target lost, nothing detected */
export const EMPTY_SNAPSHOT: DemoSnapshot = {
  simulationTime: 0,
  frame: 0,
  trackingState: "TARGET_LOST",
  target: { position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 } },
  camera: {
    panDeg: 0,
    tiltDeg: 0,
    panRateDegS: 0,
    tiltRateDegS: 0,
    horizontalFovDeg: CAMERA.horizontalFovDeg,
    verticalFovDeg: CAMERA.verticalFovDeg,
  },
  detection: { detected: false, xPx: null, yPx: null },
  tracking: {
    errorXPx: null,
    errorYPx: null,
    panErrorDeg: null,
    tiltErrorDeg: null,
    totalErrorDeg: null,
  },
  control: {
    panCommandDegS: 0,
    tiltCommandDegS: 0,
    panSaturated: false,
    tiltSaturated: false,
  },
  detectionErrorPx: null,
  targetVisible: false,
};

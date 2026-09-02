/**
 * Derives the event log from telemetry TRANSITIONS. Never random, never faked —
 * every entry is a state change already present in the C++ frames.
 */

import type { DemoSnapshot, SimEvent } from "@/lib/telemetry/types";

export function deriveEvents(frames: DemoSnapshot[]): SimEvent[] {
  const out: SimEvent[] = [];
  if (frames.length === 0) return out;

  const push = (f: DemoSnapshot, kind: SimEvent["kind"], label: string, severity: SimEvent["severity"]) =>
    out.push({ frame: f.frame, time: f.simulationTime, kind, label, severity });

  const first = frames[0];
  push(first, "RUN_START", "CMD: TRACK_INIT", "info");
  if (first.detection.detected) {
    push(first, "TRACK_ACQUIRED", "CENTROID ACQUIRED · TRACK LOOP CLOSED", "nominal");
  }

  let prev = first;
  let anyLimitActive = false;
  for (let i = 1; i < frames.length; i++) {
    const f = frames[i];

    if (prev.detection.detected && !f.detection.detected) {
      push(f, "TARGET_LOST", "TARGET LOST — beacon outside FOV", "critical");
    }
    if (!prev.detection.detected && f.detection.detected) {
      push(f, "REACQUIRED", "TARGET REACQUIRED — track loop resumed", "nominal");
    }

    if (!prev.control.panSaturated && f.control.panSaturated) {
      push(f, "PAN_LIMIT", "PAN RATE LIMIT REACHED", "warning");
      anyLimitActive = true;
    }
    if (!prev.control.tiltSaturated && f.control.tiltSaturated) {
      push(f, "TILT_LIMIT", "TILT RATE LIMIT REACHED", "warning");
      anyLimitActive = true;
    }
    if (
      anyLimitActive &&
      !f.control.panSaturated &&
      !f.control.tiltSaturated &&
      (prev.control.panSaturated || prev.control.tiltSaturated)
    ) {
      push(f, "RATE_NOMINAL", "STEERING: RATE NOMINAL", "nominal");
      anyLimitActive = false;
    }

    prev = f;
  }

  const last = frames[frames.length - 1];
  push(last, "RUN_END", "RUN COMPLETE", "info");
  return out;
}

export const EVENT_COLOR: Record<SimEvent["severity"], string> = {
  nominal: "var(--tracking)",
  info: "var(--muted)",
  warning: "var(--warning)",
  critical: "var(--lost)",
};

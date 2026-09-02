/**
 * Baseline constants — sourced verbatim from the frozen C++ v1_baseline and its
 * documentation. NOTHING here is recomputed physics; these are published facts:
 *
 *   - docs/16_BASELINE_ACCEPTANCE.md   (Step-10 gates + measured results)
 *   - docs/17_DEMO_FREEZE.md           (DemoScenario table + expected results)
 *   - docs/18_FRONTEND_DATA_CONTRACT.md (units, snapshot shape)
 *   - include/fsoc/config.hpp          (CameraConfig defaults)
 *   - src/simulation_runner.cpp        (baseline_runner_config: kp=12, ki=0, kd=0)
 *
 * The frontend visualizes reality; the C++ engine decides it.
 */

export const SIM_RATE_HZ = 50;
export const SIM_DT_S = 0.02;

export const PID_GAINS = { kp: 12, ki: 0, kd: 0 } as const;

export const CAMERA = {
  widthPx: 640,
  heightPx: 480,
  principalPointPx: { x: 320, y: 240 },
  horizontalFovDeg: 20.0,
  verticalFovDeg: 15.0,
  maxPanRateDegS: 30.0,
  maxTiltRateDegS: 30.0,
  tiltStopsDeg: { min: -80, max: 80 },
} as const;

export const IMAGE_HALF_DIAGONAL_PX = Math.hypot(
  CAMERA.widthPx / 2,
  CAMERA.heightPx / 2,
); // 400 px — PRD RMS gate is 10% of this = 40 px

export type ScenarioId = "static" | "sinusoidal" | "loss" | "open" | "closed";

export const SCENARIO_IDS: ScenarioId[] = [
  "static",
  "sinusoidal",
  "loss",
  "open",
  "closed",
];

export interface ScenarioMeta {
  id: ScenarioId;
  /** DemoScenario enum name in fsoc/demo.hpp */
  demoScenario: string;
  /** Stitch label / display name */
  label: string;
  /** short one-liner from `fsoc_demo --help` */
  blurb: string;
  controlEnabled: boolean;
  durationS: number;
  frames: number;
  /** deterministic Step-10 result for this scenario (docs/17) */
  expected: {
    detectionPct: number;
    rmsDeg: number;
    p95Deg: number;
    maxDeg: number;
    lostFrames: number;
  };
}

export const SCENARIOS: Record<ScenarioId, ScenarioMeta> = {
  static: {
    id: "static",
    demoScenario: "StaticAcquisition",
    label: "Static Acquisition",
    blurb: "initial coarse alignment onto a stationary terminal",
    controlEnabled: true,
    durationS: 4,
    frames: 200,
    expected: { detectionPct: 100.0, rmsDeg: 0.4691, p95Deg: 0.2885, maxDeg: 4.1275, lostFrames: 0 },
  },
  sinusoidal: {
    id: "sinusoidal",
    demoScenario: "SinusoidalTracking",
    label: "Sinusoidal Tracking",
    blurb: "closed-loop tracking of a nonlinear moving target (+/-12.4 deg sweep)",
    controlEnabled: true,
    durationS: 20,
    frames: 1000,
    expected: { detectionPct: 100.0, rmsDeg: 0.5461, p95Deg: 0.7899, maxDeg: 0.7982, lostFrames: 0 },
  },
  loss: {
    id: "loss",
    demoScenario: "LossReacquisition",
    label: "Target Loss & Re-entry",
    blurb: "deliberate target loss and natural re-entry (baseline hold policy)",
    controlEnabled: true,
    durationS: 8,
    frames: 400,
    expected: { detectionPct: 73.2, rmsDeg: 4.9328, p95Deg: 8.8284, maxDeg: 9.9692, lostFrames: 107 },
  },
  open: {
    id: "open",
    demoScenario: "OpenLoop",
    label: "Open Loop",
    blurb: "controller disabled reference run (open loop)",
    controlEnabled: false,
    durationS: 20,
    frames: 1000,
    expected: { detectionPct: 57.4, rmsDeg: 6.4549, p95Deg: 9.8112, maxDeg: 10.1916, lostFrames: 426 },
  },
  closed: {
    id: "closed",
    demoScenario: "ClosedLoop",
    label: "Closed Loop",
    blurb: "same trajectory as 'open' with the controller enabled",
    controlEnabled: true,
    durationS: 20,
    frames: 1000,
    expected: { detectionPct: 100.0, rmsDeg: 0.5461, p95Deg: 0.7899, maxDeg: 0.7982, lostFrames: 0 },
  },
};

/**
 * Open-vs-closed benchmark — validated nominal comparison (docs/17 + Step 10).
 * Do NOT invent "AI improvement" numbers.
 */
export const BENCHMARK = {
  open: { detectionPct: 57.4, rmsDeg: 6.4549, p95Deg: 9.8112, maxDeg: 10.1916, lostFrames: 426, frames: 1000 },
  closed: { detectionPct: 100.0, rmsDeg: 0.5461, p95Deg: 0.7899, maxDeg: 0.7982, lostFrames: 0, frames: 1000 },
  rmsImprovementFactor: 6.4549 / 0.5461, // ~11.82x
  detectionImprovementPts: 100.0 - 57.4, // +42.6 pts
} as const;

/**
 * Step-10 baseline acceptance — the 7 validated scenarios and their measured
 * results (docs/16_BASELINE_ACCEPTANCE.md / generated VALIDATION_REPORT.md).
 * Every scenario PASSED. Loss/Re-entry intentionally carries lost frames — that
 * is expected loss-semantics behaviour, not a test failure.
 */
export interface ValidationRow {
  id: string;
  scenario: string;
  detectionPct: number;
  rmsDeg: number | null;
  p95Deg: number | null;
  maxDeg: number | null;
  finalDeg: number;
  lostFrames: number;
  frames: number;
  status: "PASS";
  note?: string;
}

export const VALIDATION_ROWS: ValidationRow[] = [
  { id: "01", scenario: "Static Acquisition", detectionPct: 100.0, rmsDeg: 0.4691, p95Deg: 0.2885, maxDeg: 4.1275, finalDeg: 0.0, lostFrames: 0, frames: 200, status: "PASS" },
  { id: "02", scenario: "Slow Linear Tracking", detectionPct: 100.0, rmsDeg: 0.3908, p95Deg: 0.1028, maxDeg: 4.8863, finalDeg: 0.1013, lostFrames: 0, frames: 500, status: "PASS" },
  { id: "03", scenario: "Sinusoidal Tracking", detectionPct: 100.0, rmsDeg: 0.5461, p95Deg: 0.7899, maxDeg: 0.7982, finalDeg: 0.5942, lostFrames: 0, frames: 1000, status: "PASS" },
  { id: "04", scenario: "Near-FOV-Edge Acquisition", detectionPct: 100.0, rmsDeg: 1.5613, p95Deg: 2.8305, maxDeg: 9.9273, finalDeg: 0.0, lostFrames: 0, frames: 200, status: "PASS" },
  { id: "05", scenario: "Actuator Saturation", detectionPct: 100.0, rmsDeg: 1.8189, p95Deg: 3.2724, maxDeg: 11.3779, finalDeg: 0.0, lostFrames: 0, frames: 200, status: "PASS" },
  { id: "06", scenario: "Target Loss & Re-entry", detectionPct: 73.2, rmsDeg: 4.9328, p95Deg: 8.8284, maxDeg: 9.9692, finalDeg: 2.3898, lostFrames: 107, frames: 400, status: "PASS", note: "Intentional FOV exit; loss/hold/re-acquire semantics validated (not a tracking-quality gate)." },
  { id: "07", scenario: "Open vs Closed Loop", detectionPct: 100.0, rmsDeg: 0.5461, p95Deg: 0.7899, maxDeg: 0.7982, finalDeg: 0.5942, lostFrames: 0, frames: 1000, status: "PASS", note: "Closed loop beats open loop by 11.8x RMS / +42.6 pts detection on the same trajectory." },
];

export const VALIDATION_SUMMARY = {
  passed: 7,
  total: 7,
  reportId: "SIH26169.STEP10.BASELINE-ACCEPTANCE",
  verdict: "STEP 10 BASELINE ACCEPTANCE: PASS",
  baselineTag: "v1_baseline",
} as const;

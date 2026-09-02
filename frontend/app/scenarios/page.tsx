"use client";

import { useMemo, useState } from "react";
import { useRouter } from "next/navigation";
import Link from "next/link";
import {
  Activity,
  AlertTriangle,
  Crosshair,
  Play,
  RefreshCw,
  Repeat,
  ScanLine,
  Waves,
} from "lucide-react";

import { Screen } from "@/components/shell/AppShell";
import { Button } from "@/components/ui/Button";
import { SCENARIOS, type ScenarioId } from "@/lib/baseline/constants";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { deg, fixed, pct } from "@/lib/format";
import { cn } from "@/lib/cn";

interface Module {
  code: string;
  tier: "T0" | "T1" | "T2" | "T3";
  name: string;
  icon: typeof Waves;
  /** runnable DemoScenario id, or null for validation-only scenarios */
  scenario: ScenarioId | null;
  trajectory: string;
  description: string;
  /** validated Step-10 result */
  result: { detectionPct: number; rmsDeg: number | null; lostFrames: number; frames: number };
}

/**
 * Stitch lists 8 modules. Five map to `fsoc_demo` DemoScenarios and are RUNNABLE;
 * three are Step-10 validation-only (no CLI preset) and link to /validation.
 * (documented in STITCH_IMPLEMENTATION_MAP.md)
 */
const MODULES: Module[] = [
  {
    code: "MOD-01",
    tier: "T0",
    name: "STATIC ACQUISITION",
    icon: Crosshair,
    scenario: "static",
    trajectory: "STATIC (0 Hz) · {100, 6, 4} m",
    description:
      "Initializes terminal acquisition with a stationary target ~4.1° off boresight. Evaluates sensor noise floor, sub-pixel centroiding, and FSM/gimbal settling on closed-loop engagement.",
    result: { detectionPct: 100, rmsDeg: 0.4691, lostFrames: 0, frames: 200 },
  },
  {
    code: "MOD-02",
    tier: "T1",
    name: "SLOW LINEAR TRACKING",
    icon: Activity,
    scenario: null,
    trajectory: "LINEAR · v = {0, 2.0, 0.8} m/s",
    description:
      "Constant-velocity terminal traversal. Validates continual tracking and steady-state lag under smooth motion. Step-10 validation scenario (no CLI preset).",
    result: { detectionPct: 100, rmsDeg: 0.3908, lostFrames: 0, frames: 500 },
  },
  {
    code: "MOD-03",
    tier: "T1",
    name: "SINUSOIDAL TRACKING",
    icon: Waves,
    scenario: "sinusoidal",
    trajectory: "SINUSOID · ±12.4° Y sweep · 0.12 Hz",
    description:
      "Nonlinear periodic motion simulating orbital characteristics / platform sway. Validates loop stability and feed-forward tracking under continuous predictable motion.",
    result: { detectionPct: 100, rmsDeg: 0.5461, lostFrames: 0, frames: 1000 },
  },
  {
    code: "MOD-04",
    tier: "T2",
    name: "NEAR FOV EDGE",
    icon: ScanLine,
    scenario: null,
    trajectory: "STATIC · ~88% of half-FOV",
    description:
      "Acquisition from the operational FOV boundary. Verifies command direction and convergence from a large initial offset. Step-10 validation scenario (no CLI preset).",
    result: { detectionPct: 100, rmsDeg: 1.5613, lostFrames: 0, frames: 200 },
  },
  {
    code: "MOD-05",
    tier: "T3",
    name: "ACTUATOR SATURATION",
    icon: AlertTriangle,
    scenario: null,
    trajectory: "STATIC · ~11° initial error",
    description:
      "Drives the PID command past the 30°/s actuator rate limit for several frames. Verifies correct behaviour while rate-limited and clean recovery. Step-10 validation scenario (no CLI preset).",
    result: { detectionPct: 100, rmsDeg: 1.8189, lostFrames: 0, frames: 200 },
  },
  {
    code: "MOD-06",
    tier: "T2",
    name: "LOSS / RE-ENTRY",
    icon: RefreshCw,
    scenario: "loss",
    trajectory: "AGGRESSIVE SINUSOID · 0.30 Hz",
    description:
      "Target deliberately leaves the FOV. Validates lost-target semantics: PID reset, zero command, camera hold, and natural re-acquisition. Lost frames are expected here.",
    result: { detectionPct: 73.2, rmsDeg: 4.9328, lostFrames: 107, frames: 400 },
  },
  {
    code: "MOD-07",
    tier: "T0",
    name: "OPEN LOOP",
    icon: Play,
    scenario: "open",
    trajectory: "NOMINAL SINUSOID · control_enabled = false",
    description:
      "Controller disabled reference run over the nominal trajectory. The uncompensated baseline for the open-vs-closed comparison.",
    result: { detectionPct: 57.4, rmsDeg: 6.4549, lostFrames: 426, frames: 1000 },
  },
  {
    code: "MOD-08",
    tier: "T1",
    name: "CLOSED LOOP",
    icon: Repeat,
    scenario: "closed",
    trajectory: "NOMINAL SINUSOID · control_enabled = true",
    description:
      "Identical trajectory to OPEN LOOP with the PID controller enabled. 11.8× lower RMS error, +42.6 pts detection.",
    result: { detectionPct: 100, rmsDeg: 0.5461, lostFrames: 0, frames: 1000 },
  },
];

export default function ScenariosPage() {
  const router = useRouter();
  const { scenario, setScenario, runScenario } = useSimulation();
  const [selectedCode, setSelectedCode] = useState("MOD-03");
  const selected = useMemo(
    () => MODULES.find((m) => m.code === selectedCode) ?? MODULES[2],
    [selectedCode],
  );
  const runnable = selected.scenario != null;
  const durationS = runnable ? SCENARIOS[selected.scenario as ScenarioId].durationS : null;

  return (
    <Screen pad>
      {/* decorative spinner */}
      <div className="pointer-events-none absolute right-margin-md top-[64px] opacity-20">
        <svg className="animate-spin-slow text-on-surface" width="200" height="200" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="48" fill="none" stroke="currentColor" strokeDasharray="2 4" strokeWidth="0.5" />
          <circle cx="50" cy="50" r="40" fill="none" stroke="currentColor" strokeWidth="0.5" />
          <path d="M50 0 L50 100 M0 50 L100 50" opacity="0.5" stroke="currentColor" strokeWidth="0.5" />
        </svg>
      </div>

      <div className="z-10 mb-margin-md flex flex-col gap-unit border-l border-primary pl-margin-md">
        <h1 className="font-display-telem text-display-telem text-primary">SCENARIO SELECTION</h1>
        <p className="max-w-2xl font-body-md text-body-md text-on-surface-variant">
          Initialize a validated tracking simulation module. Parameters are copied verbatim
          from the frozen v1_baseline (src/validation.cpp) — nothing is retuned.
        </p>
      </div>

      <div className="z-10 flex min-h-0 flex-1 flex-col gap-gutter border border-outline-variant bg-surface-container-low p-gutter lg:flex-row">
        {/* module list */}
        <div className="flex flex-col gap-gutter overflow-y-auto border-r border-outline-variant bg-surface pr-gutter lg:w-1/4">
          <div className="flex items-center justify-between border-b border-outline-variant bg-surface-container px-margin-md py-margin-sm">
            <span className="font-label-xs text-label-xs uppercase tracking-widest text-on-surface-variant">
              Available Modules
            </span>
            <span className="font-data-mono text-data-mono text-primary">
              {MODULES.length.toString().padStart(2, "0")}
            </span>
          </div>
          {MODULES.map((m) => {
            const Icon = m.icon;
            const active = m.code === selectedCode;
            const isLive = m.scenario === scenario;
            return (
              <button
                key={m.code}
                onClick={() => setSelectedCode(m.code)}
                className={cn(
                  "group flex w-full items-start gap-margin-sm border-l-2 px-margin-md py-margin-md text-left transition-colors",
                  active
                    ? "border-primary bg-surface-container-high hover:bg-surface-bright"
                    : "border-transparent bg-surface-container hover:bg-surface-container-high",
                )}
              >
                <Icon
                  className={cn(
                    "mt-[2px] h-[18px] w-[18px] shrink-0 transition-opacity",
                    active ? "text-primary opacity-100" : "text-on-surface-variant opacity-50 group-hover:opacity-100",
                  )}
                  strokeWidth={1.75}
                />
                <span className="flex flex-col">
                  <span className="font-headline-sm text-headline-sm uppercase text-on-surface">{m.name}</span>
                  <span className="font-data-mono text-data-mono text-on-surface-variant opacity-70">
                    {m.code}
                    {" ∕ "}
                    {m.tier}
                    {!m.scenario && " · VALIDATION-ONLY"}
                    {isLive && " · ACTIVE"}
                  </span>
                </span>
              </button>
            );
          })}
        </div>

        {/* detail */}
        <div className="relative flex flex-1 flex-col overflow-y-auto bg-surface p-margin-md">
          <div className="pointer-events-none absolute inset-0 bg-[radial-gradient(ellipse_at_center,_rgba(111,238,225,0.05),_transparent_70%)]" />
          <div className="z-10 mb-margin-md flex items-start justify-between">
            <div className="flex flex-col gap-unit">
              <div className="flex items-center gap-margin-sm">
                <span className="block h-3 w-3 bg-primary" />
                <h2 className="font-display-telem text-display-telem text-primary">{selected.name}</h2>
              </div>
              <span className="ml-margin-md font-data-mono text-data-mono text-on-surface-variant">
                {`ID: ${selected.code} · TIER ${selected.tier} · ${runnable ? "RUNNABLE" : "VALIDATION-ONLY"}`}
              </span>
            </div>
            <div className="flex gap-margin-sm">
              <Link href="/benchmarks">
                <Button variant="ghost">Compare</Button>
              </Link>
              {runnable ? (
                <>
                  <Button
                    variant="ghost"
                    onClick={() => setScenario(selected.scenario as ScenarioId)}
                  >
                    Load
                  </Button>
                  <Button
                    variant="primary"
                    onClick={() => {
                      runScenario(selected.scenario as ScenarioId);
                      router.push("/mission");
                    }}
                  >
                    <Play className="h-[16px] w-[16px]" fill="currentColor" strokeWidth={2} /> Run
                  </Button>
                </>
              ) : (
                <Link href="/validation">
                  <Button variant="ghost">View in Validation</Button>
                </Link>
              )}
            </div>
          </div>

          <div className="z-10 mb-margin-md grid grid-cols-1 gap-gutter border border-outline-variant bg-surface-container-low md:grid-cols-2">
            <div className="flex flex-col gap-margin-sm bg-surface-container p-margin-md">
              <span className="border-b border-outline-variant pb-unit font-label-xs text-label-xs uppercase tracking-widest text-on-surface-variant">
                Technical Description
              </span>
              <p className="mt-margin-sm font-body-md text-body-md text-on-surface">{selected.description}</p>
            </div>
            <div className="flex flex-col gap-margin-sm bg-surface-container p-margin-md">
              <span className="border-b border-outline-variant pb-unit font-label-xs text-label-xs uppercase tracking-widest text-on-surface-variant">
                Validated Result (Step 10)
              </span>
              <div className="mt-margin-sm grid grid-cols-2 gap-y-margin-sm">
                <Param k="DURATION" v={durationS != null ? `${durationS.toFixed(2)}s` : "—"} />
                <Param k="TRAJECTORY" v={selected.trajectory} tone="primary" />
                <Param k="DETECTION" v={pct(selected.result.detectionPct, 1)} />
                <Param k="RMS ERROR" v={selected.result.rmsDeg == null ? "—" : deg(selected.result.rmsDeg, 4)} tone="primary" />
                <Param k="LOST FRAMES" v={String(selected.result.lostFrames)} />
                <Param k="FRAMES" v={String(selected.result.frames)} />
              </div>
            </div>
          </div>

          {/* kinematic preview */}
          <div className="z-10 flex min-h-[280px] flex-1 flex-col border border-outline-variant bg-surface-container-highest">
            <div className="p-margin-sm">
              <span className="border border-outline-variant bg-surface px-unit py-[2px] font-label-xs text-label-xs uppercase text-on-surface-variant">
                Kinematic Preview
              </span>
            </div>
            <div className="relative flex flex-1 items-center justify-center">
              <svg className="h-[80%] max-w-[420px] opacity-80" viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet">
                <line x1="0" x2="100" y1="50" y2="50" stroke="#3c4947" strokeDasharray="1 2" strokeWidth="0.5" />
                <line x1="50" x2="50" y1="0" y2="100" stroke="#3c4947" strokeDasharray="1 2" strokeWidth="0.5" />
                <circle cx="50" cy="50" r="25" fill="none" stroke="#3c4947" strokeWidth="0.2" />
                <circle cx="50" cy="50" r="45" fill="none" stroke="#3c4947" strokeWidth="0.2" />
                <PreviewPath module={selected} />
                <rect x="5" y="5" width="90" height="90" fill="none" stroke="#869491" strokeWidth="0.5" />
                <path
                  d="M 5 15 L 5 5 L 15 5 M 85 5 L 95 5 L 95 15 M 95 85 L 95 95 L 85 95 M 15 95 L 5 95 L 5 85"
                  fill="none"
                  stroke="#dee4e2"
                  strokeWidth="1"
                />
              </svg>
            </div>
            <div className="flex items-center justify-between border-t border-outline-variant bg-surface p-margin-sm">
              <div className="flex gap-margin-md">
                <Param k="TRAJECTORY" v={selected.trajectory.split(" · ")[0]} tone="primary" />
                <Param k="CONTROL" v={runnable && SCENARIOS[selected.scenario as ScenarioId].controlEnabled ? "CLOSED" : "OPEN"} tone="primary" />
              </div>
              <div className="flex items-center gap-margin-sm font-data-mono text-data-mono text-on-surface-variant">
                <span className="h-2 w-2 animate-pulse bg-primary" /> PREVIEW
              </div>
            </div>
          </div>
        </div>
      </div>
    </Screen>
  );
}

function Param({ k, v, tone = "default" }: { k: string; v: string; tone?: "default" | "primary" }) {
  return (
    <div className="flex flex-col">
      <span className="font-label-xs text-label-xs text-on-surface-variant">{k}</span>
      <span className={cn("font-data-mono text-data-mono tnum", tone === "primary" ? "text-primary" : "text-on-surface")}>
        {v}
      </span>
    </div>
  );
}

function PreviewPath({ module }: { module: Module }) {
  // simple representative kinematic sketch per module (illustrative, not telemetry)
  if (module.name.includes("SINUSOID") || module.name.includes("LOSS") || module.name.includes("LOOP")) {
    return (
      <g>
        <path
          d="M 10 50 Q 30 20 50 50 T 90 50"
          fill="none"
          stroke="#6feee1"
          strokeWidth="0.8"
          strokeDasharray="2 1"
        />
        <circle cx="50" cy="50" r="2" fill="#6feee1" />
      </g>
    );
  }
  if (module.name.includes("LINEAR")) {
    return (
      <g>
        <line x1="20" y1="70" x2="80" y2="30" stroke="#6feee1" strokeWidth="0.8" strokeDasharray="2 1" />
        <circle cx="50" cy="50" r="2" fill="#6feee1" />
      </g>
    );
  }
  // static / edge / saturation
  const cx = module.name.includes("EDGE") ? 82 : module.name.includes("SATURATION") ? 80 : 62;
  const cy = module.name.includes("EDGE") || module.name.includes("SATURATION") ? 24 : 40;
  return (
    <g className="animate-pulse">
      <circle cx={cx} cy={cy} r="2" fill="#6feee1" />
      <circle cx={cx} cy={cy} r="5" fill="none" stroke="#6feee1" strokeDasharray="2 1" strokeWidth="0.5" />
      <line x1="50" y1="50" x2={cx} y2={cy} stroke="#ffd2a2" strokeWidth="0.5" strokeDasharray="1 1" />
    </g>
  );
}

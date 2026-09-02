"use client";

import {
  Activity,
  Camera,
  CircleDot,
  Cpu,
  Frame,
  GitBranch,
  Minus,
  Radar,
  Rocket,
  TriangleAlert,
} from "lucide-react";

import { Screen } from "@/components/shell/AppShell";
import { Panel, PanelHeader } from "@/components/ui/Panel";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { PID_GAINS, SIM_RATE_HZ } from "@/lib/baseline/constants";
import { deg, fixed, signed } from "@/lib/format";
import { cn } from "@/lib/cn";

/**
 * /architecture — Stitch "Architecture".
 * The closed-loop pipeline with the frozen PIXEL MEASUREMENT BOUNDARY: control
 * receives pixel-derived error ONLY. Actuator state + error read from live
 * telemetry; the structural diagram is fixed. (Diagram is a faithful React
 * re-layout of the Stitch node graph — the animated multi-segment feedback SVGs
 * are simplified; documented in STITCH_IMPLEMENTATION_MAP.md.)
 */
export default function ArchitecturePage() {
  const { current, meta } = useSimulation();

  return (
    <Screen>
      <div className="pointer-events-none absolute inset-0 opacity-20" style={{ backgroundImage: "radial-gradient(circle at 2px 2px, rgba(111,238,225,0.15) 1px, transparent 0)", backgroundSize: "32px 32px" }} />
      <div className="pointer-events-none absolute left-0 right-0 top-0 h-32 bg-gradient-to-b from-primary/5 to-transparent" />

      <div className="z-10 flex min-h-0 flex-1 flex-col gap-margin-md overflow-y-auto p-margin-md lg:flex-row">
        {/* left column */}
        <div className="flex shrink-0 flex-col gap-margin-md lg:w-1/4">
          <Panel className="relative overflow-hidden border-l-2 border-l-primary p-margin-md">
            <div className="absolute -right-8 -top-8 h-24 w-24 rounded-full bg-primary/10 blur-xl" />
            <div className="flex items-center gap-margin-sm">
              <GitBranch className="h-5 w-5 text-primary" strokeWidth={1.75} />
              <span className="font-display-telem text-headline-sm tracking-widest text-on-surface">SYS_ARCH</span>
            </div>
            <MetaRow k="Topology" v="Closed-Loop Optical Tracking" />
            <MetaRow k="Update Rate" v={`${SIM_RATE_HZ}.0 Hz (fixed dt)`} />
            <MetaRow k="Controller" v={`PID · kp=${PID_GAINS.kp} ki=${PID_GAINS.ki} kd=${PID_GAINS.kd}`} />
            <div className="mt-margin-sm flex items-center justify-between border-t border-outline-variant/30 pt-margin-sm">
              <span className="font-label-xs text-label-xs text-on-surface-variant">NODE STATUS</span>
              <span className="flex items-center gap-unit">
                <span className="h-2 w-2 bg-primary" />
                <span className="font-data-mono text-data-mono text-primary">NOMINAL</span>
              </span>
            </div>
          </Panel>

          <Panel className="flex flex-1 flex-col gap-margin-md border-0 bg-surface-container-low p-margin-md">
            <div className="flex items-center justify-between border-b border-outline-variant pb-margin-sm">
              <span className="font-display-telem text-headline-sm text-on-surface">SIGNAL INTEGRITY</span>
              <Activity className="h-4 w-4 text-on-surface-variant" strokeWidth={1.5} />
            </div>
            <Diag label="Detector Input" value={`${current.detectionErrorPx != null ? fixed(current.detectionErrorPx, 3) : "—"} px err`} />
            <Diag label="Error Calc" value={`Δ ${deg(current.tracking.totalErrorDeg, 3)}`} />
            <Diag
              label="Drive Output"
              value={`${signed(current.camera.panRateDegS, 2)} / ${signed(current.camera.tiltRateDegS, 2)} °/s`}
              warn={current.control.panSaturated || current.control.tiltSaturated}
            />
            <div className="relative mt-auto flex h-32 items-center justify-center overflow-hidden bg-surface-container-highest">
              {/* eslint-disable-next-line @next/next/no-img-element */}
              <img src="/demo/frame-static.png" alt="actuator link" className="absolute inset-0 h-full w-full object-cover opacity-30" />
              <div className="absolute inset-0 bg-gradient-to-t from-background via-transparent to-transparent" />
              <div className="absolute bottom-margin-sm left-margin-sm flex items-center gap-unit">
                <span className="h-1.5 w-1.5 animate-pulse bg-primary" />
                <span className="font-label-xs text-label-xs uppercase tracking-widest text-primary">
                  Actuator HW Link {meta ? "Active" : "…"}
                </span>
              </div>
            </div>
          </Panel>
        </div>

        {/* pipeline */}
        <div className="relative flex min-h-[560px] flex-1 flex-col bg-surface-container-low p-margin-lg shadow-xl">
          <div className="mb-margin-lg border-l-2 border-primary bg-surface-container/80 px-margin-md py-margin-sm">
            <span className="block font-headline-sm text-headline-sm uppercase tracking-wider text-on-surface">
              Control System Pipeline
            </span>
            <span className="font-label-xs text-label-xs text-on-surface-variant">
              Closed-Loop Kinematics · SEQ_ID 994.2A
            </span>
          </div>

          <div className="relative flex flex-1 flex-col justify-center gap-margin-lg border border-outline-variant/20 p-margin-lg">
            {/* zone label: truth */}
            <span className="pointer-events-none absolute left-margin-lg top-4 select-none font-display-telem text-[44px] font-bold uppercase leading-none text-on-surface-variant/[0.07]">
              Simulation / Truth
            </span>

            {/* top row: truth */}
            <div className="relative z-10 flex items-start justify-between gap-margin-md">
              <Node icon={Rocket} title="Target Trajectory" sub="TRUTH_POS (x,y,z)" tone="warning" dot />
              <Arrow />
              <Node icon={Camera} title="Camera Geometry" sub="PROJ_MATRIX [3×4]" tone="warning" />
              <Arrow />
              <Node icon={Frame} title="Optical Frame" sub={`IMG_BUFFER [${640}×${480}]`} tone="primary" big />
            </div>

            {/* boundary */}
            <div className="relative z-10 flex items-center">
              <div className="w-full border-t-2 border-dashed border-error/40" />
              <div className="absolute left-8 -top-3 flex items-center gap-2 bg-surface-container-low px-2 font-label-xs text-label-xs uppercase tracking-widest text-error">
                <TriangleAlert className="h-3.5 w-3.5" strokeWidth={1.75} />
                Pixel Measurement Boundary
              </div>
              <div className="absolute right-8 -top-3 border border-error/40 bg-surface-container-low px-3 py-1 font-data-mono text-[10px] uppercase text-error">
                Control receives pixel-derived error only
              </div>
            </div>

            <span className="pointer-events-none absolute bottom-4 left-margin-lg select-none font-display-telem text-[44px] font-bold uppercase leading-none text-on-surface-variant/[0.07]">
              Control / Measurement
            </span>

            {/* bottom row: control (reversed flow: Detector <- Error <- PID).
                right padding keeps Beacon Detector clear of the Actuator State panel */}
            <div className="relative z-10 flex flex-row-reverse items-start justify-between gap-margin-md pr-[248px]">
              <Node icon={Radar} title="Beacon Detector" sub="CENTROID (u,v)" tone="primary" dot />
              <Arrow reversed />
              <Node icon={Minus} title="Tracking Error" sub="ΔE = TARGET − CENTRE" tone="lost" round />
              <Arrow reversed />
              <Node icon={Cpu} title="PID Controller" sub="CMD_VEL (ωx, ωy)" tone="primary" pid />
            </div>

            {/* feedback loop: PID output back up to Camera Geometry (schematic) */}
            <div className="pointer-events-none absolute bottom-[38%] left-[3%] top-[24%] z-0 w-[34%] border-l-2 border-t-2 border-dashed border-primary/25" />
            <div className="pointer-events-none absolute left-[3%] top-[24%] z-0 h-[2px] w-[8%] border-t-2 border-dashed border-primary/25" />
          </div>

          {/* actuator state (live) — floating bottom-right (Stitch) */}
          <Panel className="absolute bottom-margin-md right-margin-md z-20 w-60 p-margin-md">
            <span className="mb-margin-sm block border-b border-outline-variant/30 pb-1 font-label-xs text-label-xs uppercase tracking-widest text-on-surface-variant">
              Actuator State
            </span>
            <div className="flex flex-col gap-unit">
              <ActRow k="PAN (θ)" v={`${signed(current.camera.panDeg, 2)}°`} sat={current.control.panSaturated} />
              <ActRow k="TILT (φ)" v={`${signed(current.camera.tiltDeg, 2)}°`} sat={current.control.tiltSaturated} />
              <div className="relative mt-2 h-1 overflow-hidden bg-surface-container-highest">
                <span className="absolute left-1/2 top-0 h-full w-[2px] bg-error" />
                <span
                  className="absolute top-0 h-full w-4 bg-primary transition-all duration-75"
                  style={{
                    left: `${Math.max(0, Math.min(100, 50 + current.camera.panDeg * 3))}%`,
                  }}
                />
              </div>
            </div>
          </Panel>
        </div>
      </div>
    </Screen>
  );
}

function MetaRow({ k, v }: { k: string; v: string }) {
  return (
    <div className="mt-margin-md flex flex-col gap-unit">
      <span className="font-label-xs text-label-xs uppercase tracking-widest text-on-surface-variant">{k}</span>
      <span className="font-data-mono text-data-mono text-primary">{v}</span>
    </div>
  );
}

function Diag({ label, value, warn }: { label: string; value: string; warn?: boolean }) {
  return (
    <div className="flex items-center justify-between">
      <div className="flex flex-col gap-gutter">
        <span className="font-label-xs text-label-xs uppercase text-on-surface-variant">{label}</span>
        <span className={cn("font-data-mono text-data-mono", warn ? "text-tertiary" : "text-on-surface")}>{value}</span>
      </div>
      <div className="flex h-8 w-16 items-center gap-[2px]">
        {[20, 40, 60, 80, 50, 60, 40, 20].map((h, i) => (
          <span key={i} className="w-1 bg-primary" style={{ height: `${h}%`, opacity: 0.2 + i * 0.1 }} />
        ))}
      </div>
    </div>
  );
}

function Node({
  icon: Icon,
  title,
  sub,
  tone,
  dot,
  big,
  round,
  pid,
}: {
  icon: typeof Cpu;
  title: string;
  sub: string;
  tone: "primary" | "warning" | "lost";
  dot?: boolean;
  big?: boolean;
  round?: boolean;
  pid?: boolean;
}) {
  const border = { primary: "border-primary", warning: "border-tertiary", lost: "border-error" }[tone];
  const text = { primary: "text-primary", warning: "text-tertiary", lost: "text-error" }[tone];
  return (
    <div className="flex w-1/4 flex-col items-center gap-margin-sm">
      <div
        className={cn(
          "relative z-10 flex items-center justify-center border bg-surface-container",
          big ? "h-24 w-24 border-2 border-surface-tint" : round ? "h-16 w-16 rounded-full border-error" : "h-20 w-20 rounded-sm border-outline-variant",
          pid && "border-2 border-primary bg-primary/10 shadow-[0_0_15px_rgba(111,238,225,0.2)]",
        )}
      >
        {dot && <span className={cn("absolute -right-1 -top-1 h-2 w-2", { primary: "bg-primary", warning: "bg-tertiary", lost: "bg-error" }[tone])} />}
        {pid ? (
          <span className="font-display-telem text-[24px] font-bold text-primary">PID</span>
        ) : (
          <Icon className={cn(big ? "h-8 w-8 text-on-surface" : "h-8 w-8", text)} strokeWidth={1.5} />
        )}
      </div>
      <div className="text-center">
        <span className={cn("block font-headline-sm text-headline-sm uppercase", big ? "text-primary" : "text-on-surface")}>{title}</span>
        <span className={cn("font-data-mono text-label-xs", text)}>{sub}</span>
      </div>
    </div>
  );
}

function Arrow({ reversed }: { reversed?: boolean }) {
  return (
    <div className="mt-10 flex flex-1 items-center px-2 text-outline-variant">
      <div className="h-[2px] flex-1 bg-current" />
      <CircleDot className={cn("h-3 w-3", reversed ? "-order-1 rotate-180" : "")} strokeWidth={2} />
    </div>
  );
}

function ActRow({ k, v, sat }: { k: string; v: string; sat: boolean }) {
  return (
    <div className="flex items-center justify-between">
      <span className="font-data-mono text-data-mono text-on-surface">{k}</span>
      <span className={cn("bg-primary/10 px-1 font-data-mono text-data-mono tnum", sat ? "text-tertiary" : "text-primary")}>{v}</span>
    </div>
  );
}

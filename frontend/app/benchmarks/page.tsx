"use client";

import { useMemo } from "react";
import { Check, TrendingUp, TriangleAlert } from "lucide-react";

import { Screen } from "@/components/shell/AppShell";
import { TimeSeriesChart } from "@/components/charts/TimeSeriesChart";
import { useScenarioFrames } from "@/lib/simulation/useScenarioFrames";
import { decimate, n } from "@/lib/telemetry/series";
import { BENCHMARK } from "@/lib/baseline/constants";
import { deg, fixed, pct } from "@/lib/format";
import { cn } from "@/lib/cn";

/**
 * /benchmarks — Stitch "Benchmarks" (Open Loop // Closed Loop).
 * Uses the VALIDATED nominal comparison numbers (docs/17 + Step 10) and overlays
 * the two real telemetry runs on one synchronized error-vs-time axis. No invented
 * "AI improvement" figures.
 */
export default function BenchmarksPage() {
  const open = useScenarioFrames("open");
  const closed = useScenarioFrames("closed");

  const chartData = useMemo(() => {
    const oa = decimate(open.frames, 500);
    const ca = decimate(closed.frames, 500);
    const len = Math.min(oa.length, ca.length);
    const rows: Array<Record<string, number>> = [];
    for (let i = 0; i < len; i++) {
      rows.push({
        t: oa[i].simulationTime,
        open: n(oa[i].tracking.totalErrorDeg),
        closed: n(ca[i].tracking.totalErrorDeg),
      });
    }
    return rows;
  }, [open.frames, closed.frames]);

  const loading = open.loading || closed.loading;

  return (
    <Screen>
      {/* header */}
      <div className="relative z-10 flex flex-col gap-margin-sm border-b border-outline-variant bg-surface-container-lowest px-margin-md pb-margin-sm pt-margin-md">
        <div className="flex items-center gap-margin-sm">
          <span className="font-display-telem text-display-telem tracking-tight text-on-surface">
            CONTROL PERFORMANCE
          </span>
          <div className="mx-margin-md h-px flex-1 bg-outline-variant" />
          <span className="font-data-mono text-data-mono uppercase text-outline">
            OPEN LOOP // CLOSED LOOP
          </span>
        </div>
        <p className="max-w-2xl font-body-md text-body-md text-on-surface-variant">
          Comparative tracking integrity for the same nonlinear trajectory. Continuous
          pixel-feedback regulation vs an uncompensated open-loop reference. Values are the
          validated Step-10 measurements — the C++ engine is authoritative.
        </p>
      </div>

      {/* comparison grid */}
      <div className="relative z-10 flex min-h-0 flex-1 flex-col xl:flex-row">
        <ComparePanel
          kind="open"
          title="OPEN LOOP (BASELINE)"
          tag="Uncompensated"
          detection={BENCHMARK.open.detectionPct}
          rms={BENCHMARK.open.rmsDeg}
          lost={BENCHMARK.open.lostFrames}
          frames={open.frames}
        />

        {/* centre gain callout */}
        <div className="pointer-events-none z-20 hidden shrink-0 items-center justify-center xl:flex">
          <div className="flex -translate-x-1/2 flex-col items-center border border-primary bg-surface-container-highest px-margin-md py-margin-sm shadow-xl">
            <span className="mb-unit font-label-xs text-label-xs uppercase tracking-widest text-primary">
              Performance Gain
            </span>
            <div className="flex items-center gap-margin-sm">
              <span className="font-display-telem text-display-telem leading-none text-on-surface">
                {fixed(BENCHMARK.rmsImprovementFactor, 1)}x
              </span>
              <TrendingUp className="h-6 w-6 text-primary" strokeWidth={1.75} />
            </div>
            <span className="mt-unit border border-outline-variant bg-surface-container-low px-unit py-[2px] font-data-mono text-[10px] text-on-surface-variant">
              RMS ERROR REDUCTION · +{fixed(BENCHMARK.detectionImprovementPts, 1)} pts DETECTION
            </span>
          </div>
        </div>

        <ComparePanel
          kind="closed"
          title="CLOSED LOOP (ACTIVE)"
          tag="PID Regulated"
          detection={BENCHMARK.closed.detectionPct}
          rms={BENCHMARK.closed.rmsDeg}
          lost={BENCHMARK.closed.lostFrames}
          frames={closed.frames}
        />
      </div>

      {/* error vs time */}
      <div className="flex h-64 shrink-0 flex-col border-t border-outline-variant bg-surface-container">
        <div className="flex items-center justify-between border-b border-outline-variant bg-surface-container-lowest px-margin-md py-margin-sm">
          <span className="font-headline-sm text-headline-sm uppercase tracking-widest text-on-surface">
            Synchronized Error vs Time
          </span>
          <div className="flex items-center gap-margin-md">
            <Legend color="var(--lost)" label="OPEN LOOP" />
            <Legend color="var(--tracking)" label="CLOSED LOOP" />
          </div>
        </div>
        <div className="min-h-0 flex-1 p-margin-md">
          {loading ? (
            <LoadingBar label="loading open + closed telemetry…" />
          ) : (
            <TimeSeriesChart
              data={chartData}
              height={180}
              yUnit="°"
              series={[
                { key: "open", label: "Open loop", color: "var(--lost)", width: 1.5 },
                { key: "closed", label: "Closed loop", color: "var(--tracking)", width: 1.5 },
              ]}
            />
          )}
        </div>
      </div>
    </Screen>
  );
}

function ComparePanel({
  kind,
  title,
  tag,
  detection,
  rms,
  lost,
  frames,
}: {
  kind: "open" | "closed";
  title: string;
  tag: string;
  detection: number;
  rms: number;
  lost: number;
  frames: import("@/lib/telemetry/types").DemoSnapshot[];
}) {
  const isOpen = kind === "open";
  const tone = isOpen ? "text-error" : "text-primary";
  const border = isOpen ? "border-error" : "border-primary";

  const profile = useMemo(() => {
    const dec = decimate(frames, 100);
    if (dec.length === 0) return "";
    const max = Math.max(1, ...dec.map((f) => Math.abs(n(f.tracking.totalErrorDeg))));
    return dec
      .map((f, i) => {
        const x = (i / Math.max(1, dec.length - 1)) * 100;
        const y = 50 - (n(f.tracking.totalErrorDeg) / max) * 45;
        return `${x.toFixed(1)},${y.toFixed(1)}`;
      })
      .join(" ");
  }, [frames]);

  return (
    <div
      className={cn(
        "relative flex flex-1 flex-col overflow-hidden",
        isOpen ? "border-r border-outline-variant bg-surface-dim" : "bg-surface-container-lowest",
      )}
    >
      <div
        className={cn(
          "pointer-events-none absolute inset-0 bg-gradient-to-b to-transparent",
          isOpen ? "from-error/5" : "from-primary/5",
        )}
      />
      <div className="relative flex items-center justify-between border-b border-outline-variant bg-surface-container-lowest p-margin-md">
        {!isOpen && <div className="absolute bottom-0 left-0 top-0 w-1 bg-primary" />}
        <div className={cn("flex items-center gap-margin-sm", !isOpen && "pl-margin-sm")}>
          <span className={cn("flex h-3 w-3 items-center justify-center border border-on-surface", isOpen ? "rotate-45 bg-error" : "bg-primary")}>
            <span className="h-1 w-1 bg-surface-container-lowest" />
          </span>
          <span className="font-headline-sm text-headline-sm uppercase tracking-widest text-on-surface">
            {title}
          </span>
        </div>
        <span className={cn("flex items-center gap-unit font-data-mono text-data-mono", tone)}>
          {!isOpen && <span className="h-2 w-2 animate-pulse rounded-full bg-primary" />}
          {tag}
        </span>
      </div>

      <div className="relative grid grid-cols-2 gap-gutter bg-outline-variant p-margin-md">
        <div className="flex flex-col gap-unit bg-surface-container-low p-panel-padding">
          <span className="font-label-xs text-label-xs uppercase text-on-surface-variant">Detection Rate</span>
          <div className="flex items-end gap-margin-sm">
            <span className={cn("font-display-telem text-display-telem leading-none tnum", tone)}>
              {fixed(detection, 1)}
            </span>
            <span className={cn("mb-[3px] font-data-mono text-data-mono", tone)}>%</span>
          </div>
          <div className="mt-unit h-1 w-full bg-surface-container-highest">
            <div className={cn("h-full", isOpen ? "bg-error" : "bg-primary")} style={{ width: `${detection}%` }} />
          </div>
        </div>
        <div className="flex flex-col gap-unit bg-surface-container-low p-panel-padding">
          <span className="font-label-xs text-label-xs uppercase text-on-surface-variant">RMS Error</span>
          <div className="flex items-end gap-margin-sm">
            <span className={cn("font-display-telem text-display-telem leading-none tnum", tone)}>
              {fixed(rms, 2)}
            </span>
            <span className={cn("mb-[3px] font-data-mono text-data-mono", tone)}>deg</span>
          </div>
          <div className="mt-unit flex items-center gap-unit">
            {isOpen ? (
              <>
                <TriangleAlert className="h-3.5 w-3.5 text-error" strokeWidth={1.75} />
                <span className="font-data-mono text-[10px] leading-none text-error">Threshold Exceeded</span>
              </>
            ) : (
              <>
                <Check className="h-3.5 w-3.5 text-primary" strokeWidth={2} />
                <span className="font-data-mono text-[10px] leading-none text-primary">Lock Maintained</span>
              </>
            )}
          </div>
        </div>
        <div className="col-span-2 flex flex-col gap-unit bg-surface-container-low p-panel-padding">
          <span className="font-label-xs text-label-xs uppercase text-on-surface-variant">
            Trajectory Divergence Profiler · {lost} lost frames
          </span>
          <div className="relative mt-margin-sm h-24 w-full">
            <svg className={cn("absolute inset-0 h-full w-full", isOpen ? "text-error" : "text-primary")} preserveAspectRatio="none" viewBox="0 0 100 100">
              <line x1="0" x2="100" y1="50" y2="50" stroke="currentColor" strokeDasharray="2 2" strokeOpacity={0.3} strokeWidth={1} />
              <polyline points={profile} fill="none" stroke="currentColor" strokeWidth={1.5} vectorEffect="non-scaling-stroke" />
            </svg>
          </div>
        </div>
      </div>
      <div className={cn("mt-auto border-t p-margin-sm font-data-mono text-[10px] text-on-surface-variant", border, "border-opacity-30")}>
        {frames.length} frames · {isOpen ? "control_enabled = false" : "control_enabled = true"} · identical trajectory
      </div>
    </div>
  );
}

function Legend({ color, label }: { color: string; label: string }) {
  return (
    <div className="flex items-center gap-unit">
      <span className="h-1 w-4" style={{ background: color }} />
      <span className="font-data-mono text-[10px] text-on-surface-variant">{label}</span>
    </div>
  );
}

function LoadingBar({ label }: { label: string }) {
  return (
    <div className="flex h-full items-center justify-center font-data-mono text-label-xs text-on-surface-variant">
      <span className="mr-2 h-1.5 w-1.5 animate-pulse bg-primary" />
      {label}
    </div>
  );
}

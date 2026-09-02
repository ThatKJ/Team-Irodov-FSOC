"use client";

import { useEffect, useRef, useState } from "react";
import { Pause, Play, RotateCcw, SkipBack, SkipForward } from "lucide-react";

import { useSimulation, type PlaybackSpeed } from "@/lib/simulation/SimulationProvider";
import { EVENT_COLOR } from "@/lib/simulation/events";
import { elapsedClock, utcClock } from "@/lib/format";
import { cn } from "@/lib/cn";

const SPEEDS: PlaybackSpeed[] = [0.5, 1, 2];

/** Reproduces the Stitch transport section — scrubber, markers, transport, speeds. */
export function PlaybackControls({ className }: { className?: string }) {
  const {
    playing,
    toggle,
    reset,
    seek,
    stepFrame,
    frameIndex,
    totalFrames,
    progress,
    speed,
    setSpeed,
    events,
    simTime,
    runState,
    meta,
  } = useSimulation();

  const trackRef = useRef<HTMLDivElement>(null);
  const [sysTime, setSysTime] = useState("--:--:-- UTC");
  useEffect(() => {
    setSysTime(utcClock());
    const id = setInterval(() => setSysTime(utcClock()), 1000);
    return () => clearInterval(id);
  }, []);

  const onScrub = (clientX: number) => {
    const el = trackRef.current;
    if (!el || totalFrames === 0) return;
    const r = el.getBoundingClientRect();
    const ratio = Math.min(1, Math.max(0, (clientX - r.left) / r.width));
    seek(Math.round(ratio * (totalFrames - 1)));
  };

  const durationFrames = Math.max(1, totalFrames - 1);

  return (
    <section
      className={cn(
        "flex h-[72px] shrink-0 flex-col justify-end gap-margin-sm border-t border-outline-variant bg-surface-container-highest p-2",
        className,
      )}
      data-testid="playback-controls"
    >
      {/* scrubber */}
      <div
        ref={trackRef}
        className="group relative h-4 w-full cursor-pointer"
        role="slider"
        aria-label="Simulation timeline"
        aria-valuemin={0}
        aria-valuemax={durationFrames}
        aria-valuenow={frameIndex}
        tabIndex={0}
        onMouseDown={(e) => {
          onScrub(e.clientX);
          const move = (ev: MouseEvent) => onScrub(ev.clientX);
          const up = () => {
            window.removeEventListener("mousemove", move);
            window.removeEventListener("mouseup", up);
          };
          window.addEventListener("mousemove", move);
          window.addEventListener("mouseup", up);
        }}
        onKeyDown={(e) => {
          if (e.key === "ArrowLeft") stepFrame(e.shiftKey ? -25 : -1);
          if (e.key === "ArrowRight") stepFrame(e.shiftKey ? 25 : 1);
          if (e.key === "Home") seek(0);
          if (e.key === "End") seek(totalFrames - 1);
        }}
      >
        <div className="absolute left-0 right-0 top-1/2 h-px -translate-y-1/2 bg-outline-variant" />
        <div
          className="absolute left-0 top-1/2 h-px -translate-y-1/2 bg-primary"
          style={{ width: `${progress * 100}%` }}
        />
        {/* event markers */}
        {events
          .filter((ev) => ev.kind !== "RUN_START" && ev.kind !== "RUN_END")
          .map((ev, i) => (
            <span
              key={i}
              title={`${ev.time.toFixed(2)}s · ${ev.label}`}
              className="absolute top-1/2 h-2 w-px -translate-y-1/2"
              style={{
                left: `${(ev.frame / durationFrames) * 100}%`,
                background: EVENT_COLOR[ev.severity],
              }}
            />
          ))}
        {/* playhead */}
        <div
          className="absolute top-0 -ml-px flex h-full flex-col items-center justify-start bg-primary"
          style={{ left: `${progress * 100}%`, width: 1 }}
        >
          <span className="absolute -top-2 h-2 w-2 border border-primary bg-surface" />
        </div>
      </div>

      {/* controls */}
      <div className="flex items-center justify-between">
        <div className="flex gap-1">
          <TransportBtn label="Restart" onClick={reset}>
            <RotateCcw className="h-[16px] w-[16px]" strokeWidth={1.75} />
          </TransportBtn>
          <TransportBtn label="Back 25 frames" onClick={() => stepFrame(-25)}>
            <SkipBack className="h-[16px] w-[16px]" strokeWidth={1.75} />
          </TransportBtn>
          <button
            type="button"
            aria-label={playing ? "Pause" : "Play"}
            onClick={toggle}
            className="flex h-8 w-8 items-center justify-center border border-primary bg-primary text-background transition-colors hover:bg-primary-fixed"
          >
            {playing ? (
              <Pause className="h-[16px] w-[16px]" strokeWidth={2} fill="currentColor" />
            ) : (
              <Play className="h-[16px] w-[16px]" strokeWidth={2} fill="currentColor" />
            )}
          </button>
          <TransportBtn label="Forward 25 frames" onClick={() => stepFrame(25)}>
            <SkipForward className="h-[16px] w-[16px]" strokeWidth={1.75} />
          </TransportBtn>
        </div>

        <div className="flex items-center gap-1">
          {SPEEDS.map((s) => (
            <button
              key={s}
              type="button"
              onClick={() => setSpeed(s)}
              className={cn(
                "flex h-8 items-center justify-center border px-2 font-data-mono text-label-xs transition-colors",
                s === speed
                  ? "border-primary bg-surface-variant text-primary"
                  : "border-outline-variant bg-surface text-on-surface hover:border-primary hover:text-primary",
              )}
            >
              {s.toFixed(1)}x
            </button>
          ))}
          <span
            className={cn(
              "ml-4 flex h-8 items-center justify-center border px-2 font-data-mono text-label-xs",
              runState === "RUNNING"
                ? "border-primary text-primary"
                : "border-outline-variant text-on-surface-variant",
            )}
          >
            {runState}
          </span>
        </div>

        <div className="flex items-center gap-4 font-data-mono text-label-xs text-on-surface-variant">
          <span className="tnum">SIM {elapsedClock(simTime)}</span>
          <span className="tnum">SYS {sysTime}</span>
          {meta && (
            <span className="hidden lg:inline">
              {meta.source.toUpperCase()} · {frameIndex + 1}/{totalFrames}
            </span>
          )}
        </div>
      </div>
    </section>
  );
}

function TransportBtn({
  label,
  onClick,
  children,
}: {
  label: string;
  onClick: () => void;
  children: React.ReactNode;
}) {
  return (
    <button
      type="button"
      aria-label={label}
      onClick={onClick}
      className="flex h-8 w-8 items-center justify-center border border-outline-variant bg-surface text-on-surface transition-colors hover:border-primary hover:text-primary"
    >
      {children}
    </button>
  );
}

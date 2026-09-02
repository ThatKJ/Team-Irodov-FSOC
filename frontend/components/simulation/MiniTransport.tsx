"use client";

import { Pause, Play, RotateCcw } from "lucide-react";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { EVENT_COLOR } from "@/lib/simulation/events";
import { cn } from "@/lib/cn";

/**
 * Compact transport for screens that are visually a single viewport (Optical
 * Tracking, Spatial View). Same clock as the full PlaybackControls.
 */
export function MiniTransport({ className }: { className?: string }) {
  const { playing, toggle, reset, seek, frameIndex, totalFrames, progress, events, speed, setSpeed } =
    useSimulation();
  const durationFrames = Math.max(1, totalFrames - 1);

  return (
    <div
      className={cn(
        "flex items-center gap-margin-md border border-outline-variant/60 bg-surface-container-lowest/85 px-margin-md py-margin-sm backdrop-blur-sm",
        className,
      )}
      data-testid="mini-transport"
    >
      <button
        type="button"
        aria-label={playing ? "Pause" : "Play"}
        onClick={toggle}
        className="flex h-7 w-7 items-center justify-center border border-primary bg-primary text-background hover:bg-primary-fixed"
      >
        {playing ? (
          <Pause className="h-3.5 w-3.5" fill="currentColor" strokeWidth={2} />
        ) : (
          <Play className="h-3.5 w-3.5" fill="currentColor" strokeWidth={2} />
        )}
      </button>
      <button
        type="button"
        aria-label="Restart"
        onClick={reset}
        className="flex h-7 w-7 items-center justify-center border border-outline-variant text-on-surface hover:border-primary hover:text-primary"
      >
        <RotateCcw className="h-3.5 w-3.5" strokeWidth={1.75} />
      </button>

      <div
        className="relative h-3 w-[220px] cursor-pointer"
        role="slider"
        aria-label="Timeline"
        aria-valuemin={0}
        aria-valuemax={durationFrames}
        aria-valuenow={frameIndex}
        onMouseDown={(e) => {
          const el = e.currentTarget;
          const move = (clientX: number) => {
            const r = el.getBoundingClientRect();
            const ratio = Math.min(1, Math.max(0, (clientX - r.left) / r.width));
            seek(Math.round(ratio * durationFrames));
          };
          move(e.clientX);
          const mm = (ev: MouseEvent) => move(ev.clientX);
          const up = () => {
            window.removeEventListener("mousemove", mm);
            window.removeEventListener("mouseup", up);
          };
          window.addEventListener("mousemove", mm);
          window.addEventListener("mouseup", up);
        }}
      >
        <div className="absolute left-0 right-0 top-1/2 h-px -translate-y-1/2 bg-outline-variant" />
        <div
          className="absolute left-0 top-1/2 h-px -translate-y-1/2 bg-primary"
          style={{ width: `${progress * 100}%` }}
        />
        {events
          .filter((ev) => ev.kind !== "RUN_START" && ev.kind !== "RUN_END")
          .map((ev, i) => (
            <span
              key={i}
              className="absolute top-1/2 h-1.5 w-px -translate-y-1/2"
              style={{ left: `${(ev.frame / durationFrames) * 100}%`, background: EVENT_COLOR[ev.severity] }}
            />
          ))}
        <span
          className="absolute top-1/2 h-2 w-2 -translate-x-1/2 -translate-y-1/2 border border-primary bg-surface"
          style={{ left: `${progress * 100}%` }}
        />
      </div>

      <div className="flex gap-1">
        {([0.5, 1, 2] as const).map((s) => (
          <button
            key={s}
            type="button"
            onClick={() => setSpeed(s)}
            className={cn(
              "border px-1.5 py-0.5 font-data-mono text-label-xs",
              s === speed
                ? "border-primary text-primary"
                : "border-outline-variant text-on-surface-variant hover:text-on-surface",
            )}
          >
            {s}x
          </button>
        ))}
      </div>
      <span className="font-data-mono text-label-xs text-on-surface-variant tnum">
        {frameIndex + 1}/{totalFrames}
      </span>
    </div>
  );
}

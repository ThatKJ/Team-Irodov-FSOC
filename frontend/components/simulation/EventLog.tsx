"use client";

import { useMemo } from "react";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { EVENT_COLOR } from "@/lib/simulation/events";
import { simClock } from "@/lib/format";
import { cn } from "@/lib/cn";

/** Event log derived from telemetry transitions (never random). Streams in as playback advances. */
export function EventLog({ className, max = 40 }: { className?: string; max?: number }) {
  const { events, simTime } = useSimulation();

  const visible = useMemo(
    () => events.filter((e) => e.time <= simTime + 1e-6).slice(-max).reverse(),
    [events, simTime, max],
  );

  return (
    <div className={cn("flex flex-col gap-unit overflow-y-auto", className)} data-testid="event-log">
      {visible.length === 0 && (
        <span className="font-data-mono text-[10px] text-on-surface-variant opacity-50">
          awaiting first frame…
        </span>
      )}
      {visible.map((ev, i) => (
        <div
          key={`${ev.frame}-${ev.kind}-${i}`}
          className={cn("flex gap-margin-sm font-data-mono text-[10px]", i > 4 && "opacity-60", i > 10 && "opacity-40")}
        >
          <span className="tnum text-on-surface-variant">{simClock(ev.time)}</span>
          <span style={{ color: EVENT_COLOR[ev.severity] }}>{ev.label}</span>
        </div>
      ))}
    </div>
  );
}

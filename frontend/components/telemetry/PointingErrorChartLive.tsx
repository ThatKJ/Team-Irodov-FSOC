"use client";

import { useMemo } from "react";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { TimeSeriesChart } from "@/components/charts/TimeSeriesChart";
import { toFullChartData, n } from "@/lib/telemetry/series";
import { cn } from "@/lib/cn";

/** Full-run pointing-error trace with a playhead synced to the sim clock. */
export function PointingErrorChartLive({
  className,
  height = 60,
  compact = true,
  hideLabel = false,
}: {
  className?: string;
  height?: number;
  compact?: boolean;
  hideLabel?: boolean;
}) {
  const { frames, simTime } = useSimulation();
  const data = useMemo(
    () => toFullChartData(frames, (f) => ({ err: n(f.tracking.totalErrorDeg) }), 400),
    [frames],
  );

  return (
    <div className={cn("relative w-full", className)}>
      {!hideLabel && (
        <span className="absolute left-margin-md top-margin-sm z-10 font-label-xs text-label-xs uppercase text-on-surface-variant">
          Pointing Error [deg]
        </span>
      )}
      <TimeSeriesChart
        data={data}
        height={height}
        compact={compact}
        playhead={simTime}
        series={[{ key: "err", label: "Total angular error", color: "var(--warning)", width: 1.25 }]}
        yUnit="°"
      />
    </div>
  );
}

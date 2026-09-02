"use client";

import { useMemo } from "react";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { decimate } from "@/lib/telemetry/series";
import { TrackingFeed, type TrackingFeedProps } from "./TrackingFeed";

/** TrackingFeed bound to the shared simulation clock. */
export function TrackingFeedLive(
  props: Omit<TrackingFeedProps, "snapshot" | "errorSeries">,
) {
  const { current, frames } = useSimulation();
  // full-run pointing-error envelope for the HUD sparkline (always populated)
  const errorSeries = useMemo(
    () => decimate(frames, 160).map((f) => f.tracking.totalErrorDeg ?? 0),
    [frames],
  );
  return <TrackingFeed snapshot={current} errorSeries={errorSeries} {...props} />;
}

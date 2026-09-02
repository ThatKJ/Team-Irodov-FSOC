"use client";

import { useEffect, useState } from "react";
import type { DemoSnapshot, SimulationMeta, SimulationPayload } from "@/lib/telemetry/types";
import type { ScenarioId } from "@/lib/baseline/constants";

interface Result {
  frames: DemoSnapshot[];
  meta: SimulationMeta | null;
  loading: boolean;
  error: string | null;
}

/**
 * One-shot fetch of a scenario's telemetry, independent of the playback provider.
 * Used by comparison screens (benchmarks) that need >1 scenario at once.
 */
export function useScenarioFrames(scenario: ScenarioId, source: "auto" | "engine" | "replay" = "auto"): Result {
  const [state, setState] = useState<Result>({ frames: [], meta: null, loading: true, error: null });

  useEffect(() => {
    const ctrl = new AbortController();
    let alive = true;
    setState((s) => ({ ...s, loading: true, error: null }));
    fetch(`/api/simulation/${scenario}?source=${source}`, { signal: ctrl.signal })
      .then(async (r) => {
        const body = (await r.json()) as SimulationPayload | { error: string; detail?: string };
        if (!r.ok || !("frames" in body)) {
          throw new Error("error" in body ? body.error : `HTTP ${r.status}`);
        }
        if (alive) setState({ frames: body.frames, meta: body.meta, loading: false, error: null });
      })
      .catch((e: unknown) => {
        if (!alive || (e instanceof DOMException && e.name === "AbortError")) return;
        setState({ frames: [], meta: null, loading: false, error: e instanceof Error ? e.message : String(e) });
      });
    return () => {
      alive = false;
      ctrl.abort();
    };
  }, [scenario, source]);

  return state;
}

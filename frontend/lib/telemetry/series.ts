import type { DemoSnapshot } from "./types";

/** even-stride decimation that always keeps the last sample */
export function decimate<T>(arr: T[], maxPoints: number): T[] {
  if (arr.length <= maxPoints) return arr;
  const stride = Math.ceil(arr.length / maxPoints);
  const out: T[] = [];
  for (let i = 0; i < arr.length; i += stride) out.push(arr[i]);
  if (out[out.length - 1] !== arr[arr.length - 1]) out.push(arr[arr.length - 1]);
  return out;
}

/**
 * Builds Recharts-ready rows from telemetry frames [0..upToIndex]. `map` returns
 * the numeric fields for one row; `t` (sim seconds) is added automatically.
 */
export function toChartData(
  frames: DemoSnapshot[],
  upToIndex: number,
  map: (f: DemoSnapshot) => Record<string, number>,
  maxPoints = 400,
): Array<Record<string, number>> {
  const slice = frames.slice(0, Math.min(upToIndex + 1, frames.length));
  return decimate(slice, maxPoints).map((f) => ({ t: f.simulationTime, ...map(f) }));
}

/** whole-run chart data (used by benchmark / open-vs-closed comparisons) */
export function toFullChartData(
  frames: DemoSnapshot[],
  map: (f: DemoSnapshot) => Record<string, number>,
  maxPoints = 500,
): Array<Record<string, number>> {
  return decimate(frames, maxPoints).map((f) => ({ t: f.simulationTime, ...map(f) }));
}

/** safe number for charts — nulls become 0 so the line stays continuous where the engine has data */
export function n(v: number | null | undefined): number {
  return v == null || !Number.isFinite(v) ? 0 : v;
}

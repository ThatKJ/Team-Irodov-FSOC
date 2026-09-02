/** Display formatters. Fixed widths so telemetry digits never jitter. */

export function fixed(v: number | null | undefined, dp = 2): string {
  if (v == null || !Number.isFinite(v)) return "—";
  return v.toFixed(dp);
}

export function signed(v: number | null | undefined, dp = 2): string {
  if (v == null || !Number.isFinite(v)) return "—";
  const s = v.toFixed(dp);
  return v >= 0 ? `+${s}` : s;
}

export function deg(v: number | null | undefined, dp = 3): string {
  if (v == null || !Number.isFinite(v)) return "—";
  return `${v.toFixed(dp)}°`;
}

export function pct(v: number | null | undefined, dp = 1): string {
  if (v == null || !Number.isFinite(v)) return "—";
  return `${v.toFixed(dp)}%`;
}

export function px(v: number | null | undefined, dp = 1): string {
  if (v == null || !Number.isFinite(v)) return "—";
  return v.toFixed(dp);
}

/** sim time as SS.mmm (matches Stitch "04.520s") */
export function simClock(seconds: number): string {
  if (!Number.isFinite(seconds)) return "00.000";
  const s = Math.max(0, seconds);
  return `${Math.floor(s).toString().padStart(2, "0")}.${Math.round((s % 1) * 1000)
    .toString()
    .padStart(3, "0")}`;
}

/** mm:ss.mmm elapsed */
export function elapsedClock(seconds: number): string {
  if (!Number.isFinite(seconds)) return "00:00.000";
  const s = Math.max(0, seconds);
  const mm = Math.floor(s / 60);
  const ss = Math.floor(s % 60);
  const ms = Math.round((s % 1) * 1000);
  return `${mm.toString().padStart(2, "0")}:${ss.toString().padStart(2, "0")}.${ms
    .toString()
    .padStart(3, "0")}`;
}

/** UTC wall clock for the SYS TIME readout (cosmetic only, not simulation state) */
export function utcClock(d = new Date()): string {
  return `${d.toISOString().slice(11, 19)} UTC`;
}

/**
 * Maps the C++ 640x480 sensor plane onto a DOM viewport. Pure geometry for
 * PLACEMENT ONLY — the pixel values themselves come from the engine
 * (detection.xPx/yPx, tracking.errorXPx/errorYPx). Nothing here is control math.
 */

import { CAMERA } from "@/lib/baseline/constants";
import type { DemoSnapshot } from "@/lib/telemetry/types";

export interface SensorRect {
  x: number;
  y: number;
  w: number;
  h: number;
  cx: number; // optical centre (principal point) in viewport px
  cy: number;
}

/** `contain`-fit the 4:3 sensor inside the viewport, centred. */
export function sensorRect(viewW: number, viewH: number): SensorRect {
  const aspect = CAMERA.widthPx / CAMERA.heightPx; // 4:3
  let w = viewW;
  let h = w / aspect;
  if (h > viewH) {
    h = viewH;
    w = h * aspect;
  }
  const x = (viewW - w) / 2;
  const y = (viewH - h) / 2;
  return {
    x,
    y,
    w,
    h,
    cx: x + (CAMERA.principalPointPx.x / CAMERA.widthPx) * w,
    cy: y + (CAMERA.principalPointPx.y / CAMERA.heightPx) * h,
  };
}

export interface ReticlePlacement {
  /** detected centroid position in viewport px */
  x: number;
  y: number;
  /** centre-of-frame (optical centre) in viewport px */
  cx: number;
  cy: number;
  /** pixel error carried straight from telemetry */
  errX: number | null;
  errY: number | null;
  inFrame: boolean;
}

export function placeReticle(snap: DemoSnapshot, rect: SensorRect): ReticlePlacement | null {
  if (!snap.detection.detected || snap.detection.xPx == null || snap.detection.yPx == null) {
    return null;
  }
  const u = snap.detection.xPx / CAMERA.widthPx;
  const v = snap.detection.yPx / CAMERA.heightPx;
  const x = rect.x + u * rect.w;
  const y = rect.y + v * rect.h;
  return {
    x,
    y,
    cx: rect.cx,
    cy: rect.cy,
    errX: snap.tracking.errorXPx,
    errY: snap.tracking.errorYPx,
    inFrame: u >= 0 && u <= 1 && v >= 0 && v <= 1,
  };
}

/** trailing sparkline of total angular error (deg) for the pointing-error mini chart */
export function pointingErrorSeries(
  frames: DemoSnapshot[],
  upToIndex: number,
  window = 120,
): number[] {
  const end = Math.min(upToIndex + 1, frames.length);
  const start = Math.max(0, end - window);
  const out: number[] = [];
  for (let i = start; i < end; i++) {
    out.push(frames[i]?.tracking.totalErrorDeg ?? 0);
  }
  return out;
}

/** build an SVG polyline `points` string from a numeric series, auto-scaled */
export function polyline(series: number[], w = 100, h = 30, maxOverride?: number): string {
  if (series.length === 0) return "";
  const max = maxOverride ?? Math.max(0.001, ...series);
  const n = series.length;
  return series
    .map((val, i) => {
      const x = n === 1 ? 0 : (i / (n - 1)) * w;
      const y = h - Math.min(1, Math.max(0, val / max)) * h;
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    })
    .join(" ");
}

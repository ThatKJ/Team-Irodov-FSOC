"use client";

import { useMemo } from "react";
import { useSimulation } from "@/lib/simulation/SimulationProvider";
import { CAMERA } from "@/lib/baseline/constants";
import { deg, fixed } from "@/lib/format";
import type { WorldView } from "./WorldScene";

/**
 * 2-D schematic fallback for the spatial view when WebGL / R3F is unavailable
 * (e.g. headless capture, GPU-blocked browser). Same telemetry-driven elements —
 * camera terminal, target, line of sight, FOV wedge, trajectory — as a top-down
 * or side plan. Not decoration.
 */
export function WorldFallback2D({ view }: { view: WorldView }) {
  const { current, frames } = useSimulation();
  const lost = current.trackingState === "TARGET_LOST";
  const side = view === "SIDE";

  // world (+X fwd / +Y right / +Z up) -> 2D plan.  TOP/WORLD: x=fwd(up), y=right(right).
  // SIDE: x=fwd(right), z=up(up).
  const SC = 3.4; // px per metre-ish after normalising to ~±25 m
  const originX = 320;
  const originY = side ? 360 : 470;

  const project = (px: number, py: number, pz: number): [number, number] => {
    if (side) return [originX + px * 0.08 * SC, originY - pz * SC];
    return [originX + py * SC, originY - px * 0.08 * SC];
  };

  const [tx, ty] = project(current.target.position.x, current.target.position.y, current.target.position.z);

  const path = useMemo(() => {
    const stride = Math.max(1, Math.floor(frames.length / 240));
    const pts: string[] = [];
    for (let i = 0; i < frames.length; i += stride) {
      const p = frames[i].target.position;
      const [x, y] = project(p.x, p.y, p.z);
      pts.push(`${x.toFixed(1)},${y.toFixed(1)}`);
    }
    return pts.join(" ");
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [frames, side]);

  // FOV wedge: half-angle from boresight (pan for TOP, tilt for SIDE)
  const bore = side ? current.camera.tiltDeg : current.camera.panDeg;
  const half = (side ? CAMERA.verticalFovDeg : CAMERA.horizontalFovDeg) / 2;
  const wedge = (aDeg: number, len = 260) => {
    const a = ((aDeg - 90) * Math.PI) / 180;
    return [originX + Math.cos(a) * len, originY + Math.sin(a) * len];
  };
  const [w1x, w1y] = wedge(bore - half);
  const [w2x, w2y] = wedge(bore + half);
  const [losx, losy] = wedge(bore, 240);

  return (
    <div className="relative h-full w-full bg-background">
      <svg className="h-full w-full" viewBox="0 0 640 640" preserveAspectRatio="xMidYMid meet">
        <defs>
          <pattern id="wgrid" width="32" height="32" patternUnits="userSpaceOnUse">
            <path d="M32 0 L0 0 0 32" fill="none" stroke="#1b2120" strokeWidth="1" />
          </pattern>
        </defs>
        <rect width="640" height="640" fill="url(#wgrid)" />

        {/* trajectory */}
        <polyline points={path} fill="none" stroke="#3c4947" strokeWidth="1" />

        {/* FOV wedge */}
        <path d={`M${originX},${originY} L${w1x},${w1y} L${w2x},${w2y} Z`} fill="#8ecdff" fillOpacity="0.06" stroke="#8ecdff" strokeOpacity="0.4" strokeWidth="1" />

        {/* line of sight */}
        {!lost && <line x1={originX} y1={originY} x2={tx} y2={ty} stroke="#6feee1" strokeWidth="1" />}
        <line x1={originX} y1={originY} x2={losx} y2={losy} stroke="#6feee1" strokeOpacity="0.35" strokeDasharray="4 4" strokeWidth="1" />

        {/* camera terminal */}
        <g transform={`translate(${originX} ${originY})`}>
          <circle r="7" fill="#252b2a" stroke="#869491" strokeWidth="1" />
          <circle r="2" fill="#6feee1" />
        </g>

        {/* target */}
        <g transform={`translate(${tx} ${ty})`}>
          <circle r="10" fill="none" stroke={lost ? "#ffb4ab" : "#ffd2a2"} strokeOpacity="0.7" strokeWidth="1" />
          <circle r="3.5" fill={lost ? "#ffb4ab" : "#6feee1"} />
        </g>

        {/* axes hint */}
        <g transform={`translate(28 612)`} fontFamily="var(--font-mono)" fontSize="10">
          <line x1="0" y1="0" x2="0" y2="-24" stroke="#ffb4ab" strokeWidth="1" />
          <text x="4" y="-20" fill="#ffb4ab">{side ? "Z" : "X"}</text>
          <line x1="0" y1="0" x2="24" y2="0" stroke="#8ecdff" strokeWidth="1" />
          <text x="26" y="3" fill="#8ecdff">{side ? "X" : "Y"}</text>
        </g>
      </svg>
      <div className="absolute left-margin-md top-margin-md font-data-mono text-label-xs text-on-surface-variant">
        2D SCHEMATIC · WebGL unavailable · {view}
      </div>
      <div className="absolute bottom-margin-md left-1/2 -translate-x-1/2 border border-outline-variant bg-surface-container-lowest/80 px-margin-md py-unit font-data-mono text-label-xs text-on-surface-variant">
        pan {fixed(current.camera.panDeg, 1)}° · tilt {fixed(current.camera.tiltDeg, 1)}° · LOS err {deg(current.tracking.totalErrorDeg, 3)}
      </div>
    </div>
  );
}

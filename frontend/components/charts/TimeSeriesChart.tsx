"use client";

import {
  CartesianGrid,
  Line,
  LineChart,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";
import { cn } from "@/lib/cn";

export interface Series {
  key: string;
  label: string;
  color: string;
  dashed?: boolean;
  width?: number;
}

export interface ChartMarker {
  x: number;
  color: string;
  label?: string;
}

interface TimeSeriesChartProps {
  data: Array<Record<string, number>>;
  xKey?: string;
  series: Series[];
  /** current sim time — draws the synchronized playhead */
  playhead?: number;
  markers?: ChartMarker[];
  threshold?: { value: number; color?: string; label?: string };
  yUnit?: string;
  yDomain?: [number | "auto", number | "auto"];
  height?: number;
  className?: string;
  compact?: boolean;
}

/** Thin 1px stroke lines, no area fills, dashed grid — design.md chart treatment. */
export function TimeSeriesChart({
  data,
  xKey = "t",
  series,
  playhead,
  markers = [],
  threshold,
  yUnit,
  yDomain,
  height = 160,
  className,
  compact = false,
}: TimeSeriesChartProps) {
  // default: clamp lower bound at/below 0, add ~12% headroom so peaks don't clip
  const domain: [number | string | ((v: number) => number), number | string | ((v: number) => number)] =
    yDomain ?? [
      (min: number) => (min > 0 ? 0 : min * 1.12),
      (max: number) => (max <= 0 ? 0 : max * 1.12),
    ];

  if (data.length < 2) {
    return (
      <div
        className={cn("flex w-full items-center justify-center", className)}
        style={{ height }}
        data-testid="timeseries-chart"
      >
        <span className="font-data-mono text-label-xs text-on-surface-variant">
          <span className="mr-2 inline-block h-1.5 w-1.5 animate-pulse bg-primary" />
          awaiting telemetry…
        </span>
      </div>
    );
  }

  return (
    <div className={cn("w-full", className)} style={{ height }} data-testid="timeseries-chart">
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data} margin={{ top: 8, right: 10, bottom: compact ? 2 : 16, left: compact ? 2 : 4 }}>
          <CartesianGrid stroke="var(--border)" strokeDasharray="2 2" strokeOpacity={0.5} vertical={false} />
          {!compact && (
            <XAxis
              dataKey={xKey}
              type="number"
              domain={["dataMin", "dataMax"]}
              tick={{ fill: "var(--muted)", fontSize: 9, fontFamily: "var(--font-mono)" }}
              tickLine={{ stroke: "var(--border)" }}
              axisLine={{ stroke: "var(--border)" }}
              tickFormatter={(v: number) => `${v.toFixed(0)}s`}
              minTickGap={40}
            />
          )}
          <YAxis
            hide={compact}
            width={40}
            domain={domain}
            tick={{ fill: "var(--muted)", fontSize: 9, fontFamily: "var(--font-mono)" }}
            tickLine={{ stroke: "var(--border)" }}
            axisLine={{ stroke: "var(--border)" }}
            tickFormatter={(v: number) => (yUnit ? `${v}${yUnit}` : String(v))}
          />
          <Tooltip
            contentStyle={{
              background: "var(--surface-highest)",
              border: "1px solid var(--border)",
              borderRadius: 0,
              fontFamily: "var(--font-mono)",
              fontSize: 11,
              color: "var(--foreground)",
            }}
            labelStyle={{ color: "var(--muted)" }}
            labelFormatter={(v) => `t = ${Number(v).toFixed(2)}s`}
            formatter={(value: number, _n, item) => [
              `${Number(value).toFixed(3)}${yUnit ?? ""}`,
              (item as { name?: string }).name,
            ]}
          />
          {threshold != null && (
            <ReferenceLine
              y={threshold.value}
              stroke={threshold.color ?? "var(--lost-container)"}
              strokeWidth={1}
              label={
                threshold.label
                  ? { value: threshold.label, fill: "var(--muted)", fontSize: 9, position: "insideTopRight" }
                  : undefined
              }
            />
          )}
          {markers.map((m, i) => (
            <ReferenceLine
              key={i}
              x={m.x}
              stroke={m.color}
              strokeWidth={1}
              strokeDasharray="4 4"
              label={m.label ? { value: m.label, fill: m.color, fontSize: 8, position: "top" } : undefined}
            />
          ))}
          {playhead != null && (
            <ReferenceLine x={playhead} stroke="var(--tracking)" strokeWidth={1} ifOverflow="extendDomain" />
          )}
          {series.map((s) => (
            <Line
              key={s.key}
              type="monotone"
              dataKey={s.key}
              name={s.label}
              stroke={s.color}
              strokeWidth={s.width ?? 1}
              strokeDasharray={s.dashed ? "3 2" : undefined}
              dot={false}
              isAnimationActive={false}
              connectNulls={false}
            />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}

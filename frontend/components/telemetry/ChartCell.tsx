"use client";

import React from "react";
import { cn } from "@/lib/cn";

/** One cell of the Stitch telemetry chart grid: absolute label block + chart body. */
export function ChartCell({
  title,
  legend,
  marker,
  children,
  className,
}: {
  title: string;
  legend?: React.ReactNode;
  marker?: { label: string; tone: "warning" | "lost" };
  children: React.ReactNode;
  className?: string;
}) {
  return (
    <div className={cn("group relative flex flex-col bg-surface", className)}>
      <div className="pointer-events-none absolute left-0 top-0 z-10 flex w-full items-start justify-between p-2">
        <div>
          <div className="font-label-xs text-label-xs uppercase text-on-surface-variant">{title}</div>
          {legend != null && <div className="mt-1 flex gap-4 font-data-mono text-body-md">{legend}</div>}
        </div>
        {marker && (
          <div
            className={cn(
              "border px-1 py-0.5 font-label-xs text-label-xs",
              marker.tone === "warning"
                ? "border-tertiary bg-surface-container-highest text-tertiary"
                : "animate-pulse border-error bg-error-container text-on-error-container",
            )}
          >
            {marker.label}
          </div>
        )}
      </div>
      <div className="relative flex flex-1 items-end overflow-hidden pt-[44px]">
        {children}
        <div className="pointer-events-none absolute inset-0 border-l-2 border-t-2 border-primary opacity-0 mix-blend-screen transition-opacity group-hover:opacity-100" />
      </div>
    </div>
  );
}

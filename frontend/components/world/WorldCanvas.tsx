"use client";

import React from "react";
import dynamic from "next/dynamic";
import type { WorldView } from "./WorldScene";
import { WorldFallback2D } from "./WorldFallback2D";

/** R3F is client + WebGL only — never SSR it. */
const WorldScene = dynamic(() => import("./WorldScene"), {
  ssr: false,
  loading: () => (
    <div className="flex h-full w-full items-center justify-center bg-background">
      <span className="font-data-mono text-label-xs text-on-surface-variant">
        <span className="mr-2 inline-block h-1.5 w-1.5 animate-pulse bg-primary" />
        initializing spatial view…
      </span>
    </div>
  ),
});

/** A WebGL failure (no GPU, headless capture, blocked context) degrades to the 2D schematic. */
class GLBoundary extends React.Component<
  { view: WorldView; children: React.ReactNode },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  componentDidCatch(err: unknown) {
    // eslint-disable-next-line no-console
    console.warn("[world] R3F/WebGL failed, using 2D fallback:", err);
  }
  render() {
    if (this.state.failed) return <WorldFallback2D view={this.props.view} />;
    return this.props.children;
  }
}

export function WorldCanvas({ view }: { view: WorldView }) {
  return (
    <GLBoundary view={view}>
      <WorldScene view={view} />
    </GLBoundary>
  );
}

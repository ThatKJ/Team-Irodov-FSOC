"use client";

import { useCallback, useLayoutEffect, useRef, useState } from "react";

/** Tracks an element's content-box size with a ResizeObserver. */
export function useElementSize<T extends HTMLElement>() {
  const ref = useRef<T | null>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });

  const measure = useCallback(() => {
    const el = ref.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    setSize((prev) =>
      prev.width === r.width && prev.height === r.height ? prev : { width: r.width, height: r.height },
    );
  }, []);

  useLayoutEffect(() => {
    measure();
    const el = ref.current;
    if (!el || typeof ResizeObserver === "undefined") return;
    const ro = new ResizeObserver(measure);
    ro.observe(el);
    return () => ro.disconnect();
  }, [measure]);

  return { ref, ...size } as const;
}

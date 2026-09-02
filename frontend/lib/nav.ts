import {
  LayoutGrid,
  Crosshair,
  ScanEye,
  Globe,
  Activity,
  Network,
  Gauge,
  ClipboardCheck,
  Workflow,
  type LucideIcon,
} from "lucide-react";

/**
 * The FSOC Stitch project has exactly 9 screens. Each maps 1:1 to a route.
 * Material Symbols glyph -> closest Lucide icon (documented in STITCH_IMPLEMENTATION_MAP.md).
 */
export interface NavItem {
  path: string;
  label: string;
  /** Stitch data-path attribute */
  stitchPath: string;
  /** Stitch Material Symbol name */
  stitchIcon: string;
  icon: LucideIcon;
}

export const NAV_ITEMS: NavItem[] = [
  { path: "/", label: "Overview", stitchPath: "overview", stitchIcon: "grid_view", icon: LayoutGrid },
  { path: "/mission", label: "Mission Control", stitchPath: "mission-control", stitchIcon: "target", icon: Crosshair },
  { path: "/tracking", label: "Optical Tracking", stitchPath: "optical-tracking", stitchIcon: "center_focus_strong", icon: ScanEye },
  { path: "/world", label: "Spatial View", stitchPath: "spatial-view", stitchIcon: "language", icon: Globe },
  { path: "/telemetry", label: "Telemetry", stitchPath: "telemetry", stitchIcon: "query_stats", icon: Activity },
  { path: "/scenarios", label: "Scenarios", stitchPath: "scenarios", stitchIcon: "account_tree", icon: Network },
  { path: "/benchmarks", label: "Benchmarks", stitchPath: "benchmarks", stitchIcon: "speed", icon: Gauge },
  { path: "/validation", label: "Validation", stitchPath: "validation", stitchIcon: "fact_check", icon: ClipboardCheck },
  { path: "/architecture", label: "Architecture", stitchPath: "architecture", stitchIcon: "schema", icon: Workflow },
];

export function navLabelForPath(pathname: string): string {
  const exact = NAV_ITEMS.find((n) => n.path === pathname);
  if (exact) return exact.label;
  const nested = NAV_ITEMS.filter((n) => n.path !== "/").find((n) => pathname.startsWith(n.path));
  return nested?.label ?? "Overview";
}

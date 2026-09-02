"use client";

import { useMemo, useState } from "react";
import { Check, Download, Info, TriangleAlert } from "lucide-react";

import { Screen } from "@/components/shell/AppShell";
import { VALIDATION_ROWS, VALIDATION_SUMMARY, type ValidationRow } from "@/lib/baseline/constants";
import { deg, fixed, pct } from "@/lib/format";
import { cn } from "@/lib/cn";

/**
 * /validation — Stitch "Validation".
 * Shows the ACTUAL Step-10 baseline acceptance results (docs/16). 7/7 PASS.
 * Loss/Re-entry intentionally carries lost frames — that is validated
 * loss-semantics behaviour, NOT a failure.
 * Stitch deviations (documented): units are degrees (real Step-10), not µrad;
 * the "LOST (ms)" column is "LOST (frames)".
 */
export default function ValidationPage() {
  const [selected, setSelected] = useState<ValidationRow>(
    VALIDATION_ROWS.find((r) => r.scenario === "Sinusoidal Tracking") ?? VALIDATION_ROWS[0],
  );

  const exportLog = useMemo(
    () => () => {
      const header = "id,scenario,detection_pct,rms_deg,p95_deg,max_deg,final_deg,lost_frames,frames,status";
      const body = VALIDATION_ROWS.map((r) =>
        [r.id, `"${r.scenario}"`, r.detectionPct, r.rmsDeg ?? "", r.p95Deg ?? "", r.maxDeg ?? "", r.finalDeg, r.lostFrames, r.frames, r.status].join(","),
      ).join("\n");
      const blob = new Blob([`# ${VALIDATION_SUMMARY.verdict}\n${header}\n${body}\n`], { type: "text/csv" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "fsoc_step10_validation.csv";
      a.click();
      URL.revokeObjectURL(url);
    },
    [],
  );

  return (
    <Screen>
      <div className="flex flex-1 gap-margin-md overflow-hidden p-margin-md">
        {/* results table */}
        <div className="relative z-10 flex flex-1 flex-col border border-outline-variant bg-surface-container">
          <div className="flex h-[48px] shrink-0 items-center justify-between border-b border-outline-variant bg-surface-container-low px-margin-md">
            <div className="flex items-center gap-margin-sm">
              <span className="h-1.5 w-1.5 rounded-full bg-primary" />
              <span className="font-headline-sm text-headline-sm uppercase tracking-wider text-primary">
                {VALIDATION_SUMMARY.passed} / {VALIDATION_SUMMARY.total} Scenarios Passed
              </span>
            </div>
            <div className="flex items-center gap-margin-md">
              <span className="font-data-mono text-data-mono text-on-surface-variant">
                {VALIDATION_SUMMARY.reportId}
              </span>
              <button
                type="button"
                onClick={exportLog}
                className="flex items-center gap-unit border border-outline-variant px-margin-sm py-unit font-label-xs text-label-xs uppercase text-on-surface-variant transition-colors hover:bg-surface-bright hover:text-on-surface"
              >
                <Download className="h-3.5 w-3.5" strokeWidth={1.5} /> Export Log
              </button>
            </div>
          </div>

          <div className="flex-1 overflow-y-auto">
            <table className="w-full border-collapse text-left">
              <thead className="sticky top-0 z-20 border-b border-outline-variant bg-surface-container-high font-label-xs text-label-xs uppercase text-on-surface-variant">
                <tr>
                  {["ID", "Scenario", "DETECT %", "RMS (deg)", "P95 (deg)", "MAX (deg)", "LOST (frames)", "STATUS"].map(
                    (h, i) => (
                      <th
                        key={h}
                        className={cn(
                          "border-r border-outline-variant/30 px-margin-md py-margin-sm font-normal",
                          i >= 2 && i <= 6 && "text-right",
                          i === 7 && "border-r-0 text-center",
                          i === 0 && "w-[44px] text-center",
                        )}
                      >
                        {h}
                      </th>
                    ),
                  )}
                </tr>
              </thead>
              <tbody className="divide-y divide-outline-variant/50 font-data-mono text-data-mono">
                {VALIDATION_ROWS.map((r) => {
                  const active = r.id === selected.id;
                  return (
                    <tr
                      key={r.id}
                      onClick={() => setSelected(r)}
                      className={cn(
                        "group cursor-pointer transition-colors hover:bg-surface-bright",
                        active && "border-l-[3px] border-l-primary bg-surface-container-high",
                      )}
                    >
                      <td className="border-r border-outline-variant/30 px-margin-md py-margin-sm text-center text-on-surface-variant">
                        {r.id}
                      </td>
                      <td className="border-r border-outline-variant/30 px-margin-md py-margin-sm text-on-surface transition-colors group-hover:text-primary">
                        <span className="flex items-center gap-2">
                          {r.scenario}
                          {r.lostFrames > 0 && <TriangleAlert className="h-3.5 w-3.5 text-tertiary-container" strokeWidth={1.75} />}
                        </span>
                      </td>
                      <td className={cn("border-r border-outline-variant/30 px-margin-md py-margin-sm text-right tnum", r.detectionPct < 95 ? "text-tertiary-container" : "text-on-surface")}>
                        {fixed(r.detectionPct, 2)}
                      </td>
                      <td className="border-r border-outline-variant/30 px-margin-md py-margin-sm text-right tnum text-on-surface-variant">
                        {r.rmsDeg == null ? "N/A" : fixed(r.rmsDeg, 4)}
                      </td>
                      <td className="border-r border-outline-variant/30 px-margin-md py-margin-sm text-right tnum text-on-surface-variant">
                        {r.p95Deg == null ? "N/A" : fixed(r.p95Deg, 4)}
                      </td>
                      <td className="border-r border-outline-variant/30 px-margin-md py-margin-sm text-right tnum text-on-surface-variant">
                        {r.maxDeg == null ? "N/A" : fixed(r.maxDeg, 4)}
                      </td>
                      <td className={cn("border-r border-outline-variant/30 px-margin-md py-margin-sm text-right tnum", r.lostFrames > 0 ? "text-tertiary-container" : "text-on-surface-variant")}>
                        {r.lostFrames}
                      </td>
                      <td className="px-margin-md py-margin-sm text-center">
                        <span className="inline-flex items-center justify-center border border-primary/40 bg-primary/10 px-2 py-0.5 font-label-xs text-label-xs text-primary">
                          {r.status}
                        </span>
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
        </div>

        {/* selected scenario detail */}
        <div className="relative z-10 flex w-[320px] shrink-0 flex-col gap-margin-md">
          <div className="flex h-full flex-col border border-outline-variant bg-surface-container">
            <div className="flex items-start justify-between border-b border-outline-variant bg-surface-container-low p-margin-md">
              <div>
                <div className="mb-unit font-label-xs text-label-xs uppercase text-on-surface-variant">
                  Selected Scenario
                </div>
                <div className="font-headline-sm text-headline-sm text-on-surface">
                  {selected.id}: {selected.scenario}
                </div>
              </div>
              <span className="inline-flex h-6 w-6 items-center justify-center border border-primary/40 bg-primary/10 text-primary">
                <Check className="h-4 w-4" strokeWidth={2} />
              </span>
            </div>

            <div className="flex-1 space-y-margin-md overflow-y-auto p-margin-md">
              <div className="grid grid-cols-2 gap-gutter bg-outline-variant">
                {[
                  ["DETECTION", pct(selected.detectionPct, 2)],
                  ["FRAMES", String(selected.frames)],
                  ["RMS", selected.rmsDeg == null ? "N/A" : deg(selected.rmsDeg, 4)],
                  ["FINAL", deg(selected.finalDeg, 4)],
                  ["P95", selected.p95Deg == null ? "N/A" : deg(selected.p95Deg, 4)],
                  ["LOST", String(selected.lostFrames)],
                ].map(([k, v]) => (
                  <div key={k} className="flex flex-col gap-unit bg-surface-container-low p-margin-sm">
                    <span className="font-label-xs text-label-xs text-on-surface-variant">{k}</span>
                    <span className="font-data-mono text-data-mono tnum text-on-surface">{v}</span>
                  </div>
                ))}
              </div>

              <div>
                <div className="mb-margin-sm border-b border-outline-variant pb-unit font-label-xs text-label-xs uppercase text-on-surface-variant">
                  Verdict
                </div>
                <ul className="space-y-margin-sm font-data-mono text-[11px]">
                  {[
                    "Finite values — no NaN / Inf",
                    "Monotonic timestamps · fixed 50 Hz dt",
                    "Command rate ≤ PID limit · applied ≤ actuator limit",
                    "Deterministic replay (bit-identical second run)",
                    selected.lostFrames > 0
                      ? "Loss / hold / re-acquire semantics validated"
                      : "Tracking-quality gates met",
                  ].map((c) => (
                    <li key={c} className="flex items-start gap-margin-sm">
                      <Check className="mt-[2px] h-3.5 w-3.5 shrink-0 text-primary" strokeWidth={2} />
                      <span className="text-on-surface">{c}</span>
                    </li>
                  ))}
                </ul>
              </div>
            </div>

            <div className="border-t border-outline-variant bg-surface-container-low p-margin-md">
              <div className="flex items-start gap-margin-sm text-on-surface-variant">
                <Info className="mt-0.5 h-4 w-4 shrink-0 text-primary" strokeWidth={1.75} />
                <div className="font-body-md text-[11px] leading-tight">
                  <span className="font-bold text-primary">
                    {selected.note ? "Note:" : `${VALIDATION_SUMMARY.verdict}.`}
                  </span>{" "}
                  {selected.note ??
                    `Gates frozen before evaluation in docs/16_BASELINE_ACCEPTANCE.md and pinned to ${VALIDATION_SUMMARY.baselineTag}.`}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Screen>
  );
}

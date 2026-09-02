/**
 * SERVER-ONLY. Deterministic replay fixtures — checked-in telemetry generated
 * ONCE from the real C++ `fsoc_demo` engine (see scripts/generate-fixtures.mjs).
 *
 * These make the app deployable / demoable on machines without the C++ toolchain,
 * with byte-identical numbers to the engine. The visual components do not change
 * between LOCAL ENGINE MODE and REPLAY MODE.
 */

import { readFile } from "node:fs/promises";
import { existsSync } from "node:fs";
import path from "node:path";

import type { DemoSnapshot } from "@/lib/telemetry/types";
import type { ScenarioId } from "@/lib/baseline/constants";

function fixtureDir(): string {
  return path.join(process.cwd(), "lib", "telemetry", "fixtures");
}

export function fixturePath(scenario: ScenarioId): string {
  return path.join(fixtureDir(), `${scenario}.json`);
}

export function fixtureAvailable(scenario: ScenarioId): boolean {
  return existsSync(fixturePath(scenario));
}

interface RawFixture {
  meta?: { generatedAt?: string; enginePath?: string; source?: string };
  frames: DemoSnapshot[];
}

export async function loadFixture(
  scenario: ScenarioId,
): Promise<{ frames: DemoSnapshot[]; generatedAt?: string }> {
  const p = fixturePath(scenario);
  if (!existsSync(p)) {
    throw new Error(
      `replay fixture missing for '${scenario}' at ${p} — run: npm run gen:fixtures`,
    );
  }
  const raw = JSON.parse(await readFile(p, "utf8")) as RawFixture;
  if (!Array.isArray(raw.frames) || raw.frames.length === 0) {
    throw new Error(`replay fixture for '${scenario}' has no frames`);
  }
  return { frames: raw.frames, generatedAt: raw.meta?.generatedAt };
}

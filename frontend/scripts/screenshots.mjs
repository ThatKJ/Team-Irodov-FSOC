#!/usr/bin/env node
/**
 * Visual-parity capture: screenshot every route at desktop widths for comparison
 * against the FSOC Stitch screenshots.
 *
 *   frontend/ $  node scripts/screenshots.mjs [baseURL] [width]
 */
import { chromium } from "@playwright/test";
import { mkdir } from "node:fs/promises";
import path from "node:path";

const baseURL = process.argv[2] || "http://localhost:4317";
const width = Number(process.argv[3] || 1512);
const height = Number(process.argv[4] || 982);

const ROUTES = [
  ["overview", "/"],
  ["mission", "/mission"],
  ["tracking", "/tracking"],
  ["world", "/world"],
  ["telemetry", "/telemetry"],
  ["scenarios", "/scenarios"],
  ["benchmarks", "/benchmarks"],
  ["validation", "/validation"],
  ["architecture", "/architecture"],
];

const outDir = path.resolve(process.cwd(), "..", "generated", "frontend-shots");
await mkdir(outDir, { recursive: true });

const browser = await chromium.launch({
  channel: "chrome",
  args: [
    "--enable-unsafe-swiftshader",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--ignore-gpu-blocklist",
    "--enable-webgl",
  ],
});
const page = await browser.newPage({ viewport: { width, height } });

for (const [name, route] of ROUTES) {
  await page.goto(baseURL + route, { waitUntil: "networkidle" });
  // let telemetry + charts + R3F settle
  await page.waitForTimeout(route === "/world" ? 2500 : 1200);
  const file = path.join(outDir, `${name}-${width}.png`);
  await page.screenshot({ path: file, fullPage: false });
  console.log("  " + file);
}

await browser.close();
console.log(`\n  ${ROUTES.length} screenshots -> ${outDir}\n`);

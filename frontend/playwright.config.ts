import { defineConfig, devices } from "@playwright/test";

/**
 * E2E smoke tests for the FSOC mission-control frontend.
 * Runs against the production build on :4317 (deterministic REPLAY telemetry).
 */
export default defineConfig({
  testDir: "./tests/e2e",
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: 0,
  workers: 3,
  reporter: [["list"]],
  timeout: 30_000,
  expect: { timeout: 10_000 },
  use: {
    baseURL: process.env.FSOC_BASE_URL ?? "http://localhost:4317",
    trace: "off",
    screenshot: "off",
    viewport: { width: 1512, height: 982 },
  },
  // uses the system Google Chrome (no bundled-browser download required)
  projects: [
    {
      name: "chrome",
      use: {
        ...devices["Desktop Chrome"],
        channel: "chrome",
        launchOptions: {
          args: [
            "--enable-unsafe-swiftshader",
            "--use-gl=angle",
            "--use-angle=swiftshader",
            "--ignore-gpu-blocklist",
            "--disable-background-timer-throttling",
            "--disable-renderer-backgrounding",
            "--disable-backgrounding-occluded-windows",
          ],
        },
      },
    },
  ],
  webServer: {
    command: "npm run start",
    url: "http://localhost:4317",
    reuseExistingServer: true,
    timeout: 60_000,
  },
});

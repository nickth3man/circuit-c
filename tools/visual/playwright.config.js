/*
 * Playwright config for the vehicle-appearance inspector.
 *
 * DELIBERATELY NOT A CI GATE. docs/CI.md keeps the required checks headless and
 * platform-independent; these tests need a browser and their value is the evidence they emit,
 * not a pass/fail bit. Some of them are EXPECTED to fail today — that is the point: they are
 * executable statements of what "car-looking" requires, and they turn green as the grammar is
 * fixed. Run them by hand, read artifacts/visual/.
 */
const { defineConfig, devices } = require("@playwright/test");

module.exports = defineConfig({
  testDir: "./tests",
  outputDir: "../../artifacts/visual/test-output",
  fullyParallel: false,
  workers: 1,
  reporter: [
    ["list"],
    ["html", { outputFolder: "../../artifacts/visual/report", open: "never" }],
  ],
  use: {
    baseURL: "http://localhost:4173",
    // The cards are pixel art at game scale; a fractional device ratio would resample them and
    // every screenshot would lie about what the player sees.
    deviceScaleFactor: 1,
    screenshot: "only-on-failure",
  },
  projects: [{ name: "chromium", use: { ...devices["Desktop Chrome"], deviceScaleFactor: 1 } }],
  webServer: {
    command: "node serve.js",
    url: "http://localhost:4173/cards.json",
    reuseExistingServer: true,
    timeout: 20_000,
  },
});

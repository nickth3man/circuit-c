/*
 * capture.spec.js — the evidence generator.
 *
 * grammar.spec.js says WHAT is wrong in numbers. This produces the pictures needed to see it:
 * per-car cards at a zoom a human (or an agent reading the PNG) can actually judge, the feature
 * label map beside the paint, and a nose-to-tail strip of the sweeps where one parameter moves
 * and everything else is held still.
 *
 * Everything lands in artifacts/visual/. These tests assert almost nothing on purpose — a
 * capture that fails produces no evidence, which is the opposite of useful.
 */
const { test, expect } = require("@playwright/test");
const path = require("node:path");
const fs = require("node:fs");

const OUT = path.resolve(__dirname, "..", "..", "..", "artifacts", "visual");

test.beforeAll(() => fs.mkdirSync(OUT, { recursive: true }));

/* The five the eye goes to first: the default, two that should look sporty, one that should
 * look brutish, and one that should look purposeful. If these five are not distinguishable at a
 * glance, nothing in the tail of the corpus will be. */
const HEADLINE = [
  "archetype_00_stock_baseline",
  "archetype_05_sports_car",
  "archetype_06_supercar",
  "archetype_07_muscle_car",
  "archetype_11_high_angle_rwd",
];

async function configure(page, { zoom, labels, overlay }) {
  await page.goto("/");
  await page.waitForFunction(() => document.body.dataset.ready === "1");
  // The control bar is sticky for interactive use, which means it lands on top of whatever
  // row a full-element screenshot happens to scroll past. Pin it for captures only.
  await page.addStyleTag({ content: "#controls { position: static !important; }" });
  await page.fill("#search", "");
  await page.evaluate(
    ([z, l, o]) => {
      const set = (id, v) => {
        const el = document.getElementById(id);
        if (el.type === "checkbox") el.checked = v;
        else el.value = v;
        el.dispatchEvent(new Event("input", { bubbles: true }));
      };
      set("zoom", String(z));
      set("showLabels", l);
      set("showOverlay", o);
    },
    [zoom, labels, overlay]
  );
  await page.waitForFunction(() => document.body.dataset.ready === "1");
  await page.waitForTimeout(150);
}

async function shotCard(page, id, file) {
  const card = page.locator(`.card[data-id="${id}"]`);
  await expect(card).toBeVisible();
  await card.screenshot({ path: path.join(OUT, file) });
}

test("headline five: paint, overlay, and label map", async ({ page }) => {
  await page.setViewportSize({ width: 1600, height: 1200 });

  for (const [suffix, opts] of [
    ["paint", { zoom: 8, labels: false, overlay: false }],
    ["overlay", { zoom: 8, labels: false, overlay: true }],
    ["labels", { zoom: 8, labels: true, overlay: false }],
  ]) {
    await configure(page, opts);
    for (const id of HEADLINE) {
      await shotCard(page, id, `${id}__${suffix}.png`);
    }
  }
});

test("diagnostics panel", async ({ page }) => {
  await page.setViewportSize({ width: 1600, height: 900 });
  await configure(page, { zoom: 4, labels: false, overlay: true });
  await page.locator("#diagnostics").screenshot({ path: path.join(OUT, "diagnostics.png") });

  // Also emit the verdicts as text, so a diff between two runs is readable without opening
  // an image viewer.
  const diag = await page.evaluate(() => window.CIRCUIT_DIAGNOSTICS);
  fs.writeFileSync(
    path.join(OUT, "diagnostics.txt"),
    diag.map((d) => `[${d.cls.toUpperCase().padEnd(4)}] ${d.k}: ${d.v}`).join("\n") + "\n"
  );
  expect(diag.length).toBeGreaterThan(0);
});

test("archetype contact strip at review zoom", async ({ page }) => {
  await page.setViewportSize({ width: 1500, height: 3000 });
  await configure(page, { zoom: 6, labels: false, overlay: false });
  await page.selectOption("#group", "archetype");
  await page.waitForTimeout(250);
  await page.locator("#grid").screenshot({ path: path.join(OUT, "archetypes.png") });
});

/* The sweeps are the cleanest evidence there is: within one sweep exactly ONE spec field moves.
 * If a row of five looks identical, that parameter has no visual consequence — and the sweep
 * name says which parameter it is. */
test("per-sweep strips isolate one parameter at a time", async ({ page }) => {
  await page.setViewportSize({ width: 1500, height: 1400 });
  await configure(page, { zoom: 7, labels: false, overlay: false });

  const data = await page.evaluate(() => window.CIRCUIT_DATA);
  const sweeps = [
    ...new Set(
      data.cars
        .filter((c) => c.group === "sweep")
        .map((c) => c.id.replace(/_\d+$/, ""))
    ),
  ];

  const dir = path.join(OUT, "sweeps");
  fs.mkdirSync(dir, { recursive: true });

  for (const sweep of sweeps) {
    await page.fill("#search", sweep);
    await page.waitForTimeout(120);
    const grid = page.locator("#grid");
    if ((await grid.locator(".card").count()) === 0) continue;
    await grid.screenshot({ path: path.join(dir, `${sweep}.png`) });
  }
  expect(sweeps.length).toBeGreaterThan(0);
});

/* A grammar change should be visible as a diff, not argued about. This writes a stable, whole-
 * fleet montage; compare two of them across a change with any image differ. */
test("whole-corpus montage for before/after comparison", async ({ page }) => {
  await page.setViewportSize({ width: 1800, height: 8000 });
  await configure(page, { zoom: 3, labels: false, overlay: false });
  await page.locator("#grid").screenshot({ path: path.join(OUT, "corpus_montage.png") });
});

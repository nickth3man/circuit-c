/*
 * grammar.spec.js — executable statements of what the appearance grammar must deliver.
 *
 * These are not regression locks. Each one names a property a human uses to decide whether a
 * shape reads as a car, and checks it against the whole 100-vehicle corpus at the scale the
 * game actually draws at. Several FAIL on the current grammar; the failure message is the
 * diagnosis, and it names the parameter or rule at fault rather than reporting a pixel count.
 *
 * The data comes from window.DRIFTY_DATA, which the inspector exposes after loading cards.json.
 */
const { test, expect } = require("@playwright/test");

let DATA;

test.beforeAll(async ({ browser }) => {
  const page = await browser.newPage();
  await page.goto("/");
  await page.waitForFunction(() => window.DRIFTY_DATA);
  DATA = await page.evaluate(() => window.DRIFTY_DATA);
  await page.close();
});

const LATENTS = [
  "mass01", "size01", "low01", "grip01", "balance01",
  "power01", "aero01", "sport01", "strip01",
];

const range = (v) => Math.max(...v) - Math.min(...v);

test("corpus loaded at the scale the game draws at", () => {
  expect(DATA.cars.length).toBeGreaterThan(50);
  // PIXELS_PER_METER (24) * CAMERA_BASE_ZOOM (0.55). If this drifts, every pixel budget below
  // is being judged against a scale the player never sees.
  expect(DATA.pxPerM).toBeCloseTo(13.2, 1);
});

test("every style axis actually varies across the corpus", () => {
  const frozen = LATENTS.filter((k) => range(DATA.cars.map((c) => c.latents[k])) < 1e-6);
  expect(
    frozen,
    `These style axes are identical for all ${DATA.cars.length} vehicles, so every feature ` +
      `downstream of them is frozen too. Check whether any corpus entry varies the spec ` +
      `fields they read (car_visual.c car_visual_latents): low01<-cgHeightM, ` +
      `grip01<-tireMuLat*, balance01<-tireMuLatFront-tireMuLatRear.`
  ).toEqual([]);
});

test("each style axis uses at least a third of its normalised range", () => {
  const narrow = LATENTS.map((k) => [k, range(DATA.cars.map((c) => c.latents[k]))])
    .filter(([, r]) => r < 0.33)
    .map(([k, r]) => `${k}=${r.toFixed(3)}`);
  expect(
    narrow,
    "An axis normalised to 0..1 that only ever moves across a sliver of it cannot " +
      "differentiate cars, whatever the rules downstream do with it."
  ).toEqual([]);
});

/* Labels only record for alpha >= 128 (put_px in car_visual_raster.c). The L0 shadow is drawn
 * at alpha 55 deliberately, so that it cannot inflate the distinctness pixel count — its label
 * is therefore never written and its histogram entry is always zero. That is correct
 * behaviour, not a missing feature, so it is excluded here rather than reported forever. */
const UNLABELLABLE = new Set(["empty", "shadow"]);

test("no drawn feature is invisible across the whole fleet", () => {
  const never = DATA.labelNames
    .map((name, i) => [name, DATA.cars.reduce((a, c) => a + c.labelPixels[i], 0)])
    .filter(([name, total]) => !UNLABELLABLE.has(name) && total === 0)
    .map(([name]) => name);
  expect(
    never,
    "These CarRasterLabel features are implemented and never cover a single pixel on any of " +
      `the ${DATA.cars.length} vehicles. Either the rule that emits them never fires, or the ` +
      "geometry it produces is below the quantisation floor at 13.2 px/m."
  ).toEqual([]);
});

test("features that do appear are large enough to read", () => {
  const tiny = DATA.labelNames
    .map((name, i) => {
      const present = DATA.cars.filter((c) => c.labelPixels[i] > 0);
      if (UNLABELLABLE.has(name) || present.length === 0) return null;
      const mean = present.reduce((a, c) => a + c.labelPixels[i], 0) / present.length;
      return mean < 4 ? `${name} (${mean.toFixed(1)}px mean)` : null;
    })
    .filter(Boolean);
  expect(
    tiny,
    "A feature covering under ~4 pixels is a speck: it cannot carry shape information and " +
      "reads as noise or dirt. Either give it a presentation gain or stop drawing it."
  ).toEqual([]);
});

test("hulls taper — a car is not a rectangle from above", () => {
  const boxy = DATA.cars
    .map((c) => {
      const hw = c.hull.map((h) => h.hw);
      return { id: c.id, taper: (Math.max(...hw) - Math.min(...hw)) / Math.max(...hw) };
    })
    .filter((c) => c.taper < 0.25);
  expect(
    boxy.length,
    `${boxy.length}/${DATA.cars.length} vehicles vary their hull half-width by less than 25% ` +
      `from the widest station to the narrowest, so their silhouette is a rectangle. ` +
      `Worst: ${boxy.slice(0, 6).map((c) => `${c.id} ${(c.taper * 100).toFixed(0)}%`).join(", ")}`
  ).toBe(0);
});

test("nose and tail are distinguishable from the silhouette alone", () => {
  // Colour and lamp pixels are excluded on purpose: at this scale the lamps are one or two
  // pixels, so if the hull is symmetric the car has no readable facing.
  const symmetric = DATA.cars
    .map((c) => {
      const hw = c.hull.map((h) => h.hw);
      const n = hw.length;
      const half = Math.floor(n / 2);
      let diff = 0;
      for (let i = 0; i < half; i++) diff += Math.abs(hw[i] - hw[n - 1 - i]);
      return { id: c.id, asym: diff / half / Math.max(...hw) };
    })
    .filter((c) => c.asym < 0.1);
  expect(
    symmetric.length,
    `${symmetric.length}/${DATA.cars.length} vehicles have a hull that is within 10% of ` +
      `front-to-back mirror symmetry. A player cannot tell which way these cars point.`
  ).toBe(0);
});

test("the greenhouse sits off-centre, as a real cabin does", () => {
  // A cabin centred exactly on the CG with equal glass at both ends reads as a bathtub. Real
  // cars put the windscreen base well forward of the backlight relative to the wheelbase.
  const centred = DATA.cars.filter((c) => {
    const span = c.dims.windscreenX - c.dims.backlightX;
    return span > 0 && Math.abs(c.dims.cabinCentreX) < 0.05 * c.dims.length;
  });
  expect(
    centred.length,
    `${centred.length} vehicles place the cabin within 5% of the body centre. ` +
      `Ids: ${centred.slice(0, 8).map((c) => c.id).join(", ")}`
  ).toBeLessThan(DATA.cars.length * 0.25);
});

test("wheels are contained by the bodywork except where openWheel says otherwise", () => {
  // poke is how far the tire stands proud of the hull. A positive poke on a car that is not
  // meant to be open-wheel is exactly the "wheels bolted on the outside" read.
  const poking = DATA.cars
    .map((c) => {
      const maxPoke = Math.max(...c.wheels.map((w) => w.poke));
      return { id: c.id, maxPoke, openWheel: c.dims.openWheel, flare: c.dims.archFlare };
    })
    .filter((c) => c.openWheel < 0.5 && c.maxPoke > c.flare);
  expect(
    poking.length,
    `${poking.length} vehicles have a tire standing further outside the hull than the wheel ` +
      `arch flares to cover it, with no open-wheel intent. The arch cannot visually contain ` +
      `the tire, so the wheels read as detached. Worst: ` +
      poking
        .sort((a, b) => b.maxPoke - b.flare - (a.maxPoke - a.flare))
        .slice(0, 6)
        .map((c) => `${c.id} poke=${c.maxPoke.toFixed(3)} flare=${c.flare.toFixed(3)}`)
        .join(", ")
  ).toBe(0);
});

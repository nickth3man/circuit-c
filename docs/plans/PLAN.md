# Drifty — the remaining work

This is the single plan file for the project. It states where the work actually stands, then
every remaining task in enough detail to execute. [ROADMAP.md](ROADMAP.md) beside it is the
same content as a checklist; this file is the reasoning behind each line.

`docs/SPEC.md` remains authoritative for *what* the physics phases contain.
[AGENTS.md](../AGENTS.md) remains authoritative for *how* to work in the repo. This file is
authoritative only for *what is left*.

---

## Where the work actually stands

Verified by scenario inventory, registry inspection and file substance rather than by trusting
any earlier status document.

### Physics and gameplay — phases 0–6 all complete

Earlier status tracking marked phases 4, 5 and 6 as unstarted. That was wrong: all three are
implemented and have test coverage. Roughly 14 of the 54 scenarios exercise them.

| Phase | Evidence |
|---|---|
| 0 Foundations | `math` `units` `timestep` `oneshot` `replay` `renderscale` |
| 1 Rigid body | `vehicle` `rest` `launch-stop` `steer-sign` `lever-arm` `integration` `fixed-rate` |
| 2 Tire / drivetrain | `tire` `drivetrain` `coast-down` `brake-corner` `power-oversteer` `handbrake` `low-speed` `reverse` |
| 3 Load transfer | `accel-filter` `load-transfer` `resistance` `accel-load` `brake-load` `skidpad` `skidpad-sweep` `step-steer` `transition` `lift-off` `catchable-drift` |
| 4 Four-wheel fidelity | `lat-load-transfer` `open-diff` `lsd-diff` `ackermann` `load-sensitivity` `tire-relaxation` `surface-asymmetry`; registry carries `drive.diff_mode`, `steer.ackermann_percent`, `tire.relaxation_length`, `tire.load_sensitivity_k`, `susp.anti_roll_*`, `susp.roll_centre_*`; `physics.c` iterates `WHEEL_COUNT` |
| 5 Track / surfaces / collision | `track-surface` `collision-barrier` `checkpoint-lap`; `src/world/track.c` 251 lines, `src/world/collision.c` 470, `src/physics/surface.c` 57; lap HUD in `render.c` |
| 6 Scoring / effects / presentation | `scoring-accumulation` `scoring-rejection` `scoring-determinism` `highscore-persistence` `particle-pool` `state-machine`; `src/game/scoring.c`, `src/game/particle.c`; camera zoom in `render.c`; high-score load/save at `src/game/game.c:150` |

**No physics or gameplay phase work remains.** Everything below is vehicle appearance,
tooling, or hygiene.

### Vehicle appearance — the system exists, the output is not yet good

The grammar, rasterizer, corpus, texture path and gallery all shipped. A car's appearance is a
pure, total, deterministic function of its physics parameters; there is no hand-authored art.
The contract is [docs/CAR_VISUAL.md](../docs/CAR_VISUAL.md).

Three defects were found by measuring 77 reference top-down sprites against all 17 archetypes,
and three have been fixed:

| Fixed | Was | Now |
|---|---|---|
| Frozen style axes | `low01` `grip01` `balance01` range 0.000; `sport01` 0.142 | 0.395 / 0.524 / 0.462; `sport01` 0.444 |
| Palette hue split | median 3 saturated hue families per car (reference median 1) | median 1, max 2 |
| The pickup bed | drawn on **78/100** vehicles, 74 not trucks; 47% of the muscle car's bodywork | **1/100** — only where `body.bed_length > 0` |

Current gate state: **54 scenarios, 1111 checks, 0 failed.** Registry: **123 parameters.**
Corpus: **100 vehicles** (17 archetype, 40 sweep, 43 sampled).

### The measurement baseline to work against

Reference medians from 77 sprites, against current Drifty medians:

| metric | reference | Drifty | status |
|---|---|---|---|
| saturated hue families / car | 1 | 1 | fixed |
| nose gain over first 15% of length | 10 px | 8 px | acceptable |
| **tail closure over last 15%** | **6 px** | **2 px** | **open** |
| hull taper (max→min half-width) | — | 46% | acceptable |
| largest flat single-colour rect | 13.7% | 17.2% | acceptable |
| glass width as % of body width | 80% | 80% | acceptable |
| **% of length covered by dark band** | **29%** | **59%** | **open** |
| wheels visible on passenger cars | none | light grey blocks outboard | **open** |

Four grammar assertions are still red, and each is a real defect rather than a strict test:

1. `each style axis uses at least a third of its normalised range` — `aero01` at 0.305.
2. `features that do appear are large enough to read` — `exhaust` 2.1 px, `tow_hook` 1.0 px,
   `hood_pins` 1.4 px, `heading` 3.0 px mean.
3. `nose and tail are distinguishable from the silhouette alone` — 1/100 within 10% of mirror
   symmetry.
4. `the greenhouse sits off-centre, as a real cabin does` — 75/100 place the cabin within 5%
   of the body centre.

---

## Part 1 — Vehicle appearance: the parameter expansion

### Why parameters and not better rules

The grammar cannot express what it is not told. The bed bug is the worked example: the rule
inferred a bed from `(backlightXM − tailX) / lengthM`, which on a three-box car is the boot.
No threshold separates them, because the measurement is genuinely ambiguous. Adding
`body.bed_length` dissolved the bug that no amount of rule tuning could have fixed.

The same reasoning applies to wing span, splitter, arch flare, side-window count and the nose
and tail taper: all are computed today from unrelated physics and cannot be authored.

### The gate every candidate passes

At 13.2 px/m, **1 px = 7.58 cm**. A parameter earns a place only if it is a real measurable
vehicle quantity *and* moves at least one pixel across its plausible range. Every entry below
states its pixel effect. Candidates that fail this test are listed under "Rejected" so they
are not re-proposed.

### Remaining phases

Phases 0a, 0b and A are done. B through H remain. One PR per phase, matching the existing
`vehicle-visuals/phaseN-*` branch convention.

---

#### Phase B — silhouette control

Replaces the taper that `sport01` drives too weakly, and closes the measured 3× tail gap.

| key | meaning | range | px effect |
|---|---|---|---|
| `body.nose_width` | width at the foremost station | 0.8–2.2 m | 11–29 px |
| `body.tail_width` | width at the rearmost station | 0.8–2.4 m | 11–32 px |
| `body.shoulder_x` | station of maximum width (layout frame) | — | position over ~60 px |
| `body.fender_flare_front` | flare proud of the hull over the arch | 0–0.12 m | 0–1.6 px |
| `body.fender_flare_rear` | same, rear | 0–0.15 m | 0–2 px |

`nose_width` and `tail_width` become `[identity]` hull stations; `sport01` is demoted to
modulating only the stations *between* them.

Also fix here: `archetype_06_supercar` and `archetype_08_gt3_race_car` currently have
**negative nose gain** (−7 px and −6 px) — the splitter makes the nose the widest point on the
car. Explicit `nose_width` plus the phase F splitter parameter resolves it.

**Closes:** grammar assertion 3; the tail-closure row of the measurement table.
**Sweep candidates:** `nose_width`, `tail_width`, `shoulder_x` clear the floors.

#### Phase C — greenhouse

The largest remaining visual gap. Reference sprites are consistently **dark glass / body-
coloured roof / dark glass**; Drifty draws two dark rectangles with no roof band between them,
which reads as two sunroofs. This is also why the dark-band-per-length figure is 59% against a
reference 29%.

| key | meaning | range | px effect |
|---|---|---|---|
| `body.roof_start_x` | forward edge of the roof panel (layout frame) | — | 3–8 px band |
| `body.roof_end_x` | aft edge of the roof panel (layout frame) | — | 3–8 px band |
| `body.roof_width` | roof width at its widest (tumblehome) | 0.15–0.45 m inset/side | 2–6 px/side |
| `body.windscreen_rake` | A-pillar angle from vertical | 20–70° | ~7.5 px of glass length |
| `body.backlight_rake` | rear screen angle | 15–75° | 3–6 px |
| `body.side_window_count` | promote from derived | 2–6 | 8–10 px/segment |
| `body.quarter_window` | window aft of the rear door | 0 / 0.2–0.4 m | 0 / 3–5 px |
| `body.sunroof_length` | 0 = none | 0 / 0.4–1.0 m | 0 / 5–13 px |
| `body.door_count` | 2 / 4 / 5 | — | drives B-pillar and side glass |
| `body.cabin_rows` | seat rows | 1–3 | cabin length; ties to `mass.driver_x` |
| `body.roof_type` | fixed / targa / convertible | enum | ~15 px |

A raked windscreen shows *more* glass from directly above, which is what separates a supercar
from a van in one number; it also feeds `Cd`.

**Closes:** grammar assertion 4; the dark-band-proportion row.
**Sweep candidates:** `roof_start_x`, `windscreen_rake`, `cabin_rows`.

#### Phase D — lighting

Currently zero lighting parameters. Lamps render as a full-width band across the nose, reading
as a bumper stripe. Every reference sprite uses **bright corner wedges** as its primary facing
cue.

| key | meaning | range | px effect |
|---|---|---|---|
| `light.headlamp_width` | single lamp width | 0.15–0.45 m | 2–6 px |
| `light.headlamp_inset` | inset from the body corner | 0.05–0.35 m | 0.7–4.6 px |
| `light.headlamp_count` | 2 or 4 | — | period and class cue |
| `light.lamp_wrap` | how far the lamp wraps onto the flank | 0–0.30 m | 0–4 px |
| `light.taillamp_width` | rear lamp width | 0.10–0.60 m | 1.3–8 px |
| `light.taillamp_count` | 2 or 4 | — | — |
| `light.third_brake_light_width` | centre high-mount, 0 = none | 0 / 0.3–0.6 m | 0 / 4–8 px |
| `light.fog_lamp_count` | 0 or 2 | — | ~1.3 px each |

Lamps are not dynamics, but neither is `body.height_overall`, which exists because it feeds
frontal area and the look. `VehicleSpec` is a vehicle spec, not a solver spec, and lamps are
measurable physical geometry — the CAR_VISUAL.md contract holds.

#### Phase E — openings, roof plane, and hardware

| key | meaning | range | px effect |
|---|---|---|---|
| `body.grille_width` | main intake width; cooling demand ∝ power | 0.4–1.6 m | 5–21 px |
| `body.grille_height` | intake height; also feeds frontal area | 0.1–0.6 m | shading band |
| `body.hood_scoop_length` | explicit, vs derived `hoodBulgeStrength` | 0–0.5 m | 0–7 px |
| `body.hood_vent_count` | louvres / extractors | 0–4 | ~3 px each |
| `body.roof_rack_length` | longitudinal rails, 0 = none | 0 / 0.8–1.8 m | 0 / 11–24 px |
| `aero.roof_scoop_length` | rally / race roof intake | 0 / 0.3–0.6 m | 0 / 4–8 px |
| `body.mirror_span` | mirror-to-mirror width | 1.8–2.3 m | 1.3–3.3 px/side |
| `body.spare_wheel_diameter` | rear-mounted spare, 0 = none | 0 / 0.6–0.8 m | 0 / 8–11 px |
| `exhaust.tip_offset_y` | centre / corner / side exit | 0–1.0 m | 0–13 px |

Straight down is the one view where the roof is the largest visible surface, and nothing is
drawn on it today. Mirrors currently render as detached specks (6.3 px for the pair, separated
from the hull); `mirror_span` attaches them.

#### Phase F — aero made explicit

Wing and splitter geometry is derived from lift coefficient × reference area, so "big wing"
and "much downforce" cannot be separated.

| key | meaning | range | px effect |
|---|---|---|---|
| `aero.wing_span` | explicit, not derived | 0.8–2.0 m | 11–26 px |
| `aero.wing_chord` | explicit | 0.10–0.40 m | 1.3–5 px |
| `aero.splitter_length` | explicit | 0–0.35 m | 0–4.6 px |
| `aero.canard_count` | 0–3 | — | ~2 px each |

**Closes:** grammar assertion 1 — `aero01` is the last axis under the one-third bar at 0.305,
and it is narrow because the corpus expresses aero only through two coefficient keys.
**Sweep candidate:** `wing_span`.

#### Phase G — physics-only parameters

No visual consequence; a sweep row would be five identical cars and would fail distinctness by
construction. These get neither a sweep nor a `kVisualDrivers[]` entry.

There are **no damper parameters at all** today — `susp.wheel_rate_*` and `susp.anti_roll_*`
exist, bump and rebound do not. Transient response cannot be right without them.

- `susp.damper_bump_front` / `_rear`, `susp.damper_rebound_front` / `_rear`
- `susp.anti_dive`, `susp.anti_squat` — pitch under braking and power
- `susp.kingpin_inclination`, `susp.scrub_radius` — steering feel and self-centring
- `chassis.torsional_stiffness` — needed if `body.roof_type` is to mean anything mechanically

This phase moves physics, so it is the one that needs `mk verify` and a reviewed
`mk baselines` regeneration. Review `artifacts/regression.md` by hand and record the accepted
deltas before re-recording.

#### Phase H — deletions and small-feature cleanup

- **Remove the `spokeCount` ← `wheelInertiaKgM2` derivation** and its signature component. A
  rim is 4.4–6.7 px across; five spokes will never resolve. Its measured signature range
  across all 100 vehicles is exactly **zero** — it is already doing nothing.
- **Resolve the four sub-4px features** flagged by grammar assertion 2: `exhaust` (2.1 px),
  `tow_hook` (1.0 px), `hood_pins` (1.4 px), `heading` (3.0 px). Each is either given a
  documented presentation gain or stopped being drawn. `heading` is a gameplay affordance and
  should be gained, not deleted.
- **Consider darkening the wheels.** Across all 75 reference passenger-car sprites, wheels are
  not drawn at all — at most 1 px dark notches. Drifty's `#969ba5` rims are unlike anything in
  the reference set. Keep the wheels as separate sprites for steering legibility, but move
  their value toward the reference convention.

**Closes:** grammar assertion 2; the visible-wheels row of the measurement table.

---

### Rejected on measurement — do not re-propose

| candidate | why |
|---|---|
| `wheel.spoke_count` | rim is 4.4–6.7 px; five spokes is sub-pixel |
| `tire.tread_pattern` | contact patch 2–4.6 px wide; tread blocks sub-pixel. Could legitimately drive tyre *value* (slicks darker) with no geometry |
| `wheel.rim_style` | sub-pixel |
| `brake.caliper_pistons` | sub-pixel |
| `aero.diffuser_length` | invisible from directly above |
| `body.hood_height`, `body.deck_height` | 1–2 px shading band; only reads against a roof band, so revisit after phase C |
| `body.tow_hitch` (~2 px), `wheel.mud_flap` (3×3 px), `body.rocker_inset` (~2 px), `aero.side_skirt_depth` (~1 px) | marginal; add only if a specific archetype needs one |

---

### The repeatable mechanic for one parameter

Eleven touch points. `body.cowl_x` at `src/dev/dev_params.c:50` is the worked template. The full
procedure with its failure modes is in [AGENTS.md](../AGENTS.md) under
*"Adding a parameter: the eleven touch points"* — including the one that matters most:

> **Step 8, `car_visual_bake_key()`, fails silently.** Omit it and everything compiles, the
> slider moves, the value reaches the grammar, and the sprite never rebakes. Only the bake-key
> assertion driven by `kVisualDrivers[]` catches it, so skipping both is undetected.

### Landing each phase without visual churn

A new parameter should reproduce the current derived output, so a phase lands pixel-identical
and every later change is a reviewable diff. A constant default only achieves that for the
*stock* car, so:

1. Before landing, run `./build/tests/drifty_tests.exe --dump-corpus-cards artifacts/corpus-cards`.
   `cards.json` already carries every derived quantity per vehicle.
2. Generate `A("body.nose_width", <that car's current value>)` assignments for the 17
   archetypes from that JSON.
3. Land the parameter with those assignments in the same commit; tune in a follow-up.

**Phase A was the deliberate exception** — its entire purpose was to change 78 vehicles, so
landing it identical would have defeated it. Expect the same for any phase that is fixing a
bug rather than adding expression.

### Corpus evidence: sweeps where they work, `kVisualDrivers[]` otherwise

Every corpus **pair** must clear three floors simultaneously: ≥3.0% of the union silhouette
differing (`CV_MIN_PIXEL_DIFF`), signature L∞ ≥ 0.08 m (`CV_MIN_LINF`), and signature
L2 ≥ 0.25 (`CV_MIN_L2`). Adjacent sweep steps are the tightest pairs in the fleet, so an axis
must move roughly a fifth of its range past all three floors, four times over.

**Most parameters cannot.** `src/dev/car_corpus.c` records three worked examples with the measured
numbers that forced each decision — `tire.aspect_*`, `body.backlight_x` and `body.bed_length`.
Read them before adding or removing an axis.

**Never lower a corpus-wide threshold to accommodate one row.** A key that cannot carry a
sweep goes in `kVisualDrivers[]` instead, where the sensitivity test perturbs it across its
whole declared range and the bake-key assertion proves the sprite rebakes for it.

Sweep candidates from the remaining phases: `nose_width`, `tail_width`, `shoulder_x`,
`roof_start_x`, `windscreen_rake`, `cabin_rows`, `wing_span`. Each must be measured, not
assumed — `bed_length` looked like a certainty and failed, because its first generated step
(0.1875 m) renders as nothing and duplicated the stock baseline.

### Cross-cutting constraints

- **Layout frame ≠ body frame.** Longitudinal stations (`roof_start_x`, `roof_end_x`,
  `shoulder_x`) use the layout frame (axle-midpoint origin) like `cowl_x` / `backlight_x`, and
  convert only via `layout_to_body_x()`. Reading a layout value as a body value shifts the
  whole greenhouse by the CG offset.
- **Adding a `VehicleSpec` field changes the `Game` layout** → one manual `drifty.exe` restart
  per phase. Module hot reload is fine thereafter.
- **The `params` scenario asserts every registry `defaultValue` against
  `vehicle_spec_set_default()`** — they cannot disagree.
- **After editing `src/dev/car_corpus.c`, run
  `./build/tests/drifty_tests.exe --generate-corpus data/vehicles/corpus`** — the profiles are checked in and the
  round-trip is asserted.
- **Registry ranges are development limits, not physical claims.** Where a sweep needs a
  narrower range than physical reality (as `body.bed_length` did, capped at 1.5 m against the
  ~1.57 m the stock car can contain), say so in the entry's comment.
- **State the pixel arithmetic** in every entry's comment.

---

## Part 2 — Tooling and hygiene

### T1. A test that makes the silent bake-key failure impossible

Currently `car_visual_bake_key()` omission is caught only if the key was also added to
`kVisualDrivers[]`. Add a scenario that walks the registry and asserts every non-derived key
which reaches `CarVisual` is covered by the bake key. This converts the most dangerous step of
the eleven-point procedure from documentation into a gate, and it pays for itself over the
~37 parameters still to be added.

### T2. Tracy profiler

The zone macros and the `mk profile` selection exist; the distribution is not vendored. Drop
it into `third_party/tracy/` and the build picks it up. Deliberately deferred, not forgotten.

### T3. Visual regression in CI

`mk screenshots` and `mk visual-test` work locally. Hosted runners have no GPU, so the gate
stays on the developer's machine. The headless appearance measurements (`mk visual-diagnose`)
are the CI-safe substitute and could be promoted to a required check once the four red grammar
assertions are green — a gate that is expected to fail is not a gate.

### T4. clang-format adoption

`.clang-format` matches the existing style but the tree has not been normalised, so the CI
check is advisory. Land the normalisation as its own commit, touching nothing else, then make
the check required.

### T5. Unverified replay checksum in the Phase 3 record

`docs/PHASE3_VALIDATION.md` records replay checksum `f0b4580e`. It has not been re-verified
since the parameter expansion. Either confirm it still reproduces or record the new value with
an explanation — a determinism claim that nobody checks is worse than none.

---

## Verification, every phase

Headless, and required:

```bash
./build.sh --tests && ./build/tests/drifty_tests.exe
```

All scenarios must stay green — especially `params`, `car-visual` and `corpus`.

Appearance measurement, and expected to improve rather than merely pass:

```bash
mk visual-diagnose
```

Read `artifacts/visual/diagnostics.txt` and the grammar suite. Per-phase expectations:

| phase | expected movement |
|---|---|
| B | tail closure median 2 px → toward the reference 6 px; `nose and tail are distinguishable` passes; supercar and GT3 nose gain stops being negative |
| C | dark-band-per-length 59% → toward 29%; `the greenhouse sits off-centre` passes |
| D | corner lamp wedges replace the full-width nose band |
| E | roof plane stops being empty; mirrors attach to the hull |
| F | `aero01` range 0.305 → past 0.33; `each style axis uses at least a third` passes |
| G | no visual movement expected; `mk verify` and a reviewed `mk baselines` instead |
| H | `spoke_level` leaves the sub-pixel signature list; `features are large enough to read` passes |

Compare the montage across each phase:

```bash
mk cards
```

then diff `artifacts/visual/corpus_montage.png` before against after.

## Out of scope

- Any change to `src/render/render.c`'s bake/compose path — the five-sprite split and the
  `car_visual_bake_key()` cache are sufficient throughout.
- Multiple cars and AI opponents; skid-mark decals; track file loading; split-screen and ghost
  replays; in-menu car spec selection; full Magic Formula MF6.1. Rationale for each is in
  `docs/SPEC.md`.
- Lowering any corpus distinctness threshold.

# Drifty — Roadmap and checklist

Status at a glance. [PLAN.md](PLAN.md) beside it carries the reasoning, the measured evidence,
and the execution detail for every unchecked line here.

`docs/SPEC.md` is authoritative for what each physics phase contains.
[AGENTS.md](../AGENTS.md) is authoritative for how to work in the repo.

**Current gate state:** 54 scenarios, 1111 checks, 0 failed · registry 123 parameters ·
corpus 100 vehicles.

---

## Physics and gameplay — complete

Two workstreams number their phases independently. These are the `docs/SPEC.md` phases.

| Phase | Scope | Status |
|-------|-------|--------|
| 0 | Foundations and test harness | ✅ Complete |
| 1 | Rigid-body vehicle | ✅ Complete |
| 2 | Tire, drivetrain, braking, combined slip | ✅ Complete |
| 3 | Load transfer and handling validation | ✅ Complete |
| 4 | Four-wheel fidelity | ✅ Complete |
| 5 | Track, surfaces, collision | ✅ Complete |
| 6 | Scoring, effects, presentation | ✅ Complete |

Phases 4–6 were tracked as unstarted for some time; they are not. Roughly 14 of the 54
scenarios exercise them — see PLAN.md for the per-phase evidence.

**No physics or gameplay phase work remains.**

---

## Vehicle appearance — the system ships, the output needs work

| Phase | Scope | Status |
|-------|-------|--------|
| 0–6 | Grammar, rasterizer, corpus, texture path, gallery, CI | ✅ Complete |
| 0a | Unfreeze the three dead style axes | ✅ Complete |
| 0b | Palette hue unity | ✅ Complete |
| A | `body.bed_length` — retire the 78/100 bed bug | ✅ Complete |
| B | Silhouette control | ⬜ Not started |
| C | Greenhouse | ⬜ Not started |
| D | Lighting | ⬜ Not started |
| E | Openings, roof plane, hardware | ⬜ Not started |
| F | Aero made explicit | ⬜ Not started |
| G | Physics-only suspension parameters | ⬜ Not started |
| H | Deletions and small-feature cleanup | ⬜ Not started |

### Phase B — silhouette control

Closes the measured 3× tail-closure gap and the negative nose gain on supercar and GT3.

- [ ] `body.nose_width` — 0.8–2.2 m, 11–29 px
- [ ] `body.tail_width` — 0.8–2.4 m, 11–32 px
- [ ] `body.shoulder_x` — station of maximum width, layout frame
- [ ] `body.fender_flare_front` — 0–0.12 m
- [ ] `body.fender_flare_rear` — 0–0.15 m
- [ ] Demote `sport01` to modulating only the stations between nose and tail
- [ ] Measure sweep eligibility for `nose_width`, `tail_width`, `shoulder_x`

**Complete when:** tail closure median moves from 2 px toward the reference 6 px; the
`nose and tail are distinguishable from the silhouette alone` assertion passes; no archetype
has negative nose gain.

### Phase C — greenhouse

The largest remaining visual gap: reference sprites are dark glass / body-coloured roof / dark
glass; Drifty draws two dark rectangles with no roof band.

- [ ] `body.roof_start_x`, `body.roof_end_x` — layout frame
- [ ] `body.roof_width` — tumblehome, 0.15–0.45 m inset per side
- [ ] `body.windscreen_rake` — 20–70°, ~7.5 px of glass length
- [ ] `body.backlight_rake` — 15–75°
- [ ] `body.side_window_count` — promote from derived, 2–6
- [ ] `body.quarter_window` — 0 / 0.2–0.4 m
- [ ] `body.sunroof_length` — 0 / 0.4–1.0 m
- [ ] `body.door_count` — 2 / 4 / 5
- [ ] `body.cabin_rows` — 1–3
- [ ] `body.roof_type` — fixed / targa / convertible
- [ ] Measure sweep eligibility for `roof_start_x`, `windscreen_rake`, `cabin_rows`

**Complete when:** dark band per length moves from 59% toward the reference 29%; the
`greenhouse sits off-centre` assertion passes.

### Phase D — lighting

Zero lighting parameters exist today. Lamps render as a full-width nose band; every reference
sprite uses corner wedges as its facing cue.

- [ ] `light.headlamp_width` — 0.15–0.45 m, 2–6 px
- [ ] `light.headlamp_inset` — 0.05–0.35 m
- [ ] `light.headlamp_count` — 2 or 4
- [ ] `light.lamp_wrap` — 0–0.30 m
- [ ] `light.taillamp_width` — 0.10–0.60 m
- [ ] `light.taillamp_count` — 2 or 4
- [ ] `light.third_brake_light_width` — 0 / 0.3–0.6 m
- [ ] `light.fog_lamp_count` — 0 or 2

**Complete when:** corner wedges replace the full-width band and facing is readable without
relying on the heading marker.

### Phase E — openings, roof plane, hardware

- [ ] `body.grille_width` — 0.4–1.6 m, cooling demand ∝ power
- [ ] `body.grille_height` — 0.1–0.6 m, also feeds frontal area
- [ ] `body.hood_scoop_length` — 0–0.5 m
- [ ] `body.hood_vent_count` — 0–4
- [ ] `body.roof_rack_length` — 0 / 0.8–1.8 m
- [ ] `aero.roof_scoop_length` — 0 / 0.3–0.6 m
- [ ] `body.mirror_span` — 1.8–2.3 m; attaches the currently detached mirrors
- [ ] `body.spare_wheel_diameter` — 0 / 0.6–0.8 m
- [ ] `exhaust.tip_offset_y` — 0–1.0 m

**Complete when:** the roof plane is no longer empty and mirrors read as attached to the hull.

### Phase F — aero made explicit

- [ ] `aero.wing_span` — 0.8–2.0 m, 11–26 px
- [ ] `aero.wing_chord` — 0.10–0.40 m
- [ ] `aero.splitter_length` — 0–0.35 m
- [ ] `aero.canard_count` — 0–3
- [ ] Measure sweep eligibility for `wing_span`

**Complete when:** `aero01` range moves past 0.33 and the
`each style axis uses at least a third of its normalised range` assertion passes.

### Phase G — physics-only

No visual consequence; no sweep, no `kVisualDrivers[]` entry. There are no damper parameters
at all today.

- [ ] `susp.damper_bump_front` / `_rear`
- [ ] `susp.damper_rebound_front` / `_rear`
- [ ] `susp.anti_dive`, `susp.anti_squat`
- [ ] `susp.kingpin_inclination`, `susp.scrub_radius`
- [ ] `chassis.torsional_stiffness`
- [ ] `mk verify`, then a hand-reviewed `mk baselines` with the accepted deltas recorded

**Complete when:** transient response is damped, and the baseline regeneration is an inspected
change with its rationale written down.

### Phase H — deletions and small-feature cleanup

- [ ] Remove the `spokeCount` ← `wheelInertiaKgM2` derivation and its signature component
      (measured range across all 100 vehicles: exactly zero)
- [ ] Resolve the four sub-4px features: `exhaust` 2.1 px, `tow_hook` 1.0 px, `hood_pins`
      1.4 px, `heading` 3.0 px — gain or delete each; `heading` is a gameplay affordance and
      should be gained
- [ ] Move wheel value toward the reference convention (no reference passenger car draws
      visible wheels; Drifty's `#969ba5` rims have no counterpart there)

**Complete when:** `spoke_level` leaves the sub-pixel signature list and the
`features that do appear are large enough to read` assertion passes.

---

## Tooling and hygiene

- [ ] **T1** — a scenario walking the registry to assert bake-key coverage, so the silent
      `car_visual_bake_key()` omission becomes impossible rather than merely documented
- [ ] **T2** — vendor Tracy into `third_party/tracy/`; macros and `mk profile` already exist
- [ ] **T3** — promote `mk visual-diagnose` to a required CI check once the four red grammar
      assertions are green
- [ ] **T4** — normalise the tree with clang-format as its own commit, then make the check
      required
- [ ] **T5** — re-verify or re-record the replay checksum in `docs/PHASE3_VALIDATION.md`

---

## Standing constraints

These hold in every phase and are not renegotiated by a later one:

- SI units internally; pixels exist only at the render boundary, via `units.h`.
- `physics.*`, `vehicle.*`, `tire.*`, `drivetrain.*`, `car_visual.*`, `car_visual_raster.*`
  call no raylib function — that is what keeps `drifty_tests` headless.
- Drift is emergent. The handbrake is brake torque; scoring observes physics and never
  modifies it.
- Appearance is a pure, total, deterministic function of the physics parameters. No
  `body.type` enum, no per-archetype drawing branch, no per-car art asset, no geometry from a
  hash of spec bytes.
- Nothing reachable from `Game` may point into the game module's code or static data, and no
  function pointers live in persistent state.
- No allocation during gameplay.
- Every build command terminates.
- No corpus distinctness threshold is lowered to accommodate a row.

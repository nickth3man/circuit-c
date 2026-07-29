# Parametric vehicle appearance

A car's appearance in Drifty is a **pure, total, deterministic function of its physics
parameters**. There is no hand-authored art for any vehicle, no `body.type` enum, and no
per-car drawing branch. Two cars with different parameters look different; two cars with
identical parameters render identically, bit for bit.

This document is the contract. It states what every drawn feature reads, what is honest
physics and what is a stated interpretation, where the render-only amplifications are and why
each one exists, and which rules a change must not break.

- `docs/generated/CORPUS.md` — the 100 demonstration vehicles, generated
- `docs/generated/PARAMETERS.md` — the tunable registry, generated

---

## Where the code lives

```
VehicleSpec ─► car_visual_derive() ─► CarVisual ─► car_raster_draw_part() ─► RGBA8
               src/render/car_visual.c                    src/render/car_visual_raster.c      │
               THE GRAMMAR                         THE RASTERIZER               │
                     │                                                          │
                     ├─► car_visual_signature()  the diagnostic feature vector  │
                     └─► car_visual_bake_key()   the texture cache key          │
                                                                                │
                              ┌─────────────────────────────────────────────────┤
                    tests/support/car_sheet.c                       src/render/render_vehicle.c
                    headless PNG contact sheet                    Texture2D, drawn rotated
                    (no GPU, no window, CI-safe)                   (the same pixels)
```

Four rules make this work, and each one is load-bearing:

1. **`src/render/car_visual.c` is the only place a styling decision may live.** The
   `src/render/render_vehicle.c` texture consumer and the contact-sheet writer are dumb consumers;
   neither may invent geometry. A rule kept in the rasterizer would be invisible to the signature,
   and therefore to every test that reads it — this has happened once already (the roof and glass
   rules) and was moved.

2. **`car_visual.c` and `car_visual_raster.c` are raylib-free.** They include `raylib.h` for
   the `Color` and `Vector2` *types* and call no raylib function, exactly as `src/core/units.h`
   does. That is what lets both live in `SHARED_SRCS` and be linked into the headless
   `build/tests/drifty_tests.exe`. It matters because `render.c` stubs its whole draw path out under
   `DRIFTY_HEADLESS`: anything decided there is unverifiable.

3. **One rasterizer, two consumers.** The contact sheet and the in-game sprite come from the
   same pixel buffer, so the gallery cannot drift away from the game.

4. **`CarVisual` is a stack local**, derived per bake. It is never stored in `Game` — that
   would be a layout change for no gain — and never cached across a spec edit, because the
   Physics Lab writes `game->spec` live.

---

## Latent style axes

Nine normalised axes are derived first, so that thirty-odd features move **together** instead
of independently. A heavy, tall, soft spec reads as van-like across silhouette, arches, wing
and exhaust at once, rather than as unrelated procedural detail.

All are bounded to `[0, 1]`, none is produced from noise, and each cites its inputs.

| Axis | Reads | Meaning |
|---|---|---|
| `mass01` | `massKg` over 500–8000 kg | how heavy |
| `size01` | `wheelbaseM` over 1.60–6.50 m | how big |
| `low01` | `cgHeightM` over 0.10–2.00 m, inverted | how planted |
| `grip01` | `max(tireMuLatFront, tireMuLatRear)` over 0.40–2.50 | how much grip |
| `balance01` | `tireMuLatFront − tireMuLatRear` over −0.20–0.45 | understeer/oversteer bias |
| `power01` | peak engine torque × `finalDriveRatio` ÷ `massKg` | tractive effort per kilogram |
| `aero01` | `dragCoefficient × frontalAreaM2` over 0.20–4.50 | aerodynamic bulk |
| `sport01` | 0.35·`grip01` + 0.30·`low01` + 0.35·`power01` | composite sportiness |
| `strip01` | `sport01` × (1 − mass-per-wheelbase over 200–1200 kg/m) | light for its size ⇒ racecar |

---

## Frames and units

Everything in `CarVisual` is **metres in the body frame**: `+X` forward (nose), `+Y` left,
origin at the **centre of mass**. This is the `src/core/units.h` convention.
Consumers apply their own metres-to-pixels scale.

### The layout frame is not the body frame

The registry states mass particles and glass stations in the **layout frame**, whose origin is
the **axle midpoint** (`src/physics/vehicle.h`, `src/dev/dev_params.c`: *"layout frame (axle midpoint
origin)"*). `vehicle.c` derives the CG from those same particles, so the offset between the
two frames is exactly

```
xCg_layout = 0.5 * wheelbase − cgToFront
```

and a layout station sits at `xLayout − xCg_layout` in the body frame. `layout_to_body_x()` in
`car_visual.c` is the only conversion; reading a layout value as if it were a body value
shifts the whole greenhouse by the CG offset — 0.125 m on the stock car, and far more on a
rear-engined one.

---

## The fidelity budget, and why it constrains everything

| Source | Scale |
|---|---|
| Reference `Sedan`, 30 × 78 px for a ~4.7 m car | ~16.6 px/m |
| Reference `Compact`, 26 × 58 px for a ~3.7 m car | ~15.7 px/m |
| Drifty world scale: `PIXELS_PER_METER` 24 × `CAMERA_BASE_ZOOM` 0.55 | **13.2 px/m** |

Two consequences discipline the whole feature list:

1. **The visibility floor is ~7.6 cm ≈ 1 px.** Any feature whose geometric effect is smaller
   than that is invisible and must be either dropped or amplified.
2. **The distinctness threshold has a physical meaning.** `L∞ ≥ 0.08 m` is *exactly one
   visible pixel at the world scale*, which is why that is the number in the test rather than
   an arbitrary constant.

---

## Taxonomy: every feature is one of two kinds

`car_visual.c` labels each rule inline.

- **`[identity]`** — the drawn value **is** the physical value. Wheel centres, wheelbase, tire
  diameter, body width. These may never be fudged: the `car-visual` scenario asserts the drawn
  wheel centres equal `vehicle.c`'s `set_wheel_positions()` exactly, and the drawn tire
  diameter equals `2 × wheelRadius[Front|Rear]M`.
- **`[rule]`** — a documented, deterministic styling mapping from named parameters. Not a law
  of physics, but it cites its inputs and is stable. Where the physical input is not uniquely
  invertible, the rule says so.

### Feature mappings

| Drawn feature | Reads | Kind |
|---|---|---|
| Overall length | `cgToFrontM`, `cgToRearM`, `frontOverhangM`, `rearOverhangM` | identity |
| Overall width | `bodyHalfWidthM` (from `widthOverallM`) | identity |
| Wheelbase, axle span | `wheelbaseM` | identity |
| Wheel centres | `cgToFrontM`, `cgToRearM`, `trackWidth[Front\|Rear]M` | identity |
| Tire diameter | `wheelRadius[Front\|Rear]M` (from section × aspect × rim) | identity |
| Tire width | `tireSectionWidth[Front\|Rear]Mm` | identity |
| Rim diameter, rim width | `tireRimDiameter[Front\|Rear]In`, `tireRimWidth[Front\|Rear]In` | identity |
| Visible sidewall | `(tire diameter − rim diameter) / 2`, i.e. section × aspect | identity |
| Wheel poke | `trackWidth[Front\|Rear]M`, tire width, `widthOverallM` | identity |
| Arch gap | `rideHeight[Front\|Rear]M` + 0.35 × `suspTravel[Front\|Rear]M` | identity |
| Arch flare | track vs `bodyHalfWidthM`, mean tire width, arch gap, `openWheelWeight` | rule |
| Brake disc | `brakeDiscRadius[Front\|Rear]M`, scaled by `maxBrakeTorqueNm` | rule |
| Spoke count | `wheelInertiaKgM2` | rule |
| Greenhouse band | `cowlXM`, `backlightXM`, `massDriverXM` (layout frame), plus the 9%-of-body-length forward package bias | rule |
| Cabin width | `heightOverallM` via `heightVisual` × `bodyHalfWidthM` | rule |
| Roof panel length | `cabinLengthM`, `heightVisual` | rule |
| Glass band width | `cabinHalfWidthM`, `heightVisual` | rule |
| Nose taper | `noseWidthM`, `shoulderXM`, `widthOverallM`; gradual nose exponent plus anchored sub-pixel facing correction | identity/rule |
| Tail taper | `tailWidthM`, `shoulderXM`, `widthOverallM`; abrupt Kamm-tail exponent plus anchored sub-pixel facing correction | identity/rule |
| Waist pinch | `sport01` | rule |
| Wing span / chord | `aeroLiftCoefRear` (signed), `aeroRefAreaRearM2` | rule |
| Splitter | front downforce demand: `aeroLiftCoefFront`, `aeroRefAreaFrontM2` | rule |
| Canards | front downforce demand | rule |
| Exhaust count | `engineCylinders` | rule |
| Exhaust bore | `engineDisplacementL` | rule |
| Hood bulge | `engineDisplacementL` + `engineCylinders` (engine *bulk*), `massEngineXM` | rule |
| Pickup bed length | `bedLengthM`, clamped to the space behind the greenhouse | identity |
| Pickup bed weight | `bedLengthM` (expression strength for the drawn width) | rule |
| Van/bus windows, segment count | `heightVisual` × greenhouse span | rule |
| Open-wheel weight | `trackWidth[Front\|Rear]M` vs `widthOverallM` | rule |
| Race details (cage, mirrors, tow hook, hood pins) | mass-per-length, `grip01`, downforce, `strip01`; marker diameters are presentation-gained and tow-hook X clears the heading base | rule |
| Heading marker geometry | overall body length and width, with a four-pixel presentation floor | rule |
| Stripes | `strip01` | rule |
| Wheel static angle | `suspToe[Front\|Rear]Rad`, presentation-gained | rule |
| Camber stance | `suspCamber[Front\|Rear]Rad`, presentation-gained | rule |

### Aerodynamic sign convention

A lift coefficient is **up-positive**, so downforce is the negative side of it.

- **Wings, splitters and canards are downforce devices.** They read `max(0, −Cl) × refArea`
  and are simply absent on a spec whose bodywork lifts. The rear device ramps continuously
  from an ordinary deck lip through to a full wing.
- **Body taper reads the SIGNED coefficient**, because a lifting tail and a downforce tail are
  different *shapes*, not different amounts of the same shape.

Both mappings are normalised against the registry envelope (`aero.lift_rear` ∈ [−3, 1],
`aero.ref_area_rear` ∈ [0.05, 2.0], so rear downforce demand spans [0, 6]) so that a mapping
never saturates part-way through a sweep of its own key.

### Body type is a region of parameter space, not an enum

There is **no `body.type` key and no per-archetype branch.** The reference forms emerge:

| Form | Emerges when |
|---|---|
| Pickup | a short cabin plus a non-zero `body.bed_length` — see the note below |
| Van / bus | a tall body whose greenhouse covers most of its length ⇒ a repeated side-window band |
| Limousine | long wheelbase + long greenhouse + modest width |
| Supercar | low `heightOverallM`, wide, strong rear downforce ⇒ wing, splitter, canards |
| Kei / compact | short wheelbase, narrow, tall-ish, small tires |
| Race car | light for its size ⇒ cage, mirrors deleted, stripes, tow hook |
| Open-wheel | `trackWidth` ≫ `widthOverallM` ⇒ wheels drawn clear of a narrowed body |

There is still no `body.type` enum: `body.bed_length` is a continuous dimension in metres, not
a category, and nothing branches on it. But it marks where inference has to stop.

**When a form must be declared instead of inferred.** The bed used to emerge from
`(backlightXM − tailX) / lengthM`, on the theory that a long rear deck reads as an open bed.
It does not: on every three-box car the boot *is* the body behind the rear glass, so the two
are the same measurement. The rule fired on **78 of 100** corpus vehicles, 74 of them not
trucks, and covered 47% of the muscle car's bodywork. No threshold separates them, because
there is nothing in the geometry to separate. When two forms are genuinely indistinguishable
from the parameters in hand, the answer is a new parameter, not a better guess.

### Transition bands

Every discrete feature ramps across a `smoothstep` band. A 0.001 change in a coefficient never
snaps a full-size wing, splitter, bed or exhaust into existence. Presence stays discrete for
signature separation; magnitude fades in.

---

## Presentation gains

Several real quantities are far below the one-pixel visibility floor. Each gets a **named,
documented, render-only** amplification. No solver reads any of them.

| Constant | Value | Raw effect | Why |
|---|---:|---|---|
| `CV_TOE_VISUAL_GAIN` | 8.0 | static toe ~0.15° ⇒ ~0.01 px of tire-edge arc | ungained, static toe is invisible; the gained pair creates ~1 px of toe-in/out divergence that reads as a steering-geometry cue |
| `CV_CAMBER_VISUAL_GAIN` | 4.0 | camber ~1.5° ⇒ cos reduction of 0.99966, ~0.001 px | ungained, camber has no drawn consequence at all; gained, it narrows the drawn footprint enough to read as stance |
| `CV_REST_ANGLE_GAIN` | 6.0 | legacy Ackermann-derived rest angle | kept for consistency until every preset has migrated to the per-axle `suspToe` primaries |
| `CV_EXHAUST_VISUAL_GAIN` | 4.0 | tip bore 40–120 mm ⇒ 0.5–1.6 px | ungained, exhaust tips rasterize to *nothing*; gained, the 4.9 px fleet mean carries one pipe against four |
| `CV_TOW_HOOK_DIAMETER_M` | 0.20 m | a 50 mm safety marker quantized to one pixel | a 2.6 px diameter plus a station clear of L9 produces a 5.0 px mean cluster |
| `CV_HOOD_PIN_DIAMETER_M` | 0.13 m | a 40 mm fastener quantized to one or two pixels per pair | the gained pair averages 4.1 px and remains subordinate to the hood |
| `headingLengthM`, `headingHalfWidthM` | 0.22–0.30 m, 0.12–0.18 m | the old fixed triangle averaged 3.0 px | body-scaled dimensions with floors average 6.4 px, preserving the gameplay cue |
| `steerVisualGain` (`render_vehicle.c`) | 1.25 | — | pre-existing; makes steer unmistakable at top-down scale. The physics angle is untouched |

`CV_MIN_CABIN_M` (0.35 m) is not a gain but a floor: a windscreen and a rear glass need
somewhere to sit even when the two declared stations coincide, and a zero-length cabin would
make the roof, glass and side-window layers degenerate.

`CV_CABIN_FORWARD_BIAS` shifts the station-controlled greenhouse forward by 9% of overall
body length before driver containment and hull clamping. It preserves every `cowl_x` and
`backlight_x` delta while preventing a symmetric bathtub read; it is a package-presentation
rule, not a solver dimension.

The two `CV_HULL_FACING_*` coefficients add a longitudinally varying cubic below one world
pixel. The correction is exactly zero at the nose, tail and declared shoulder, so the three
identity anchors remain exact while intermediate station pairs cannot become mirror-symmetric
as shoulder position or overhang changes.

---

## The colour-hash exception

**`car_visual.c` may contain no hash of raw spec bytes for geometry.** Byte-hashing would
trivially guarantee distinctness while destroying the property the distinctness test exists to
protect. Every geometric feature cites the parameters it reads.

**Colour is the single stated exception.** It is explicitly arbitrary but stable per car, so
captures stay deterministic, and it is **excluded from the distinctness metric** so that shape
has to carry the result.

`car_visual_colour_seed()` is FNV-1a over an **explicitly listed set of spec fields**, never
over the raw struct bytes — those include padding, which is unspecified and would make the
colour depend on the compiler.

Livery stripe **colour** may also use the colour-seed bits. Stripe **geometry** — placement,
span, pattern — is a deterministic function of `stripeWeight` and the body extents.

---

## The bake key: canonical serialization

`car_visual_bake_key()` is the texture cache key. `src/render/render.c` rebakes its GPU textures when
it changes and does nothing when it does not.

Three rules:

- **Over `CarVisual`, not `VehicleSpec`.** `CarVisual` is exactly what the rasterizer reads, so
  a key over it cannot miss a driver: if the picture would change, the key changes. A key over
  the spec would have to re-list which fields the grammar happens to consume, and would go
  stale the first time a rule was rewired.
- **Field by field, never raw bytes and never `memcmp`.** Both structs contain padding whose
  contents are unspecified; a key built from them could differ between two identical cars or
  between two compilers and rebake for nothing. Floats are hashed through their exact bit
  pattern, with `−0.0` normalised to `+0.0` so the two zeroes cannot disagree about one
  picture.
- **A hash may invalidate a cache and may seed colour. It may never produce geometry.**

Adding a field to `CarVisual` means adding it to the key. The `car-visual` scenario asserts
every designated visual driver moves the key, and that no two corpus vehicles collide onto
one key — which is what catches the omission.

---

## The raster

### Layer stack

Explicit, fixed, and **the in-file draw order is the stack, in order**. No caller may reorder
features, and no feature may be hoisted out of its layer.

| Layer | Contents |
|---|---|
| L0 | shadow / ground contact |
| L1 | body silhouette + outline |
| L2 | body secondary shading |
| L3 | greenhouse roof panel |
| L4 | windscreen, side glass, rear glass (+ cage) |
| L5 | head, tail, brake and reverse lights |
| L6 | **wheels**: arch brow, then tread / sidewall / rim / disc |
| L7 | appendages: splitter, wing, canards, mirrors, exhaust, bed rails, hood bulge, tow hook, hood pins |
| L8 | livery: stripes, panels |
| L9 | heading marker |

**L6 is drawn over L1, not under it.** That is what the layer table says and what the
reference sprites show — the tires read as dark blocks at the four corners, overlapping the
bodywork. Drawing them underneath hid the tire geometry almost completely and made every
tire-derived feature invisible to the pixel metric.

L0 shadow pixels stay `CAR_LABEL_EMPTY` (their alpha is below the label threshold), so a
shadow can never inflate the distinctness ratio.

**The heading marker stays.** The gold nose wedge is a gameplay affordance for top-down
heading legibility, not decoration. The nose *shape* is derived; the marker sits on top.

The L7 tow hook is derived behind the L9 triangle's base. This preserves the fixed layer order
without letting the always-on heading affordance overwrite the hook's entire pixel cluster.

### Composable parts

| Part | Contents |
|---|---|
| `CAR_RASTER_PART_ALL` | the whole car, wheels in place at their static angles |
| `CAR_RASTER_PART_BODY` | everything except the tires — hull, glass, **arches**, appendages |
| `CAR_RASTER_PART_WHEEL` | one wheel, about its hub, unrotated |

The split is at the joint that actually moves. **Wheel arches are bodywork and stay with the
body** — a fender does not steer — while the tire, sidewall, rim and disc travel with the
wheel. Composited body-then-wheels at zero steer, the result is the same picture `ALL` draws,
in the same order; the `car-visual` scenario asserts this pixel for pixel.

### Pivots and bounds

| Part | Pivot | Bounds |
|---|---|---|
| ALL / BODY | body-space (0,0) — the CG — at `(originXPx, originYPx)`; rotate about it by heading | everything drawn: hull, wheels, wing, splitter, mirrors, canards, one pixel of outline, and the shadow offset |
| WHEEL | the wheel's own hub, at the centre of its buffer | the largest ring drawn, in both axes |

The wheel sprite is **axis-aligned**: the derived static toe/camber angle is *not* baked in.
The caller rotates by `heading + steer + staticAngleRad`. Baking it in would make it
impossible to steer the wheel afterwards without double-counting it.

`DrawTexturePro` measures its `origin` in *destination* units, so the pivot scales with the
sprite. Getting that wrong shifts the car off its own centre of mass as the scale changes —
the kind of bug a still screenshot hides.

### Metres to pixels

`to_px()` maps `+X` forward to increasing pixel X and `+Y` left to *decreasing* pixel Y — the
same mapping as `src/core/units.h`. A consumer wanting the nose-up orientation of the reference
sheets rotates the finished buffer by an exact 90°, which is an index permutation and
therefore lossless.

Fills are **hard-edged**: a pixel centre is inside a shape or it is not. No anti-aliasing,
matching `resources/sprite_examples/`. Sub-pixel features are therefore genuinely invisible,
which is why the grammar amplifies the ones that matter rather than relying on the rasterizer
to hint them.

### The scale chain

`src/core/config.h` reconciles the five numbers end to end. Summary:

| | | |
|---|---|---|
| `PIXELS_PER_METER` | 24 | the render layer's world scale |
| `CAMERA_BASE_ZOOM` | 0.55 | the `Camera2D` zoom at rest |
| world scale | **13.2 px/m** | what the world is rasterized at inside the low-resolution target |
| sprite bake scale | **13.2 px/m** | the same number, so a vehicle texel is exactly one target pixel at rest and is never resampled |
| `PIXEL_ART_UPSCALE` | 2 | integer, nearest-neighbour, applied once: 640×360 → 1280×720 |

13.2 px/m is not arbitrary — it is what the grammar is calibrated to. The HUD, raygui and the
Physics Lab draw *after* the upscale at native resolution.

A `Camera2D` translates by `(offset − target × zoom)`; left fractional, the pixel grid crawls
as the camera follows the car. `units_snap_camera_offset_axis()` snaps it to whole target
pixels, and the `units` scenario asserts the property.

---

## Distinctness

Three assertions, because "any two different specs look different" is unachievable — two specs
differing by 1 N·m of peak torque cannot be required to look different.

1. **Purity and totality** — the same spec gives a bit-identical `CarVisual`, signature and
   raster; every registry range corner and every corpus car yields finite, non-negative,
   in-range output.
2. **Sensitivity** — twenty-one designated visual-driver keys, each perturbed across its whole
   declared range from stock. Both metrics must move. Catches a deleted or dead rule.
3. **Corpus distinctness** — all pairs among the 100 exceed the thresholds.

### The two metrics

- **Pixel difference — the real test.** Both cars rasterized at a canonical pose and a shared
  scale, compared on a **feature-label map**: one byte of feature identity per pixel, not
  colour. Colour here is arbitrary, so any colour-based metric would let two identically
  shaped cars in different paint "differ". A label map is colour-blind by construction. The
  requirement is **≥ 3% of the union silhouette differing**.
- **Feature vector — the diagnostic.** ~60 components normalised to approximate visible
  metres, asserted at **L2 ≥ 0.25** and **L∞ ≥ 0.08 m** (one visible pixel). Its job is the
  *error message*: it names which feature is too similar, which a pixel count cannot.

The signature is computed **from `CarVisual`, not re-derived from the spec** — otherwise the
test would verify a copy of the grammar rather than the grammar itself.

### Thresholds, and where they differ

| Threshold | Value | What it asks |
|---|---:|---|
| `CV_MIN_PIXEL_DIFF` | 0.030 | separates two **different vehicles** in a hundred-car fleet |
| `CV_MIN_L2` | 0.25 | the pair is not a rounding error apart across the whole vector |
| `CV_MIN_LINF` | 0.080 m | one visible pixel, in a **named** feature |
| `CV_MIN_SENSITIVITY_DIFF` | 0.015 | is this key wired to the picture **at all**? |

The sensitivity floor is deliberately lower, and the two ask different questions. A mis-wired
or deleted rule scores `0.0000`, not `0.0200`, so that floor only has to clear quantisation
noise — and the companion `CV_MIN_LINF` assertion additionally demands the change reach a
named feature by at least one screen pixel, which a pixel count alone cannot say. The number
matters because the fidelity budget is binding: tire aspect ratio swung across its whole
25–80% range repaints ~2.5% of the car — a 45% change in tire diameter, unmistakable on the
contact sheet — yet would fail a 3% bar simply because two of four wheels are a small share of
a car seen from above. Failing that would be the test lying, not the grammar.

### Failure artifacts

A distinctness failure writes `artifacts/car_visual_failures/<idA>__<idB>/`:

```
car_a.png  car_b.png   both cars, nose-up, at the comparison scale
diff.png               per-pixel label agreement, disagreements in red
car_a.txt  car_b.txt   both specs as tuning profiles, loadable in the Physics Lab
report.txt             ids, pixel ratio, union/differing counts, L2, L∞,
                       and the three largest signature-component gaps by name
```

Suppressed by `--no-bundle`, like every other failure bundle in the suite.

---

## Derived parameters

`vehicle_spec_refresh_derived()` is a **staged** recompute. The order is load-bearing because
later stages read earlier ones:

| Stage | Produces | From |
|---|---|---|
| 1 — dimensions | `lengthOverallM`, `bodyHalfWidthM`, `frontalAreaM2` | wheelbase, overhangs, `widthOverallM`, `heightOverallM` |
| 2 — mass particles | `massKg`, `cgToFrontM`, `cgToRearM`, `cgHeightM`, `yawInertiaKgM2` | the five `{kg, x, z}` particles + `wheelbaseM` (stage 1's frame) |
| 3 — tires | `wheelRadius[Front\|Rear]M`, `tireLoadRefPerWheelN` | section width, aspect, rim diameter, and stage 2's `massKg` |

Two inversions from the pre-Phase-2 model deserve care:

- **`body.wheelbase` is primary**; `cg_to_front` / `cg_to_rear` derive from it plus the mass
  particles. More physical — a car *has* a wheelbase, and its CG falls where the masses put it.
- **`wheelRadiusM` is per axle.** Staggered setups are normal and the visual difference is
  large.

`DevParameter.derived` marks a read-only readout: `dev_param_set` refuses it, the Physics Lab
draws a value instead of a slider, and a profile naming one is migrated onto the primaries
that produce it. `DevParameter.tier` (essential / advanced / expert) keeps the Lab legible now
that the registry describes the whole vehicle.

### Profile compatibility

**A vehicle is a tuning profile.** Since appearance is derived there is nothing else to store:
the existing `# drifty tuning profile v1` format, `dev_params_save` / `dev_params_load`, and
`fuzz/fuzz_profile.c` all work unchanged. **No new file format and no new parser.**

A profile written against the pre-Phase-2 keys still produces the car it names, because
`dev_params_apply_assignments()` migrates each legacy derived key onto the primaries it used
to stand for:

| Legacy key | Migrated by |
|---|---|
| `body.mass` | scaling all five particle masses to the requested total |
| `body.cg_to_front` / `body.cg_to_rear` | shifting every particle X so the CG lands where asked (collected together, so the pair migrates as one) |
| `body.cg_height` | shifting every particle Z likewise |
| `wheel.radius` | solving rim diameter from the requested loaded radius, keeping section and aspect |

Unknown keys are counted and skipped rather than treated as errors, so a profile written by a
newer build still loads. Parsing is deliberately defensive — it is a fuzz target.

---

## Commands

```bash
./build.sh --tests && ./build/tests/drifty_tests.exe --scenario car-visual   # grammar: purity, sensitivity, monotonicity, scale
./build/tests/drifty_tests.exe --scenario corpus                              # corpus: validity, round-trip, all-pairs distinctness

./build/tests/drifty_tests.exe --generate-corpus data/vehicles/corpus                # export the fleet as tuning profiles
./build/tests/drifty_tests.exe --dump-corpus-index docs/generated/CORPUS.md             # the corpus table
./build/tests/drifty_tests.exe --dump-corpus-sheet artifacts/gallery          # headless PNG contact sheet + index.html
```

`artifacts/gallery/index.html` is the primary human acceptance check, and **it works without a
GPU, a window, or `build/dev/drifty.exe`**. Compare it side by side against
`resources/sprite_examples/bk_cars1.a.png` to judge whether the range is being hit.

```bash
mk gallery      # the in-game gallery, every page, through the production texture path
mk visual-test  # whole-scene regression against tests/visual/baseline
```

`build/dev/drifty.exe --gallery-page N` is bounded and exits on its own, like every capture here. The
gallery is a **human-review artifact, deliberately not a GPU regression baseline**: a hundred
cars behind an RMSE gate, on hardware that rasterizes differently per vendor, is a maintenance
sinkhole with no CI value. The headless sheet and the `corpus` scenario are the actual gates.

---

## Demonstration-only vehicles

The registry ranges deliberately reach the reference extremes, so some corpus vehicles are
expressible and recognisable but **drive badly**. That is acceptable for a gallery and is
labelled rather than hidden. Entries whose description ends in `VISUAL DEMO ONLY`:

| Id | Why |
|---|---|
| `archetype_10_open_wheel` | track far wider than the body; the collision geometry that follows is unstable |
| `archetype_14_bus` | extreme length and height; 8 t is already at the `tireLoadRefPerWheelN` ceiling |
| `archetype_16_box_truck` | tall, boxy and heavy well outside the handling model's comfortable range |

The `sweep` group is a demonstration too, by construction: each row holds one key at values
across its whole declared range, including both ends, which is a parameter study rather than a
set of cars anyone would drive. The `sampled` group is quasi-random over a documented box and
makes no drivability claim at all.

---

## Rules a change must not break

- No geometric feature may be generated from a hash of raw spec data. Colour is the single
  stated exception.
- No `body.type` enum, no per-archetype drawing branch, no per-car art asset.
- No styling decision outside `src/render/car_visual.c`.
- `CarVisual` stays a stack local. Never in `Game`, never in `DevState`.
- GPU textures are released in `game_pre_reload` and re-acquired after, the `audio.c` pattern.
- Float determinism holds only **within one binary** (`game.dll` at `-O0`, `drifty_tests` at
  `-O2`). Never compare a module-computed raster against a test-computed one, and do not add
  `-ffast-math`.
- Reference sprites in `resources/sprite_examples/` are references for *how appearance maps
  from proportions*. They are never shipped as assets.

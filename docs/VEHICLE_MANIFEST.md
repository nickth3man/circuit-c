# Vehicle Manifest (`circuit/vehicle` v1)

A vehicle manifest is the external, human-reviewable form of one immutable `VehicleDefinition`
plus its default `VehicleSetup` and the content metadata a roster needs (display name, class tags,
controller eligibility, provenance). It replaces the hard-coded C roster (`car_roster.c`) with
reviewed data, so a new car is a file, not a recompile.

This is the format defined by issue #29. The loader lives in
`src/content/vehicle_manifest.{h,c}`; the strict JSON reader it builds on lives in
`src/core/json.{h,c}`. The parameter ownership classification comes from the issue #12 audit
(see `docs/VEHICLE_PARAMETERS.md`).

## Why a new format

The built-in roster is C data compiled into the binary, and the existing `dev_params` key=value
profile format cannot represent arrays of structured values cleanly. Issue #29 asks for a
versioned, cross-platform representation that:

- carries a stable id, metadata, class tags, and provenance,
- applies physics and setup parameters through the audited owner classification,
- hashes identically regardless of where it was loaded, and
- is headless-usable (no raylib, no window).

A single strict JSON reader serves both this format and the track format (#34).

## Schema

```json
{
  "schema": "circuit/vehicle",
  "version": 1,
  "id": "rwd_grip",
  "displayName": "RWD Grip",
  "description": "balanced rear-wheel-drive road car",
  "contentVersion": 1,
  "appearanceId": "rwd_grip",
  "contentKind": "player-selectable",
  "classTags": ["rwd", "road"],
  "controllerEligibility": ["human", "ai"],
  "provenance": { "source": "roster", "author": "circuit-c" },
  "physics": { "body.wheelbase": 2.55, "tire.lat_front.mu": 1.35 },
  "setup":   { "brake.bias_front": 0.57, "drive.gear_count": 6 }
}
```

### Top-level fields

| Field            | Type     | Required | Notes |
|------------------|----------|----------|-------|
| `schema`         | string   | yes      | Must be `"circuit/vehicle"`. |
| `version`        | number   | yes      | Must be `1`. |
| `id`             | string   | yes      | Stable content id: `[a-z0-9]` then up to 62 of `[a-z0-9._-]`. |
| `displayName`    | string   | yes      | Human label (≤ 128 chars). |
| `description`    | string   | no       | Free text (≤ 384 chars). Absent ⇒ empty. |
| `contentVersion` | number   | yes      | Integer; stored into `VehicleDefinition::contentVersion`. |
| `appearanceId`   | string   | yes      | Links to the appearance sheet (≤ 128 chars). |
| `contentKind`    | string   | no       | One of the four kinds below. Absent ⇒ `player-selectable`. |
| `classTags`      | string[] | no       | Up to 8 tags, 32 chars each. |
| `controllerEligibility` | string[] | no | Subset of `["human", "ai"]`. Absent ⇒ both eligible. |
| `provenance`     | object   | no       | `source` and `author` strings (≤ 128 chars each). |
| `physics`        | object   | no       | Definition-owned registry keys. |
| `setup`          | object   | no       | Setup-owned registry keys. |

Unknown top-level keys are rejected.

### `contentKind`

What a manifest is allowed to be used for. One of:

| Kind                 | Meaning |
|----------------------|---------|
| `"visual-sample"`    | Appearance-corpus sample; never race content. |
| `"prototype"`        | In review; not validated. |
| `"validated"`        | Passed validation; not yet player-facing. |
| `"player-selectable"`| Validated and listed for players. |

The field is optional for back-compat — a manifest without it is `"player-selectable"`, the only
kind the roster knew before the appearance corpus was split from race content. An unknown value is
rejected. Only `"player-selectable"` manifests are visible to car selection (`car_roster_*`); the
other kinds load into the catalog (so appearance and review tooling can read them) but never
surface as race cars.

### `physics` and `setup`

Both are flat objects of `"<dotted.key>": <number>`. Every key must be a real entry in the
`dev_params` registry, and its **owner** decides which section it belongs to:

- A `physics` key must be **definition-owned**. It is written into `VehicleDefinition::spec`, so it
  becomes part of the immutable car and feeds `contentHash`.
- A `setup` key must be **setup-owned**. It is written into `VehicleSetup`, the authored baseline a
  driver dials away from.
- A **derived** key (computed from others, e.g. `body.length_overall`) is rejected outright.
- A **session-rules** key is rejected outright.

This mirrors the issue #12 audit exactly, so a manifest cannot move a definition constant into the
setup dials (or vice versa). `drive.gear_count` is an integer special case: it is validated as a
whole number in `[1, MAX_GEARS]` before being stored.

An unspecified key keeps its value from `vehicle_spec_set_default()` / the default setup, so every
platform sees one documented value.

### `controllerEligibility`

Any combination of `"human"` and `"ai"`. An unknown value is rejected. When the field is absent the
vehicle is eligible for both controllers (the historical default).

## Determinism

Loading is a pure function of the text:

- Physics/setup keys apply to a spec that starts from `vehicle_spec_set_default()`.
- The **content hash** (`VehicleDefinition::contentHash`) is FNV-1a over the resulting spec fields,
  so two manifests that produce the same spec produce the same hash.
- The **manifest hash** (`VehicleManifest::manifestHash`) is `json_canonical_hash` over the whole
  document: object members hashed in sorted-key order, numbers by IEEE-754 double bits (`-0.0`
  normalized to `+0.0`), strings by decoded UTF-8 bytes. It is stable under formatting-only
  changes.
- A catalog is sorted by stable id after discovery, so the order files are enumerated cannot change
  the roster order or any compatibility checksum. Duplicate ids are a load error.

## Lifecycle and ownership

| Field class       | Owner                | Where it lives | Example |
|-------------------|----------------------|----------------|---------|
| Identity          | content author       | top-level      | `id`, `displayName` |
| Metadata          | content author       | top-level      | `classTags`, `provenance` |
| Physics constant  | definition (immutable) | `physics`     | `body.wheelbase` |
| Setup baseline    | setup (driver-tunable) | `setup`       | `brake.bias_front` |
| Derived           | computed, never authored | (rejected) | `body.length_overall` |

## Defaults and migration

A manifest with no `physics` or `setup` sections parses to the stock car
(`vehicle_definition_set_default`). Adding a car is additive: drop a `*.vehicle.json` file into the
catalog directory; no recompile, no roster edit.

## Authoring and verification

The `vehicle-manifest` scenario asserts, for one car of each drivetrain layout (RWD, FWD, AWD),
that writing then re-reading the roster spec reproduces its content hash exactly. It also covers:
unknown-key rejection, wrong-section key rejection (setup key in physics and vice versa), derived-
key rejection, out-of-range values, invalid ids, wrong schema/version, and duplicate-id detection
in a catalog. Discovery-order independence is checked by writing fixtures with out-of-order ids and
asserting the catalog comes back sorted.

## Classes (`circuit/vehicle-class` v1)

A class tag on a manifest (`"road"`, `"race"`) is a display string, not a contract: nothing stops
a typo from tagging a 1600 kg race car as "road". A class file is the reviewed, versioned
statement of what a class means numerically. The loader lives in
`src/content/vehicle_class.{h,c}` and uses the same strict JSON reader and conventions as the
vehicle manifest: unknown top-level keys are rejected, errors are `field: reason`, and a catalog
is id-sorted with duplicate ids rejected. Class files live in `data/vehicles/classes/`
(`*.vehicle-class.json`).

### Schema

```json
{
  "schema": "circuit/vehicle-class",
  "version": 1,
  "id": "road",
  "displayName": "Road",
  "description": "Street-oriented cars: modest mass, moderate torque, road tires, front- or rear-wheel drive.",
  "rules": {
    "mass_kg": [700, 1300],
    "peak_torque_nm": [50, 350],
    "max_tire_mu": 1.6,
    "layouts": ["rwd", "fwd"]
  }
}
```

### Fields

| Field         | Type     | Required | Notes |
|---------------|----------|----------|-------|
| `schema`      | string   | yes      | Must be `"circuit/vehicle-class"`. |
| `version`     | number   | yes      | Must be `1`. |
| `id`          | string   | yes      | Stable class id, same `[a-z0-9][a-z0-9._-]{0,62}` rule as a vehicle id. This id is what a manifest's `classTags` entry must match. |
| `displayName` | string   | yes      | Human label (≤ 128 chars). Display only — never part of eligibility. |
| `description` | string   | no       | Free text (≤ 384 chars). Display only — never part of eligibility. |
| `rules`       | object   | no       | Optional numeric bounds; see below. Absent ⇒ all rules unconstrained. |

### Rules (all optional — absent = unconstrained)

| Rule              | Shape      | Meaning |
|-------------------|------------|---------|
| `mass_kg`         | `[min, max]` | Inclusive bound on the manifest's total mass (`spec.massKg`, the sum of the authored mass particles). |
| `peak_torque_nm`  | `[min, max]` | Inclusive bound on the max of the engine torque curve (`engine.torque_p0..p6`). |
| `max_tire_mu`     | number      | Inclusive upper bound on `max(tire.lat_front.mu, tire.lat_rear.mu)`. |
| `layouts`         | string[]    | Whitelist of `"rwd"`/`"fwd"`/`"awd"` (up to 3, no duplicates). Absent or empty ⇒ any layout. |

Each bound pair must be two finite numbers with `min <= max`; unknown rule keys, unknown layout
names, and out-of-range rule values are load errors, not silent no-ops. Bounds are stored as
32-bit floats, so a number that is finite as JSON but too large to narrow (e.g. `1e100`) is also
a load error — narrowing it would record an infinity and quietly turn an authored bound into
"unconstrained".

### Eligibility

`vehicle_class_check_eligibility(class, manifest)` returns membership as a two-part check:

1. **Tag rule** — the manifest must carry the class `id` in its `classTags`. This is the primary
   membership signal a roster UI groups by.
2. **Numeric rules** — every rule the class constrains must be satisfied by the manifest's
   derived spec. A class with no `rules` object is a pure tag filter; a tag with no rules is
   still required for membership, so an untagged car can never be absorbed by overlapping
   numeric bounds.

Eligibility never consults display strings: `displayName` and `description` are human review
text and play no part in membership. The one-line evidence detail names the failed rule with
the observed value and bound (e.g. `mass 1400 kg outside [700, 1300] kg`), or quotes the checked
values on success (`tag=road mass=760kg torque=65Nm mu=0.95 layout=fwd eligible`).

### Where the rules are enforced

Class rules and the issue #31 promotion checklist are both applied where it matters — when
`car_roster` builds the live gameplay roster (`src/game/car_roster.c`). A manifest reaches the
roster only if it is `player-selectable`, passes every promotion check, and satisfies the numeric
rules of every class tag that names a class file actually present in `data/vehicles/classes/`.
Anything refused is kept out of gameplay and recorded, with its id and the failing rule, in the
rejection log read through `car_roster_rejection_count()` / `car_roster_rejection()`.

A class tag with no matching class file constrains nothing — it stays a grouping label. That is
deliberate: rejecting every car whose tag has no reviewed rule file would empty the roster the
moment a packaged build shipped without `classes/`. When a class file does exist, its rules are
binding, which is what turns the tag from a display string into a contract. The shipped roster is
held to both gates by the `roster-gate` scenario; the class-rule logic itself is covered
independently by `vehicle-class` using authored fixtures rather than roster cars, so a routine
physics content bump cannot flip a rule-logic test.

The menu applies a second, narrower gate at start (`game_can_start_race`) covering only what the
player can change from the menu: which car is selected and what the setup editor did to its
setup. A refused start keeps the player on the menu with the reason on screen.

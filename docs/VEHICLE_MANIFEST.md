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
| `classTags`      | string[] | no       | Up to 8 tags, 32 chars each. |
| `controllerEligibility` | string[] | no | Subset of `["human", "ai"]`. Absent ⇒ both eligible. |
| `provenance`     | object   | no       | `source` and `author` strings (≤ 128 chars each). |
| `physics`        | object   | no       | Definition-owned registry keys. |
| `setup`          | object   | no       | Setup-owned registry keys. |

Unknown top-level keys are rejected.

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

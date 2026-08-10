# Track Format (`circuit/track` v2)

A track file is the external, human-reviewable form of one immutable `TrackDefinition`: the
authored centreline, surface bands, ordered checkpoints, and identity metadata. It replaces the
hard-coded C layouts (`track_init`, `track_load_chicane`, `track_load_sprint`,
`track_load_technical`) with reviewed data, so a new circuit is a file, not a recompile.

This is the format defined by issues #34 and #37. The loader lives in `src/content/track_manifest.{h,c}`;
the strict JSON reader it builds on lives in `src/core/json.{h,c}`.

## Why a new format

The built-in layouts are C structs compiled into the binary. Issue #34 asks for a versioned,
cross-platform representation that:

- faithfully reproduces every current built-in track (round-trips the geometry hash), and
- hashes identically regardless of where it was loaded, so content-compatibility and replay/save
  checks are deterministic.

JSON was chosen over a bespoke text format because the reader is shared with the vehicle manifest
(issue #29), and because the strict reader already rejects every deviation that could make two
byte-different files load as the same value.

## Schema

```json
{
  "schema": "circuit/track",
  "version": 2,
  "id": "chicane",
  "displayName": "Chicane Validation Circuit",
  "description": "the lap every car is validated against",
  "contentVersion": "chicane_v1",
  "route": {
    "closed": true,
    "nodes": [
      { "x": -100, "y": 0, "halfWidth": 8, "runoffHalfWidth": 12, "surface": "asphalt" }
    ]
  },
  "surfaces": { "offTrack": "grass", "runoff": "grass" },
  "parkingLot": { "minX": -200, "maxX": 200, "minY": -150, "maxY": 150 },
  "checkpoints": [
    { "x": -60, "y": 0, "forwardX": 1, "forwardY": 0, "halfWidth": 10, "required": true }
  ],
  "sectors": [
    { "x": -60, "y": 0, "forwardX": 1, "forwardY": 0, "halfWidth": 10 }
  ],
  "startFinish": { "x": -60, "y": 0, "forwardX": 1, "forwardY": 0, "halfWidth": 10 },
  "grid": [
    { "x": -70, "y": -2, "heading": 0 },
    { "x": -70, "y": 2, "heading": 0 }
  ],
  "pit": {
    "entry": { "x": 50, "y": 90, "forwardX": -1, "forwardY": 0, "halfWidth": 6 },
    "exit": { "x": -50, "y": 0, "forwardX": 1, "forwardY": 0, "halfWidth": 6 },
    "speedLine": { "x": 0, "y": 90, "forwardX": -1, "forwardY": 0, "halfWidth": 6 },
    "serviceBoxes": [
      { "minX": -10, "minY": 95, "maxX": 10, "maxY": 105 }
    ]
  }
}
```

### Top-level fields

| Field            | Type     | Required | Notes |
|------------------|----------|----------|-------|
| `schema`         | string   | yes      | Must be `"circuit/track"`. |
| `version`        | number   | yes      | `1` or `2`. v1 requires `route.closed: true`; v2 allows `false` for open point-to-point routes. |
| `id`             | string   | yes      | Stable content id. Matches `[a-z0-9][a-z0-9._-]{0,30}` (≤31 chars, fits `TRACK_ID_CHARS`). |
| `displayName`    | string   | yes      | Human label (≤ 128 chars). |
| `description`    | string   | no       | Free text (≤ 256 chars). Absent ⇒ empty. |
| `contentVersion` | string   | yes      | Stored verbatim into `TrackDefinition::version`. |
| `route`          | object   | yes      | The centreline. |
| `surfaces`       | object   | no       | Off-track and runoff surface names. Absent ⇒ grass. |
| `parkingLot`     | object   | no       | Open-area bounds. Present iff `isParkingLot`. |
| `checkpoints`    | array    | no       | Ordered route-validation gates. Absent ⇒ derived from nodes. |
| `sectors`        | array    | no       | Ordered sector splits, independent of route validation. |
| `startFinish`    | object   | no       | Explicit start/finish line. Absent ⇒ gate 0 is the lap boundary. |
| `grid`           | array    | no       | Grid slots, deterministic poses. Validated for overlap and track placement. |
| `pit`            | object   | no       | Pit geometry: `entry`, `exit`, `speedLine` gates and `serviceBoxes` rectangles. Authoring pit geometry does not automatically enable pit rules. |

Unknown top-level keys are rejected. v1 files with new keys load (they are optional), but v1 `route.closed` must be `true`; v2 relaxes that for open sprint routes.

### `route`

| Field    | Type   | Required | Notes |
|----------|--------|----------|-------|
| `closed` | bool   | yes      | `true` = closed circuit (laps wrap), `false` = open point-to-point (sprint). v1 requires `true`. |
| `nodes`  | array  | yes      | Ordered centreline nodes (≥ 2). |
### node

| Field             | Type   | Notes |
|-------------------|--------|-------|
| `x`, `y`          | number | Centre point, world metres. |
| `halfWidth`       | number | Drivable half-width. Must be > 0. |
| `runoffHalfWidth` | number | Runoff band half-width. 0 ⇒ no runoff band. |
| `surface`         | string | One of `asphalt`, `gravel`, `grass`, `snow`. |

### `checkpoints[]`

Ordered route-validation gates. A gate scores when the car's motion crosses its line travelling the forward half-plane.

| Field       | Type   | Notes |
|-------------|--------|-------|
| `x`, `y`    | number | Gate centre, world metres. |
| `forwardX`, `forwardY` | number | Travel direction. Must be unit-length (√(fx²+fy²) ≈ 1), **or** the zero vector `(0, 0)` for a degenerate placeholder. Stored verbatim, never renormalized. |
| `halfWidth` | number | Gate half-width. |
| `required`  | bool   | Defaults to `true` when absent. |

The zero-vector exception exists for the parking lot: its perimeter closes with a node that
coincides with the first, so the derived gate faces no direction and
`track_build_checkpoints_from_nodes()` stores `(0, 0)`. A clearly non-zero, non-unit vector is
still rejected as a self-contradictory marker.

### `sectors[]`

Independent timing splits. Same geometry as a checkpoint but without `required`: crossing advances the sector counter without affecting lap validity.

| Field       | Type   | Notes |
|-------------|--------|-------|
| `x`, `y`    | number | Gate centre. |
| `forwardX`, `forwardY` | number | Must be unit-length. |
| `halfWidth` | number | Half-width, (0, 100]. |

### `startFinish`

Single explicit start/finish line. When present it is the authoritative lap boundary; otherwise gate 0 is used. Prevents lap validation and timing from being conflated.

| Field       | Type   | Notes |
|-------------|--------|-------|
| `x`, `y`    | number | Line centre. |
| `forwardX`, `forwardY` | number | Unit-length forward. |
| `halfWidth` | number | Half-width. |

### `grid[]`

Deterministic starting grid: each slot is a pose. Validated for overlap (≥ 3 m apart) and for being on the racing surface.

| Field     | Type   | Notes |
|-----------|--------|-------|
| `x`, `y`  | number | Slot position, world metres. |
| `heading` | number | Yaw radians CCW from +X. Also accepted as `headingRad`. |

### `pit`

Pit geometry is authored but does not automatically enable pit rules — `RaceSession` decides.

| Field          | Type   | Notes |
|----------------|--------|-------|
| `entry`        | object | Pit entry line: `x`, `y`, `forwardX`, `forwardY`, `halfWidth`. |
| `exit`         | object | Pit exit line, same shape. |
| `speedLine`    | object | Pit speed-limit line, same shape. Aliases: `speedline`, `speed`. |
| `serviceBoxes` | array  | Service rectangles: each `{ minX, minY, maxX, maxY }` with `max > min`. |

### `surfaces`

| Field      | Type   | Notes |
|------------|--------|-------|
| `offTrack` | string | Surface beyond the runoff band. |
| `runoff`   | string | Surface in the runoff band. |

### `parkingLot`

| Field  | Type   | Notes |
|--------|--------|-------|
| `minX`, `maxX`, `minY`, `maxY` | number | Open-area bounds, world metres. |
## Coordinate system (fixed)

World metres, +X east, +Y north. Headings are radians counter-clockwise from +X (the `atan2`
convention). Closed routes wind counter-clockwise, so the circuit interior is on the left of the
travel direction. This is documented here rather than carried as a per-file field: v1/v2 share one
coordinate system.

## Determinism

- Numbers parse to a `double` via `strtod` and are stored as `float`. The export serializes with
  `%.9g` (`FLT_DECIMAL_DIG`), so an authored float round-trips its bits exactly on the supported
  toolchains (mingw-w64 UCRT64 and glibc/clang libc, which provide correctly-rounded `strtod`).
- The **manifest hash** (`json_canonical_hash`, written to `*manifestHashOut`) is stable under
  formatting-only changes: object members are hashed in sorted-key order, numbers by their IEEE-754
  double bits (`-0.0` normalized to `+0.0`), strings by decoded UTF-8 bytes.
- The **geometry hash** (`track_geometry_hash`) is the runtime identity of a track: it covers every
  node, checkpoint, sector, start/finish, grid, pit and service-box geometry and is stable under any change that does not move that geometry. A loaded file is asserted to reproduce its compiled geometry hash exactly.

## Field mapping (TrackDefinition)

| `TrackDefinition` field | Manifest source | Derived at load | Runtime only |
|-------------------------|-----------------|-----------------|--------------|
| `nodes`, `count`        | `route.nodes`   |                 |              |
| `routeClosed`           | `route.closed`  |                 |              |
| `checkpoints`, `checkpointCount` | `checkpoints` | `track_build_checkpoints_from_nodes()` when absent | |
| `sectorMarkers`, `sectorMarkerCount` | `sectors` |            |              |
| `hasStartFinish`, `startFinish` | `startFinish` |              | |
| `gridSlots`, `gridSlotCount` | `grid`      |                 |              |
| `hasPitEntry`, `pitEntry` | `pit.entry`   |                 |              |
| `hasPitExit`, `pitExit` | `pit.exit`      |                 |              |
| `hasPitSpeedLine`, `pitSpeedLine` | `pit.speedLine` |       |              |
| `serviceBoxes`, `serviceBoxCount` | `pit.serviceBoxes` |      |              |
| `id`                    | `id`            |                 |              |
| `version`               | `contentVersion`|                 |              |
| `offTrackSurfaceId`     | `surfaces.offTrack` |            |              |
| `runoffSurfaceId`       | `surfaces.runoff` |              |              |
| `isParkingLot`          | `parkingLot` present |           |              |
| `lotMin/MaxXM`, `lotMin/MaxYM` | `parkingLot` |            |              |

Generate the committed examples from the compiled layouts:

```sh
./build/tests/circuit_tests.exe --generate-tracks data/tracks
```

The `track-format` scenario asserts, for each built-in layout, that writing then re-reading
reproduces the compiled geometry hash, and that the committed `data/tracks/*.track.json` files
still parse and match. A hand edit to a committed file is therefore caught rather than silently
shipping a wrong track.

## Discovery and runtime selection (issue #36)

Tracks are discovered by stable content id, not by a compiled enum. `track_catalog_load()` scans
`data/tracks/*.track.json`, parses each file, sorts the catalog by stable id, and rejects
duplicate ids. `track_load_by_id()` loads one track by building the canonical path
`data/tracks/<id>.track.json` and validating the id before touching the filesystem.

`GameRunConfig` carries a `trackId` string: empty means "keep the current track", otherwise it
must be a valid id and the corresponding file must parse. `RaceSession` records the same id
string, which is hashed into the rolling checksum as bytes rather than as an integer enum.
Command-line `--track` values are normalized (`lot` → `parking_lot`) and then resolved through
the catalog; unknown, missing, or uppercase ids are rejected with a diagnostic instead of being
silently ignored.

The four built-in tracks (`parking_lot`, `chicane`, `sprint`, `technical`) all load from
external files in headless and interactive builds. `sprint` and `technical` are explicit,
reviewable JSON files rather than runtime transforms of the chicane: the `track-migration`
scenario asserts that each catalog entry's geometry hash is bit-identical to the legacy
`track_load_*` output, and that node arrays match exactly.

The legacy loaders (`track_init`, `track_load_chicane`, `track_load_sprint`,
`track_load_technical`) remain in the codebase only for the temporary legacy-vs-loaded
comparisons and for `--generate-tracks`. No session code depends on them:
`game_configure_run()` resolves tracks through the catalog, and `game_init()` (interactive)
loads `parking_lot` through the catalog as well — a missing, corrupt, or duplicate catalog
leaves the session without a track rather than silently falling back to compiled geometry,
and filename/id mismatches (`foo.track.json` containing `id: bar`) are rejected at catalog
load and at `track_load_by_id()` time.

## Forward compatibility

v2 adds: open routes (`route.closed: false`), sector/start-finish/grid/pit geometry. Each is
optional and gated on the version field: v1 files must have `route.closed: true`, v2 relaxes that.
A future v3 will add environment/presentation hooks; the v2 loader will reject a v3 file outright so an
old binary never silently loads a newer file as something it is not.

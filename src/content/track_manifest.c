/*
 * track_manifest.c — versioned external track format loader/exporter. See the header.
 */
#include "content/track_manifest.h"

#include "core/json.h"

#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* A node half-width of 100 m is generous for any conceivable racing surface and keeps a stray
 * exponent from silently turning into a 1 km-wide ribbon. */
#define TRACK_MAX_HALF_WIDTH_M 100.0f

static const struct {
    SurfaceId id;
    const char *name;
} kSurfaces[] = {
    { SURFACE_ASPHALT, "asphalt" },
    { SURFACE_GRAVEL, "gravel" },
    { SURFACE_GRASS, "grass" },
    { SURFACE_SNOW, "snow" },
};

const char *track_manifest_surface_name(SurfaceId surface)
{
    for (size_t i = 0; i < sizeof(kSurfaces) / sizeof(kSurfaces[0]); i++) {
        if (kSurfaces[i].id == surface) return kSurfaces[i].name;
    }
    return NULL;
}

SurfaceId track_manifest_surface_id(const char *name)
{
    if (name == NULL) return (SurfaceId)-1;
    for (size_t i = 0; i < sizeof(kSurfaces) / sizeof(kSurfaces[0]); i++) {
        if (strcmp(kSurfaces[i].name, name) == 0) return kSurfaces[i].id;
    }
    return (SurfaceId)-1;
}

/* The keys accepted at the top level. New v2 keys (sectors, startFinish, grid, pit) are
 * optional and accepted for both v1 and v2 files; a v1 file simply has no reason to carry
 * them. This keeps the parser forward-compatible without making equivalent v1-vs-v2 content
 * load differently when the new keys are absent. */
typedef enum {
    TM_KEY_SCHEMA,
    TM_KEY_VERSION,
    TM_KEY_ID,
    TM_KEY_DISPLAY_NAME,
    TM_KEY_DESCRIPTION,
    TM_KEY_CONTENT_VERSION,
    TM_KEY_ROUTE,
    TM_KEY_SURFACES,
    TM_KEY_PARKING_LOT,
    TM_KEY_CHECKPOINTS,
    TM_KEY_SECTORS,
    TM_KEY_START_FINISH,
    TM_KEY_GRID,
    TM_KEY_PIT,
    TM_KEY_COUNT
} TrackTopKey;

static const char *const kTopKeyNames[TM_KEY_COUNT] = {
    [TM_KEY_SCHEMA] = "schema",
    [TM_KEY_VERSION] = "version",
    [TM_KEY_ID] = "id",
    [TM_KEY_DISPLAY_NAME] = "displayName",
    [TM_KEY_DESCRIPTION] = "description",
    [TM_KEY_CONTENT_VERSION] = "contentVersion",
    [TM_KEY_ROUTE] = "route",
    [TM_KEY_SURFACES] = "surfaces",
    [TM_KEY_PARKING_LOT] = "parkingLot",
    [TM_KEY_CHECKPOINTS] = "checkpoints",
    [TM_KEY_SECTORS] = "sectors",
    [TM_KEY_START_FINISH] = "startFinish",
    [TM_KEY_GRID] = "grid",
    [TM_KEY_PIT] = "pit",
};

static bool is_known_top_key(const char *name)
{
    for (int i = 0; i < TM_KEY_COUNT; i++) {
        if (strcmp(name, kTopKeyNames[i]) == 0) return true;
    }
    return false;
}

static bool track_id_is_valid(const char *id)
{
    if (id == NULL || id[0] == '\0') return false;
    const char c0 = id[0];
    if (!((c0 >= 'a' && c0 <= 'z') || (c0 >= '0' && c0 <= '9'))) return false;
    size_t len = 1;
    for (size_t i = 1; id[i] != '\0'; i++) {
        const char c = id[i];
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' || c == '_' ||
              c == '-'))
            return false;
        len++;
        if (len >= TRACK_ID_CHARS) return false;
    }
    return true;
}

static void set_error(char *error, size_t errorCap, const char *field, const char *reason)
{
    if (error == NULL || errorCap == 0) return;
    if (field != NULL) {
        snprintf(error, errorCap, "%s: %s", field, reason);
    } else {
        snprintf(error, errorCap, "%s", reason);
    }
}

static bool finite_number(const JsonValue *value, float *out, const char *key, char *error,
                          size_t errorCap)
{
    if (value == NULL || !json_is_number(value)) {
        set_error(error, errorCap, key, "expected a finite number");
        return false;
    }
    const double d = json_as_number(value);
    const float f = (float)d;
    if (!isfinite(d) || !isfinite(f)) {
        set_error(error, errorCap, key, "expected a finite number");
        return false;
    }
    *out = f;
    return true;
}

/* Parse one route node. runoffHalfWidth and surface default, matching the struct's existing
 * "leave them off and you get the pre-runoff behaviour" contract. */
static bool parse_node(const JsonValue *node, TrackNode *out, int index, char *error,
                       size_t errorCap)
{
    if (node == NULL || !json_is_object(node)) {
        set_error(error, errorCap, "route.nodes", "each node must be an object");
        return false;
    }
    char field[48];
    float x = 0.0f, y = 0.0f, halfWidth = 0.0f;
    snprintf(field, sizeof(field), "route.nodes[%d].x", index);
    if (!finite_number(json_object_get(node, "x"), &x, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "route.nodes[%d].y", index);
    if (!finite_number(json_object_get(node, "y"), &y, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "route.nodes[%d].halfWidth", index);
    if (!finite_number(json_object_get(node, "halfWidth"), &halfWidth, field, error, errorCap))
        return false;
    if (!(halfWidth > 0.0f && halfWidth <= TRACK_MAX_HALF_WIDTH_M)) {
        snprintf(field, sizeof(field), "route.nodes[%d].halfWidth", index);
        set_error(error, errorCap, field, "halfWidth must be in (0, 100] metres");
        return false;
    }

    float runoff = 0.0f;
    const JsonValue *runoffVal = json_object_get(node, "runoffHalfWidth");
    if (runoffVal != NULL) {
        snprintf(field, sizeof(field), "route.nodes[%d].runoffHalfWidth", index);
        if (!finite_number(runoffVal, &runoff, field, error, errorCap)) return false;
        if (runoff < 0.0f || runoff > TRACK_MAX_HALF_WIDTH_M) {
            snprintf(field, sizeof(field), "route.nodes[%d].runoffHalfWidth", index);
            set_error(error, errorCap, field, "runoffHalfWidth must be in [0, 100] metres");
            return false;
        }
    }

    SurfaceId surface = SURFACE_ASPHALT;
    const JsonValue *surfaceVal = json_object_get(node, "surface");
    if (surfaceVal != NULL) {
        const char *name = json_as_string(surfaceVal);
        const SurfaceId parsed = track_manifest_surface_id(name);
        if (parsed == (SurfaceId)-1) {
            snprintf(field, sizeof(field), "route.nodes[%d].surface", index);
            set_error(error, errorCap, field, "unknown surface (asphalt/gravel/grass/snow)");
            return false;
        }
        surface = parsed;
    }

    out->centerM = (Vector2){ x, y };
    out->halfWidthM = halfWidth;
    out->surfaceId = surface;
    out->runoffHalfWidthM = runoff;
    return true;
}

static bool parse_route(const JsonValue *route, TrackDefinition *out, char *error,
                        size_t errorCap)
{
    if (route == NULL || !json_is_object(route)) {
        set_error(error, errorCap, "route", "must be an object");
        return false;
    }
    const JsonValue *closed = json_object_get(route, "closed");
    if (closed == NULL || !json_is_bool(closed)) {
        set_error(error, errorCap, "route.closed", "must be a boolean");
        return false;
    }
    out->routeClosed = json_as_bool(closed);
    const JsonValue *nodes = json_object_get(route, "nodes");
    if (nodes == NULL || !json_is_array(nodes) || json_array_count(nodes) < 2) {
        set_error(error, errorCap, "route.nodes", "must be an array of at least two nodes");
        return false;
    }
    const int count = json_array_count(nodes);
    TrackNode *nodeArr = (TrackNode *)calloc((size_t)count, sizeof(TrackNode));
    if (nodeArr == NULL) {
        set_error(error, errorCap, NULL, "out of memory");
        return false;
    }
    for (int i = 0; i < count; i++) {
        if (!parse_node(json_array_at(nodes, i), &nodeArr[i], i, error, errorCap)) {
            free(nodeArr);
            return false;
        }
    }
    /* Validate route segment continuity: consecutive nodes must not coincide.
     * Closed routes validate the wrap-around closing segment; open routes do not. */
    const bool closesByDuplicateNode =
        (count > 1 && nodeArr[0].centerM.x == nodeArr[count - 1].centerM.x &&
         nodeArr[0].centerM.y == nodeArr[count - 1].centerM.y);
    int segmentsToCheck = 0;
    if (out->routeClosed) {
        segmentsToCheck = (!closesByDuplicateNode) ? count : (count - 1);
    } else {
        segmentsToCheck = count - 1;
    }

    for (int i = 0; i < segmentsToCheck; i++) {
        const int nextIdx = (i + 1) % count;
        const float dx = nodeArr[nextIdx].centerM.x - nodeArr[i].centerM.x;
        const float dy = nodeArr[nextIdx].centerM.y - nodeArr[i].centerM.y;
        if (dx * dx + dy * dy < 1.0e-8f) {
            char field[48];
            snprintf(field, sizeof(field), "route.nodes[%d]", nextIdx);
            set_error(error, errorCap, field,
                      "consecutive nodes coincide (segment length zero)");
            free(nodeArr);
            return false;
        }
    }
    out->nodes = nodeArr;
    out->count = count;
    return true;
}

static bool parse_surfaces(const JsonValue *surfaces, TrackDefinition *out, char *error,
                           size_t errorCap)
{
    /* Defaults match every current built-in track: grass off-track, grass runoff. */
    out->offTrackSurfaceId = SURFACE_GRASS;
    out->runoffSurfaceId = SURFACE_GRASS;
    if (surfaces == NULL) return true;
    if (!json_is_object(surfaces)) {
        set_error(error, errorCap, "surfaces", "must be an object");
        return false;
    }
    const JsonValue *offVal = json_object_get(surfaces, "offTrack");
    if (offVal != NULL) {
        if (!json_is_string(offVal)) {
            set_error(error, errorCap, "surfaces.offTrack", "expected a string");
            return false;
        }
        const char *offName = json_as_string(offVal);
        const SurfaceId parsed = track_manifest_surface_id(offName);
        if (parsed == (SurfaceId)-1) {
            set_error(error, errorCap, "surfaces.offTrack",
                      "unknown surface (asphalt/gravel/grass/snow)");
            return false;
        }
        out->offTrackSurfaceId = parsed;
    }
    const JsonValue *runoffVal = json_object_get(surfaces, "runoff");
    if (runoffVal != NULL) {
        if (!json_is_string(runoffVal)) {
            set_error(error, errorCap, "surfaces.runoff", "expected a string");
            return false;
        }
        const char *runoffName = json_as_string(runoffVal);
        const SurfaceId parsed = track_manifest_surface_id(runoffName);
        if (parsed == (SurfaceId)-1) {
            set_error(error, errorCap, "surfaces.runoff",
                      "unknown surface (asphalt/gravel/grass/snow)");
            return false;
        }
        out->runoffSurfaceId = parsed;
    }
    return true;
}

static bool parse_parking_lot(const JsonValue *parkingLot, TrackDefinition *out, char *error,
                              size_t errorCap)
{
    if (parkingLot == NULL) {
        out->isParkingLot = false;
        return true;
    }
    if (!json_is_object(parkingLot)) {
        set_error(error, errorCap, "parkingLot", "must be an object");
        return false;
    }
    float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
    if (!finite_number(json_object_get(parkingLot, "minX"), &minX, "parkingLot.minX", error,
                       errorCap) ||
        !finite_number(json_object_get(parkingLot, "maxX"), &maxX, "parkingLot.maxX", error,
                       errorCap) ||
        !finite_number(json_object_get(parkingLot, "minY"), &minY, "parkingLot.minY", error,
                       errorCap) ||
        !finite_number(json_object_get(parkingLot, "maxY"), &maxY, "parkingLot.maxY", error,
                       errorCap)) {
        return false;
    }
    if (!(maxX > minX) || !(maxY > minY)) {
        set_error(error, errorCap, "parkingLot",
                  "maxX must exceed minX and maxY must exceed minY");
        return false;
    }
    out->isParkingLot = true;
    out->lotMinXM = minX;
    out->lotMaxXM = maxX;
    out->lotMinYM = minY;
    out->lotMaxYM = maxY;
    return true;
}

static bool parse_checkpoint(const JsonValue *cp, Checkpoint *out, int index, char *error,
                             size_t errorCap)
{
    if (cp == NULL || !json_is_object(cp)) {
        set_error(error, errorCap, "checkpoints", "each checkpoint must be an object");
        return false;
    }
    char field[48];
    float x = 0.0f, y = 0.0f, fx = 0.0f, fy = 0.0f, halfWidth = 0.0f;
    snprintf(field, sizeof(field), "checkpoints[%d].x", index);
    if (!finite_number(json_object_get(cp, "x"), &x, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "checkpoints[%d].y", index);
    if (!finite_number(json_object_get(cp, "y"), &y, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "checkpoints[%d].forwardX", index);
    if (!finite_number(json_object_get(cp, "forwardX"), &fx, field, error, errorCap))
        return false;
    snprintf(field, sizeof(field), "checkpoints[%d].forwardY", index);
    if (!finite_number(json_object_get(cp, "forwardY"), &fy, field, error, errorCap))
        return false;

    /* A forward vector is a unit direction for a real gate, or a zero placeholder for a
     * degenerate one: the parking lot's perimeter closes with a node that coincides with the
     * first, so the derived gate faces no direction and track_build_checkpoints_from_nodes()
     * stores (0, 0). Anything clearly nonzero but not unit is a self-contradictory marker. The
     * components are stored verbatim (not renormalized) so a round-trip preserves the exact
     * geometry hash. */
    const float len = sqrtf(fx * fx + fy * fy);
    if (len > 1e-3f && fabsf(len - 1.0f) > 1e-3f) {
        snprintf(field, sizeof(field), "checkpoints[%d].forward", index);
        set_error(error, errorCap, field, "forward vector must be unit-length within 1e-3");
        return false;
    }
    snprintf(field, sizeof(field), "checkpoints[%d].halfWidth", index);
    if (!finite_number(json_object_get(cp, "halfWidth"), &halfWidth, field, error, errorCap))
        return false;
    if (!(halfWidth > 0.0f && halfWidth <= TRACK_MAX_HALF_WIDTH_M)) {
        snprintf(field, sizeof(field), "checkpoints[%d].halfWidth", index);
        set_error(error, errorCap, field, "halfWidth must be in (0, 100] metres");
        return false;
    }

    const JsonValue *required = json_object_get(cp, "required");
    if (required != NULL) {
        if (!json_is_bool(required)) {
            snprintf(field, sizeof(field), "checkpoints[%d].required", index);
            set_error(error, errorCap, field, "required must be a boolean");
            return false;
        }
        out->required = json_as_bool(required);
    } else {
        out->required = true;
    }
    out->centerM = (Vector2){ x, y };
    out->forwardUnit = (Vector2){ fx, fy };
    out->halfWidthM = halfWidth;
    return true;
}

static bool parse_sector_marker(const JsonValue *m, SectorMarker *out, int index, char *error,
                                size_t errorCap)
{
    if (m == NULL || !json_is_object(m)) {
        set_error(error, errorCap, "sectors", "each sector must be an object");
        return false;
    }
    char field[48];
    float x = 0.0f, y = 0.0f, fx = 0.0f, fy = 0.0f, halfWidth = 0.0f;
    snprintf(field, sizeof(field), "sectors[%d].x", index);
    if (!finite_number(json_object_get(m, "x"), &x, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "sectors[%d].y", index);
    if (!finite_number(json_object_get(m, "y"), &y, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "sectors[%d].forwardX", index);
    if (!finite_number(json_object_get(m, "forwardX"), &fx, field, error, errorCap))
        return false;
    snprintf(field, sizeof(field), "sectors[%d].forwardY", index);
    if (!finite_number(json_object_get(m, "forwardY"), &fy, field, error, errorCap))
        return false;
    const float len = sqrtf(fx * fx + fy * fy);
    if (len < 1e-3f || fabsf(len - 1.0f) > 1e-3f) {
        snprintf(field, sizeof(field), "sectors[%d].forward", index);
        set_error(error, errorCap, field, "forward vector must be unit-length within 1e-3");
        return false;
    }
    snprintf(field, sizeof(field), "sectors[%d].halfWidth", index);
    if (!finite_number(json_object_get(m, "halfWidth"), &halfWidth, field, error, errorCap))
        return false;
    if (!(halfWidth > 0.0f && halfWidth <= TRACK_MAX_HALF_WIDTH_M)) {
        snprintf(field, sizeof(field), "sectors[%d].halfWidth", index);
        set_error(error, errorCap, field, "halfWidth must be in (0, 100] metres");
        return false;
    }
    out->centerM = (Vector2){ x, y };
    out->forwardUnit = (Vector2){ fx, fy };
    out->halfWidthM = halfWidth;
    return true;
}

static bool parse_sectors(const JsonValue *sectors, TrackDefinition *out, char *error,
                          size_t errorCap)
{
    if (sectors == NULL) return true;
    if (!json_is_array(sectors)) {
        set_error(error, errorCap, "sectors", "must be an array");
        return false;
    }
    const int count = json_array_count(sectors);
    if (count == 0) return true;
    if (count > TRACK_MAX_SECTOR_MARKERS) {
        set_error(error, errorCap, "sectors", "too many sector markers");
        return false;
    }
    SectorMarker *arr = (SectorMarker *)calloc((size_t)count, sizeof(SectorMarker));
    if (arr == NULL) {
        set_error(error, errorCap, NULL, "out of memory");
        return false;
    }
    for (int i = 0; i < count; i++) {
        if (!parse_sector_marker(json_array_at(sectors, i), &arr[i], i, error, errorCap)) {
            free(arr);
            return false;
        }
    }
    out->sectorMarkers = arr;
    out->sectorMarkerCount = count;
    return true;
}

static bool parse_start_finish(const JsonValue *sf, TrackDefinition *out, char *error,
                               size_t errorCap)
{
    if (sf == NULL) return true;
    if (!json_is_object(sf)) {
        set_error(error, errorCap, "startFinish", "must be an object");
        return false;
    }
    float x = 0.0f, y = 0.0f, fx = 0.0f, fy = 0.0f, halfWidth = 0.0f;
    if (!finite_number(json_object_get(sf, "x"), &x, "startFinish.x", error, errorCap))
        return false;
    if (!finite_number(json_object_get(sf, "y"), &y, "startFinish.y", error, errorCap))
        return false;
    if (!finite_number(json_object_get(sf, "forwardX"), &fx, "startFinish.forwardX", error,
                       errorCap))
        return false;
    if (!finite_number(json_object_get(sf, "forwardY"), &fy, "startFinish.forwardY", error,
                       errorCap))
        return false;
    const float len = sqrtf(fx * fx + fy * fy);
    if (len < 1e-3f || fabsf(len - 1.0f) > 1e-3f) {
        set_error(error, errorCap, "startFinish.forward",
                  "forward vector must be unit-length within 1e-3");
        return false;
    }
    if (!finite_number(json_object_get(sf, "halfWidth"), &halfWidth, "startFinish.halfWidth",
                       error, errorCap))
        return false;
    if (!(halfWidth > 0.0f && halfWidth <= TRACK_MAX_HALF_WIDTH_M)) {
        set_error(error, errorCap, "startFinish.halfWidth",
                  "halfWidth must be in (0, 100] metres");
        return false;
    }
    out->hasStartFinish = true;
    out->startFinish.centerM = (Vector2){ x, y };
    out->startFinish.forwardUnit = (Vector2){ fx, fy };
    out->startFinish.halfWidthM = halfWidth;
    return true;
}

static bool parse_grid(const JsonValue *grid, TrackDefinition *out, char *error,
                       size_t errorCap)
{
    if (grid == NULL) return true;
    if (!json_is_array(grid)) {
        set_error(error, errorCap, "grid", "must be an array");
        return false;
    }
    const int count = json_array_count(grid);
    if (count == 0) return true;
    if (count > TRACK_MAX_GRID_SLOTS) {
        set_error(error, errorCap, "grid", "too many grid slots");
        return false;
    }
    GridSlot *arr = (GridSlot *)calloc((size_t)count, sizeof(GridSlot));
    if (arr == NULL) {
        set_error(error, errorCap, NULL, "out of memory");
        return false;
    }
    for (int i = 0; i < count; i++) {
        const JsonValue *slot = json_array_at(grid, i);
        if (slot == NULL || !json_is_object(slot)) {
            set_error(error, errorCap, "grid", "each grid slot must be an object");
            free(arr);
            return false;
        }
        char field[48];
        float x = 0.0f, y = 0.0f, heading = 0.0f;
        snprintf(field, sizeof(field), "grid[%d].x", i);
        if (!finite_number(json_object_get(slot, "x"), &x, field, error, errorCap)) {
            free(arr);
            return false;
        }
        snprintf(field, sizeof(field), "grid[%d].y", i);
        if (!finite_number(json_object_get(slot, "y"), &y, field, error, errorCap)) {
            free(arr);
            return false;
        }
        const JsonValue *hVal = json_object_get(slot, "heading");
        if (hVal == NULL) hVal = json_object_get(slot, "headingRad");
        snprintf(field, sizeof(field), "grid[%d].heading", i);
        if (!finite_number(hVal, &heading, field, error, errorCap)) {
            free(arr);
            return false;
        }
        if (!isfinite(x) || !isfinite(y) || !isfinite(heading)) {
            set_error(error, errorCap, field, "grid pose must be finite");
            free(arr);
            return false;
        }
        arr[i].positionM = (Vector2){ x, y };
        arr[i].headingRad = heading;
    }
    out->gridSlots = arr;
    out->gridSlotCount = count;
    char gridError[256];
    if (!track_validate_grid_slots(out, gridError, sizeof(gridError))) {
        set_error(error, errorCap, "grid", gridError);
        free(arr);
        out->gridSlots = NULL;
        out->gridSlotCount = 0;
        return false;
    }
    return true;
}

static bool parse_pit_gate(const JsonValue *obj, PitGate *out, const char *key, char *error,
                           size_t errorCap)
{
    if (obj == NULL || !json_is_object(obj)) {
        char field[48];
        snprintf(field, sizeof(field), "pit.%s", key);
        set_error(error, errorCap, field, "must be an object");
        return false;
    }
    char field[48];
    float x = 0.0f, y = 0.0f, fx = 0.0f, fy = 0.0f, halfWidth = 0.0f;
    snprintf(field, sizeof(field), "pit.%s.x", key);
    if (!finite_number(json_object_get(obj, "x"), &x, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "pit.%s.y", key);
    if (!finite_number(json_object_get(obj, "y"), &y, field, error, errorCap)) return false;
    snprintf(field, sizeof(field), "pit.%s.forwardX", key);
    if (!finite_number(json_object_get(obj, "forwardX"), &fx, field, error, errorCap))
        return false;
    snprintf(field, sizeof(field), "pit.%s.forwardY", key);
    if (!finite_number(json_object_get(obj, "forwardY"), &fy, field, error, errorCap))
        return false;
    const float len = sqrtf(fx * fx + fy * fy);
    if (len < 1e-3f || fabsf(len - 1.0f) > 1e-3f) {
        snprintf(field, sizeof(field), "pit.%s.forward", key);
        set_error(error, errorCap, field, "forward vector must be unit-length within 1e-3");
        return false;
    }
    snprintf(field, sizeof(field), "pit.%s.halfWidth", key);
    if (!finite_number(json_object_get(obj, "halfWidth"), &halfWidth, field, error, errorCap))
        return false;
    if (!(halfWidth > 0.0f && halfWidth <= TRACK_MAX_HALF_WIDTH_M)) {
        snprintf(field, sizeof(field), "pit.%s.halfWidth", key);
        set_error(error, errorCap, field, "halfWidth must be in (0, 100] metres");
        return false;
    }
    out->centerM = (Vector2){ x, y };
    out->forwardUnit = (Vector2){ fx, fy };
    out->halfWidthM = halfWidth;
    return true;
}

static bool parse_pit(const JsonValue *pit, TrackDefinition *out, char *error, size_t errorCap)
{
    if (pit == NULL) return true;
    if (!json_is_object(pit)) {
        set_error(error, errorCap, "pit", "must be an object");
        return false;
    }
    const JsonValue *entry = json_object_get(pit, "entry");
    if (entry != NULL) {
        if (!parse_pit_gate(entry, &out->pitEntry, "entry", error, errorCap)) return false;
        out->hasPitEntry = true;
    }
    const JsonValue *exit = json_object_get(pit, "exit");
    if (exit != NULL) {
        if (!parse_pit_gate(exit, &out->pitExit, "exit", error, errorCap)) return false;
        out->hasPitExit = true;
    }
    const JsonValue *speed = json_object_get(pit, "speedLine");
    if (speed == NULL) speed = json_object_get(pit, "speedline");
    if (speed == NULL) speed = json_object_get(pit, "speed");
    if (speed != NULL) {
        if (!parse_pit_gate(speed, &out->pitSpeedLine, "speedLine", error, errorCap))
            return false;
        out->hasPitSpeedLine = true;
    }
    const JsonValue *boxes = json_object_get(pit, "serviceBoxes");
    if (boxes == NULL) boxes = json_object_get(pit, "serviceBox");
    if (boxes != NULL) {
        if (!json_is_array(boxes)) {
            set_error(error, errorCap, "pit.serviceBoxes", "must be an array");
            return false;
        }
        const int count = json_array_count(boxes);
        if (count > TRACK_MAX_SERVICE_BOXES) {
            set_error(error, errorCap, "pit.serviceBoxes", "too many service boxes");
            return false;
        }
        if (count > 0) {
            ServiceBox *arr = (ServiceBox *)calloc((size_t)count, sizeof(ServiceBox));
            if (arr == NULL) {
                set_error(error, errorCap, NULL, "out of memory");
                return false;
            }
            for (int i = 0; i < count; i++) {
                const JsonValue *box = json_array_at(boxes, i);
                if (box == NULL || !json_is_object(box)) {
                    set_error(error, errorCap, "pit.serviceBoxes",
                              "each box must be an object");
                    free(arr);
                    return false;
                }
                char field[48];
                float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;
                snprintf(field, sizeof(field), "pit.serviceBoxes[%d].minX", i);
                if (!finite_number(json_object_get(box, "minX"), &minX, field, error,
                                   errorCap)) {
                    free(arr);
                    return false;
                }
                snprintf(field, sizeof(field), "pit.serviceBoxes[%d].minY", i);
                if (!finite_number(json_object_get(box, "minY"), &minY, field, error,
                                   errorCap)) {
                    free(arr);
                    return false;
                }
                snprintf(field, sizeof(field), "pit.serviceBoxes[%d].maxX", i);
                if (!finite_number(json_object_get(box, "maxX"), &maxX, field, error,
                                   errorCap)) {
                    free(arr);
                    return false;
                }
                snprintf(field, sizeof(field), "pit.serviceBoxes[%d].maxY", i);
                if (!finite_number(json_object_get(box, "maxY"), &maxY, field, error,
                                   errorCap)) {
                    free(arr);
                    return false;
                }
                if (!(maxX > minX) || !(maxY > minY)) {
                    set_error(error, errorCap, field, "max must exceed min");
                    free(arr);
                    return false;
                }
                arr[i].minM = (Vector2){ minX, minY };
                arr[i].maxM = (Vector2){ maxX, maxY };
            }
            out->serviceBoxes = arr;
            out->serviceBoxCount = count;
        }
    }
    for (int i = 0; i < json_object_count(pit); i++) {
        const char *k = json_object_key_at(pit, i);
        if (strcmp(k, "entry") != 0 && strcmp(k, "exit") != 0 && strcmp(k, "speedLine") != 0 &&
            strcmp(k, "speedline") != 0 && strcmp(k, "speed") != 0 &&
            strcmp(k, "serviceBoxes") != 0 && strcmp(k, "serviceBox") != 0) {
            char reason[96];
            snprintf(reason, sizeof(reason), "unknown pit key '%s'", k);
            set_error(error, errorCap, "pit", reason);
            return false;
        }
    }
    return true;
}

static bool parse_checkpoints(const JsonValue *checkpoints, TrackDefinition *out, char *error,
                              size_t errorCap)
{
    if (checkpoints == NULL) {
        /* Absent checkpoints fall back to one gate per node, the scheme a hand-built ribbon used
         * before gates became explicit data. Lets an author ship a node list without authoring
         * gates by hand. */
        return track_build_checkpoints_from_nodes(out);
    }
    if (!json_is_array(checkpoints) || json_array_count(checkpoints) < 1) {
        set_error(error, errorCap, "checkpoints", "must be a non-empty array");
        return false;
    }
    const int count = json_array_count(checkpoints);
    Checkpoint *cpArr = (Checkpoint *)calloc((size_t)count, sizeof(Checkpoint));
    if (cpArr == NULL) {
        set_error(error, errorCap, NULL, "out of memory");
        return false;
    }
    for (int i = 0; i < count; i++) {
        if (!parse_checkpoint(json_array_at(checkpoints, i), &cpArr[i], i, error, errorCap)) {
            free(cpArr);
            return false;
        }
    }
    out->checkpoints = cpArr;
    out->checkpointCount = count;
    return true;
}

bool track_manifest_parse(const char *text, size_t length, TrackDefinition *out,
                          uint32_t *manifestHashOut, char *error, size_t errorCap)
{
    if (out == NULL) return false;
    track_free(out);
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';

    JsonDocument *doc = json_parse(text, length, error, errorCap);
    if (doc == NULL) return false;
    const JsonValue *root = json_document_root(doc);

    if (root == NULL || !json_is_object(root)) {
        set_error(error, errorCap, NULL, "track file must be a JSON object");
        json_document_free(doc);
        return false;
    }

    for (int i = 0; i < json_object_count(root); i++) {
        if (!is_known_top_key(json_object_key_at(root, i))) {
            static char reason[160];
            snprintf(reason, sizeof(reason), "unknown top-level key '%s'",
                     json_object_key_at(root, i));
            set_error(error, errorCap, NULL, reason);
            json_document_free(doc);
            return false;
        }
    }

    const JsonValue *schema = json_object_get(root, kTopKeyNames[TM_KEY_SCHEMA]);
    if (schema == NULL || !json_is_string(schema) ||
        strcmp(json_as_string(schema), TRACK_MANIFEST_SCHEMA) != 0) {
        set_error(error, errorCap, "schema", "must be \"" TRACK_MANIFEST_SCHEMA "\"");
        json_document_free(doc);
        return false;
    }

    const JsonValue *versionVal = json_object_get(root, kTopKeyNames[TM_KEY_VERSION]);
    if (versionVal == NULL || !json_is_number(versionVal)) {
        set_error(error, errorCap, "version", "must be a number");
        json_document_free(doc);
        return false;
    }
    const double ver = json_as_number(versionVal);
    if (ver != 1.0 && ver != 2.0 && ver != (double)TRACK_MANIFEST_VERSION) {
        static char reason[96];
        snprintf(reason, sizeof(reason), "version must be 1 or %d", TRACK_MANIFEST_VERSION);
        set_error(error, errorCap, "version", reason);
        json_document_free(doc);
        return false;
    }

    const char *id = json_as_string(json_object_get(root, kTopKeyNames[TM_KEY_ID]));
    if (!track_id_is_valid(id)) {
        set_error(error, errorCap, "id", "must match [a-z0-9][a-z0-9._-]{0,30} (≤31 chars)");
        json_document_free(doc);
        return false;
    }
    if (strlen(id) + 1 > sizeof(out->id)) {
        set_error(error, errorCap, "id", "value is too long");
        json_document_free(doc);
        return false;
    }
    const char *contentVersion =
        json_as_string(json_object_get(root, kTopKeyNames[TM_KEY_CONTENT_VERSION]));
    if (contentVersion == NULL || contentVersion[0] == '\0') {
        set_error(error, errorCap, "contentVersion", "must be a non-empty string");
        json_document_free(doc);
        return false;
    }
    if (strlen(contentVersion) + 1 > sizeof(out->version)) {
        set_error(error, errorCap, "contentVersion", "value is too long");
        json_document_free(doc);
        return false;
    }

    /* Geometry first; it owns the heap allocations that every later step reads. */
    if (!parse_route(json_object_get(root, kTopKeyNames[TM_KEY_ROUTE]), out, error, errorCap)) {
        json_document_free(doc);
        track_free(out);
        return false;
    }
    if (ver == 1.0 && !out->routeClosed) {
        set_error(error, errorCap, "route.closed",
                  "must be true for version 1 (open routes require version 2)");
        json_document_free(doc);
        track_free(out);
        return false;
    }
    if (!parse_surfaces(json_object_get(root, kTopKeyNames[TM_KEY_SURFACES]), out, error,
                        errorCap) ||
        !parse_parking_lot(json_object_get(root, kTopKeyNames[TM_KEY_PARKING_LOT]), out, error,
                           errorCap) ||
        !parse_checkpoints(json_object_get(root, kTopKeyNames[TM_KEY_CHECKPOINTS]), out, error,
                           errorCap)) {
        json_document_free(doc);
        track_free(out);
        return false;
    }
    if (!parse_sectors(json_object_get(root, kTopKeyNames[TM_KEY_SECTORS]), out, error,
                       errorCap) ||
        !parse_start_finish(json_object_get(root, kTopKeyNames[TM_KEY_START_FINISH]), out,
                            error, errorCap) ||
        !parse_grid(json_object_get(root, kTopKeyNames[TM_KEY_GRID]), out, error, errorCap) ||
        !parse_pit(json_object_get(root, kTopKeyNames[TM_KEY_PIT]), out, error, errorCap)) {
        json_document_free(doc);
        track_free(out);
        return false;
    }

    snprintf(out->id, sizeof(out->id), "%s", id);
    snprintf(out->version, sizeof(out->version), "%s", contentVersion);

    if (manifestHashOut != NULL) *manifestHashOut = json_canonical_hash(root);
    json_document_free(doc);
    return true;
}

bool track_manifest_load(const char *path, TrackDefinition *out, uint32_t *manifestHashOut,
                         char *error, size_t errorCap)
{
    if (out == NULL) return false;
    track_free(out);
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (path == NULL) {
        set_error(error, errorCap, NULL, "no track path given");
        return false;
    }
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        set_error(error, errorCap, path, "could not open file");
        return false;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        set_error(error, errorCap, path, "could not seek file");
        return false;
    }
    const long size = ftell(file);
    rewind(file);
    if (size < 0) {
        fclose(file);
        set_error(error, errorCap, path, "could not tell file size");
        return false;
    }
    char *buffer = (char *)malloc((size_t)size + 1u);
    if (buffer == NULL) {
        fclose(file);
        set_error(error, errorCap, NULL, "out of memory");
        return false;
    }
    const size_t read = fread(buffer, 1u, (size_t)size, file);
    fclose(file);
    buffer[read] = '\0';
    const bool ok = track_manifest_parse(buffer, read, out, manifestHashOut, error, errorCap);
    free(buffer);
    return ok;
}

bool track_manifest_id_is_valid(const char *id)
{
    return track_id_is_valid(id);
}

static int compare_catalog_entry_by_id(const void *a, const void *b)
{
    const TrackCatalogEntry *ea = (const TrackCatalogEntry *)a;
    const TrackCatalogEntry *eb = (const TrackCatalogEntry *)b;
    return strcmp(ea->definition.id, eb->definition.id);
}

bool track_catalog_load(const char *dir, TrackCatalog *out, char *error, size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';
    const char *useDir = (dir != NULL) ? dir : TRACK_CATALOG_DIR;
    if (useDir == NULL) {
        set_error(error, errorCap, NULL, "no catalog directory given");
        return false;
    }
    DIR *d = opendir(useDir);
    if (d == NULL) {
        set_error(error, errorCap, useDir, "could not open directory");
        return false;
    }
    int fileCount = 0;
    const struct dirent *entry;
    const size_t suffixLen = strlen(TRACK_CATALOG_SUFFIX);
    while ((entry = readdir(d)) != NULL) {
        const char *name = entry->d_name;
        const size_t len = strlen(name);
        if (len > suffixLen && strcmp(name + len - suffixLen, TRACK_CATALOG_SUFFIX) == 0) {
            fileCount++;
        }
    }
    rewinddir(d);
    TrackCatalogEntry *items = NULL;
    if (fileCount > 0) {
        items = (TrackCatalogEntry *)calloc((size_t)fileCount, sizeof(TrackCatalogEntry));
        if (items == NULL) {
            closedir(d);
            set_error(error, errorCap, NULL, "out of memory");
            return false;
        }
    }
    int count = 0;
    bool ok = true;
    while (ok && (entry = readdir(d)) != NULL) {
        const char *name = entry->d_name;
        const size_t len = strlen(name);
        if (len <= suffixLen || strcmp(name + len - suffixLen, TRACK_CATALOG_SUFFIX) != 0)
            continue;
        char path[1024];
        const int written = snprintf(path, sizeof(path), "%s/%s", useDir, name);
        if (written < 0 || written >= (int)sizeof(path)) {
            set_error(error, errorCap, name, "path is too long");
            ok = false;
            break;
        }
        if (count >= fileCount) {
            set_error(error, errorCap, useDir, "catalog directory changed during load");
            ok = false;
            break;
        }
        uint32_t manifestHash = 0;
        if (!track_manifest_load(path, &items[count].definition, &manifestHash, error,
                                 errorCap)) {
            ok = false;
            break;
        }
        /* Filename must match stable id: a file named foo.track.json that contains
         * id=bar would be selectable as bar via the catalog but missed by
         * track_load_by_id("bar") which builds the canonical path. Rejecting here
         * makes the mismatch visible rather than silently shadowing content. */
        {
            char base[TRACK_ID_CHARS + 16];
            const size_t baseLen = len - suffixLen;
            if (baseLen >= sizeof(base) ||
                snprintf(base, sizeof(base), "%.*s", (int)baseLen, name) >= (int)sizeof(base)) {
                set_error(error, errorCap, name, "filename is too long");
                track_free(&items[count].definition);
                ok = false;
                break;
            }
            if (strcmp(base, items[count].definition.id) != 0) {
                char reason[192];
                snprintf(reason, sizeof(reason),
                         "filename '%s' does not match manifest id '%s'", name,
                         items[count].definition.id);
                set_error(error, errorCap, name, reason);
                track_free(&items[count].definition);
                ok = false;
                break;
            }
        }
        items[count].manifestHash = manifestHash;
        count++;
    }
    closedir(d);
    if (!ok) {
        for (int i = 0; i < count; i++) track_free(&items[i].definition);
        free(items);
        out->entries = NULL;
        out->count = 0;
        return false;
    }
    qsort(items, (size_t)count, sizeof(TrackCatalogEntry), compare_catalog_entry_by_id);
    for (int i = 1; i < count; i++) {
        if (strcmp(items[i - 1].definition.id, items[i].definition.id) == 0) {
            char reason[192];
            snprintf(reason, sizeof(reason), "duplicate content id '%s'",
                     items[i].definition.id);
            set_error(error, errorCap, NULL, reason);
            for (int j = 0; j < count; j++) track_free(&items[j].definition);
            free(items);
            out->entries = NULL;
            out->count = 0;
            return false;
        }
    }
    out->entries = items;
    out->count = count;
    return true;
}

void track_catalog_free(TrackCatalog *catalog)
{
    if (catalog == NULL) return;
    for (int i = 0; i < catalog->count; i++) track_free(&catalog->entries[i].definition);
    free(catalog->entries);
    catalog->entries = NULL;
    catalog->count = 0;
}

int track_catalog_find(const TrackCatalog *catalog, const char *id)
{
    if (catalog == NULL || id == NULL) return -1;
    for (int i = 0; i < catalog->count; i++) {
        if (strcmp(catalog->entries[i].definition.id, id) == 0) return i;
    }
    return -1;
}

bool track_load_by_id(const char *id, TrackDefinition *out, uint32_t *manifestHashOut,
                      char *error, size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (!track_manifest_id_is_valid(id)) {
        set_error(error, errorCap, "id", "invalid track id");
        return false;
    }
    char path[1024];
    const int written =
        snprintf(path, sizeof(path), "%s/%s%s", TRACK_CATALOG_DIR, id, TRACK_CATALOG_SUFFIX);
    if (written < 0 || written >= (int)sizeof(path)) {
        set_error(error, errorCap, id, "path is too long");
        return false;
    }
    if (!track_manifest_load(path, out, manifestHashOut, error, errorCap)) return false;
    if (strcmp(out->id, id) != 0) {
        track_free(out);
        memset(out, 0, sizeof(*out));
        set_error(error, errorCap, "id", "does not match requested track id");
        return false;
    }
    return true;
}

/* ----------------------------------------------------------------------------------------------- export */

static void write_json_string(FILE *out, const char *s)
{
    fputc('"', out);
    for (const char *p = s; *p != '\0'; p++) {
        const unsigned char c = (unsigned char)*p;
        switch (c) {
            case '"': fputs("\\\"", out); break;
            case '\\': fputs("\\\\", out); break;
            case '\n': fputs("\\n", out); break;
            case '\r': fputs("\\r", out); break;
            case '\t': fputs("\\t", out); break;
            default:
                if (c < 0x20) {
                    fprintf(out, "\\u%04x", (unsigned int)c);
                } else {
                    fputc((char)c, out);
                }
                break;
        }
    }
    fputc('"', out);
}

bool track_manifest_write(const TrackDefinition *track, const char *displayName,
                          const char *description, FILE *out)
{
    if (track == NULL || out == NULL) return false;

    fprintf(out, "{\n");
    fprintf(out, "  \"schema\": \"%s\",\n", TRACK_MANIFEST_SCHEMA);
    fprintf(out, "  \"version\": %d,\n", TRACK_MANIFEST_VERSION);
    fprintf(out, "  \"id\": ");
    write_json_string(out, track->id);
    fprintf(out, ",\n  \"displayName\": ");
    write_json_string(out, displayName != NULL ? displayName : track->id);
    fprintf(out, ",\n  \"description\": ");
    write_json_string(out, description != NULL ? description : "");
    fprintf(out, ",\n  \"contentVersion\": ");
    write_json_string(out, track->version);
    fprintf(out, ",\n");

    fprintf(out, "  \"route\": {\n    \"closed\": %s,\n    \"nodes\": [\n",
            track->routeClosed ? "true" : "false");
    for (int i = 0; i < track->count; i++) {
        const TrackNode *n = &track->nodes[i];
        const char *surface = track_manifest_surface_name(n->surfaceId);
        if (surface == NULL) surface = "asphalt";
        fprintf(out,
                "      { \"x\": %.9g, \"y\": %.9g, \"halfWidth\": %.9g, "
                "\"runoffHalfWidth\": %.9g, \"surface\": \"%s\" }%s\n",
                (double)n->centerM.x, (double)n->centerM.y, (double)n->halfWidthM,
                (double)n->runoffHalfWidthM, surface, (i + 1 < track->count) ? "," : "");
    }
    fprintf(out, "    ]\n  },\n");

    const bool hasParking = track->isParkingLot;
    const bool hasCheckpoints = (track->checkpointCount > 0);
    const bool hasSectors = (track->sectorMarkerCount > 0);
    const bool hasStartFinishFlag = track->hasStartFinish;
    const bool hasGrid = (track->gridSlotCount > 0);
    const bool hasPit = track_pit_has_geometry(track);

    const bool surfacesNeedsComma =
        hasParking || hasCheckpoints || hasSectors || hasStartFinishFlag || hasGrid || hasPit;
    fprintf(out, "  \"surfaces\": { \"offTrack\": \"%s\", \"runoff\": \"%s\" }%s\n",
            track_manifest_surface_name(track->offTrackSurfaceId),
            track_manifest_surface_name(track->runoffSurfaceId), surfacesNeedsComma ? "," : "");

    if (hasParking) {
        const bool needsComma =
            hasCheckpoints || hasSectors || hasStartFinishFlag || hasGrid || hasPit;
        fprintf(out,
                "  \"parkingLot\": { \"minX\": %.9g, \"maxX\": %.9g, \"minY\": %.9g, "
                "\"maxY\": %.9g }%s\n",
                (double)track->lotMinXM, (double)track->lotMaxXM, (double)track->lotMinYM,
                (double)track->lotMaxYM, needsComma ? "," : "");
    }

    if (hasCheckpoints) {
        fprintf(out, "  \"checkpoints\": [\n");
        for (int i = 0; i < track->checkpointCount; i++) {
            const Checkpoint *c = &track->checkpoints[i];
            fprintf(out,
                    "      { \"x\": %.9g, \"y\": %.9g, \"forwardX\": %.9g, \"forwardY\": %.9g, "
                    "\"halfWidth\": %.9g, \"required\": %s }%s\n",
                    (double)c->centerM.x, (double)c->centerM.y, (double)c->forwardUnit.x,
                    (double)c->forwardUnit.y, (double)c->halfWidthM,
                    c->required ? "true" : "false",
                    (i + 1 < track->checkpointCount) ? "," : "");
        }
        const bool needsComma = hasSectors || hasStartFinishFlag || hasGrid || hasPit;
        fprintf(out, "  ]%s\n", needsComma ? "," : "");
    }
    if (hasSectors) {
        fprintf(out, "  \"sectors\": [\n");
        for (int i = 0; i < track->sectorMarkerCount; i++) {
            const SectorMarker *s = &track->sectorMarkers[i];
            fprintf(out,
                    "      { \"x\": %.9g, \"y\": %.9g, \"forwardX\": %.9g, \"forwardY\": %.9g, "
                    "\"halfWidth\": %.9g }%s\n",
                    (double)s->centerM.x, (double)s->centerM.y, (double)s->forwardUnit.x,
                    (double)s->forwardUnit.y, (double)s->halfWidthM,
                    (i + 1 < track->sectorMarkerCount) ? "," : "");
        }
        const bool needsComma = hasStartFinishFlag || hasGrid || hasPit;
        fprintf(out, "  ]%s\n", needsComma ? "," : "");
    }
    if (hasStartFinishFlag) {
        const StartFinishLine *sf = &track->startFinish;
        const bool needsComma = hasGrid || hasPit;
        fprintf(out,
                "  \"startFinish\": { \"x\": %.9g, \"y\": %.9g, \"forwardX\": %.9g, "
                "\"forwardY\": %.9g, \"halfWidth\": %.9g }%s\n",
                (double)sf->centerM.x, (double)sf->centerM.y, (double)sf->forwardUnit.x,
                (double)sf->forwardUnit.y, (double)sf->halfWidthM, needsComma ? "," : "");
    }
    if (hasGrid) {
        fprintf(out, "  \"grid\": [\n");
        for (int i = 0; i < track->gridSlotCount; i++) {
            const GridSlot *g = &track->gridSlots[i];
            fprintf(out, "      { \"x\": %.9g, \"y\": %.9g, \"heading\": %.9g }%s\n",
                    (double)g->positionM.x, (double)g->positionM.y, (double)g->headingRad,
                    (i + 1 < track->gridSlotCount) ? "," : "");
        }
        fprintf(out, "  ]%s\n", hasPit ? "," : "");
    }
    if (hasPit) {
        fprintf(out, "  \"pit\": {\n");
        bool first = true;
        if (track->hasPitEntry) {
            const PitGate *pg = &track->pitEntry;
            fprintf(out,
                    "    \"entry\": { \"x\": %.9g, \"y\": %.9g, \"forwardX\": %.9g, "
                    "\"forwardY\": %.9g, \"halfWidth\": %.9g }",
                    (double)pg->centerM.x, (double)pg->centerM.y, (double)pg->forwardUnit.x,
                    (double)pg->forwardUnit.y, (double)pg->halfWidthM);
            first = false;
        }
        if (track->hasPitExit) {
            const PitGate *pg = &track->pitExit;
            if (!first) fprintf(out, ",\n");
            fprintf(out,
                    "    \"exit\": { \"x\": %.9g, \"y\": %.9g, \"forwardX\": %.9g, "
                    "\"forwardY\": %.9g, \"halfWidth\": %.9g }",
                    (double)pg->centerM.x, (double)pg->centerM.y, (double)pg->forwardUnit.x,
                    (double)pg->forwardUnit.y, (double)pg->halfWidthM);
            first = false;
        }
        if (track->hasPitSpeedLine) {
            const PitGate *pg = &track->pitSpeedLine;
            if (!first) fprintf(out, ",\n");
            fprintf(out,
                    "    \"speedLine\": { \"x\": %.9g, \"y\": %.9g, \"forwardX\": %.9g, "
                    "\"forwardY\": %.9g, \"halfWidth\": %.9g }",
                    (double)pg->centerM.x, (double)pg->centerM.y, (double)pg->forwardUnit.x,
                    (double)pg->forwardUnit.y, (double)pg->halfWidthM);
            first = false;
        }
        if (track->serviceBoxCount > 0) {
            if (!first) fprintf(out, ",\n");
            fprintf(out, "    \"serviceBoxes\": [\n");
            for (int i = 0; i < track->serviceBoxCount; i++) {
                const ServiceBox *b = &track->serviceBoxes[i];
                fprintf(out,
                        "      { \"minX\": %.9g, \"minY\": %.9g, \"maxX\": %.9g, \"maxY\": "
                        "%.9g }%s\n",
                        (double)b->minM.x, (double)b->minM.y, (double)b->maxM.x,
                        (double)b->maxM.y, (i + 1 < track->serviceBoxCount) ? "," : "");
            }
            fprintf(out, "    ]");
        }
        fprintf(out, "\n  }\n");
    }
    fprintf(out, "}\n");
    return true;
}

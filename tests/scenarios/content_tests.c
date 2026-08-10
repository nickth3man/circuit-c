/*
 * content_tests.c — scenarios for the versioned vehicle and track content formats.
 *
 * Three groups of assertions, kept out of the existing scenario files so the registry stays one
 * concern per file:
 *
 *   json-parser        the strict reader and its canonical content hash
 *   vehicle-manifest   issue #29: manifest load, validation, round-trip, and catalog discovery
 *   track-format       issue #34: external track load, faithful round-trip, and hash stability
 *
 * Every scenario is headless and deterministic: no window, no audio, no working-directory
 * assumption beyond the shared artifacts/telemetry scratch directory.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "test_commands.h"
#include "test_scenarios.h"
#include "support/test_harness.h"

#include "core/config.h"
#include "core/json.h"
#include "content/track_manifest.h"
#include "content/vehicle_manifest.h"
#include "dev/dev_params.h"
#include "game/car_roster.h"
#include "game/telemetry.h"
#include "physics/vehicle.h"
#include "world/track.h"

/* A minimal manifest used by several checks. It carries one physics and one setup override so the
 * round-trip and owner-routing paths are exercised without depending on the full registry. */
static const char kSampleManifest[] =
    "{\n"
    "  \"schema\": \"circuit/vehicle\", \"version\": 1,\n"
    "  \"id\": \"sample_rwd\", \"displayName\": \"Sample RWD\",\n"
    "  \"contentVersion\": 1, \"appearanceId\": \"sample_rwd\",\n"
    "  \"physics\": { \"body.wheelbase\": 2.60, \"tire.lat_front.mu\": 1.40 },\n"
    "  \"setup\": { \"brake.bias_front\": 0.58, \"drive.gear_count\": 5 }\n"
    "}\n";

/* ---------------------------------------------------------------------------------------------- json */

static void scenario_json_parser(void)
{
    char error[256];

    /* A representative document exercises every value type and nested structure. */
    JsonDocument *doc = json_parse(
        "{\n"
        "  \"a\": 1, \"b\": [true, null, \"x\"], \"c\": { \"d\": 0.25e0 }, \"neg\": -3.5,\n"
        "  \"esc\": \"a\\tb\\u00e9\"\n"
        "}\n",
        0, error, sizeof(error));
    check(doc != NULL, "valid document parses (error: %s)", error);
    if (doc != NULL) {
        const JsonValue *root = json_document_root(doc);
        check(json_is_object(root), "root is an object");
        check(json_as_number(json_object_get(root, "a")) == 1.0, "number member reads back");
        const JsonValue *b = json_object_get(root, "b");
        check(json_is_array(b) && json_array_count(b) == 3, "array has three elements");
        check(json_as_bool(json_array_at(b, 0)) == true, "array bool is true");
        check(json_is_null(json_array_at(b, 1)), "array null is null");
        check(strcmp(json_as_string(json_array_at(b, 2)), "x") == 0, "array string is \"x\"");
        check(json_as_number(json_object_get(json_object_get(root, "c"), "d")) == 0.25,
              "nested number reads back");
        check(json_as_number(json_object_get(root, "neg")) == -3.5,
              "negative number reads back");
        const char *esc = json_as_string(json_object_get(root, "esc"));
        check(esc != NULL && strcmp(esc, "a\tb\xc3\xa9") == 0,
              "escape sequences decode (tab + U+00E9)");
        json_document_free(doc);
    }

    /* Whitespace before colon and formatting variants parse and yield identical canonical hashes. */
    JsonDocument *d1 = json_parse("{\"a\":1,\"b\":2}", 0, error, sizeof(error));
    JsonDocument *d2 =
        json_parse("{\n  \"b\" : 2.0,\n  \"a\" : 1\n}\n", 0, error, sizeof(error));
    if (d1 != NULL && d2 != NULL) {
        check(json_canonical_hash(json_document_root(d1)) ==
                  json_canonical_hash(json_document_root(d2)),
              "canonical hash matches across key order, whitespace, and number spelling");
    }
    json_document_free(d1);
    json_document_free(d2);

    /* Strict rejections: each is a real deviation, not a tolerated quirk. */
    static const struct {
        const char *text;
        const char *what;
    } bad[] = {
        { "", "empty input" },
        { "{", "unterminated object" },
        { "{\"a\":1,}", "trailing comma" },
        { "{\"a\":1,\"a\":2}", "duplicate key" },
        { "{} garbage", "trailing characters" },
        { "{a:1}", "unquoted key" },
        { "{\"a\":1e}", "malformed exponent" },
        { "{\"a\":012}", "leading zero" },
        { "[1e999]", "non-finite number" },
        { "{\"a\":\"hello\\u0000world\"}", "escaped NUL byte" },
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        JsonDocument *fail = json_parse(bad[i].text, strlen(bad[i].text), error, sizeof(error));
        check(fail == NULL, "strict parser rejects %s", bad[i].what);
        json_document_free(fail);
    }

    /* The depth cap rejects pathological nesting rather than overflowing the C stack. */
    char nested[8192];
    for (size_t i = 0; i < sizeof(nested) - 1; i++) nested[i] = '[';
    nested[sizeof(nested) - 1] = '\0';
    JsonDocument *deep = json_parse(nested, strlen(nested), error, sizeof(error));
    check(deep == NULL, "deeply nested input hits the depth limit");
    json_document_free(deep);
}

/* --------------------------------------------------------------------------------------- vehicle manifest */

/* Build a manifest text buffer from a roster spec, parse it back, and compare hashes. Returns the
 * parsed manifest hash, or 0 on any failure. The temp file lives under the shared scratch dir so
 * the scenario needs no extra setup. */
static uint32_t roundtrip_roster_manifest(int rosterIndex, VehicleManifest *out)
{
    char id[128], path[512];
    car_roster_id(rosterIndex, id, sizeof(id));

    VehicleSpec spec;
    if (!car_roster_spec(rosterIndex, &spec)) return 0;

    /* The original definition's hash is the reference the round-trip must reproduce. */
    VehicleDefinition ref;
    if (!vehicle_definition_init(&ref, id, id, 1u, &spec)) return 0;

    snprintf(path, sizeof(path), "%s/_manifest_%s.vehicle.json", TELEMETRY_DIR, id);
    FILE *file = fopen(path, "wb");
    if (file == NULL) return 0;
    const bool wrote = vehicle_manifest_write(&spec, id, id, id, 1u, "roster", file);
    fclose(file);
    if (!wrote) return 0;

    char error[256];
    if (!vehicle_manifest_load(path, out, error, sizeof(error))) return 0;
    return ref.contentHash;
}

static void scenario_vehicle_manifest(void)
{
    char error[256];

    /* Golden parse: required fields land, overrides apply, hashes are populated. */
    VehicleManifest m;
    check(vehicle_manifest_parse(kSampleManifest, strlen(kSampleManifest), &m, error,
                                 sizeof(error)),
          "sample manifest parses (error: %s)", error);
    if (vehicle_manifest_parse(kSampleManifest, strlen(kSampleManifest), &m, error,
                               sizeof(error))) {
        check(strcmp(m.definition.id, "sample_rwd") == 0, "id round-trips");
        check(strcmp(m.displayName, "Sample RWD") == 0, "displayName round-trips");
        check(m.definition.contentVersion == 1u, "contentVersion round-trips");
        check(m.definition.contentHash != 0u, "definition has a content hash");
        check(m.manifestHash != 0u, "manifest has a canonical hash");
        check(fabsf(m.definition.spec.wheelbaseM - 2.60f) < 1e-5f &&
                  fabsf(m.definition.spec.tireMuLatFront - 1.40f) < 1e-5f,
              "physics overrides apply to the spec");
        check(fabsf(m.defaultSetup.brakeBiasFront - 0.58f) < 1e-5f,
              "setup override lands in the default setup");
        check(m.defaultSetup.gearCount == 5, "gear_count setup override applies");
        check(m.eligibleHuman && m.eligibleAi, "absent eligibility defaults to human+ai");
    }

    /* Defaults: a manifest with no physics or setup overrides is the stock car. */
    static const char kDefaultManifest[] =
        "{\n"
        "  \"schema\": \"circuit/vehicle\", \"version\": 1,\n"
        "  \"id\": \"stock\", \"displayName\": \"Stock\", \"contentVersion\": 1,\n"
        "  \"appearanceId\": \"stock\"\n"
        "}\n";
    VehicleManifest dm;
    check(vehicle_manifest_parse(kDefaultManifest, strlen(kDefaultManifest), &dm, error,
                                 sizeof(error)),
          "default-only manifest parses");
    VehicleDefinition defaultDef;
    vehicle_definition_set_default(&defaultDef);
    check(dm.definition.contentHash == defaultDef.contentHash,
          "manifest with no overrides matches vehicle_definition_set_default");

    /* Round-trip one car of each drivetrain layout; the parsed content hash must equal the
     * reference built straight from the roster spec, proving the format loses nothing. */
    check(telemetry_ensure_dir(TELEMETRY_DIR),
          "scratch directory exists for manifest round-trip");
    const int layoutIndices[] = { 0, 2, 4 }; /* RWD, FWD, AWD */
    for (size_t i = 0; i < sizeof(layoutIndices) / sizeof(layoutIndices[0]); i++) {
        VehicleManifest parsed;
        const uint32_t refHash = roundtrip_roster_manifest(layoutIndices[i], &parsed);
        check(refHash != 0u, "roster entry %d builds a reference definition", layoutIndices[i]);
        if (refHash != 0u) {
            check(parsed.definition.contentHash == refHash,
                  "roster entry %d round-trips with an identical content hash",
                  layoutIndices[i]);
        }
    }

    /* Validation rejections: each is a distinct way a manifest can lie about a car. */
    struct {
        const char *text;
        const char *what;
    } bad[] = {
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\",\"bogus\":1}",
          "unknown top-level key" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\","
          "\"physics\":{\"brake.bias_front\":0.6}}",
          "setup key placed in physics" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\","
          "\"setup\":{\"body.wheelbase\":2.5}}",
          "definition key placed in setup" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\","
          "\"physics\":{\"body.length_overall\":4.0}}",
          "derived key authored" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\","
          "\"physics\":{\"body.wheelbase\":999.0}}",
          "out-of-range value" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"Bad "
          "ID\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\"}",
          "invalid id" },
        { "{\"schema\":\"circuit/other\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\"}",
          "wrong schema" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":2,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\"}",
          "unsupported version" },
        { "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"x\",\"displayName\":\"X\","
          "\"contentVersion\":1,\"appearanceId\":\"x\","
          "\"setup\":{\"drive.gear_count\":99}}",
          "gear_count out of range" },
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        VehicleManifest rejected;
        const bool ok = vehicle_manifest_parse(bad[i].text, strlen(bad[i].text), &rejected,
                                               error, sizeof(error));
        check(!ok, "manifest rejected for %s (error: %s)", bad[i].what, ok ? "(none)" : error);
        check(rejected.definition.contentVersion == 0 && rejected.definition.id[0] == '\0',
              "rejected manifest is zeroed for %s", bad[i].what);
    }

    /* Catalog discovery: write three manifests with out-of-order ids and confirm the catalog comes
     * back sorted by stable id, so filesystem enumeration order cannot change the roster. */
    const char *ids[] = { "charlie", "alpha", "bravo" };
    char dir[512];
    snprintf(dir, sizeof(dir), "%s/_catalog", TELEMETRY_DIR);
    telemetry_ensure_dir(dir);
    for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        char path[640], text[512];
        snprintf(path, sizeof(path), "%s/%02d_%s.vehicle.json", dir, (int)i, ids[i]);
        snprintf(text, sizeof(text),
                 "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"%s\","
                 "\"displayName\":\"%s\",\"contentVersion\":1,\"appearanceId\":\"%s\"}",
                 ids[i], ids[i], ids[i]);
        FILE *file = fopen(path, "wb");
        check(file != NULL, "catalog fixture %zu opens", i);
        if (file != NULL) {
            fputs(text, file);
            fclose(file);
        }
    }
    VehicleCatalog catalog;
    const bool loaded = vehicle_manifest_load_dir(dir, &catalog, error, sizeof(error));
    check(loaded && catalog.count == 3, "catalog loads three fixtures (error: %s)",
          loaded ? "(none)" : error);
    if (loaded && catalog.count == 3) {
        check(strcmp(catalog.items[0].definition.id, "alpha") == 0 &&
                  strcmp(catalog.items[1].definition.id, "bravo") == 0 &&
                  strcmp(catalog.items[2].definition.id, "charlie") == 0,
              "catalog is sorted by stable id regardless of file order");
    }
    vehicle_catalog_free(&catalog);

    /* Duplicate ids are a load error even when both files individually parse. */
    {
        char dupPath[640];
        snprintf(dupPath, sizeof(dupPath), "%s/zz_dup.vehicle.json", dir);
        FILE *file = fopen(dupPath, "wb");
        if (file != NULL) {
            fputs("{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"alpha\","
                  "\"displayName\":\"Dup\",\"contentVersion\":1,\"appearanceId\":\"alpha\"}",
                  file);
            fclose(file);
        }
        VehicleCatalog dup;
        const bool ok = vehicle_manifest_load_dir(dir, &dup, error, sizeof(error));
        check(!ok, "catalog with a duplicate id is rejected (error: %s)",
              ok ? "(none)" : error);
        vehicle_catalog_free(&dup);
        remove(dupPath);
    }
}

/* ------------------------------------------------------------------------------------------- track format */

/* Build a track from a compiled-in loader, write it, read it back, and compare geometry hashes.
 * Equal hashes prove the external format faithfully represents the built-in geometry. */
static bool track_roundtrip(void (*load)(TrackDefinition *), const char *id)
{
    char path[512];
    TrackDefinition compiled;
    memset(&compiled, 0, sizeof(compiled)); /* loaders free existing state first */
    load(&compiled);
    const uint32_t compiledHash = track_geometry_hash(&compiled);

    snprintf(path, sizeof(path), "%s/_track_%s.track.json", TELEMETRY_DIR, id);
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        track_free(&compiled);
        return false;
    }
    const bool wrote = track_manifest_write(&compiled, id, id, file);
    fclose(file);
    track_free(&compiled);
    if (!wrote) return false;

    char error[256];
    TrackDefinition loaded;
    memset(&loaded, 0, sizeof(loaded));
    const bool ok = track_manifest_load(path, &loaded, NULL, error, sizeof(error));
    if (!ok) {
        track_free(&loaded);
        return false;
    }
    const bool equal = (track_geometry_hash(&loaded) == compiledHash);
    track_free(&loaded);
    return equal;
}

static void scenario_track_format(void)
{
    char error[256];
    check(telemetry_ensure_dir(TELEMETRY_DIR), "scratch directory exists for track round-trip");

    /* Each built-in layout round-trips with an identical geometry hash: the external format
     * reproduces the authored centreline and gates bit-for-bit through %.9g serialization. */
    struct {
        const char *id;
        void (*load)(TrackDefinition *);
    } layouts[] = {
        { "parking_lot", track_init },
        { "chicane", track_load_chicane },
        { "sprint", track_load_sprint },
        { "technical", track_load_technical },
    };
    for (size_t i = 0; i < sizeof(layouts) / sizeof(layouts[0]); i++) {
        check(track_roundtrip(layouts[i].load, layouts[i].id),
              "%s round-trips with an identical geometry hash", layouts[i].id);
    }

    /* The committed schema examples must still parse and match their compiled geometry, so a
     * hand edit to a committed file is caught rather than silently shipping a wrong track. */
    for (size_t i = 0; i < sizeof(layouts) / sizeof(layouts[0]); i++) {
        char path[512];
        snprintf(path, sizeof(path), "data/tracks/%s.track.json", layouts[i].id);
        TrackDefinition compiled;
        memset(&compiled, 0, sizeof(compiled));
        layouts[i].load(&compiled);
        const uint32_t compiledHash = track_geometry_hash(&compiled);
        track_free(&compiled);

        TrackDefinition loaded;
        memset(&loaded, 0, sizeof(loaded));
        const bool ok = track_manifest_load(path, &loaded, NULL, error, sizeof(error));
        check(ok, "committed %s example loads (error: %s)", layouts[i].id,
              ok ? "(none)" : error);
        if (ok) {
            check(track_geometry_hash(&loaded) == compiledHash,
                  "committed %s example matches its compiled geometry hash", layouts[i].id);
            track_free(&loaded);
        }
    }

    /* Canonical hash stability: two textually different but semantically equal track files hash
     * the same, which is what content-compatibility checks rely on. */
    static const char *kCompact =
        "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
        "\"route\":{\"closed\":true,\"nodes\":["
        "{\"x\":0,\"y\":0,\"halfWidth\":8,\"runoffHalfWidth\":12,\"surface\":\"asphalt\"},"
        "{\"x\":10,\"y\":0,\"halfWidth\":8,\"runoffHalfWidth\":12,\"surface\":\"asphalt\"}]}}";
    static const char *kSpaced =
        "{\n  \"schema\": \"circuit/track\",\n  \"version\": 1,\n  \"contentVersion\": "
        "\"v1\",\n"
        "  \"id\": \"t\",\n  \"route\": {\n    \"closed\": true,\n    \"nodes\": [\n"
        "      { \"y\": 0.0, \"x\": 0, \"halfWidth\": 8.0, \"surface\": \"asphalt\", "
        "\"runoffHalfWidth\": 12.0 },\n      { \"x\": 1e1, \"y\": 0, \"halfWidth\": 8, "
        "\"runoffHalfWidth\": 12, \"surface\": \"asphalt\" }\n    ]\n  }\n}\n";
    TrackDefinition a, b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    uint32_t ha = 0, hb = 0;
    const bool oka =
        track_manifest_parse(kCompact, strlen(kCompact), &a, &ha, error, sizeof(error));
    const bool okb =
        track_manifest_parse(kSpaced, strlen(kSpaced), &b, &hb, error, sizeof(error));
    check(oka && okb, "hash-variant track files parse");
    if (oka && okb) {
        check(ha == hb, "canonical manifest hash is stable under formatting-only changes");
    }
    if (oka) track_free(&a);
    if (okb) track_free(&b);

    /* Validation rejections: each names a distinct malformed or self-contradictory file. */
    struct {
        const char *text;
        const char *what;
    } bad[] = {
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":0}]}}",
          "non-positive node halfWidth" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":false,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]}}",
          "open route (unsupported in v1)" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8,"
          "\"surface\":\"ice\"}]}}",
          "unknown surface name" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]},"
          "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":1,\"halfWidth\":10}]}",
          "non-unit checkpoint forward vector" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]},\"extra\":1}",
          "unknown top-level key" },
        { "{\"schema\":\"circuit/other\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]}}",
          "wrong schema" },
        { "{\"schema\":\"circuit/track\",\"version\":2,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]}}",
          "unsupported version" },
        { "{\"schema\":\"circuit/"
          "track\",\"version\":1,\"id\":\"Chicane\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]}}",
          "uppercase track id" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]},"
          "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10,"
          "\"required\":\"yes\"}]}",
          "non-boolean checkpoint required" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]},"
          "\"surfaces\":{\"offTrack\":123}}",
          "non-string surface name" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":1e100,\"y\":0,\"halfWidth\":8}]}}",
          "coordinate out of float range" },
        { "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":0,\"y\":0,\"halfWidth\":8}]}}",
          "consecutive nodes coincide" },
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TrackDefinition rejected;
        memset(&rejected, 0, sizeof(rejected));
        const bool ok = track_manifest_parse(bad[i].text, strlen(bad[i].text), &rejected, NULL,
                                             error, sizeof(error));
        check(!ok, "track rejected for %s (error: %s)", bad[i].what, ok ? "(none)" : error);
        track_free(&rejected);
    }
}

/* --------------------------------------------------------------------------------------------- registry */

static const TestScenario kScenarios[] = {
    { "json-parser", "strict JSON reader, escapes, and canonical hash", scenario_json_parser },
    { "vehicle-manifest",
      "issue #29: manifest load, validation, roster round-trip, catalog discovery",
      scenario_vehicle_manifest },
    { "track-format",
      "issue #34: external track load, faithful round-trip, hash stability, validation",
      scenario_track_format },
};

TestScenarioGroup test_content_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kScenarios;
    group.count = sizeof(kScenarios) / sizeof(kScenarios[0]);
    return group;
}

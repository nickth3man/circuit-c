/*
 * content_tests.c — scenarios for the versioned vehicle and track content formats.
 *
 * Six groups of assertions, kept out of the existing scenario files so the registry stays one
 * concern per file:
 *
 *   json-parser          the strict reader and its canonical content hash
 *   vehicle-manifest     issue #29: manifest load, validation, round-trip, and catalog discovery
 *   roster-promotion     issue #31: the promotion checklist gates player-selectable content
 *   roster-content-kind  issue #31: content-kind separation keeps corpus samples out of the roster
 *   vehicle-class        issue #33: class rules and eligibility for the roster cars
 *   track-format         issue #34: external track load, faithful round-trip, and hash stability
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
#include "content/roster_promotion.h"
#include "content/track_manifest.h"
#include "content/vehicle_class.h"
#include "content/vehicle_manifest.h"
#include "dev/dev_params.h"
#include "game/car_roster.h"
#include "game/car_selection.h"
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

/* A complete, reviewable manifest: every promotion-checklist row has its evidence — a stable
 * identity with provenance, authored physics, a class tag, a valid baseline setup, an explicit
 * player-facing content kind, and human+ai eligibility. The roster_promotion scenario starts
 * from this and breaks one row at a time. */
static const char kPromotableManifest[] =
    "{\n"
    "  \"schema\": \"circuit/vehicle\", \"version\": 1,\n"
    "  \"id\": \"promo_rwd\", \"displayName\": \"Promo RWD\",\n"
    "  \"contentVersion\": 1, \"appearanceId\": \"promo_rwd\",\n"
    "  \"contentKind\": \"player-selectable\",\n"
    "  \"classTags\": [\"road\"],\n"
    "  \"controllerEligibility\": [\"human\", \"ai\"],\n"
    "  \"provenance\": { \"source\": \"tests\", \"author\": \"content_tests\" },\n"
    "  \"physics\": { \"body.wheelbase\": 2.60 },\n"
    "  \"setup\": { \"brake.bias_front\": 0.58 }\n"
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


/* ---------------------------------------------------------------------------------- roster promotion */

static void scenario_roster_promotion(void)
{
    char error[256];
    VehicleManifest good;
    check(vehicle_manifest_parse(kPromotableManifest, strlen(kPromotableManifest), &good, error, sizeof(error)), "promotable manifest parses (error: %s)", error);
    VehiclePromotionReport report;
    check(vehicle_promotion_evaluate(&good, &report), "promotable manifest passes the full checklist");
    check(report.count == PROMOTION_CHECK_COUNT, "promotion report has %d rows (got %d)", PROMOTION_CHECK_COUNT, report.count);
    for (int i = 0; i < report.count; i++) {
        check(report.checks[i].pass, "promotion check '%s' passes: %s", report.checks[i].name, report.checks[i].detail);
    }
    struct {
        const char *breakKind;
        const char *manifestText;
        const char *expectedCheck;
    } breaks[] = {
        { "identity-provenance (empty displayName)",
          "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"player-selectable\",\"classTags\":[\"road\"],\"controllerEligibility\":[\"human\",\"ai\"],\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "identity-provenance" },
        { "class-assignment (no tags)",
          "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"Promo RWD\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"player-selectable\",\"controllerEligibility\":[\"human\",\"ai\"],\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "class-assignment" },
        { "review-status (visual-sample)",
          "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"Promo RWD\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"visual-sample\",\"classTags\":[\"road\"],\"controllerEligibility\":[\"human\",\"ai\"],\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "review-status" },
        { "ai-compatibility (human only)",
          "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"Promo RWD\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"player-selectable\",\"classTags\":[\"road\"],\"controllerEligibility\":[\"human\"],\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "ai-compatibility" },
    };
    for (size_t b = 0; b < sizeof(breaks) / sizeof(breaks[0]); b++) {
        VehicleManifest m;
        bool parsed = vehicle_manifest_parse(breaks[b].manifestText, strlen(breaks[b].manifestText), &m, error, sizeof(error));
        check(parsed, "break case '%s' still parses (error: %s)", breaks[b].breakKind, parsed ? "(none)" : error);
        if (!parsed) continue;
        VehiclePromotionReport r;
        const bool ok = vehicle_promotion_evaluate(&m, &r);
        check(!ok, "promotion fails for %s", breaks[b].breakKind);
        check(r.count == PROMOTION_CHECK_COUNT, "report still has %d rows for %s", PROMOTION_CHECK_COUNT, breaks[b].breakKind);
        bool found = false;
        for (int i = 0; i < r.count; i++) {
            if (strcmp(r.checks[i].name, breaks[b].expectedCheck) == 0) {
                check(!r.checks[i].pass, "check '%s' fails for %s: %s", r.checks[i].name, breaks[b].breakKind, r.checks[i].detail);
                found = true;
            }
        }
        check(found, "expected failing check '%s' present for %s", breaks[b].expectedCheck, breaks[b].breakKind);
    }
    {
        VehicleManifest m = good;
        m.definition.contentHash = 0;
        VehiclePromotionReport r;
        check(!vehicle_promotion_evaluate(&m, &r), "promotion fails for physical-data-complete (zero hash)");
        bool found = false;
        for (int i = 0; i < r.count; i++) if (strcmp(r.checks[i].name, "physical-data-complete")==0) { check(!r.checks[i].pass, "physical-data-complete fails for zero hash: %s", r.checks[i].detail); found=true; }
        check(found, "physical-data-complete check present");
    }
    {
        VehicleManifest m = good;
        m.defaultSetup.brakeBiasFront = 10.0f;
        VehiclePromotionReport r;
        check(!vehicle_promotion_evaluate(&m, &r), "promotion fails for default-setup-valid (bad brake bias)");
        bool found = false;
        for (int i = 0; i < r.count; i++) if (strcmp(r.checks[i].name, "default-setup-valid")==0) { check(!r.checks[i].pass, "default-setup-valid fails for bad setup: %s", r.checks[i].detail); found=true; }
        check(found, "default-setup-valid check present");
    }
    {
        VehicleManifest m = good;
        m.definition.spec.bodyHalfWidthM = 2.0f;
        m.definition.spec.lengthOverallM = 20.0f;
        VehiclePromotionReport r;
        check(!vehicle_promotion_evaluate(&m, &r), "promotion fails for collision-dimensions (excessive envelope)");
        bool found = false;
        for (int i = 0; i < r.count; i++) if (strcmp(r.checks[i].name, "collision-dimensions")==0) { check(!r.checks[i].pass, "collision-dimensions fails for excessive envelope: %s", r.checks[i].detail); found=true; }
        check(found, "collision-dimensions check present");
    }
    VehiclePromotionReport empty;
    check(!vehicle_promotion_evaluate(NULL, &empty), "NULL manifest fails promotion");
    check(empty.count == 0, "NULL manifest yields empty report");
}

static void scenario_roster_content_kind(void)
{
    char error[256];
    const char *kinds[] = { "visual-sample", "prototype", "validated", "player-selectable" };
    for (size_t i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++) {
        char text[512];
        snprintf(text, sizeof(text),
                 "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"kind_%zu\",\"displayName\":\"Kind\",\"contentVersion\":1,\"appearanceId\":\"kind_%zu\",\"contentKind\":\"%s\"}",
                 i, i, kinds[i]);
        VehicleManifest m;
        check(vehicle_manifest_parse(text, strlen(text), &m, error, sizeof(error)), "contentKind '%s' parses", kinds[i]);
        if (m.contentKind < VEHICLE_CONTENT_VISUAL_SAMPLE || m.contentKind > VEHICLE_CONTENT_PLAYER_SELECTABLE) {
            check(false, "contentKind '%s' yielded out-of-range enum %d", kinds[i], (int)m.contentKind);
        } else {
            check(strcmp(vehicle_content_kind_name(m.contentKind), kinds[i]) == 0, "contentKind '%s' round-trips through name table", kinds[i]);
        }
    }
    {
        const char *bad = "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"contentVersion\":1,\"appearanceId\":\"bad\",\"contentKind\":\"unknown-kind\"}";
        VehicleManifest m;
        check(!vehicle_manifest_parse(bad, strlen(bad), &m, error, sizeof(error)), "unknown contentKind is rejected (error: %s)", error);
    }
    char dir[512];
    snprintf(dir, sizeof(dir), "%s/_content_kind", TELEMETRY_DIR);
    telemetry_ensure_dir(dir);
    const char *mixIds[] = { "mix_a", "mix_b", "mix_c", "mix_d" };
    const char *mixKinds[] = { "visual-sample", "prototype", "validated", "player-selectable" };
    for (size_t i = 0; i < 4; i++) {
        char path[640], text[512];
        snprintf(path, sizeof(path), "%s/%s.vehicle.json", dir, mixIds[i]);
        snprintf(text, sizeof(text),
                 "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"%s\",\"displayName\":\"%s\",\"contentVersion\":1,\"appearanceId\":\"%s\",\"contentKind\":\"%s\"}",
                 mixIds[i], mixIds[i], mixIds[i], mixKinds[i]);
        FILE *f = fopen(path, "wb");
        if (f != NULL) { fputs(text, f); fclose(f); }
    }
    VehicleCatalog cat;
    check(vehicle_manifest_load_dir(dir, &cat, error, sizeof(error)), "mixed-kind catalog loads (error: %s)", error);
    if (cat.count == 4) {
        check(cat.count == 4, "catalog holds all 4 kinds (got %d)", cat.count);
        int playable = 0;
        for (int i = 0; i < cat.count; i++) if (cat.items[i].contentKind == VEHICLE_CONTENT_PLAYER_SELECTABLE) playable++;
        check(playable == 1, "only 1 of 4 is player-selectable (got %d)", playable);
    }
    vehicle_catalog_free(&cat);
    for (size_t i = 0; i < 4; i++) { char p[640]; snprintf(p, sizeof(p), "%s/%s.vehicle.json", dir, mixIds[i]); remove(p); }
    check(car_roster_count() == 6, "live roster still has 6 player-selectable cars (got %d)", car_roster_count());
}

static void scenario_vehicle_class(void)
{
    char error[256];
    const char *kRoadClass = "{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"road\",\"displayName\":\"Road\"}";
    VehicleClass road;
    check(vehicle_class_parse(kRoadClass, strlen(kRoadClass), &road, error, sizeof(error)), "road class with no rules parses (error: %s)", error);
    VehicleManifest tagged, untagged;
    const char *kTagged = "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"tagged\",\"displayName\":\"Tagged\",\"contentVersion\":1,\"appearanceId\":\"tagged\",\"classTags\":[\"road\"]}";
    const char *kUntagged = "{\"schema\":\"circuit/vehicle\",\"version\":1,\"id\":\"untagged\",\"displayName\":\"Untagged\",\"contentVersion\":1,\"appearanceId\":\"untagged\"}";
    check(vehicle_manifest_parse(kTagged, strlen(kTagged), &tagged, error, sizeof(error)), "tagged manifest parses");
    check(vehicle_manifest_parse(kUntagged, strlen(kUntagged), &untagged, error, sizeof(error)), "untagged manifest parses");
    char detail[256];
    check(vehicle_class_check_eligibility(&road, &tagged, detail, sizeof(detail)), "tagged car is eligible for tag-only class: %s", detail);
    check(!vehicle_class_check_eligibility(&road, &untagged, detail, sizeof(detail)), "untagged car is ineligible for tag-only class: %s", detail);
    const char *kBounded = "{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"light\",\"displayName\":\"Light\",\"description\":\"ignored\",\"rules\":{\"mass_kg\":[600,900],\"peak_torque_nm\":[10,200],\"max_tire_mu\":1.5,\"layouts\":[\"fwd\"]}}";
    VehicleClass light;
    check(vehicle_class_parse(kBounded, strlen(kBounded), &light, error, sizeof(error)), "bounded class parses (error: %s)", error);
    const VehicleManifest *fwdLight = car_roster_manifest(car_roster_find("fwd_light"));
    const VehicleManifest *rwdPower = car_roster_manifest(car_roster_find("rwd_power"));
    if (fwdLight != NULL) {
        VehicleManifest tweaked = *fwdLight;
        bool hasTag = false;
        for (int i = 0; i < tweaked.classTagCount; i++) if (strcmp(tweaked.classTags[i], "light") == 0) hasTag = true;
        if (!hasTag && tweaked.classTagCount < VEHICLE_MANIFEST_MAX_CLASS_TAGS) {
            snprintf(tweaked.classTags[tweaked.classTagCount++], VEHICLE_MANIFEST_CLASS_TAG_CHARS, "light");
        }
        check(vehicle_class_check_eligibility(&light, &tweaked, detail, sizeof(detail)), "fwd_light tweaked to tag 'light' is eligible: %s", detail);
    }
    if (rwdPower != NULL) {
        check(!vehicle_class_check_eligibility(&light, rwdPower, detail, sizeof(detail)), "rwd_power is ineligible for light/FWD class: %s", detail);
    }
    const char *badRules[] = {
        "{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{\"unknown_rule\":1}}",
        "{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{\"layouts\":[\"hover\"]}}",
        "{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{\"mass_kg\":[900,600]}}",
    };
    for (size_t i = 0; i < sizeof(badRules)/sizeof(badRules[0]); i++) {
        VehicleClass bad;
        check(!vehicle_class_parse(badRules[i], strlen(badRules[i]), &bad, error, sizeof(error)), "bad class %zu rejected (error: %s)", i, error);
    }
    char dir2[512];
    snprintf(dir2, sizeof(dir2), "%s/_vehicle_class", TELEMETRY_DIR);
    telemetry_ensure_dir(dir2);
    const char *ids[] = { "zeta", "alpha", "middle" };
    for (size_t i = 0; i < 3; i++) {
        char path[640], text[512];
        snprintf(path, sizeof(path), "%s/%s.vehicle-class.json", dir2, ids[i]);
        snprintf(text, sizeof(text), "{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"%s\",\"displayName\":\"%s\"}", ids[i], ids[i]);
        FILE *f = fopen(path, "wb");
        if (f != NULL) { fputs(text, f); fclose(f); }
    }
    VehicleClassCatalog cat2;
    check(vehicle_class_load_dir(dir2, &cat2, error, sizeof(error)), "class catalog loads (error: %s)", error);
    if (cat2.count == 3) {
        check(strcmp(cat2.items[0].id, "alpha")==0 && strcmp(cat2.items[1].id, "middle")==0 && strcmp(cat2.items[2].id, "zeta")==0, "class catalog sorted by id");
    }
    vehicle_class_catalog_free(&cat2);
    {
        char dup[640]; snprintf(dup, sizeof(dup), "%s/_dup.vehicle-class.json", dir2);
        FILE *f = fopen(dup, "wb"); if (f!=NULL){ fputs("{\"schema\":\"circuit/vehicle-class\",\"version\":1,\"id\":\"alpha\",\"displayName\":\"Dup\"}", f); fclose(f); }
        VehicleClassCatalog dupCat; check(!vehicle_class_load_dir(dir2, &dupCat, error, sizeof(error)), "duplicate class id rejected (error: %s)", error); vehicle_class_catalog_free(&dupCat); remove(dup);
    }
    for (size_t i=0;i<3;i++){ char p[640]; snprintf(p,sizeof(p),"%s/%s.vehicle-class.json", dir2, ids[i]); remove(p); }
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
    /* Schema version migration & version bounds: unsupported versions (e.g. 0, 2) fail with
     * version-specific error diagnostics before a session can start. */
    {
        static const char *kVer0 =
            "{\"schema\":\"circuit/"
            "track\",\"version\":0,\"id\":\"t\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
            "{\"x\":10,\"y\":0,\"halfWidth\":8}]}}";
        static const char *kVer2 =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"t\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
            "{\"x\":10,\"y\":0,\"halfWidth\":8}]}}";
        TrackDefinition v0Track, v2Track;
        memset(&v0Track, 0, sizeof(v0Track));
        memset(&v2Track, 0, sizeof(v2Track));
        const bool ok0 =
            track_manifest_parse(kVer0, strlen(kVer0), &v0Track, NULL, error, sizeof(error));
        check(!ok0 && strstr(error, "version") != NULL,
              "schema version 0 rejected with version diagnostic (%s)", ok0 ? "(none)" : error);
        const bool ok2 =
            track_manifest_parse(kVer2, strlen(kVer2), &v2Track, NULL, error, sizeof(error));
        check(!ok2 && strstr(error, "version") != NULL,
              "schema version 2 rejected with version diagnostic (%s)", ok2 ? "(none)" : error);
    }

    /* Randomized file enumeration & hash stability across file enumeration order. */
    {
        char enumDir[512];
        snprintf(enumDir, sizeof(enumDir), "%s/_track_enum", TELEMETRY_DIR);
        check(telemetry_ensure_dir(enumDir),
              "scratch directory created for track file enumeration");
        static const char *filenames[] = {
            "z_first.track.json",
            "a_second.track.json",
            "m_third.track.json",
        };
        for (size_t i = 0; i < 3; i++) {
            char path[640];
            snprintf(path, sizeof(path), "%s/%s", enumDir, filenames[i]);
            FILE *f = fopen(path, "wb");
            if (f != NULL) {
                fprintf(f,
                        "{\"schema\":\"circuit/track\",\"version\":1,\"id\":\"track_%d\","
                        "\"contentVersion\":\"v1\",\"route\":{\"closed\":true,\"nodes\":["
                        "{\"x\":%d,\"y\":0,\"halfWidth\":8},{\"x\":%d,\"y\":0,\"halfWidth\":8}]"
                        "}}\n",
                        (int)i, (int)i * 10, (int)i * 10 + 5);
                fclose(f);
            }
        }
        uint32_t hashesFirst[3], hashesSecond[3];
        for (size_t i = 0; i < 3; i++) {
            char path[640];
            snprintf(path, sizeof(path), "%s/%s", enumDir, filenames[i]);
            TrackDefinition td;
            memset(&td, 0, sizeof(td));
            const bool ok =
                track_manifest_load(path, &td, &hashesFirst[i], error, sizeof(error));
            check(ok, "randomized enumeration fixture %s loads", filenames[i]);
            track_free(&td);
        }
        for (int i = 2; i >= 0; i--) {
            char path[640];
            snprintf(path, sizeof(path), "%s/%s", enumDir, filenames[(size_t)i]);
            TrackDefinition td;
            memset(&td, 0, sizeof(td));
            const bool ok =
                track_manifest_load(path, &td, &hashesSecond[(size_t)i], error, sizeof(error));
            check(ok, "randomized enumeration fixture %s loads on pass 2",
                  filenames[(size_t)i]);
            track_free(&td);
        }
        for (size_t i = 0; i < 3; i++) {
            check(hashesFirst[i] == hashesSecond[i],
                  "track manifest hash for %s is stable across randomized loading order",
                  filenames[i]);
        }
        /* Clean up scratch files. */
        for (size_t i = 0; i < 3; i++) {
            char path[640];
            snprintf(path, sizeof(path), "%s/%s", enumDir, filenames[i]);
            remove(path);
        }
    }

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
    { "vehicle-manifest", "issue #29: manifest load, validation, roster round-trip, catalog discovery", scenario_vehicle_manifest },
    { "roster-promotion", "issue #31: the promotion checklist gates player-selectable content", scenario_roster_promotion },
    { "roster-content-kind", "issue #31: content-kind separation keeps corpus samples out of the roster", scenario_roster_content_kind },
    { "vehicle-class", "issue #33: class rules and eligibility for the roster cars", scenario_vehicle_class },
    { "track-format", "issue #34: external track load, faithful round-trip, hash stability, validation", scenario_track_format },
};

TestScenarioGroup test_content_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kScenarios;
    group.count = sizeof(kScenarios) / sizeof(kScenarios[0]);
    return group;
}

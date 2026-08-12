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
#include <stdlib.h>
#include <string.h>

#include "test_commands.h"
#include "test_scenarios.h"
#include "support/test_harness.h"

#include "core/config.h"
#include "core/json.h"
#include "core/math_utils.h"
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
    check(vehicle_manifest_parse(kPromotableManifest, strlen(kPromotableManifest), &good, error,
                                 sizeof(error)),
          "promotable manifest parses (error: %s)", error);
    VehiclePromotionReport report;
    check(vehicle_promotion_evaluate(&good, &report),
          "promotable manifest passes the full checklist");
    check(report.count == PROMOTION_CHECK_COUNT, "promotion report has %d rows (got %d)",
          PROMOTION_CHECK_COUNT, report.count);
    for (int i = 0; i < report.count; i++) {
        check(report.checks[i].pass, "promotion check '%s' passes: %s", report.checks[i].name,
              report.checks[i].detail);
    }
    struct {
        const char *breakKind;
        const char *manifestText;
        const char *expectedCheck;
    } breaks[] = {
        { "identity-provenance (empty displayName)",
          "{\"schema\":\"circuit/"
          "vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"\","
          "\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"player-"
          "selectable\",\"classTags\":[\"road\"],\"controllerEligibility\":[\"human\",\"ai\"],"
          "\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{"
          "\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "identity-provenance" },
        { "class-assignment (no tags)",
          "{\"schema\":\"circuit/"
          "vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"Promo "
          "RWD\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"player-"
          "selectable\",\"controllerEligibility\":[\"human\",\"ai\"],\"provenance\":{"
          "\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{\"body.wheelbase\":"
          "2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "class-assignment" },
        { "review-status (visual-sample)",
          "{\"schema\":\"circuit/"
          "vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"Promo "
          "RWD\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"visual-"
          "sample\",\"classTags\":[\"road\"],\"controllerEligibility\":[\"human\",\"ai\"],"
          "\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{"
          "\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "review-status" },
        { "ai-compatibility (human only)",
          "{\"schema\":\"circuit/"
          "vehicle\",\"version\":1,\"id\":\"promo_rwd\",\"displayName\":\"Promo "
          "RWD\",\"contentVersion\":1,\"appearanceId\":\"promo_rwd\",\"contentKind\":\"player-"
          "selectable\",\"classTags\":[\"road\"],\"controllerEligibility\":[\"human\"],"
          "\"provenance\":{\"source\":\"tests\",\"author\":\"content_tests\"},\"physics\":{"
          "\"body.wheelbase\":2.60},\"setup\":{\"brake.bias_front\":0.58}}",
          "ai-compatibility" },
    };
    for (size_t b = 0; b < sizeof(breaks) / sizeof(breaks[0]); b++) {
        VehicleManifest m;
        bool parsed = vehicle_manifest_parse(
            breaks[b].manifestText, strlen(breaks[b].manifestText), &m, error, sizeof(error));
        check(parsed, "break case '%s' still parses (error: %s)", breaks[b].breakKind,
              parsed ? "(none)" : error);
        if (!parsed) continue;
        VehiclePromotionReport r;
        const bool ok = vehicle_promotion_evaluate(&m, &r);
        check(!ok, "promotion fails for %s", breaks[b].breakKind);
        check(r.count == PROMOTION_CHECK_COUNT, "report still has %d rows for %s",
              PROMOTION_CHECK_COUNT, breaks[b].breakKind);
        bool found = false;
        for (int i = 0; i < r.count; i++) {
            if (strcmp(r.checks[i].name, breaks[b].expectedCheck) == 0) {
                check(!r.checks[i].pass, "check '%s' fails for %s: %s", r.checks[i].name,
                      breaks[b].breakKind, r.checks[i].detail);
                found = true;
            }
        }
        check(found, "expected failing check '%s' present for %s", breaks[b].expectedCheck,
              breaks[b].breakKind);
    }
    {
        VehicleManifest m = good;
        m.definition.contentHash = 0;
        VehiclePromotionReport r;
        check(!vehicle_promotion_evaluate(&m, &r),
              "promotion fails for physical-data-complete (zero hash)");
        bool found = false;
        for (int i = 0; i < r.count; i++)
            if (strcmp(r.checks[i].name, "physical-data-complete") == 0) {
                check(!r.checks[i].pass, "physical-data-complete fails for zero hash: %s",
                      r.checks[i].detail);
                found = true;
            }
        check(found, "physical-data-complete check present");
    }
    {
        VehicleManifest m = good;
        m.defaultSetup.brakeBiasFront = 10.0f;
        VehiclePromotionReport r;
        check(!vehicle_promotion_evaluate(&m, &r),
              "promotion fails for default-setup-valid (bad brake bias)");
        bool found = false;
        for (int i = 0; i < r.count; i++)
            if (strcmp(r.checks[i].name, "default-setup-valid") == 0) {
                check(!r.checks[i].pass, "default-setup-valid fails for bad setup: %s",
                      r.checks[i].detail);
                found = true;
            }
        check(found, "default-setup-valid check present");
    }
    {
        VehicleManifest m = good;
        m.definition.spec.bodyHalfWidthM = 2.0f;
        m.definition.spec.lengthOverallM = 20.0f;
        VehiclePromotionReport r;
        check(!vehicle_promotion_evaluate(&m, &r),
              "promotion fails for collision-dimensions (excessive envelope)");
        bool found = false;
        for (int i = 0; i < r.count; i++)
            if (strcmp(r.checks[i].name, "collision-dimensions") == 0) {
                check(!r.checks[i].pass,
                      "collision-dimensions fails for excessive envelope: %s",
                      r.checks[i].detail);
                found = true;
            }
        check(found, "collision-dimensions check present");
    }
    {
        /*
         * Evidence must never look complete when it is not. A car carrying more class tags
         * than the detail buffer can hold gets an explicit "..." rather than a list that
         * silently stops, so a reviewer reading a report can tell it was cut short.
         */
        VehicleManifest m;
        check(vehicle_manifest_parse(kPromotableManifest, strlen(kPromotableManifest), &m,
                                     error, sizeof(error)),
              "promotable manifest parses for the tag-truncation case (error: %s)", error);
        m.classTagCount = 0;
        for (int i = 0; i < VEHICLE_MANIFEST_MAX_CLASS_TAGS; i++) {
            snprintf(m.classTags[m.classTagCount++], VEHICLE_MANIFEST_CLASS_TAG_CHARS,
                     "class_tag_number_%02d_padded_out", i);
        }
        VehiclePromotionReport r;
        (void)vehicle_promotion_evaluate(&m, &r);
        const PromotionCheck *classCheck = NULL;
        for (int i = 0; i < r.count; i++) {
            if (strcmp(r.checks[i].name, "class-assignment") == 0) classCheck = &r.checks[i];
        }
        check(classCheck != NULL, "class-assignment check present for the truncation case");
        if (classCheck != NULL) {
            check(classCheck->pass, "many tags still satisfy class-assignment");
            check(strlen(classCheck->detail) < PROMOTION_CHECK_DETAIL_CHARS,
                  "class-assignment evidence stays inside its buffer (%zu bytes)",
                  strlen(classCheck->detail));
            check(strstr(classCheck->detail, "...") != NULL,
                  "truncated tag evidence is marked with '...' (got '%s')", classCheck->detail);
        }
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
                 "{\"schema\":\"circuit/"
                 "vehicle\",\"version\":1,\"id\":\"kind_%zu\",\"displayName\":\"Kind\","
                 "\"contentVersion\":1,\"appearanceId\":\"kind_%zu\",\"contentKind\":\"%s\"}",
                 i, i, kinds[i]);
        VehicleManifest m;
        check(vehicle_manifest_parse(text, strlen(text), &m, error, sizeof(error)),
              "contentKind '%s' parses", kinds[i]);
        if (m.contentKind < VEHICLE_CONTENT_VISUAL_SAMPLE ||
            m.contentKind > VEHICLE_CONTENT_PLAYER_SELECTABLE) {
            check(false, "contentKind '%s' yielded out-of-range enum %d", kinds[i],
                  (int)m.contentKind);
        } else {
            check(strcmp(vehicle_content_kind_name(m.contentKind), kinds[i]) == 0,
                  "contentKind '%s' round-trips through name table", kinds[i]);
        }
    }
    {
        const char *bad =
            "{\"schema\":\"circuit/"
            "vehicle\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"contentVersion\":"
            "1,\"appearanceId\":\"bad\",\"contentKind\":\"unknown-kind\"}";
        VehicleManifest m;
        check(!vehicle_manifest_parse(bad, strlen(bad), &m, error, sizeof(error)),
              "unknown contentKind is rejected (error: %s)", error);
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
                 "{\"schema\":\"circuit/"
                 "vehicle\",\"version\":1,\"id\":\"%s\",\"displayName\":\"%s\","
                 "\"contentVersion\":1,\"appearanceId\":\"%s\",\"contentKind\":\"%s\"}",
                 mixIds[i], mixIds[i], mixIds[i], mixKinds[i]);
        FILE *f = fopen(path, "wb");
        if (f != NULL) {
            fputs(text, f);
            fclose(f);
        }
    }
    VehicleCatalog cat;
    check(vehicle_manifest_load_dir(dir, &cat, error, sizeof(error)),
          "mixed-kind catalog loads (error: %s)", error);
    /* The count is asserted rather than assumed: guarding the assertion with the same condition
     * made the check unreachable when it mattered, so a short catalog would have passed silently. */
    check(cat.count == 4, "catalog holds all 4 kinds (got %d)", cat.count);
    int playable = 0;
    for (int i = 0; i < cat.count; i++)
        if (cat.items[i].contentKind == VEHICLE_CONTENT_PLAYER_SELECTABLE) playable++;
    check(playable == 1, "only 1 of 4 is player-selectable (got %d)", playable);
    vehicle_catalog_free(&cat);
    for (size_t i = 0; i < 4; i++) {
        char p[640];
        snprintf(p, sizeof(p), "%s/%s.vehicle.json", dir, mixIds[i]);
        remove(p);
    }
    check(car_roster_count() == 6, "live roster still has 6 player-selectable cars (got %d)",
          car_roster_count());
}

static void scenario_vehicle_class(void)
{
    char error[256];
    const char *kRoadClass =
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"road\",\"displayName\":\"Road\"}";
    VehicleClass road;
    check(vehicle_class_parse(kRoadClass, strlen(kRoadClass), &road, error, sizeof(error)),
          "road class with no rules parses (error: %s)", error);
    VehicleManifest tagged, untagged;
    const char *kTagged =
        "{\"schema\":\"circuit/"
        "vehicle\",\"version\":1,\"id\":\"tagged\",\"displayName\":\"Tagged\","
        "\"contentVersion\":1,\"appearanceId\":\"tagged\",\"classTags\":[\"road\"]}";
    const char *kUntagged = "{\"schema\":\"circuit/"
                            "vehicle\",\"version\":1,\"id\":\"untagged\",\"displayName\":"
                            "\"Untagged\",\"contentVersion\":1,\"appearanceId\":\"untagged\"}";
    check(vehicle_manifest_parse(kTagged, strlen(kTagged), &tagged, error, sizeof(error)),
          "tagged manifest parses");
    check(vehicle_manifest_parse(kUntagged, strlen(kUntagged), &untagged, error, sizeof(error)),
          "untagged manifest parses");
    char detail[256];
    check(vehicle_class_check_eligibility(&road, &tagged, detail, sizeof(detail)),
          "tagged car is eligible for tag-only class: %s", detail);
    check(!vehicle_class_check_eligibility(&road, &untagged, detail, sizeof(detail)),
          "untagged car is ineligible for tag-only class: %s", detail);
    const char *kBounded =
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"light\",\"displayName\":\"Light\","
        "\"description\":\"ignored\",\"rules\":{\"mass_kg\":[600,900],\"peak_torque_nm\":[10,"
        "200],\"max_tire_mu\":1.5,\"layouts\":[\"fwd\"]}}";
    VehicleClass light;
    check(vehicle_class_parse(kBounded, strlen(kBounded), &light, error, sizeof(error)),
          "bounded class parses (error: %s)", error);
    /*
     * Eligibility is asserted against manifests authored right here, not against roster cars.
     * A roster-based assertion silently depends on two specific cars staying inside the class
     * bounds, so an ordinary physics content bump would fail this scenario even though the
     * class-rule logic it is meant to cover never changed. Authored fixtures pin every value a
     * rule reads — mass, peak torque, tire mu, layout — so the only thing that can move the
     * verdict is the rule evaluation itself. (The shipped roster is separately checked against
     * the shipped classes by the `roster-gate` gameplay scenario.)
     */
    const char *kFixture =
        "{\"schema\":\"circuit/"
        "vehicle\",\"version\":1,\"id\":\"fixture\",\"displayName\":\"Fixture\","
        "\"contentVersion\":1,\"appearanceId\":\"fixture\",\"classTags\":[\"light\"]}";
    VehicleManifest inside;
    check(vehicle_manifest_parse(kFixture, strlen(kFixture), &inside, error, sizeof(error)),
          "class fixture manifest parses (error: %s)", error);
    /* Comfortably inside every bound of the "light" class above. */
    inside.definition.spec.massKg = 750.0f;
    inside.definition.spec.drivetrainLayout = (float)DRIVE_LAYOUT_FWD;
    inside.definition.spec.tireMuLatFront = 1.2f;
    inside.definition.spec.tireMuLatRear = 1.2f;
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++)
        inside.definition.spec.engineTorqueCurveNm[i] = 0.0f;
    inside.definition.spec.engineTorqueCurveNm[0] = 150.0f;
    check(vehicle_class_check_eligibility(&light, &inside, detail, sizeof(detail)),
          "a car inside every 'light' bound is eligible: %s", detail);

    /* One rule at a time, so each failure names the rule that produced it. */
    {
        VehicleManifest heavy = inside;
        heavy.definition.spec.massKg = 1500.0f; /* above mass_kg max 900 */
        check(!vehicle_class_check_eligibility(&light, &heavy, detail, sizeof(detail)),
              "mass above the class maximum is ineligible: %s", detail);
    }
    {
        VehicleManifest torquey = inside;
        torquey.definition.spec.engineTorqueCurveNm[0] =
            400.0f; /* above peak_torque_nm max 200 */
        check(!vehicle_class_check_eligibility(&light, &torquey, detail, sizeof(detail)),
              "peak torque above the class maximum is ineligible: %s", detail);
    }
    {
        VehicleManifest sticky = inside;
        sticky.definition.spec.tireMuLatRear = 1.9f; /* above max_tire_mu 1.5 */
        check(!vehicle_class_check_eligibility(&light, &sticky, detail, sizeof(detail)),
              "tire grip above the class maximum is ineligible: %s", detail);
    }
    {
        VehicleManifest rwd = inside;
        rwd.definition.spec.drivetrainLayout = (float)DRIVE_LAYOUT_RWD; /* layouts = ["fwd"] */
        check(!vehicle_class_check_eligibility(&light, &rwd, detail, sizeof(detail)),
              "a layout outside the class whitelist is ineligible: %s", detail);
    }
    {
        /* The tag is required as well as the numbers: satisfying every rule without carrying
         * the class id must not grant membership. */
        VehicleManifest untaggedButInside = inside;
        untaggedButInside.classTagCount = 0;
        check(!vehicle_class_check_eligibility(&light, &untaggedButInside, detail,
                                               sizeof(detail)),
              "rules alone do not grant membership without the tag: %s", detail);
    }
    const char *badRules[] = {
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{"
        "\"unknown_rule\":1}}",
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{"
        "\"layouts\":[\"hover\"]}}",
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{"
        "\"mass_kg\":[900,600]}}",
        /* Finite as a JSON double, but not representable as the float the rule is stored in.
         * Narrowing it would record an infinity and quietly turn an authored upper bound into
         * "unconstrained" — the file must be rejected instead. */
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{"
        "\"mass_kg\":[0,1e100]}}",
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{"
        "\"peak_torque_nm\":[-1e300,200]}}",
        "{\"schema\":\"circuit/"
        "vehicle-class\",\"version\":1,\"id\":\"bad\",\"displayName\":\"Bad\",\"rules\":{"
        "\"max_tire_mu\":1e100}}",
    };
    for (size_t i = 0; i < sizeof(badRules) / sizeof(badRules[0]); i++) {
        VehicleClass bad;
        check(
            !vehicle_class_parse(badRules[i], strlen(badRules[i]), &bad, error, sizeof(error)),
            "bad class %zu rejected (error: %s)", i, error);
    }
    char dir2[512];
    snprintf(dir2, sizeof(dir2), "%s/_vehicle_class", TELEMETRY_DIR);
    telemetry_ensure_dir(dir2);
    const char *ids[] = { "zeta", "alpha", "middle" };
    for (size_t i = 0; i < 3; i++) {
        char path[640], text[512];
        snprintf(path, sizeof(path), "%s/%s.vehicle-class.json", dir2, ids[i]);
        snprintf(text, sizeof(text),
                 "{\"schema\":\"circuit/"
                 "vehicle-class\",\"version\":1,\"id\":\"%s\",\"displayName\":\"%s\"}",
                 ids[i], ids[i]);
        FILE *f = fopen(path, "wb");
        if (f != NULL) {
            fputs(text, f);
            fclose(f);
        }
    }
    VehicleClassCatalog cat2;
    check(vehicle_class_load_dir(dir2, &cat2, error, sizeof(error)),
          "class catalog loads (error: %s)", error);
    if (cat2.count == 3) {
        check(strcmp(cat2.items[0].id, "alpha") == 0 &&
                  strcmp(cat2.items[1].id, "middle") == 0 &&
                  strcmp(cat2.items[2].id, "zeta") == 0,
              "class catalog sorted by id");
    }
    vehicle_class_catalog_free(&cat2);
    {
        char dup[640];
        snprintf(dup, sizeof(dup), "%s/_dup.vehicle-class.json", dir2);
        FILE *f = fopen(dup, "wb");
        if (f != NULL) {
            fputs("{\"schema\":\"circuit/"
                  "vehicle-class\",\"version\":1,\"id\":\"alpha\",\"displayName\":\"Dup\"}",
                  f);
            fclose(f);
        }
        VehicleClassCatalog dupCat;
        check(!vehicle_class_load_dir(dir2, &dupCat, error, sizeof(error)),
              "duplicate class id rejected (error: %s)", error);
        vehicle_class_catalog_free(&dupCat);
        remove(dup);
    }
    for (size_t i = 0; i < 3; i++) {
        char p[640];
        snprintf(p, sizeof(p), "%s/%s.vehicle-class.json", dir2, ids[i]);
        remove(p);
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
    /* Schema version migration & version bounds: unsupported versions (e.g. 0, 3) fail with
     * version-specific error diagnostics before a session can start. Version 2 is now accepted. */
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
        static const char *kVer3 =
            "{\"schema\":\"circuit/"
            "track\",\"version\":3,\"id\":\"t\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
            "{\"x\":10,\"y\":0,\"halfWidth\":8}]}}";
        TrackDefinition v0Track, v2Track, v3Track;
        memset(&v0Track, 0, sizeof(v0Track));
        memset(&v2Track, 0, sizeof(v2Track));
        memset(&v3Track, 0, sizeof(v3Track));
        const bool ok0 =
            track_manifest_parse(kVer0, strlen(kVer0), &v0Track, NULL, error, sizeof(error));
        check(!ok0 && strstr(error, "version") != NULL,
              "schema version 0 rejected with version diagnostic (%s)", ok0 ? "(none)" : error);
        const bool ok2 =
            track_manifest_parse(kVer2, strlen(kVer2), &v2Track, NULL, error, sizeof(error));
        check(ok2, "schema version 2 accepted (error: %s)", ok2 ? "(none)" : error);
        if (ok2) track_free(&v2Track);
        const bool ok3 =
            track_manifest_parse(kVer3, strlen(kVer3), &v3Track, NULL, error, sizeof(error));
        check(!ok3 && strstr(error, "version") != NULL,
              "schema version 3 rejected with version diagnostic (%s)", ok3 ? "(none)" : error);
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
        { "{\"schema\":\"circuit/"
          "track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":0}]}}",
          "non-positive node halfWidth" },
        { "{\"schema\":\"circuit/"
          "track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":false,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]}}",
          "open route (unsupported in v1)" },
        { "{\"schema\":\"circuit/"
          "track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8,"
          "\"surface\":\"ice\"}]}}",
          "unknown surface name" },
        { "{\"schema\":\"circuit/"
          "track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]},"
          "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":1,"
          "\"halfWidth\":10}]}",
          "non-unit checkpoint forward vector" },
        { "{\"schema\":\"circuit/"
          "track\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]},\"extra\":1}",
          "unknown top-level key" },
        { "{\"schema\":\"circuit/"
          "other\",\"version\":1,\"id\":\"t\",\"contentVersion\":\"v1\","
          "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},"
          "{\"x\":5,\"y\":0,\"halfWidth\":8}]}}",
          "wrong schema" },
        { "{\"schema\":\"circuit/"
          "track\",\"version\":3,\"id\":\"t\",\"contentVersion\":\"v1\","
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

static void scenario_track_markers(void)
{
    char error[256];

    /* Closed circuit: 4 gates around a loop, lap closes after 4. */
    {
        const char *closedJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"closed_test\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":10,"
            "\"y\":0,\"halfWidth\":8},"
            "{\"x\":10,\"y\":10,\"halfWidth\":8},{\"x\":0,\"y\":10,\"halfWidth\":8}]},"
            "\"checkpoints\":["
            "{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10},"
            "{\"x\":10,\"y\":0,\"forwardX\":0,\"forwardY\":1,\"halfWidth\":10},"
            "{\"x\":10,\"y\":10,\"forwardX\":-1,\"forwardY\":0,\"halfWidth\":10},"
            "{\"x\":0,\"y\":10,\"forwardX\":0,\"forwardY\":-1,\"halfWidth\":10}"
            "],\"sectors\":[{\"x\":10,\"y\":0,\"forwardX\":0,\"forwardY\":1,\"halfWidth\":10}],"
            "\"startFinish\":{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10},"
            "\"grid\":[{\"x\":-2,\"y\":-2,\"heading\":0},{\"x\":-2,\"y\":2,\"heading\":0}],"
            "\"pit\":{\"entry\":{\"x\":5,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":6}"
            ","
            "\"exit\":{\"x\":8,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":6},"
            "\"speedLine\":{\"x\":6,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":6},"
            "\"serviceBoxes\":[{\"minX\":-5,\"minY\":-5,\"maxX\":5,\"maxY\":5}]}}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(closedJson, strlen(closedJson), &track, NULL,
                                             error, sizeof(error));
        check(ok, "closed circuit with sectors/startFinish/grid/pit parses (error: %s)",
              ok ? "(none)" : error);
        if (ok) {
            check(track.routeClosed, "closed circuit reports routeClosed true");
            check(track.sectorMarkerCount == 1, "one sector marker present");
            check(track.hasStartFinish, "startFinish present");
            check(track.gridSlotCount == 2, "two grid slots present");
            check(track_pit_has_geometry(&track), "pit geometry detected");
            check(track_validate_grid_slots(&track, error, sizeof(error)),
                  "grid slots valid (error: %s)", error);
            /* Lap validation and sector timing independent: advance lap and sector separately. */
            RacerProgress prog;
            memset(&prog, 0, sizeof(prog));
            track_reset_progress_at(&prog, &track, 0);
            Vector2 prev = { -5, 0 }, curr = { 5, 0 };
            TrackCheckpointEvent cev = track_update_checkpoints(&track, &prog, prev, curr);
            check(!cev.crossed || cev.index != -1, "first checkpoint crossing observed");
            if (cev.crossed && !cev.outOfOrder) {
                check(prog.nextCheckpoint == 1 || prog.nextCheckpoint == 2,
                      "nextCheckpoint advanced after first gate (got %d)", prog.nextCheckpoint);
            }
            /* Sector independent: crossing sector gate should not advance route. */
            RacerProgress prog2;
            memset(&prog2, 0, sizeof(prog2));
            track_reset_progress_at(&prog2, &track, 0);
            prev = (Vector2){ 10, -5 };
            curr = (Vector2){ 10, 5 };
            TrackSectorEvent sev = track_update_sectors(&track, &prog2, prev, curr);
            check(sev.crossed && sev.index == 0, "sector crossing detected");
            check(prog2.nextCheckpoint == 1,
                  "sector crossing did not advance route checkpoint (next %d)",
                  prog2.nextCheckpoint);
            /* Reverse crossing should not advance. */
            RacerProgress progRev;
            memset(&progRev, 0, sizeof(progRev));
            track_reset_progress_at(&progRev, &track, 0);
            prev = (Vector2){ 5, 0 };
            curr = (Vector2){ -5, 0 }; /* reverse over gate 0 */
            TrackCheckpointEvent rev = track_update_checkpoints(&track, &progRev, prev, curr);
            check(!rev.crossed, "reverse crossing does not score");
            check(progRev.nextCheckpoint == 1,
                  "reverse did not advance nextCheckpoint (got %d)", progRev.nextCheckpoint);
            /* Skipped checkpoint marks lapInvalid. */
            RacerProgress progSkip;
            memset(&progSkip, 0, sizeof(progSkip));
            track_reset_progress_at(&progSkip, &track, 0);
            /* Directly jump to gate 2 (skip gate 1): at (10,10) facing -1,0 - need motion -X through x=10 */
            prev = (Vector2){ 15, 10 };
            curr = (Vector2){ 5, 10 };
            TrackCheckpointEvent skip = track_update_checkpoints(&track, &progSkip, prev, curr);
            check(skip.outOfOrder, "skipped checkpoint reported outOfOrder");
            check(progSkip.lapInvalid, "required skipped checkpoint marks lapInvalid");
            /* Two entrants on same tick, different orders: isolated. */
            RacerProgress a, b;
            memset(&a, 0, sizeof(a));
            memset(&b, 0, sizeof(b));
            track_reset_progress_at(&a, &track, 0);
            track_reset_progress_at(&b, &track, 0);
            /* A crosses expected gate 1, B crosses out-of-order gate 2 on same tick. */
            Vector2 aPrev = { 10, -5 }, aCurr = { 10, 5 }; /* gate 1 expected */
            Vector2 bPrev = { 15, 10 }, bCurr = { 5, 10 }; /* gate 2 out-of-order */
            TrackCheckpointEvent aEv = track_update_checkpoints(&track, &a, aPrev, aCurr);
            TrackCheckpointEvent bEv = track_update_checkpoints(&track, &b, bPrev, bCurr);
            check(!aEv.outOfOrder && aEv.crossed, "entrant A in-order");
            check(bEv.outOfOrder, "entrant B out-of-order");
            check(a.nextCheckpoint == 2, "entrant A advanced (next %d)", a.nextCheckpoint);
            check(b.nextCheckpoint == 1, "entrant B did not advance (next %d)",
                  b.nextCheckpoint);
            check(b.lapInvalid, "entrant B lapInvalid without affecting A");
            check(!a.lapInvalid, "entrant A lap remains valid");
            track_free(&track);
        }
    }

    /* Open point-to-point: closed false, finishes without wrapping. */
    {
        const char *openJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"open_test\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":false,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":"
            "10,\"y\":0,\"halfWidth\":8},"
            "{\"x\":20,\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":["
            "{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10},"
            "{\"x\":10,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10},"
            "{\"x\":20,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}"
            "]}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(openJson, strlen(openJson), &track, NULL, error,
                                             sizeof(error));
        check(ok, "open point-to-point parses (error: %s)", ok ? "(none)" : error);
        if (ok) {
            check(!track.routeClosed, "open route reports routeClosed false");
            RacerProgress prog;
            memset(&prog, 0, sizeof(prog));
            track_reset_progress_at(&prog, &track, 0);
            check(prog.nextCheckpoint == 1, "open start next is 1");
            /* Cross gates 1 and 2 in order, finish at end without wrapping. */
            Vector2 p0 = { 5, 0 }, c0 = { 15, 0 }; /* through gate 1 at x=10 */
            TrackCheckpointEvent e1 = track_update_checkpoints(&track, &prog, p0, c0);
            check(e1.crossed && !e1.outOfOrder, "open gate 1 in-order");
            check(prog.nextCheckpoint == 2, "open next after gate1 is 2");
            Vector2 p1 = { 15, 0 }, c1 = { 25, 0 }; /* through gate 2 at x=20 */
            TrackCheckpointEvent e2 = track_update_checkpoints(&track, &prog, p1, c1);
            check(e2.crossed && e2.lapCompleted, "open final gate completes route");
            check(prog.routeFinished, "open routeFinished true");
            check(prog.nextCheckpoint == track.checkpointCount,
                  "open sentinel nextCheckpoint == count (%d)", prog.nextCheckpoint);
            /* Further crossings after finish do nothing. */
            TrackCheckpointEvent e3 = track_update_checkpoints(&track, &prog, p1, c1);
            check(!e3.crossed, "no crossing after open finished");
            check(track_has_required_markers_for_mode(&track, "sprint", error, sizeof(error)),
                  "open route satisfies sprint mode");
            check(!track_has_required_markers_for_mode(&track, "race", error, sizeof(error)),
                  "open without grid fails race check");
            track_free(&track);
        }
    }

    /* Grid overlap validation. */
    {
        const char *overlapJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"overlap_test\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{"
            "\"x\":10,\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,"
            "\"halfWidth\":10}],"
            "\"grid\":[{\"x\":0,\"y\":0,\"heading\":0},{\"x\":1,\"y\":0,\"heading\":0}]"
            "}"; /* 1m apart < 3m */
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(overlapJson, strlen(overlapJson), &track, NULL,
                                             error, sizeof(error));
        check(!ok, "overlapping grid slots rejected (error: %s)", ok ? "(none)" : error);
        if (!ok) {
            check(strstr(error, "overlap") != NULL || strstr(error, "grid") != NULL,
                  "grid overlap diagnostic mentions grid/overlap");
        }
        track_free(&track);
    }

    /* Pit geometry authorable without enabling pit rules. */
    {
        const char *pitJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"pit_test\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":10,"
            "\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}]"
            ","
            "\"pit\":{\"entry\":{\"x\":2,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":6}"
            ","
            "\"exit\":{\"x\":8,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":6},"
            "\"speedLine\":{\"x\":5,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":6},"
            "\"serviceBoxes\":[{\"minX\":4,\"minY\":1,\"maxX\":6,\"maxY\":3}]}}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok =
            track_manifest_parse(pitJson, strlen(pitJson), &track, NULL, error, sizeof(error));
        check(ok, "pit geometry parses (error: %s)", ok ? "(none)" : error);
        if (ok) {
            check(track_pit_has_geometry(&track), "pit geometry present");
            check(track_point_in_service_box(&track, (Vector2){ 5, 2 }),
                  "point inside service box");
            check(!track_point_in_service_box(&track, (Vector2){ 0, 0 }),
                  "point outside service box not inside");
            /* Pit geometry does not automatically fail non-pit modes. */
            check(
                track_has_required_markers_for_mode(&track, "time_trial", error, sizeof(error)),
                "time_trial with pit still valid");
            track_free(&track);
        }
    }

    /* Sector boundary: crossing sector does not affect lap. */
    {
        const char *sectorJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"sector_test\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":10,"
            "\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10},"
            "{\"x\":10,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}],"
            "\"sectors\":[{\"x\":5,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}]}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(sectorJson, strlen(sectorJson), &track, NULL,
                                             error, sizeof(error));
        check(ok, "sector boundary parses");
        if (ok) {
            RacerProgress prog;
            memset(&prog, 0, sizeof(prog));
            track_reset_progress_at(&prog, &track, 0);
            /* Sector at x=5 facing 1,0: motion 1,0 through x=5, y=0 */
            Vector2 prev = { 0, 0 };
            Vector2 curr = { 10.1f, 0 };
            /* This will cross both checkpoint 1 and sector; test isolation. */
            TrackCheckpointEvent cev = track_update_checkpoints(&track, &prog, prev, curr);
            TrackSectorEvent sev = track_update_sectors(&track, &prog, prev, curr);
            check(cev.crossed, "checkpoint crossed");
            check(sev.crossed, "sector crossed");
            check(sev.index == 0, "sector index 0");
            check(prog.nextCheckpoint == 0, "checkpoint advanced to 0");
            track_free(&track);
        }
    }

    /* Schema errors for missing/ambiguous required markers by session mode. */
    {
        const char *noGridJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"nogrid\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":10,"
            "\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}]"
            "}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(noGridJson, strlen(noGridJson), &track, NULL,
                                             error, sizeof(error));
        check(ok, "track without grid parses");
        if (ok) {
            check(!track_has_required_markers_for_mode(&track, "race", error, sizeof(error)),
                  "race without grid fails required check");
            check(strstr(error, "grid") != NULL, "race grid missing diagnostic mentions grid");
            check(
                track_has_required_markers_for_mode(&track, "time_trial", error, sizeof(error)),
                "time_trial without grid still valid");
            check(!track_has_required_markers_for_mode(&track, "sprint", error, sizeof(error)),
                  "sprint on closed without open fails");
            track_free(&track);
        }
    }
    {
        const char *badSectorJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"badsector\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":10,"
            "\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}]"
            ","
            "\"sectors\":[{\"x\":0,\"y\":0,\"forwardX\":0.5,\"forwardY\":0,\"halfWidth\":10}]}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(badSectorJson, strlen(badSectorJson), &track, NULL,
                                             error, sizeof(error));
        check(!ok, "non-unit sector forward vector rejected");
        track_free(&track);
    }
    {
        const char *badGridHeadingJson =
            "{\"schema\":\"circuit/"
            "track\",\"version\":2,\"id\":\"badgrid\",\"contentVersion\":\"v1\","
            "\"route\":{\"closed\":true,\"nodes\":[{\"x\":0,\"y\":0,\"halfWidth\":8},{\"x\":10,"
            "\"y\":0,\"halfWidth\":8}]},"
            "\"checkpoints\":[{\"x\":0,\"y\":0,\"forwardX\":1,\"forwardY\":0,\"halfWidth\":10}]"
            ","
            "\"grid\":[{\"x\":0,\"y\":0}]}";
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        const bool ok = track_manifest_parse(badGridHeadingJson, strlen(badGridHeadingJson),
                                             &track, NULL, error, sizeof(error));
        check(!ok, "grid missing heading rejected");
        track_free(&track);
    }
}

static void scenario_track_migration(void)
{
    char error[256];
    /* The catalog discovers the four built-in tracks from data/tracks/. */
    TrackCatalog catalog;
    memset(&catalog, 0, sizeof(catalog));
    check(track_catalog_load(NULL, &catalog, error, sizeof(error)),
          "track catalog loads from " TRACK_CATALOG_DIR " (error: %s)",
          catalog.count > 0 ? "(none)" : error);
    check(catalog.count == 4, "catalog holds four tracks (got %d)", catalog.count);
    if (catalog.count != 4) {
        track_catalog_free(&catalog);
        return;
    }
    /* Id-sorted catalog order is independent of filesystem enumeration. */
    const char *expectedIds[] = { "chicane", "parking_lot", "sprint", "technical" };
    for (int i = 0; i < 4 && i < catalog.count; i++) {
        check(strcmp(catalog.entries[i].definition.id, expectedIds[i]) == 0,
              "catalog entry %d is %s (got %s)", i, expectedIds[i],
              catalog.entries[i].definition.id);
    }
    for (int i = 0; i < catalog.count; i++) {
        check(track_catalog_find(&catalog, catalog.entries[i].definition.id) == i,
              "catalog find returns %d for %s", i, catalog.entries[i].definition.id);
    }
    check(track_catalog_find(&catalog, "does_not_exist") == -1,
          "catalog find returns -1 for missing id");

    /* Legacy vs loaded equivalence: each compiled-in loader must produce the same geometry
     * hash as the catalog entry with the same id. This is the temporary comparison that proves
     * the migration preserves surface queries, barriers, checkpoints, lap completion, rendering
     * and AI validation before the legacy geometry is removed. */
    struct {
        const char *id;
        void (*load)(TrackDefinition *);
    } legacy[] = {
        { "parking_lot", track_init },
        { "chicane", track_load_chicane },
        { "sprint", track_load_sprint },
        { "technical", track_load_technical },
    };
    for (size_t i = 0; i < sizeof(legacy) / sizeof(legacy[0]); i++) {
        TrackDefinition compiled;
        memset(&compiled, 0, sizeof(compiled));
        legacy[i].load(&compiled);
        const uint32_t compiledHash = track_geometry_hash(&compiled);
        const int idx = track_catalog_find(&catalog, legacy[i].id);
        check(idx >= 0, "catalog contains legacy id %s", legacy[i].id);
        if (idx >= 0) {
            const TrackDefinition *loaded = &catalog.entries[idx].definition;
            const uint32_t loadedHash = track_geometry_hash(loaded);
            check(loadedHash == compiledHash,
                  "legacy vs catalog geometry hash for %s matches (%08x)", legacy[i].id,
                  loadedHash);
            check(loaded->count == compiled.count, "%s node count matches (%d)", legacy[i].id,
                  loaded->count);
            check(loaded->checkpointCount == compiled.checkpointCount,
                  "%s checkpoint count matches (%d)", legacy[i].id, loaded->checkpointCount);
            bool nodesEqual = true;
            for (int n = 0; n < loaded->count && n < compiled.count; n++) {
                if (loaded->nodes[n].centerM.x != compiled.nodes[n].centerM.x ||
                    loaded->nodes[n].centerM.y != compiled.nodes[n].centerM.y ||
                    loaded->nodes[n].halfWidthM != compiled.nodes[n].halfWidthM ||
                    loaded->nodes[n].runoffHalfWidthM != compiled.nodes[n].runoffHalfWidthM ||
                    loaded->nodes[n].surfaceId != compiled.nodes[n].surfaceId) {
                    nodesEqual = false;
                    break;
                }
            }
            check(nodesEqual, "%s node arrays are bit-identical", legacy[i].id);
            /* Sprint and technical must be explicit content, not a runtime transform of the
             * chicane. The catalog entry's id/version comes from the file, and the hash proves
             * the file's geometry is the same as the legacy transform — so the file is a reviewed
             * copy, not a derivation performed at startup. */
            if (strcmp(legacy[i].id, "sprint") == 0 || strcmp(legacy[i].id, "technical") == 0) {
                check(strcmp(loaded->id, legacy[i].id) == 0, "%s catalog id is explicit (%s)",
                      legacy[i].id, loaded->id);
            }
        }
        track_free(&compiled);
    }

    /* Missing and corrupt track handling: stable-id discovery must fail loudly, not silently
     * skip a validation step. */
    {
        TrackDefinition missing;
        memset(&missing, 0, sizeof(missing));
        const bool ok =
            track_load_by_id("does_not_exist", &missing, NULL, error, sizeof(error));
        check(!ok, "missing track id is rejected (error: %s)", ok ? "(none)" : error);
        track_free(&missing);
    }
    {
        TrackDefinition bad;
        memset(&bad, 0, sizeof(bad));
        const bool ok = track_load_by_id("Chicane", &bad, NULL, error, sizeof(error));
        check(!ok, "uppercase track id is rejected (error: %s)", ok ? "(none)" : error);
        track_free(&bad);
    }
    {
        TrackCatalog badCatalog;
        memset(&badCatalog, 0, sizeof(badCatalog));
        const bool ok =
            track_catalog_load("data/tracks/does_not_exist", &badCatalog, error, sizeof(error));
        check(!ok, "catalog load from missing directory is rejected (error: %s)",
              ok ? "(none)" : error);
        track_catalog_free(&badCatalog);
    }
    /* Boundary coverage for fixed TRACK_ID_CHARS storage (31-char max). The public
     * rule for tracks is 0,30 additional chars, not 0,62, so exercise the limit. */
    {
        char id31[32] = { 0 };
        memset(id31, 'a', 31);
        id31[31] = '\0';
        check(track_manifest_id_is_valid(id31), "31-char track id is valid");
        char id32[33] = { 0 };
        memset(id32, 'a', 32);
        id32[32] = '\0';
        check(!track_manifest_id_is_valid(id32), "32-char track id is rejected (too long)");
        char id63[64] = { 0 };
        memset(id63, 'a', 63);
        id63[63] = '\0';
        check(!track_manifest_id_is_valid(id63),
              "63-char track id is rejected for track storage");
    }

    track_catalog_free(&catalog);
}

/* ------------------------------------------------------------------ issue #38: localization */

/*
 * A deliberately hostile closed layout: two straights 6 m apart carrying traffic in OPPOSITE
 * directions, joined by a turn at each end.
 *
 *     y=6   <-------------------------  segments 7..11 (returning, -X)
 *     y=0   ------------------------->  segments 0..4  (outbound, +X)
 *
 * With a 2 m racing half-width and a 3 m barrier, a car at y = 3 is exactly equidistant from
 * both strands and a car at y = 3.2 is CLOSER to the one going the other way. That is the whole
 * failure this issue exists to remove: a nearest-segment scan puts such a car on the wrong
 * strand, which reverses its heading error, reverses its progress, and takes its lap count with
 * it. Every localization check below is anchored to this geometry.
 */
#define LOC_TRACK_NODES 14
#define LOC_HALF_WIDTH_M 2.0f
#define LOC_RUNOFF_M 3.0f

static void loc_build_track(TrackDefinition *track, bool closed)
{
    memset(track, 0, sizeof(*track));
    static const Vector2 kCenters[LOC_TRACK_NODES] = {
        { 0.0f, 0.0f },   { 20.0f, 0.0f },  { 40.0f, 0.0f },  { 60.0f, 0.0f },  { 80.0f, 0.0f },
        { 100.0f, 0.0f }, { 110.0f, 3.0f }, { 100.0f, 6.0f }, { 80.0f, 6.0f },  { 60.0f, 6.0f },
        { 40.0f, 6.0f },  { 20.0f, 6.0f },  { 0.0f, 6.0f },   { -10.0f, 3.0f },
    };
    track->count = LOC_TRACK_NODES;
    track->nodes = (TrackNode *)calloc((size_t)track->count, sizeof(TrackNode));
    if (track->nodes == NULL) {
        track->count = 0;
        return;
    }
    for (int i = 0; i < LOC_TRACK_NODES; i++) {
        track->nodes[i] =
            (TrackNode){ kCenters[i], LOC_HALF_WIDTH_M, SURFACE_ASPHALT, LOC_RUNOFF_M };
    }
    track->routeClosed = closed;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    track_build_checkpoints_from_nodes(track);
    snprintf(track->id, sizeof(track->id), "%s", "loc_test");
    snprintf(track->version, sizeof(track->version), "%s", "v1");
}

/* Advance one racer by a single tick from prev to curr at the given heading. */
static TrackProgressEvent loc_step(const TrackDefinition *track, RacerProgress *progress,
                                   Vector2 prevM, Vector2 currM, float headingRad)
{
    return track_update_progress(track, progress, prevM, currM, headingRad, FIXED_DT_S);
}

static void scenario_route_localization(void)
{
    TrackDefinition track;
    loc_build_track(&track, true);
    check(track.count == LOC_TRACK_NODES, "localization fixture built (%d nodes)", track.count);
    if (track.count != LOC_TRACK_NODES) return;

    const float lengthM = track_length_m(&track);
    check(lengthM > 200.0f, "fixture route has a usable length (%.2f m)", (double)lengthM);

    /* --- parallel-route ambiguity: the same point, two defensible answers ------------------ */
    {
        const Vector2 betweenM = { 50.0f, 3.0f };
        const RouteLocation global = route_localize_global(&track, betweenM, 0.0f);
        check(global.valid && global.segmentIndex == 2,
              "equidistant point resolves to the LOWEST segment index without history (got %d)",
              global.segmentIndex);
        /* Two priors, each unambiguously on one strand, then the same ambiguous point. */
        const RouteLocation onReturning =
            route_localize_global(&track, (Vector2){ 50.0f, 6.0f }, CIRCUIT_PI);
        const RouteLocation onOutbound =
            route_localize_global(&track, (Vector2){ 50.0f, 0.0f }, 0.0f);
        check(onReturning.segmentIndex == 9 && onOutbound.segmentIndex == 2,
              "the two anchors sit on the strands the fixture says they do (%d, %d)",
              onReturning.segmentIndex, onOutbound.segmentIndex);
        const RouteLocation held = route_localize_near(&track, &onReturning, betweenM, 0.0f);
        check(held.valid && held.segmentIndex == 9,
              "continuity holds the returning strand where a global scan would jump to the "
              "outbound one (got %d)",
              held.segmentIndex);
        const RouteLocation outbound = route_localize_near(&track, &onOutbound, betweenM, 0.0f);
        check(outbound.valid && outbound.segmentIndex == 2,
              "continuity holds the outbound strand from the other side (got %d)",
              outbound.segmentIndex);
        /* Heading error is measured against whichever strand was chosen, so the two answers
         * disagree by half a turn. That is exactly why the choice cannot be left to chance. */
        check(fabsf(held.headingErrorRad - outbound.headingErrorRad) > 3.0f,
              "the two strands report opposite heading errors (%.3f vs %.3f rad)",
              (double)held.headingErrorRad, (double)outbound.headingErrorRad);
        /* No hint is the same as a global scan, by contract. */
        const RouteLocation noHint = route_localize_near(&track, NULL, betweenM, 0.0f);
        check(noHint.segmentIndex == global.segmentIndex,
              "a missing prior degenerates to the global scan (%d vs %d)", noHint.segmentIndex,
              global.segmentIndex);
    }

    /* --- cached vs brute force over a whole lap ------------------------------------------- */
    {
        RouteLocation cached;
        memset(&cached, 0, sizeof(cached));
        int disagreements = 0;
        int samples = 0;
        /* Sample strictly inside each segment so no point is equidistant from two of them:
         * where the nearest segment is unique, continuity must reproduce the brute-force
         * answer exactly, and any disagreement is the local search losing the road. */
        for (int seg = 0; seg < LOC_TRACK_NODES; seg++) {
            const Vector2 aM = track.nodes[seg].centerM;
            const Vector2 bM = track.nodes[(seg + 1) % LOC_TRACK_NODES].centerM;
            for (int step = 1; step < 10; step++) {
                const float t = (float)step * 0.1f;
                const Vector2 pM = { aM.x + t * (bM.x - aM.x), aM.y + t * (bM.y - aM.y) };
                const RouteLocation global = route_localize_global(&track, pM, 0.0f);
                cached = route_localize_near(&track, cached.valid ? &cached : NULL, pM, 0.0f);
                samples++;
                if (cached.segmentIndex != global.segmentIndex) disagreements++;
            }
        }
        check(samples == LOC_TRACK_NODES * 9, "lap sampled at every segment (%d points)",
              samples);
        check(disagreements == 0,
              "cached localization reproduces brute force on the unambiguous centreline "
              "(%d disagreements)",
              disagreements);
    }

    /* --- race distance across the lap wrap ------------------------------------------------ */
    {
        RacerProgress prog;
        memset(&prog, 0, sizeof(prog));
        track_reset_progress_at(&prog, &track, 0);
        /* Start a little way into segment 0 and drive the whole lap in 0.5 m steps. */
        Vector2 prevM = { 1.0f, 0.0f };
        prog.location = route_localize_global(&track, prevM, 0.0f);
        float previousDistanceM = prog.raceDistanceM;
        bool monotone = true;
        float maxJumpM = 0.0f;
        int crossedSeam = 0;
        const int steps = 900;
        for (int i = 0; i < steps; i++) {
            const float alongM = 1.0f + 0.5f * (float)(i + 1);
            /* Walk the polyline by arc length rather than by node, so the seam at node 0 is
             * driven through rather than started from. */
            float remainingM = fmodf(alongM, lengthM);
            if (alongM >= lengthM && remainingM < 0.5f) crossedSeam++;
            int seg = 0;
            while (seg < LOC_TRACK_NODES) {
                const Vector2 aM = track.nodes[seg].centerM;
                const Vector2 bM = track.nodes[(seg + 1) % LOC_TRACK_NODES].centerM;
                const float segLenM =
                    sqrtf((bM.x - aM.x) * (bM.x - aM.x) + (bM.y - aM.y) * (bM.y - aM.y));
                if (remainingM <= segLenM) break;
                remainingM -= segLenM;
                seg++;
            }
            if (seg >= LOC_TRACK_NODES) seg = LOC_TRACK_NODES - 1;
            const Vector2 aM = track.nodes[seg].centerM;
            const Vector2 bM = track.nodes[(seg + 1) % LOC_TRACK_NODES].centerM;
            const float segLenM =
                sqrtf((bM.x - aM.x) * (bM.x - aM.x) + (bM.y - aM.y) * (bM.y - aM.y));
            const float t = (segLenM > 0.0f) ? (remainingM / segLenM) : 0.0f;
            const Vector2 currM = { aM.x + t * (bM.x - aM.x), aM.y + t * (bM.y - aM.y) };
            const float headingRad = atan2f(bM.y - aM.y, bM.x - aM.x);
            loc_step(&track, &prog, prevM, currM, headingRad);
            const float deltaM = prog.raceDistanceM - previousDistanceM;
            if (deltaM < -0.001f) monotone = false;
            if (fabsf(deltaM) > maxJumpM) maxJumpM = fabsf(deltaM);
            previousDistanceM = prog.raceDistanceM;
            prevM = currM;
        }
        check(crossedSeam > 0, "the traversal drove through the start/finish seam");
        check(monotone, "race distance never goes backwards while driving forwards");
        check(maxJumpM < 2.0f,
              "no step produced a lap-sized distance jump at the seam (largest %.3f m)",
              (double)maxJumpM);
        check(prog.raceDistanceM > lengthM,
              "race distance passes one lap length rather than wrapping to zero (%.2f m of "
              "%.2f m lap)",
              (double)prog.raceDistanceM, (double)lengthM);
        check(!prog.wrongWay, "a clean forward lap never raises the wrong-way flag");
    }

    /* --- a lane running parallel to opposing traffic (the pit-branch case) ----------------- */
    {
        /* y = 3.2 is nearer the RETURNING strand than the outbound one, so every tick of this
         * drive would localize backwards under a global scan. */
        RacerProgress prog;
        memset(&prog, 0, sizeof(prog));
        track_reset_progress_at(&prog, &track, 0);
        Vector2 prevM = { 10.0f, 3.2f };
        const RouteLocation enteredFrom =
            route_localize_global(&track, (Vector2){ 10.0f, 0.0f }, 0.0f);
        prog.location = route_localize_near(&track, &enteredFrom, prevM, 0.0f);
        check(prog.location.segmentIndex == 0, "branch drive starts on the outbound strand");
        bool stayedOutbound = true;
        bool advanced = true;
        float lastLongitudinalM = prog.location.longitudinalM;
        for (int i = 0; i < 80; i++) {
            const Vector2 currM = { prevM.x + 1.0f, 3.2f };
            loc_step(&track, &prog, prevM, currM, 0.0f);
            if (prog.location.segmentIndex > 4) stayedOutbound = false;
            if (prog.location.longitudinalM < lastLongitudinalM - 0.001f) advanced = false;
            lastLongitudinalM = prog.location.longitudinalM;
            prevM = currM;
        }
        const RouteLocation naive = route_localize_global(&track, prevM, 0.0f);
        check(naive.segmentIndex > 4,
              "a global scan really does put this lane on the opposing strand (segment %d)",
              naive.segmentIndex);
        check(stayedOutbound, "the branch drive never swapped onto the opposing strand");
        check(advanced, "branch longitudinal progress advanced every tick");
        check(!prog.wrongWay,
              "driving a parallel branch the right way round does not read as wrong-way");
        check(prog.location.confidence <= 0.0f && !prog.location.onRoute,
              "a car outside the barrier corridor reports zero route confidence (%.3f)",
              (double)prog.location.confidence);
    }

    /* --- off-track excursion and rejoin ---------------------------------------------------- */
    {
        RacerProgress prog;
        memset(&prog, 0, sizeof(prog));
        track_reset_progress_at(&prog, &track, 0);
        Vector2 prevM = { 30.0f, 0.0f };
        prog.location = route_localize_global(&track, prevM, 0.0f);
        check(prog.location.onRoute && prog.location.confidence >= 1.0f,
              "a car on the racing surface reports full route confidence (%.3f)",
              (double)prog.location.confidence);
        /* Run wide onto the runoff, then off it, then back. */
        const float offsets[] = { -2.5f, -4.0f, -7.0f, -4.0f, -1.0f, 0.0f };
        float runoffConfidence = -1.0f;
        bool keptStrand = true;
        for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
            const Vector2 currM = { prevM.x + 2.0f, offsets[i] };
            loc_step(&track, &prog, prevM, currM, 0.0f);
            if (prog.location.segmentIndex > 4) keptStrand = false;
            if (i == 0) runoffConfidence = prog.location.confidence;
            prevM = currM;
        }
        check(runoffConfidence > 0.0f && runoffConfidence < 1.0f,
              "confidence decays across the runoff band rather than stepping to zero (%.3f)",
              (double)runoffConfidence);
        check(keptStrand, "an off-track excursion and rejoin never re-localized the car");
        check(prog.location.onRoute,
              "the rejoined car is back on the racing surface (lateral %.3f m)",
              (double)prog.location.lateralM);
        check(!prog.wrongWay, "an excursion at speed is not wrong-way travel");
    }

    /* --- reverse travel sets wrong-way, but only after the hold ---------------------------- */
    {
        RacerProgress prog;
        memset(&prog, 0, sizeof(prog));
        track_reset_progress_at(&prog, &track, 0);
        Vector2 prevM = { 80.0f, 0.0f };
        prog.location = route_localize_global(&track, prevM, CIRCUIT_PI);
        const int holdTicks = (int)(ROUTE_WRONG_WAY_HOLD_S / FIXED_DT_S);
        bool earlyFlag = false;
        for (int i = 0; i < holdTicks - 2; i++) {
            const Vector2 currM = { prevM.x - 0.2f, 0.0f };
            loc_step(&track, &prog, prevM, currM, CIRCUIT_PI);
            if (prog.wrongWay) earlyFlag = true;
            prevM = currM;
        }
        check(!earlyFlag, "wrong-way does not latch before the hysteresis hold elapses");
        bool changedEvent = false;
        for (int i = 0; i < 8; i++) {
            const Vector2 currM = { prevM.x - 0.2f, 0.0f };
            const TrackProgressEvent ev = loc_step(&track, &prog, prevM, currM, CIRCUIT_PI);
            if (ev.wrongWayChanged) changedEvent = true;
            prevM = currM;
        }
        check(prog.wrongWay, "sustained reverse travel latches wrong-way");
        check(changedEvent, "the tick the flag latched reported wrongWayChanged");
        /* Turn round: it must take the same sustained evidence to clear. */
        for (int i = 0; i < 4; i++) {
            const Vector2 currM = { prevM.x + 0.2f, 0.0f };
            loc_step(&track, &prog, prevM, currM, 0.0f);
            prevM = currM;
        }
        check(prog.wrongWay, "wrong-way does not clear on the first correct tick");
        for (int i = 0; i < holdTicks + 4; i++) {
            const Vector2 currM = { prevM.x + 0.2f, 0.0f };
            loc_step(&track, &prog, prevM, currM, 0.0f);
            prevM = currM;
        }
        check(!prog.wrongWay, "wrong-way clears once the car is going the right way again");
    }

    /* --- a spin does not flicker the flag --------------------------------------------------- */
    {
        RacerProgress prog;
        memset(&prog, 0, sizeof(prog));
        track_reset_progress_at(&prog, &track, 0);
        Vector2 prevM = { 22.0f, 0.0f };
        prog.location = route_localize_global(&track, prevM, 0.0f);
        /* A full rotation over 1.5 s while the car keeps sliding forwards: the heading spends
         * roughly half a second past 120 degrees, which is less than the hold, and the car
         * never loses ground. Both guards would have to fail for the flag to raise. */
        const int spinTicks = (int)(1.5f / FIXED_DT_S);
        bool flagged = false;
        float maxHeadingErrorRad = 0.0f;
        for (int i = 0; i < spinTicks; i++) {
            const float headingRad =
                wrap_angle(CIRCUIT_TWO_PI * (float)(i + 1) / (float)spinTicks);
            const Vector2 currM = { prevM.x + 0.15f, 0.0f };
            loc_step(&track, &prog, prevM, currM, headingRad);
            if (prog.wrongWay) flagged = true;
            if (fabsf(prog.location.headingErrorRad) > maxHeadingErrorRad)
                maxHeadingErrorRad = fabsf(prog.location.headingErrorRad);
            prevM = currM;
        }
        check(maxHeadingErrorRad > ROUTE_WRONG_WAY_ENTER_RAD,
              "the spin really did point the car past the wrong-way angle (%.3f rad)",
              (double)maxHeadingErrorRad);
        check(!flagged, "a spin that keeps carrying the car forwards never raises wrong-way");

        /* Now a recovery that briefly reverses: still short of the hold, still no flag. */
        bool recoveryFlagged = false;
        for (int i = 0; i < (int)(0.4f / FIXED_DT_S); i++) {
            const Vector2 currM = { prevM.x - 0.1f, 0.0f };
            loc_step(&track, &prog, prevM, currM, CIRCUIT_PI);
            if (prog.wrongWay) recoveryFlagged = true;
            prevM = currM;
        }
        check(!recoveryFlagged, "a brief backwards recovery does not raise wrong-way");
    }

    /* --- several entrants, one track, independent and order-independent -------------------- */
    {
        RacerProgress racers[3];
        Vector2 posM[3] = { { 10.0f, 0.0f }, { 30.0f, 0.0f }, { 50.0f, 0.0f } };
        /* Every entrant stays on the y = 0 straight, so each one's accumulated distance is
         * exactly the ground it covered and the ordering below is not an artefact of where the
         * fixture's corners happen to fall. */
        const float stepM[3] = { 0.4f, 0.2f, 0.4f };
        for (int i = 0; i < 3; i++) {
            memset(&racers[i], 0, sizeof(racers[i]));
            track_reset_progress_at(&racers[i], &track, 0);
            racers[i].location = route_localize_global(&track, posM[i], 0.0f);
        }
        /* Entrant 0 is a lap up on the other two, so the ordering is tested ACROSS a lap
         * boundary rather than only within one. */
        racers[0].raceDistanceM = lengthM;

        /* Update in ascending order, then repeat the identical inputs in descending order into
         * a second set: an entrant that could disturb another would diverge. */
        RacerProgress mirrored[3];
        Vector2 mirroredPosM[3];
        for (int i = 0; i < 3; i++) {
            mirrored[i] = racers[i];
            mirroredPosM[i] = posM[i];
        }
        for (int step = 0; step < 40; step++) {
            for (int i = 0; i < 3; i++) {
                const Vector2 currM = { posM[i].x + stepM[i], 0.0f };
                loc_step(&track, &racers[i], posM[i], currM, 0.0f);
                posM[i] = currM;
            }
            for (int i = 2; i >= 0; i--) {
                const Vector2 currM = { mirroredPosM[i].x + stepM[i], 0.0f };
                loc_step(&track, &mirrored[i], mirroredPosM[i], currM, 0.0f);
                mirroredPosM[i] = currM;
            }
        }
        bool identical = true;
        for (int i = 0; i < 3; i++) {
            if (racers[i].raceDistanceM != mirrored[i].raceDistanceM ||
                racers[i].location.segmentIndex != mirrored[i].location.segmentIndex ||
                racers[i].location.longitudinalM != mirrored[i].location.longitudinalM) {
                identical = false;
            }
        }
        check(identical, "entrant results do not depend on the order entrants are updated in");
        check_near((double)racers[2].raceDistanceM, 16.0, 0.01,
                   "the faster of the two on the lead lap covered its 16 m");
        check_near((double)racers[1].raceDistanceM, 8.0, 0.01,
                   "the slower of the two on the lead lap covered its 8 m");
        check(racers[0].raceDistanceM > racers[2].raceDistanceM &&
                  racers[2].raceDistanceM > racers[1].raceDistanceM,
              "race distance orders a lapped field correctly (%.2f > %.2f > %.2f)",
              (double)racers[0].raceDistanceM, (double)racers[2].raceDistanceM,
              (double)racers[1].raceDistanceM);
        check(racers[1].location.segmentIndex != racers[2].location.segmentIndex,
              "each entrant carries its own route position (%d vs %d)",
              racers[1].location.segmentIndex, racers[2].location.segmentIndex);
    }

    track_free(&track);

    /* --- open point-to-point route: no wrap, no phantom segment past the end --------------- */
    {
        TrackDefinition open;
        loc_build_track(&open, false);
        check(open.count == LOC_TRACK_NODES, "open fixture built");
        if (open.count == LOC_TRACK_NODES) {
            /* The closing segment 13 -> 0 does not exist on an open route, so a point beyond
             * the last node must clamp to the final real segment rather than wrap. */
            const RouteLocation past =
                route_localize_global(&open, (Vector2){ -30.0f, 3.0f }, 0.0f);
            check(past.valid && past.segmentIndex == LOC_TRACK_NODES - 2,
                  "an open route ends at its last real segment (got %d, expected %d)",
                  past.segmentIndex, LOC_TRACK_NODES - 2);
            /* Segment LOC_TRACK_NODES-1 is the closing segment a CLOSED route would have. An
             * open route has no such segment, so a prior naming it must be rejected rather
             * than indexed. */
            RouteLocation stalePrior;
            memset(&stalePrior, 0, sizeof(stalePrior));
            stalePrior.valid = true;
            stalePrior.segmentIndex = LOC_TRACK_NODES - 1;
            const RouteLocation hinted =
                route_localize_near(&open, &stalePrior, (Vector2){ -30.0f, 3.0f }, 0.0f);
            check(hinted.segmentIndex == past.segmentIndex,
                  "a prior naming a segment the open route does not have falls back cleanly "
                  "(got %d)",
                  hinted.segmentIndex);

            RacerProgress prog;
            memset(&prog, 0, sizeof(prog));
            track_reset_progress_at(&prog, &open, 0);
            Vector2 prevM = { 2.0f, 0.0f };
            prog.location = route_localize_global(&open, prevM, 0.0f);
            for (int i = 0; i < 40; i++) {
                const Vector2 currM = { prevM.x + 1.0f, 0.0f };
                loc_step(&open, &prog, prevM, currM, 0.0f);
                prevM = currM;
            }
            check_near((double)prog.raceDistanceM, 40.0, 0.01,
                       "open-route race distance is plain arc length with no wrap term");
        }
        track_free(&open);
    }

    /* --- segments far longer than the search window ---------------------------------------- */
    {
        /* The committed parking lot is a 400 m x 300 m rectangle: every one of its four
         * segments dwarfs ROUTE_LOCALIZE_WINDOW_M. A search that spent the whole of the
         * previous segment before looking at its neighbour could never see the edge a car is
         * about to turn onto, so it would clamp to the corner, freeze longitudinal progress,
         * and then jump it when the acceptance corridor finally rejected the stale answer. */
        TrackDefinition big;
        memset(&big, 0, sizeof(big));
        big.count = 4;
        big.nodes = (TrackNode *)calloc((size_t)big.count, sizeof(TrackNode));
        check(big.nodes != NULL, "long-segment fixture allocated");
        if (big.nodes != NULL) {
            const Vector2 corners[4] = {
                { 0.0f, 0.0f }, { 100.0f, 0.0f }, { 100.0f, 100.0f }, { 0.0f, 100.0f }
            };
            for (int i = 0; i < 4; i++) {
                big.nodes[i] = (TrackNode){ corners[i], 4.0f, SURFACE_ASPHALT, 0.0f };
            }
            big.routeClosed = true;
            big.offTrackSurfaceId = SURFACE_GRASS;
            big.runoffSurfaceId = SURFACE_GRASS;
            check(100.0f > ROUTE_LOCALIZE_WINDOW_M,
                  "the fixture's segments really are longer than the search window");

            RacerProgress prog;
            memset(&prog, 0, sizeof(prog));
            track_reset_progress_at(&prog, &big, 0);
            Vector2 prevM = { 90.0f, 0.0f };
            prog.location = route_localize_global(&big, prevM, 0.0f);
            check(prog.location.segmentIndex == 0, "long-segment drive starts on segment 0");

            int segmentAfterCorner = -1;
            float minDeltaM = INFINITY;
            float maxDeltaM = 0.0f;
            float previousDistanceM = prog.raceDistanceM;
            for (int i = 0; i < 30; i++) {
                /* Ten metres east to the corner, then twenty north up the next edge. */
                const Vector2 currM = (i < 10) ? (Vector2){ prevM.x + 1.0f, 0.0f }
                                               : (Vector2){ 100.0f, prevM.y + 1.0f };
                const float headingRad = (i < 10) ? 0.0f : (CIRCUIT_PI * 0.5f);
                loc_step(&big, &prog, prevM, currM, headingRad);
                if (i == 10) segmentAfterCorner = prog.location.segmentIndex;
                const float deltaM = prog.raceDistanceM - previousDistanceM;
                if (deltaM < minDeltaM) minDeltaM = deltaM;
                if (deltaM > maxDeltaM) maxDeltaM = deltaM;
                previousDistanceM = prog.raceDistanceM;
                prevM = currM;
            }
            check(segmentAfterCorner == 1,
                  "the first tick past the corner is already on the next segment (got %d)",
                  segmentAfterCorner);
            check(minDeltaM > 0.5f,
                  "longitudinal progress never froze at the corner (smallest step %.4f m)",
                  (double)minDeltaM);
            check(maxDeltaM < 1.5f,
                  "and never jumped to catch up afterwards (largest step %.4f m)",
                  (double)maxDeltaM);
            check_near((double)prog.raceDistanceM, 30.0, 0.01,
                       "race distance equals the ground covered around the corner");
        }
        track_free(&big);
    }

    /* --- containment is the real distance, not the lateral component ----------------------- */
    {
        TrackDefinition open;
        loc_build_track(&open, false);
        if (open.count == LOC_TRACK_NODES) {
            /* Straight on past the last node, along that segment's own direction. The
             * displacement is then almost entirely LONGITUDINAL, so the lateral component is
             * ~0 and a containment test built on it would call this pose on-track with full
             * confidence while the car is fifty metres into the scenery. */
            const Vector2 lastM = open.nodes[LOC_TRACK_NODES - 1].centerM;
            const Vector2 prevNodeM = open.nodes[LOC_TRACK_NODES - 2].centerM;
            const float dx = lastM.x - prevNodeM.x;
            const float dy = lastM.y - prevNodeM.y;
            const float len = sqrtf(dx * dx + dy * dy);
            const Vector2 farM = { lastM.x + 50.0f * dx / len, lastM.y + 50.0f * dy / len };
            const RouteLocation past = route_localize_global(&open, farM, 0.0f);
            check(past.valid, "a pose past the end of an open route still localizes");
            check(fabsf(past.lateralM) < 0.01f,
                  "and its LATERAL offset really is ~zero, which is the trap (%.4f m)",
                  (double)past.lateralM);
            check(!past.onRoute,
                  "yet it is not reported as on the racing surface (%.1f m past the end)",
                  50.0);
            check(past.confidence <= 0.0f, "and its route confidence is zero (%.3f)",
                  (double)past.confidence);
        }
        track_free(&open);
    }

    /* --- a route with nothing to localize against ------------------------------------------ */
    {
        TrackDefinition empty;
        memset(&empty, 0, sizeof(empty));
        const RouteLocation none = route_localize_global(&empty, (Vector2){ 0.0f, 0.0f }, 0.0f);
        check(!none.valid, "an empty definition localizes to an explicitly invalid result");
        RacerProgress prog;
        memset(&prog, 0, sizeof(prog));
        const TrackProgressEvent ev = track_update_progress(
            NULL, &prog, (Vector2){ 0.0f, 0.0f }, (Vector2){ 1.0f, 0.0f }, 0.0f, FIXED_DT_S);
        check(!ev.checkpoint.crossed && ev.checkpoint.index == -1 && !ev.wrongWay,
              "a NULL track produces an empty progress event rather than a crash");
        check(prog.raceDistanceM == 0.0f, "a NULL track leaves race distance untouched");
    }
}

/* --------------------------------------------------------------------------------------------- registry */

/*
 * track-query-cache — issue #39: the hinted local-neighbourhood search returns the exact
 * same surface as the brute-force scan for every sampled point.
 *
 * Property test: for a fixed deterministic point grid around each built-in track, query the
 * surface with a fresh runtime (hint = 0, then hint evolves) and compare against the
 * brute-force result obtained with a NULL runtime. The results must match exactly.
 */
static void scenario_track_query_cache(void)
{
    typedef void (*LoadFn)(TrackDefinition *);
    const LoadFn loads[] = { track_load_chicane, track_load_sprint, track_load_technical };
    const char *names[] = { "chicane", "sprint", "technical" };

    for (size_t l = 0; l < sizeof(loads) / sizeof(loads[0]); l++) {
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        loads[l](&track);
        check(track.nodes != NULL && track.count > 0, "%s loaded for query-cache test",
              names[l]);

        TrackRuntime runtime;
        memset(&runtime, 0, sizeof(runtime));

        /* Deterministic grid of probe points around the track: sample the centreline box
         * plus a margin, stepping at 7 m. */
        float minX = 1e30f, minY = 1e30f, maxX = -1e30f, maxY = -1e30f;
        for (int i = 0; i < track.count; i++) {
            minX = fminf(minX, track.nodes[i].centerM.x);
            minY = fminf(minY, track.nodes[i].centerM.y);
            maxX = fmaxf(maxX, track.nodes[i].centerM.x);
            maxY = fmaxf(maxY, track.nodes[i].centerM.y);
        }
        minX -= 20.0f;
        minY -= 20.0f;
        maxX += 20.0f;
        maxY += 20.0f;

        int mismatches = 0;
        int points = 0;
        for (float y = minY; y <= maxY; y += 7.0f) {
            for (float x = minX; x <= maxX; x += 7.0f) {
                const Vector2 p = { x, y };
                const SurfaceId brute = Track_SurfaceAt(&track, NULL, p);
                const SurfaceId hinted = Track_SurfaceAt(&track, &runtime, p);
                points++;
                if (brute != hinted) mismatches++;
            }
        }
        check(points > 0, "%s query-cache probe grid covered points", names[l]);
        check(mismatches == 0,
              "%s hinted surface == brute force on all %d points (%d mismatches)", names[l],
              points, mismatches);

        track_free(&track);
    }
}

/*
 * track-validate — issue #42: the authoring validator accepts every shipped track and
 * rejects planted geometry errors, and the canonical hash round-trips through the loader.
 */
static void scenario_track_validate(void)
{
    /* ---- 1. Every shipped track validates and round-trips its hash. ---- */
    {
        TrackCatalog catalog;
        memset(&catalog, 0, sizeof(catalog));
        char error[512] = "";
        check(track_catalog_load(NULL, &catalog, error, sizeof(error)),
              "track catalog loads (%s)", error);
        check(catalog.count >= 4, "at least the four built-in tracks ship (%d)", catalog.count);
        for (int i = 0; i < catalog.count; i++) {
            char why[256] = "";
            const TrackDefinition *track = &catalog.entries[i].definition;
            check(track_validate(track, why, sizeof(why)), "shipped track '%s' validates (%s)",
                  track->id, why);
            /* Load by id and compare the canonical hash. */
            TrackDefinition reloaded;
            memset(&reloaded, 0, sizeof(reloaded));
            uint32_t hash = 0u;
            check(track_load_by_id(track->id, &reloaded, &hash, error, sizeof(error)),
                  "track '%s' reloads by id (%s)", track->id, error);
            check(track_geometry_hash(&reloaded) == track_geometry_hash(track),
                  "track '%s' canonical hash round-trips", track->id);
            track_free(&reloaded);
        }
        track_catalog_free(&catalog);
    }

    /* ---- 2. Planted errors are caught. ---- */
    {
        TrackDefinition track;
        memset(&track, 0, sizeof(track));
        track_load_chicane(&track);
        char why[256] = "";

        /* Non-finite node. */
        track.nodes[5].centerM.x = NAN;
        check(!track_validate(&track, why, sizeof(why)), "non-finite node rejected (%s)", why);
        track.nodes[5].centerM.x = 0.0f;

        /* Degenerate segment. */
        track.nodes[10] = track.nodes[11];
        check(!track_validate(&track, why, sizeof(why)), "degenerate segment rejected (%s)",
              why);
        track_free(&track);
    }
}

static const TestScenario kScenarios[] = {
    { "json-parser", "strict JSON reader, escapes, and canonical hash", scenario_json_parser },
    { "vehicle-manifest",
      "issue #29: manifest load, validation, roster round-trip, catalog discovery",
      scenario_vehicle_manifest },
    { "roster-promotion", "issue #31: the promotion checklist gates player-selectable content",
      scenario_roster_promotion },
    { "roster-content-kind",
      "issue #31: content-kind separation keeps corpus samples out of the roster",
      scenario_roster_content_kind },
    { "vehicle-class", "issue #33: class rules and eligibility for the roster cars",
      scenario_vehicle_class },
    { "track-format",
      "issue #34: external track load, faithful round-trip, hash stability, validation",
      scenario_track_format },
    { "track-markers",
      "issue #37: typed route/sector/start-finish/grid/pit markers, open routes, debounce, "
      "mode checks",
      scenario_track_markers },
    { "track-migration",
      "issue #36: catalog discovery, legacy vs loaded equivalence, missing handling",
      scenario_track_migration },
    { "route-localization",
      "issue #38: continuity localization, wrong-way hysteresis, race distance, per-entrant "
      "isolation",
      scenario_route_localization },
    { "track-query-cache",
      "issue #39: hinted local-neighbourhood surface queries match brute force exactly",
      scenario_track_query_cache },
    { "track-validate",
      "issue #42: authoring validator accepts shipped tracks, catches planted errors, hash "
      "round-trips",
      scenario_track_validate },
};

TestScenarioGroup test_content_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kScenarios;
    group.count = sizeof(kScenarios) / sizeof(kScenarios[0]);
    return group;
}

/*
 * vehicle_class.c — versioned class-rule manifest loader + eligibility. See the header.
 */
#include "content/vehicle_class.h"

#include "core/json.h"

#include <dirent.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The only keys permitted at the top level of a version-1 class file. Anything else is a typo
 * and is rejected rather than silently dropped, so a misspelled section is a loud error. */
typedef enum {
    VC_KEY_SCHEMA,
    VC_KEY_VERSION,
    VC_KEY_ID,
    VC_KEY_DISPLAY_NAME,
    VC_KEY_DESCRIPTION,
    VC_KEY_RULES,
    VC_KEY_COUNT
} ClassTopKey;

static const char *const kTopKeyNames[VC_KEY_COUNT] = {
    [VC_KEY_SCHEMA] = "schema",
    [VC_KEY_VERSION] = "version",
    [VC_KEY_ID] = "id",
    [VC_KEY_DISPLAY_NAME] = "displayName",
    [VC_KEY_DESCRIPTION] = "description",
    [VC_KEY_RULES] = "rules",
};

/* The only rule keys permitted inside "rules". An unknown one is a typo (e.g. "mass_kg "
 * with a trailing space) and must not silently become an unconstrained rule. */
typedef enum {
    VC_RULE_MASS_KG,
    VC_RULE_PEAK_TORQUE_NM,
    VC_RULE_MAX_TIRE_MU,
    VC_RULE_LAYOUTS,
    VC_RULE_COUNT
} ClassRuleKey;

static const char *const kRuleKeyNames[VC_RULE_COUNT] = {
    [VC_RULE_MASS_KG] = "mass_kg",
    [VC_RULE_PEAK_TORQUE_NM] = "peak_torque_nm",
    [VC_RULE_MAX_TIRE_MU] = "max_tire_mu",
    [VC_RULE_LAYOUTS] = "layouts",
};

static int top_key_index(const char *name)
{
    for (int i = 0; i < VC_KEY_COUNT; i++) {
        if (strcmp(name, kTopKeyNames[i]) == 0) return i;
    }
    return -1;
}

static int rule_key_index(const char *name)
{
    for (int i = 0; i < VC_RULE_COUNT; i++) {
        if (strcmp(name, kRuleKeyNames[i]) == 0) return i;
    }
    return -1;
}

/* Write a `field: reason` error, truncating safely. */
static void set_error(char *error, size_t errorCap, const char *field, const char *reason)
{
    if (error == NULL || errorCap == 0) return;
    if (field != NULL) {
        snprintf(error, errorCap, "%s: %s", field, reason);
    } else {
        snprintf(error, errorCap, "%s", reason);
    }
}

static bool copy_text_field(char *dst, size_t cap, const char *src, const char *fieldName,
                            char *error, size_t errorCap)
{
    if (src == NULL) {
        set_error(error, errorCap, fieldName, "expected a string");
        return false;
    }
    if (strlen(src) + 1 > cap) {
        set_error(error, errorCap, fieldName, "value is too long");
        return false;
    }
    snprintf(dst, cap, "%s", src);
    return true;
}

/* A number whose fractional part is zero and which fits in an int. Used for version. */
static bool integer_value(const JsonValue *value, int min, int max, int *out)
{
    if (value == NULL || !json_is_number(value)) return false;
    const double n = json_as_number(value);
    if (!isfinite(n) || floor(n) != n) return false;
    if (n < (double)min || n > (double)max) return false;
    *out = (int)n;
    return true;
}

/* Parse an optional [min, max] bound pair. Absent means unconstrained; a present value must
 * be exactly two finite numbers with min <= max, because a reversed or non-numeric pair is a
 * review error, not a subtle empty rule. */
static bool parse_range_rule(const JsonValue *rules, const char *key, bool *hasOut,
                             float *minOut, float *maxOut, char *error, size_t errorCap)
{
    const JsonValue *value = json_object_get(rules, key);
    if (value == NULL) return true;
    if (!json_is_array(value) || json_array_count(value) != 2) {
        set_error(error, errorCap, key, "expected [min, max]");
        return false;
    }
    const JsonValue *lo = json_array_at(value, 0);
    const JsonValue *hi = json_array_at(value, 1);
    if (!json_is_number(lo) || !json_is_number(hi)) {
        set_error(error, errorCap, key, "expected [min, max] numbers");
        return false;
    }
    const double loN = json_as_number(lo);
    const double hiN = json_as_number(hi);
    if (!isfinite(loN) || !isfinite(hiN) || loN > hiN) {
        set_error(error, errorCap, key, "bounds must be finite with min <= max");
        return false;
    }
    *hasOut = true;
    *minOut = (float)loN;
    *maxOut = (float)hiN;
    return true;
}

/* Parse the optional max_tire_mu upper bound. Absent means unconstrained. */
static bool parse_max_tire_mu(VehicleClass *out, const JsonValue *rules, char *error,
                              size_t errorCap)
{
    const JsonValue *value = json_object_get(rules, kRuleKeyNames[VC_RULE_MAX_TIRE_MU]);
    if (value == NULL) return true;
    if (!json_is_number(value)) {
        set_error(error, errorCap, kRuleKeyNames[VC_RULE_MAX_TIRE_MU], "expected a number");
        return false;
    }
    const double mu = json_as_number(value);
    if (!isfinite(mu)) {
        set_error(error, errorCap, kRuleKeyNames[VC_RULE_MAX_TIRE_MU], "must be finite");
        return false;
    }
    out->hasMaxTireMu = true;
    out->maxTireMu = (float)mu;
    return true;
}

/* Parse the optional drivetrain whitelist. Absent (or empty) means any layout; present
 * entries must be "rwd"/"fwd"/"awd" with no duplicates, so a typo cannot create a rule no
 * car can ever satisfy. */
static bool parse_layouts(VehicleClass *out, const JsonValue *rules, char *error,
                          size_t errorCap)
{
    const JsonValue *value = json_object_get(rules, kRuleKeyNames[VC_RULE_LAYOUTS]);
    if (value == NULL) return true;
    if (!json_is_array(value)) {
        set_error(error, errorCap, kRuleKeyNames[VC_RULE_LAYOUTS],
                  "expected an array of strings");
        return false;
    }
    const int n = json_array_count(value);
    if (n > VEHICLE_CLASS_MAX_LAYOUTS) {
        set_error(error, errorCap, kRuleKeyNames[VC_RULE_LAYOUTS], "too many layouts");
        return false;
    }
    for (int i = 0; i < n; i++) {
        const char *s = json_as_string(json_array_at(value, i));
        if (s == NULL) {
            set_error(error, errorCap, kRuleKeyNames[VC_RULE_LAYOUTS],
                      "entries must be strings");
            return false;
        }
        if (strcmp(s, "rwd") != 0 && strcmp(s, "fwd") != 0 && strcmp(s, "awd") != 0) {
            static char reason[96];
            snprintf(reason, sizeof(reason),
                     "unknown layout '%s' (expected \"rwd\", \"fwd\", or \"awd\")", s);
            set_error(error, errorCap, kRuleKeyNames[VC_RULE_LAYOUTS], reason);
            return false;
        }
        for (int j = 0; j < out->layoutCount; j++) {
            if (strcmp(out->layouts[j], s) == 0) {
                set_error(error, errorCap, kRuleKeyNames[VC_RULE_LAYOUTS], "duplicate layout");
                return false;
            }
        }
        snprintf(out->layouts[out->layoutCount], sizeof(out->layouts[out->layoutCount]), "%s",
                 s);
        out->layoutCount++;
    }
    return true;
}

static bool parse_rules(VehicleClass *out, const JsonValue *rules, char *error, size_t errorCap)
{
    if (rules == NULL) return true; /* optional section absent: all rules unconstrained */
    if (!json_is_object(rules)) {
        set_error(error, errorCap, kTopKeyNames[VC_KEY_RULES], "expected an object");
        return false;
    }
    for (int i = 0; i < json_object_count(rules); i++) {
        if (rule_key_index(json_object_key_at(rules, i)) < 0) {
            static char reason[128];
            snprintf(reason, sizeof(reason), "unknown rule key '%s'",
                     json_object_key_at(rules, i));
            set_error(error, errorCap, kTopKeyNames[VC_KEY_RULES], reason);
            return false;
        }
    }
    if (!parse_range_rule(rules, kRuleKeyNames[VC_RULE_MASS_KG], &out->hasMassRange,
                          &out->minMassKg, &out->maxMassKg, error, errorCap) ||
        !parse_range_rule(rules, kRuleKeyNames[VC_RULE_PEAK_TORQUE_NM], &out->hasTorqueRange,
                          &out->minPeakTorqueNm, &out->maxPeakTorqueNm, error, errorCap) ||
        !parse_max_tire_mu(out, rules, error, errorCap) ||
        !parse_layouts(out, rules, error, errorCap)) {
        return false;
    }
    return true;
}

bool vehicle_class_parse(const char *text, size_t length, VehicleClass *out, char *error,
                         size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';

    JsonDocument *doc = json_parse(text, length, error, errorCap);
    if (doc == NULL) return false;
    const JsonValue *root = json_document_root(doc);

    if (root == NULL || !json_is_object(root)) {
        set_error(error, errorCap, NULL, "class must be a JSON object");
        json_document_free(doc);
        return false;
    }

    /* Reject unknown top-level keys before doing anything else: a typo like "ruls" should not
     * silently produce an unconstrained class. */
    for (int i = 0; i < json_object_count(root); i++) {
        if (top_key_index(json_object_key_at(root, i)) < 0) {
            static char reason[160];
            snprintf(reason, sizeof(reason), "unknown top-level key '%s'",
                     json_object_key_at(root, i));
            set_error(error, errorCap, NULL, reason);
            json_document_free(doc);
            return false;
        }
    }

    const JsonValue *schema = json_object_get(root, kTopKeyNames[VC_KEY_SCHEMA]);
    if (schema == NULL || !json_is_string(schema) ||
        strcmp(json_as_string(schema), VEHICLE_CLASS_SCHEMA) != 0) {
        set_error(error, errorCap, "schema", "must be \"" VEHICLE_CLASS_SCHEMA "\"");
        json_document_free(doc);
        return false;
    }

    int version = 0;
    if (!integer_value(json_object_get(root, kTopKeyNames[VC_KEY_VERSION]),
                       VEHICLE_CLASS_VERSION, VEHICLE_CLASS_VERSION, &version)) {
        static char reason[96];
        snprintf(reason, sizeof(reason), "version must be %d (the only supported version)",
                 VEHICLE_CLASS_VERSION);
        set_error(error, errorCap, "version", reason);
        json_document_free(doc);
        return false;
    }

    const char *id = json_as_string(json_object_get(root, kTopKeyNames[VC_KEY_ID]));
    if (!vehicle_manifest_id_is_valid(id)) {
        set_error(error, errorCap, "id",
                  "must match [a-z0-9][a-z0-9._-]{0,62} (lowercase, filesystem-safe)");
        json_document_free(doc);
        return false;
    }
    if (strlen(id) + 1 > sizeof(out->id)) {
        set_error(error, errorCap, "id", "value is too long for the class id field");
        json_document_free(doc);
        return false;
    }
    snprintf(out->id, sizeof(out->id), "%s", id);

    if (!copy_text_field(
            out->displayName, VEHICLE_CLASS_TEXT_CHARS,
            json_as_string(json_object_get(root, kTopKeyNames[VC_KEY_DISPLAY_NAME])),
            "displayName", error, errorCap)) {
        json_document_free(doc);
        memset(out, 0, sizeof(*out));
        return false;
    }
    /* description is optional: omitting it leaves an empty string, but a present value must be
     * a string so a typo (e.g. a number) is still rejected. */
    const JsonValue *description = json_object_get(root, kTopKeyNames[VC_KEY_DESCRIPTION]);
    if (description != NULL && !json_is_string(description)) {
        set_error(error, errorCap, "description", "expected a string");
        json_document_free(doc);
        memset(out, 0, sizeof(*out));
        return false;
    }
    if (description != NULL &&
        !copy_text_field(out->description, VEHICLE_CLASS_DESC_CHARS,
                         json_as_string(description), "description", error, errorCap)) {
        json_document_free(doc);
        memset(out, 0, sizeof(*out));
        return false;
    }

    if (!parse_rules(out, json_object_get(root, kTopKeyNames[VC_KEY_RULES]), error, errorCap)) {
        json_document_free(doc);
        memset(out, 0, sizeof(*out));
        return false;
    }

    json_document_free(doc);
    return true;
}

bool vehicle_class_load(const char *path, VehicleClass *out, char *error, size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (path == NULL) {
        set_error(error, errorCap, NULL, "no class path given");
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
    const bool ok = vehicle_class_parse(buffer, read, out, error, errorCap);
    free(buffer);
    return ok;
}

static int compare_class_by_id(const void *a, const void *b)
{
    const VehicleClass *ca = (const VehicleClass *)a;
    const VehicleClass *cb = (const VehicleClass *)b;
    return strcmp(ca->id, cb->id);
}

/* Read a whole file into a malloc'd buffer. Returns NULL (and sets the error) on failure. */
static char *read_file_bytes(const char *path, size_t *bytesReadOut, char *error,
                             size_t errorCap)
{
    if (bytesReadOut != NULL) *bytesReadOut = 0;
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        set_error(error, errorCap, path, "could not open file");
        return NULL;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        set_error(error, errorCap, path, "could not seek file");
        return NULL;
    }
    const long size = ftell(file);
    rewind(file);
    if (size < 0) {
        fclose(file);
        set_error(error, errorCap, path, "could not tell file size");
        return NULL;
    }
    char *buffer = (char *)malloc((size_t)size + 1u);
    if (buffer == NULL) {
        fclose(file);
        set_error(error, errorCap, NULL, "out of memory");
        return NULL;
    }
    const size_t read = fread(buffer, 1u, (size_t)size, file);
    fclose(file);
    buffer[read] = '\0';
    if (bytesReadOut != NULL) *bytesReadOut = read;
    return buffer;
}

bool vehicle_class_load_dir(const char *dir, VehicleClassCatalog *out, char *error,
                            size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (dir == NULL) {
        set_error(error, errorCap, NULL, "no class catalog directory given");
        return false;
    }

    DIR *d = opendir(dir);
    if (d == NULL) {
        set_error(error, errorCap, dir, "could not open directory");
        return false;
    }

    /* First pass: count candidate files so the result array is one allocation. Filesystem
     * enumeration order is explicitly not relied upon — the catalog is sorted by id afterwards. */
    const size_t suffixLen = strlen(".vehicle-class.json");
    int fileCount = 0;
    const struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        const char *name = entry->d_name;
        const size_t len = strlen(name);
        if (len > suffixLen && strcmp(name + len - suffixLen, ".vehicle-class.json") == 0) {
            fileCount++;
        }
    }
    rewinddir(d);

    VehicleClass *items = NULL;
    if (fileCount > 0) {
        items = (VehicleClass *)calloc((size_t)fileCount, sizeof(VehicleClass));
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
        if (len <= suffixLen || strcmp(name + len - suffixLen, ".vehicle-class.json") != 0) {
            continue;
        }

        char path[1024];
        const int written = snprintf(path, sizeof(path), "%s/%s", dir, name);
        if (written < 0 || written >= (int)sizeof(path)) {
            set_error(error, errorCap, name, "path is too long");
            ok = false;
            break;
        }
        if (count >= fileCount) {
            set_error(error, errorCap, dir, "catalog directory changed during load");
            ok = false;
            break;
        }
        size_t readBytes = 0;
        char *text = read_file_bytes(path, &readBytes, error, errorCap);
        if (text == NULL) {
            ok = false;
            break;
        }
        if (!vehicle_class_parse(text, readBytes, &items[count], error, errorCap)) {
            free(text);
            ok = false;
            break;
        }
        free(text);
        count++;
    }
    closedir(d);

    if (!ok) {
        free(items);
        out->items = NULL;
        out->count = 0;
        return false;
    }

    /* Sort by stable id so the catalog order is independent of the filesystem. */
    qsort(items, (size_t)count, sizeof(VehicleClass), compare_class_by_id);

    /* Duplicate ids would let two files claim one class identity; reject after sorting so the
     * report names the colliding pair. */
    for (int i = 1; i < count; i++) {
        if (strcmp(items[i - 1].id, items[i].id) == 0) {
            static char reason[192];
            snprintf(reason, sizeof(reason), "duplicate class id '%s'", items[i].id);
            set_error(error, errorCap, NULL, reason);
            free(items);
            out->items = NULL;
            out->count = 0;
            return false;
        }
    }

    out->items = items;
    out->count = count;
    return true;
}

void vehicle_class_catalog_free(VehicleClassCatalog *catalog)
{
    if (catalog == NULL) return;
    free(catalog->items);
    catalog->items = NULL;
    catalog->count = 0;
}

/* ------------------------------------------------------------------------------------- eligibility */

/* DrivetrainLayout is stored as a float tunable; the documented comparison pattern is a cast
 * to the enum. A value outside the enum is a data error and reads as "?" so the evidence
 * line stays honest instead of pretending to a layout. */
static const char *layout_name(int layout)
{
    switch (layout) {
        case DRIVE_LAYOUT_RWD: return "rwd";
        case DRIVE_LAYOUT_FWD: return "fwd";
        case DRIVE_LAYOUT_AWD: return "awd";
        default: return "?";
    }
}

/* Write one line of evidence, safely ignoring a NULL/zero-cap buffer. */
static void set_detail(char *detail, size_t detailCap, const char *fmt, ...)
{
    if (detail == NULL || detailCap == 0) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(detail, detailCap, fmt, args);
    va_end(args);
}

bool vehicle_class_check_eligibility(const VehicleClass *cls, const VehicleManifest *manifest,
                                     char *detail, size_t detailCap)
{
    if (detail != NULL && detailCap > 0) detail[0] = '\0';
    if (cls == NULL || manifest == NULL) {
        set_detail(detail, detailCap, "%s", cls == NULL ? "no class" : "no manifest");
        return false;
    }

    const VehicleSpec *spec = &manifest->definition.spec;

    /* Rule 0 — the tag rule: the manifest must carry this class id in classTags. This is the
     * primary membership signal a roster UI groups by; the numeric rules below only gate cars
     * that are already tagged, so a class cannot accidentally absorb an untagged car. */
    bool tagged = false;
    for (int i = 0; i < manifest->classTagCount; i++) {
        if (strcmp(manifest->classTags[i], cls->id) == 0) {
            tagged = true;
            break;
        }
    }
    if (!tagged) {
        set_detail(detail, detailCap,
                   "classTags missing \"%s\" (tag rule: membership requires the class tag)",
                   cls->id);
        return false;
    }

    /* Numeric rules read the manifest's derived spec: massKg is the sum of the authored mass
     * particles, peak torque is the max of the engine curve, tire mu is the stickier axle,
     * and layout is the drivetrainLayout enum. These are the same values a reviewer computes
     * from the physics section, so the class file bounds are checkable by hand. */
    const float massKg = spec->massKg;
    float peakTorqueNm = 0.0f;
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++) {
        if (spec->engineTorqueCurveNm[i] > peakTorqueNm) {
            peakTorqueNm = spec->engineTorqueCurveNm[i];
        }
    }
    const float maxMu =
        spec->tireMuLatFront > spec->tireMuLatRear ? spec->tireMuLatFront : spec->tireMuLatRear;
    const char *layout = layout_name((int)spec->drivetrainLayout);

    if (cls->hasMassRange && (massKg < cls->minMassKg || massKg > cls->maxMassKg)) {
        set_detail(detail, detailCap, "mass %.0f kg outside [%.0f, %.0f] kg", massKg,
                   cls->minMassKg, cls->maxMassKg);
        return false;
    }
    if (cls->hasTorqueRange &&
        (peakTorqueNm < cls->minPeakTorqueNm || peakTorqueNm > cls->maxPeakTorqueNm)) {
        set_detail(detail, detailCap, "peak torque %.0f Nm outside [%.0f, %.0f] Nm",
                   peakTorqueNm, cls->minPeakTorqueNm, cls->maxPeakTorqueNm);
        return false;
    }
    if (cls->hasMaxTireMu && maxMu > cls->maxTireMu) {
        set_detail(detail, detailCap, "max tire mu %.2f above the bound %.2f", maxMu,
                   cls->maxTireMu);
        return false;
    }
    if (cls->layoutCount > 0) {
        bool allowed = false;
        for (int i = 0; i < cls->layoutCount; i++) {
            if (strcmp(cls->layouts[i], layout) == 0) {
                allowed = true;
                break;
            }
        }
        if (!allowed) {
            char allowedList[VEHICLE_CLASS_MAX_LAYOUTS * 4 + 2];
            size_t used = 0;
            for (int i = 0; i < cls->layoutCount && used < sizeof(allowedList); i++) {
                used += (size_t)snprintf(allowedList + used, sizeof(allowedList) - used, "%s%s",
                                         i > 0 ? ", " : "", cls->layouts[i]);
            }
            set_detail(detail, detailCap, "drive layout '%s' not in allowed layouts (%s)",
                       layout, allowedList);
            return false;
        }
    }

    set_detail(detail, detailCap, "tag=%s mass=%.0fkg torque=%.0fNm mu=%.2f layout=%s eligible",
               cls->id, massKg, peakTorqueNm, maxMu, layout);
    return true;
}

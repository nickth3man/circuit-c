/*
 * vehicle_manifest.c — versioned vehicle content manifest loader/exporter. See the header.
 */
#include "content/vehicle_manifest.h"

#include "core/config.h"
#include "core/json.h"
#include "dev/dev_params.h"

#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The only keys permitted at the top level of a version-1 manifest. Anything else is a typo and
 * is rejected rather than silently dropped, so a misspelled section is a loud error. */
typedef enum {
    VM_KEY_SCHEMA,
    VM_KEY_VERSION,
    VM_KEY_ID,
    VM_KEY_DISPLAY_NAME,
    VM_KEY_DESCRIPTION,
    VM_KEY_CONTENT_VERSION,
    VM_KEY_APPEARANCE_ID,
    VM_KEY_CLASS_TAGS,
    VM_KEY_CONTROLLER_ELIGIBILITY,
    VM_KEY_PROVENANCE,
    VM_KEY_PHYSICS,
    VM_KEY_SETUP,
    VM_KEY_COUNT
} ManifestTopKey;

static const char *const kTopKeyNames[VM_KEY_COUNT] = {
    [VM_KEY_SCHEMA] = "schema",
    [VM_KEY_VERSION] = "version",
    [VM_KEY_ID] = "id",
    [VM_KEY_DISPLAY_NAME] = "displayName",
    [VM_KEY_DESCRIPTION] = "description",
    [VM_KEY_CONTENT_VERSION] = "contentVersion",
    [VM_KEY_APPEARANCE_ID] = "appearanceId",
    [VM_KEY_CLASS_TAGS] = "classTags",
    [VM_KEY_CONTROLLER_ELIGIBILITY] = "controllerEligibility",
    [VM_KEY_PROVENANCE] = "provenance",
    [VM_KEY_PHYSICS] = "physics",
    [VM_KEY_SETUP] = "setup",
};

static int top_key_index(const char *name)
{
    for (int i = 0; i < VM_KEY_COUNT; i++) {
        if (strcmp(name, kTopKeyNames[i]) == 0) return i;
    }
    return -1;
}

bool vehicle_manifest_id_is_valid(const char *id)
{
    if (id == NULL || id[0] == '\0') return false;
    /* First character: alphanumeric only. */
    if (!(isalnum((unsigned char)id[0]))) return false;
    for (size_t i = 1; id[i] != '\0'; i++) {
        const char c = id[i];
        if (!(isalnum((unsigned char)c) || c == '.' || c == '_' || c == '-')) return false;
        if (i > 62) return false; /* total length cap from the ownership contract */
    }
    return true;
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

/* A number whose fractional part is zero and which fits in an int. Used for contentVersion and
 * gearCount. */
static bool integer_value(const JsonValue *value, int min, int max, int *out)
{
    if (value == NULL || !json_is_number(value)) return false;
    const double n = json_as_number(value);
    if (!isfinite(n) || floor(n) != n) return false;
    if (n < (double)min || n > (double)max) return false;
    *out = (int)n;
    return true;
}

/* Apply one physics/setup float key. Validates that the key is real, belongs to the requested
 * owner, is not derived, and is in range before writing — errors are loud because a manifest is
 * reviewed content, not a tolerant profile. */
static bool apply_float_key(VehicleSpec *spec, const char *key, double value,
                            DevParamOwner requiredOwner, char *error, size_t errorCap)
{
    const DevParameter *param = dev_param_find(key);
    if (param == NULL) {
        set_error(error, errorCap, key, "unknown parameter");
        return false;
    }
    if (param->derived) {
        set_error(error, errorCap, key, "derived parameters are computed, not authored");
        return false;
    }
    if (param->owner != (int)requiredOwner) {
        static char reason[160];
        snprintf(reason, sizeof(reason),
                 "parameter belongs to %s, not %s — move it to the %s section",
                 dev_param_owner_name(param->owner), dev_param_owner_name((int)requiredOwner),
                 dev_param_owner_name(param->owner));
        set_error(error, errorCap, key, reason);
        return false;
    }
    if (!isfinite((float)value) || value < (double)param->minimum ||
        value > (double)param->maximum) {
        static char reason[160];
        snprintf(reason, sizeof(reason), "value %g is outside the valid range [%g, %g]", value,
                 (double)param->minimum, (double)param->maximum);
        set_error(error, errorCap, key, reason);
        return false;
    }
    /* dev_param_set clamps and refreshes; the value is already in range so it applies verbatim. */
    dev_param_set(spec, param, (float)value);
    return true;
}

/* drive.gear_count is the one setup value that is an int rather than a registry float, so it has
 * its own path. It is only valid in the setup section. */
static bool apply_gear_count(VehicleSpec *spec, const JsonValue *value, char *error,
                             size_t errorCap)
{
    int count = 0;
    if (!integer_value(value, 1, MAX_GEARS, &count)) {
        set_error(error, errorCap, "drive.gear_count", "expected an integer in [1, MAX_GEARS]");
        return false;
    }
    spec->gearCount = count;
    return true;
}

static bool apply_section(VehicleSpec *spec, const JsonValue *section, DevParamOwner owner,
                          char *error, size_t errorCap)
{
    if (section == NULL) return true; /* optional section absent */
    if (!json_is_object(section)) {
        set_error(error, errorCap, (owner == DEV_OWNER_DEFINITION) ? "physics" : "setup",
                  "expected an object");
        return false;
    }
    const int n = json_object_count(section);
    for (int i = 0; i < n; i++) {
        const char *key = json_object_key_at(section, i);
        const JsonValue *val = json_object_value_at(section, i);
        /* gear_count is setup-owned but lives in the typed audit table, not the float registry. */
        if (owner == DEV_OWNER_SETUP && strcmp(key, "drive.gear_count") == 0) {
            if (!apply_gear_count(spec, val, error, errorCap)) return false;
            continue;
        }
        if (!json_is_number(val)) {
            set_error(error, errorCap, key, "expected a number");
            return false;
        }
        if (!apply_float_key(spec, key, json_as_number(val), owner, error, errorCap)) {
            return false;
        }
    }
    return true;
}

static bool parse_class_tags(VehicleManifest *out, const JsonValue *tags, char *error,
                             size_t errorCap)
{
    if (tags == NULL) return true;
    if (!json_is_array(tags)) {
        set_error(error, errorCap, "classTags", "expected an array of strings");
        return false;
    }
    const int n = json_array_count(tags);
    if (n > VEHICLE_MANIFEST_MAX_CLASS_TAGS) {
        set_error(error, errorCap, "classTags", "too many tags");
        return false;
    }
    for (int i = 0; i < n; i++) {
        const JsonValue *tag = json_array_at(tags, i);
        const char *s = json_as_string(tag);
        if (s == NULL || s[0] == '\0') {
            set_error(error, errorCap, "classTags", "tags must be non-empty strings");
            return false;
        }
        if (strlen(s) + 1 > VEHICLE_MANIFEST_CLASS_TAG_CHARS) {
            set_error(error, errorCap, "classTags", "tag is too long");
            return false;
        }
        snprintf(out->classTags[i], VEHICLE_MANIFEST_CLASS_TAG_CHARS, "%s", s);
    }
    out->classTagCount = n;
    return true;
}

static bool parse_eligibility(VehicleManifest *out, const JsonValue *elig, char *error,
                              size_t errorCap)
{
    /* Absent eligibility means both human and AI may drive the car, which is the roster default. */
    out->eligibleHuman = true;
    out->eligibleAi = true;
    if (elig == NULL) return true;
    if (!json_is_array(elig)) {
        set_error(error, errorCap, "controllerEligibility", "expected an array of strings");
        return false;
    }
    bool sawHuman = false, sawAi = false;
    const int n = json_array_count(elig);
    for (int i = 0; i < n; i++) {
        const char *s = json_as_string(json_array_at(elig, i));
        if (s == NULL) {
            set_error(error, errorCap, "controllerEligibility", "entries must be strings");
            return false;
        }
        if (strcmp(s, "human") == 0) {
            sawHuman = true;
        } else if (strcmp(s, "ai") == 0) {
            sawAi = true;
        } else {
            static char reason[128];
            snprintf(reason, sizeof(reason),
                     "unknown controller '%s' (expected \"human\" or \"ai\")", s);
            set_error(error, errorCap, "controllerEligibility", reason);
            return false;
        }
    }
    out->eligibleHuman = sawHuman;
    out->eligibleAi = sawAi;
    return true;
}

static bool parse_provenance(VehicleManifest *out, const JsonValue *provenance, char *error,
                             size_t errorCap)
{
    if (provenance == NULL) return true;
    if (!json_is_object(provenance)) {
        set_error(error, errorCap, "provenance", "expected an object");
        return false;
    }
    if (!copy_text_field(out->provenanceSource, VEHICLE_MANIFEST_TEXT_CHARS,
                         json_as_string(json_object_get(provenance, "source")),
                         "provenance.source", error, errorCap))
        return false;
    if (!copy_text_field(out->provenanceAuthor, VEHICLE_MANIFEST_TEXT_CHARS,
                         json_as_string(json_object_get(provenance, "author")),
                         "provenance.author", error, errorCap))
        return false;
    return true;
}

bool vehicle_manifest_parse(const char *text, size_t length, VehicleManifest *out, char *error,
                            size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';

    JsonDocument *doc = json_parse(text, length, error, errorCap);
    if (doc == NULL) return false;
    const JsonValue *root = json_document_root(doc);

    const bool ok = (root != NULL && json_is_object(root));
    if (!ok) {
        set_error(error, errorCap, NULL, "manifest must be a JSON object");
        json_document_free(doc);
        return false;
    }

    /* Reject unknown top-level keys before doing anything else: a typo like "phyiscs" should not
     * silently produce a stock car. */
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

    const JsonValue *schema = json_object_get(root, kTopKeyNames[VM_KEY_SCHEMA]);
    if (schema == NULL || !json_is_string(schema) ||
        strcmp(json_as_string(schema), VEHICLE_MANIFEST_SCHEMA) != 0) {
        set_error(error, errorCap, "schema", "must be \"" VEHICLE_MANIFEST_SCHEMA "\"");
        json_document_free(doc);
        return false;
    }

    int version = 0;
    if (!integer_value(json_object_get(root, kTopKeyNames[VM_KEY_VERSION]),
                       VEHICLE_MANIFEST_VERSION, VEHICLE_MANIFEST_VERSION, &version)) {
        static char reason[96];
        snprintf(reason, sizeof(reason), "version must be %d (the only supported version)",
                 VEHICLE_MANIFEST_VERSION);
        set_error(error, errorCap, "version", reason);
        json_document_free(doc);
        return false;
    }

    const char *id = json_as_string(json_object_get(root, kTopKeyNames[VM_KEY_ID]));
    if (!vehicle_manifest_id_is_valid(id)) {
        set_error(error, errorCap, "id",
                  "must match [a-z0-9][a-z0-9._-]{0,62} (lowercase, filesystem-safe)");
        json_document_free(doc);
        return false;
    }
    if (strlen(id) + 1 > sizeof(out->definition.id)) {
        set_error(error, errorCap, "id", "value is too long for the content id field");
        json_document_free(doc);
        return false;
    }

    int contentVersion = 0;
    if (!integer_value(json_object_get(root, kTopKeyNames[VM_KEY_CONTENT_VERSION]), 1,
                       0x7fffffff, &contentVersion)) {
        set_error(error, errorCap, "contentVersion", "expected a positive integer");
        json_document_free(doc);
        return false;
    }

    const char *appearanceId =
        json_as_string(json_object_get(root, kTopKeyNames[VM_KEY_APPEARANCE_ID]));
    if (appearanceId == NULL || appearanceId[0] == '\0') {
        set_error(error, errorCap, "appearanceId", "must be a non-empty string");
        json_document_free(doc);
        return false;
    }
    if (strlen(appearanceId) + 1 > sizeof(out->definition.appearanceId)) {
        set_error(error, errorCap, "appearanceId", "value is too long");
        json_document_free(doc);
        return false;
    }

    if (!copy_text_field(
            out->displayName, VEHICLE_MANIFEST_TEXT_CHARS,
            json_as_string(json_object_get(root, kTopKeyNames[VM_KEY_DISPLAY_NAME])),
            "displayName", error, errorCap)) {
        json_document_free(doc);
        return false;
    }
    /* description is optional: omitting it leaves an empty string, but a present value must be a
     * string so a typo (e.g. a number) is still rejected. */
    const JsonValue *description = json_object_get(root, kTopKeyNames[VM_KEY_DESCRIPTION]);
    if (description != NULL && !json_is_string(description)) {
        set_error(error, errorCap, "description", "expected a string");
        json_document_free(doc);
        return false;
    }
    if (description != NULL &&
        !copy_text_field(out->description, VEHICLE_MANIFEST_DESC_CHARS,
                         json_as_string(description), "description", error, errorCap)) {
        json_document_free(doc);
        return false;
    }

    if (!parse_class_tags(out, json_object_get(root, kTopKeyNames[VM_KEY_CLASS_TAGS]), error,
                          errorCap) ||
        !parse_eligibility(out,
                           json_object_get(root, kTopKeyNames[VM_KEY_CONTROLLER_ELIGIBILITY]),
                           error, errorCap) ||
        !parse_provenance(out, json_object_get(root, kTopKeyNames[VM_KEY_PROVENANCE]), error,
                          errorCap)) {
        json_document_free(doc);
        return false;
    }

    /* Build the spec from defaults, then apply authored physics and setup keys in that order.
     * Both sections write VehicleSpec fields; the owner check above is what keeps a key in the
     * section its issue-12 owner assigns it to. */
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    if (!apply_section(&spec, json_object_get(root, kTopKeyNames[VM_KEY_PHYSICS]),
                       DEV_OWNER_DEFINITION, error, errorCap) ||
        !apply_section(&spec, json_object_get(root, kTopKeyNames[VM_KEY_SETUP]),
                       DEV_OWNER_SETUP, error, errorCap)) {
        json_document_free(doc);
        return false;
    }
    dev_params_refresh_derived(&spec);
    if (!vehicle_spec_is_valid(&spec)) {
        set_error(error, errorCap, NULL, "resulting spec fails vehicle_spec_is_valid()");
        json_document_free(doc);
        return false;
    }

    if (!vehicle_definition_init(&out->definition, id, appearanceId, (uint32_t)contentVersion,
                                 &spec)) {
        set_error(error, errorCap, NULL,
                  "could not build a VehicleDefinition from the manifest");
        json_document_free(doc);
        return false;
    }
    /* The default setup mirrors the authored setup fields straight out of the definition spec,
     * so a manifest's setup section is its default setup and the two cannot disagree. */
    vehicle_setup_set_default(&out->definition, &out->defaultSetup);
    if (!vehicle_setup_is_valid(&out->definition, &out->defaultSetup)) {
        set_error(error, errorCap, NULL,
                  "default setup fails validation against its definition");
        json_document_free(doc);
        return false;
    }

    out->manifestHash = json_canonical_hash(root);
    json_document_free(doc);
    return true;
}

bool vehicle_manifest_load(const char *path, VehicleManifest *out, char *error, size_t errorCap)
{
    if (out == NULL) return false;
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (path == NULL) {
        set_error(error, errorCap, NULL, "no manifest path given");
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
    const bool ok = vehicle_manifest_parse(buffer, read, out, error, errorCap);
    free(buffer);
    return ok;
}

static int compare_manifest_by_id(const void *a, const void *b)
{
    const VehicleManifest *ma = (const VehicleManifest *)a;
    const VehicleManifest *mb = (const VehicleManifest *)b;
    return strcmp(ma->definition.id, mb->definition.id);
}

/* Read a whole file into a malloc'd buffer. Returns NULL (and sets the error) on failure. */
static char *read_file_text(const char *path, char *error, size_t errorCap)
{
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
    return buffer;
}

bool vehicle_manifest_load_dir(const char *dir, VehicleCatalog *out, char *error,
                               size_t errorCap)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (error != NULL && errorCap > 0) error[0] = '\0';
    if (dir == NULL) {
        set_error(error, errorCap, NULL, "no catalog directory given");
        return false;
    }

    DIR *d = opendir(dir);
    if (d == NULL) {
        set_error(error, errorCap, dir, "could not open directory");
        return false;
    }

    /* First pass: count candidate files so the result array is one allocation. Filesystem
     * enumeration order is explicitly not relied upon — the catalog is sorted by id afterwards. */
    int fileCount = 0;
    const struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        const char *name = entry->d_name;
        const size_t len = strlen(name);
        if (len > strlen(".vehicle.json") &&
            strcmp(name + len - strlen(".vehicle.json"), ".vehicle.json") == 0) {
            fileCount++;
        }
    }
    rewinddir(d);

    VehicleManifest *items = NULL;
    if (fileCount > 0) {
        items = (VehicleManifest *)calloc((size_t)fileCount, sizeof(VehicleManifest));
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
        const size_t suffixLen = strlen(".vehicle.json");
        if (len <= suffixLen || strcmp(name + len - suffixLen, ".vehicle.json") != 0) continue;

        char path[1024];
        const int written = snprintf(path, sizeof(path), "%s/%s", dir, name);
        if (written < 0 || written >= (int)sizeof(path)) {
            set_error(error, errorCap, name, "path is too long");
            ok = false;
            break;
        }
        char *text = read_file_text(path, error, errorCap);
        if (text == NULL) {
            ok = false;
            break;
        }
        if (!vehicle_manifest_parse(text, strlen(text), &items[count], error, errorCap)) {
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
    qsort(items, (size_t)count, sizeof(VehicleManifest), compare_manifest_by_id);

    /* Duplicate ids would let two files claim one content identity; reject after sorting so the
     * report names the colliding pair. */
    for (int i = 1; i < count; i++) {
        if (strcmp(items[i - 1].definition.id, items[i].definition.id) == 0) {
            static char reason[192];
            snprintf(reason, sizeof(reason), "duplicate content id '%s'",
                     items[i].definition.id);
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

void vehicle_catalog_free(VehicleCatalog *catalog)
{
    if (catalog == NULL) return;
    free(catalog->items);
    catalog->items = NULL;
    catalog->count = 0;
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
            case '\b': fputs("\\b", out); break;
            case '\f': fputs("\\f", out); break;
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

bool vehicle_manifest_write(const VehicleSpec *spec, const char *id, const char *displayName,
                            const char *appearanceId, uint32_t contentVersion,
                            const char *source, FILE *out)
{
    if (spec == NULL || id == NULL || displayName == NULL || appearanceId == NULL ||
        out == NULL)
        return false;

    fprintf(out, "{\n");
    fprintf(out, "  \"schema\": \"%s\",\n", VEHICLE_MANIFEST_SCHEMA);
    fprintf(out, "  \"version\": %d,\n", VEHICLE_MANIFEST_VERSION);
    fprintf(out, "  \"id\": ");
    write_json_string(out, id);
    fprintf(out, ",\n  \"displayName\": ");
    write_json_string(out, displayName);
    fprintf(out, ",\n  \"description\": \"\",\n");
    fprintf(out, "  \"contentVersion\": %u,\n", contentVersion);
    fprintf(out, "  \"appearanceId\": ");
    write_json_string(out, appearanceId);
    fprintf(out, ",\n  \"classTags\": [],\n");
    fprintf(out, "  \"controllerEligibility\": [\"human\", \"ai\"],\n");
    fprintf(out, "  \"provenance\": { \"source\": ");
    write_json_string(out, source != NULL ? source : "");
    fprintf(out, ", \"author\": \"circuit-c\" },\n");

    /* Emit every non-derived registry key the spec sets, routed to physics or setup by its
     * issue-12 owner. %.9g round-trips every float through a double back to the same float bits. */
    bool firstPhysics = true;
    bool firstSetup = true;
    fprintf(out, "  \"physics\": {");
    for (int i = 0; i < dev_params_count(); i++) {
        const DevParameter *param = dev_param_at(i);
        if (param->derived) continue;
        if (param->owner == DEV_OWNER_DEFINITION) {
            fprintf(out, "%s\n    ", firstPhysics ? "" : ",");
            write_json_string(out, param->name);
            fprintf(out, ": %.9g", (double)dev_param_get(spec, param));
            firstPhysics = false;
        }
    }
    fprintf(out, "%s  },\n", firstPhysics ? "" : "\n");

    fprintf(out, "  \"setup\": {");
    fprintf(out, "%s\n    \"drive.gear_count\": %d", firstSetup ? "" : ",", spec->gearCount);
    firstSetup = false;
    for (int i = 0; i < dev_params_count(); i++) {
        const DevParameter *param = dev_param_at(i);
        if (param->derived) continue;
        if (param->owner == DEV_OWNER_SETUP) {
            fprintf(out, ",\n    ");
            write_json_string(out, param->name);
            fprintf(out, ": %.9g", (double)dev_param_get(spec, param));
            (void)firstSetup;
        }
    }
    fprintf(out, "\n  }\n}\n");
    return true;
}

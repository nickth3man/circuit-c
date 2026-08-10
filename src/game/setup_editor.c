/*
 * setup_editor.c — see setup_editor.h. The dev_params registry is the single source of truth
 * for which setup knobs exist, their bounds, units, and explanations; this file adds only the
 * key -> VehicleSetup field mapping, which mirrors the writes in static vehicle_setup_apply().
 *
 * ONE BINDING TABLE, NOT TWO LISTS. The registry says which keys are editable; kSetupFields
 * says where each key lives inside VehicleSetup. Those two must agree, so the table is the only
 * place the mapping is written and init() admits a registry key *only* if the table binds it.
 * Adding a setup-owned physics input to dev_params.c without a binding here therefore produces
 * no item at all — a visible omission the `setup-editor` scenario asserts against — instead of
 * a knob that displays 0.0 and does nothing when turned.
 *
 * DEV REGISTRY OFFSETS ARE DELIBERATELY UNUSED. DevParameter::offset indexes VehicleSpec, and
 * VehicleSetup is a different struct with a different layout; using it here would write the
 * wrong memory. The offsets below are this module's own, taken against VehicleSetup.
 */
#include "game/setup_editor.h"

#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/dev_params.h"

/*
 * key -> VehicleSetup float field, mirroring vehicle_setup_apply() field by field. Every entry
 * is a float, so one byte offset is enough to read and write it.
 *
 * Only gear1..gear5 are bound even though VehicleSetup carries MAX_GEARS ratios: the registry
 * publishes five, and the editor must never offer a gear count it has no ratio control for
 * (see kEditableGearRatios below).
 */
typedef struct {
    const char *key;
    size_t offset;
    bool isGearRatio;
} SetupFieldBinding;

static const SetupFieldBinding kSetupFields[] = {
    { "drive.gear1", offsetof(VehicleSetup, gearRatios[0]), true },
    { "drive.gear2", offsetof(VehicleSetup, gearRatios[1]), true },
    { "drive.gear3", offsetof(VehicleSetup, gearRatios[2]), true },
    { "drive.gear4", offsetof(VehicleSetup, gearRatios[3]), true },
    { "drive.gear5", offsetof(VehicleSetup, gearRatios[4]), true },
    { "drive.reverse", offsetof(VehicleSetup, reverseGearRatio), false },
    { "drive.final", offsetof(VehicleSetup, finalDriveRatio), false },
    { "drive.diff_mode", offsetof(VehicleSetup, differentialMode), false },
    { "drive.diff_bias_ratio", offsetof(VehicleSetup, differentialBiasRatio), false },
    { "drive.diff_preload", offsetof(VehicleSetup, differentialPreloadNm), false },
    { "brake.bias_front", offsetof(VehicleSetup, brakeBiasFront), false },
    /* Static alignment became an active dynamics input in issue #14, so it is offered here:
     * the audit rule is that a setup-owned physics input the player cannot reach is a knob the
     * car has and the menu hides. Camber and caster stay out until their own issues activate
     * them. */
    { "susp.toe_front", offsetof(VehicleSetup, suspToeFrontRad), false },
    { "susp.toe_rear", offsetof(VehicleSetup, suspToeRearRad), false },
};

static const int kSetupFieldCount = (int)(sizeof(kSetupFields) / sizeof(kSetupFields[0]));

static const SetupFieldBinding *setup_field_binding(const char *key)
{
    for (int i = 0; i < kSetupFieldCount; i++) {
        if (strcmp(kSetupFields[i].key, key) == 0) return &kSetupFields[i];
    }
    return NULL;
}

/* How many forward gear ratios the editor can actually reach. This — not MAX_GEARS — is the
 * ceiling on drive.gear_count: raising the count past the last editable ratio would commit the
 * player to a gear whose ratio is whatever the manifest left there (often 0, which
 * vehicle_setup_is_valid rejects) with no control in the menu to repair it. */
static int editable_gear_ratio_count(void)
{
    int count = 0;
    for (int i = 0; i < kSetupFieldCount; i++) {
        if (kSetupFields[i].isGearRatio) count++;
    }
    return count < MAX_GEARS ? count : MAX_GEARS;
}

/* Copy a bounded string with a fallback for NULL or empty sources ("-" for unitless). */
static void copy_text(char *dst, size_t cap, const char *src, const char *fallback)
{
    if (src == NULL || src[0] == '\0') src = fallback;
    (void)snprintf(dst, cap, "%.*s", (int)(cap - 1), src);
}

/* Unknown keys read 0.0f and are ignored on write: setup_editor_init only admits bound keys,
 * so this is defensive, not a supported path. */
static float setup_field_value(const VehicleSetup *setup, const char *key)
{
    const SetupFieldBinding *binding = setup_field_binding(key);
    if (binding == NULL) return 0.0f;
    float value = 0.0f;
    memcpy(&value, (const char *)setup + binding->offset, sizeof(value));
    return value;
}

static void setup_field_set(VehicleSetup *setup, const char *key, float value)
{
    const SetupFieldBinding *binding = setup_field_binding(key);
    if (binding == NULL) return;
    memcpy((char *)setup + binding->offset, &value, sizeof(value));
}

void setup_editor_init(SetupEditor *ed, const VehicleDefinition *def,
                       const VehicleSetup *baseline)
{
    if (ed == NULL) return;
    memset(ed, 0, sizeof(*ed));
    if (def == NULL || baseline == NULL) return;
    ed->base = def;
    ed->baseline = *baseline;
    ed->working = *baseline;

    /* Registry sweep: every setup-owned entry a dynamics path actually reads. The
     * inactive/appearance setup keys are excluded here on purpose — see setup_editor.h. */
    for (int i = 0; i < dev_params_count() && ed->itemCount < SETUP_EDITOR_MAX_ITEMS; i++) {
        const DevParameter *param = dev_param_at(i);
        if (param->owner != DEV_OWNER_SETUP) continue;
        if (param->classification != DEV_CLASS_PHYSICS_INPUT) continue;
        /* No binding, no item: an editable-looking knob that writes nothing is worse than an
         * absent one, and `unboundKeyCount` makes the omission assertable. */
        if (setup_field_binding(param->name) == NULL) {
            ed->unboundKeyCount++;
            continue;
        }

        SetupEditorItem *item = &ed->items[ed->itemCount++];
        copy_text(item->key, sizeof(item->key), param->name, "");
        copy_text(item->unit, sizeof(item->unit), param->unit, "-");
        copy_text(item->explanation, sizeof(item->explanation), param->description, "");
        item->min = param->minimum;
        item->max = param->maximum;
        item->step = param->step;
        item->isGearCount = false;
    }

    /* The typed int gear count is not a float registry entry (DevSpecFieldAudit holds it),
     * so it is appended as the model's own item, capped at the ratios the editor exposes. */
    if (ed->itemCount < SETUP_EDITOR_MAX_ITEMS) {
        SetupEditorItem *item = &ed->items[ed->itemCount++];
        copy_text(item->key, sizeof(item->key), "drive.gear_count", "");
        copy_text(item->unit, sizeof(item->unit), "", "-");
        copy_text(item->explanation, sizeof(item->explanation),
                  "Active forward gear count; the ratios themselves are the drive.gearN items.",
                  "");
        item->min = 1.0f;
        item->max = (float)editable_gear_ratio_count();
        item->step = 1.0f;
        item->isGearCount = true;
    }
}

bool setup_editor_adjust(SetupEditor *ed, int itemIndex, int direction)
{
    if (ed == NULL || itemIndex < 0 || itemIndex >= ed->itemCount || direction == 0)
        return false;

    const SetupEditorItem *item = &ed->items[itemIndex];
    if (item->isGearCount) {
        /* Clamped to the item's own maximum, which is the editable-ratio count rather than
         * MAX_GEARS — so every count the menu can reach has an editable ratio behind it. */
        const int ceiling = (int)item->max;
        const int current = ed->working.gearCount;
        int next = current + (direction > 0 ? 1 : -1);
        if (next < 1) next = 1;
        /* A setup authored with more gears than the editor exposes keeps them: the count can
         * be lowered from there, just never raised past the last ratio the player can edit. */
        if (next > ceiling && next > current) next = current;
        ed->working.gearCount = next;
        return true;
    }

    const char *key = item->key;
    float next =
        setup_field_value(&ed->working, key) + (direction > 0 ? item->step : -item->step);
    if (next < item->min) next = item->min;
    if (next > item->max) next = item->max;
    setup_field_set(&ed->working, key, next);
    return true;
}

void setup_editor_reset(SetupEditor *ed)
{
    if (ed == NULL) return;
    ed->working = ed->baseline;
}

bool setup_editor_is_valid(const SetupEditor *ed)
{
    if (ed == NULL || ed->base == NULL) return false;
    return vehicle_setup_is_valid(ed->base, &ed->working);
}

uint32_t setup_editor_hash(const SetupEditor *ed)
{
    if (ed == NULL) return vehicle_setup_hash(NULL);
    return vehicle_setup_hash(&ed->working);
}
float setup_editor_value(const SetupEditor *ed, int itemIndex)
{
    if (ed == NULL || itemIndex < 0 || itemIndex >= ed->itemCount) return 0.0f;
    if (ed->items[itemIndex].isGearCount) return (float)ed->working.gearCount;
    return setup_field_value(&ed->working, ed->items[itemIndex].key);
}

bool setup_editor_can_start(const SetupEditor *ed, char *reason, size_t reasonCap)
{
    if (ed == NULL || ed->base == NULL) {
        if (reason != NULL && reasonCap > 0) snprintf(reason, reasonCap, "no editor or base");
        return false;
    }
    if (!vehicle_setup_is_valid(ed->base, &ed->working)) {
        if (reason != NULL && reasonCap > 0)
            snprintf(reason, reasonCap, "setup outside vehicle bounds");
        return false;
    }
    return true;
}

bool setup_editor_save(const SetupEditor *ed, const char *path, char *error, size_t errorCap)
{
    if (ed == NULL || path == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "no editor or path");
        return false;
    }
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "cannot open %s", path);
        return false;
    }
    for (int i = 0; i < ed->itemCount; i++) {
        const float v = setup_editor_value(ed, i);
        if (ed->items[i].isGearCount) {
            fprintf(f, "%s=%d\n", ed->items[i].key, (int)v);
        } else {
            fprintf(f, "%s=%.9g\n", ed->items[i].key, (double)v);
        }
    }
    /* Append deterministic hash anchor so a replay can verify byte-for-byte equality. */
    fprintf(f, "setup.hash=%08x\n", setup_editor_hash(ed));
    if (fclose(f) != 0) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "close failed for %s", path);
        return false;
    }
    return true;
}

bool setup_editor_load(SetupEditor *ed, const char *path, char *error, size_t errorCap)
{
    if (ed == NULL || ed->base == NULL || path == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "no editor/base or path");
        return false;
    }
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "cannot open %s", path);
        return false;
    }
    VehicleSetup loaded = ed->working;
    char line[256];
    int lineNo = 0;
    uint32_t fileHash = 0;
    bool sawHash = false;
    while (fgets(line, sizeof(line), f) != NULL) {
        lineNo++;
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) line[--len] = '\0';
        if (len == 0 || line[0] == '#') continue;
        char *eq = strchr(line, '=');
        if (eq == NULL) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "line %d: missing =", lineNo);
            fclose(f);
            return false;
        }
        *eq = '\0';
        const char *key = line;
        const char *valStr = eq + 1;
        if (strcmp(key, "setup.hash") == 0) {
            /*
             * A hash line that does not parse must be an error, not a shrug. Leaving `sawHash`
             * false on a malformed value silently turns verification off for the whole file —
             * so the one line whose job is to detect corruption would be disabled by exactly
             * the corruption it exists to catch. The format save writes is eight hex digits,
             * and nothing else is accepted.
             */
            bool hexOk = (strlen(valStr) == 8);
            for (const char *p = valStr; hexOk && *p != '\0'; p++) {
                hexOk = isxdigit((unsigned char)*p) != 0;
            }
            unsigned int h = 0;
            if (!hexOk || sscanf(valStr, "%x", &h) != 1) {
                if (error != NULL && errorCap > 0)
                    snprintf(error, errorCap, "line %d: malformed setup.hash '%s'", lineNo,
                             valStr);
                fclose(f);
                return false;
            }
            fileHash = (uint32_t)h;
            sawHash = true;
            continue;
        }
        /* Locate editable item. */
        int idx = -1;
        for (int i = 0; i < ed->itemCount; i++)
            if (strcmp(ed->items[i].key, key) == 0) {
                idx = i;
                break;
            }
        if (idx < 0) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "line %d: unknown key %s", lineNo, key);
            fclose(f);
            return false;
        }
        char *end = NULL;
        double d = strtod(valStr, &end);
        if (end == valStr || !isfinite(d)) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "line %d: bad value %s", lineNo, valStr);
            fclose(f);
            return false;
        }
        if (ed->items[idx].isGearCount) {
            /* The range test happens in double precision, BEFORE narrowing. lrint() of a
             * finite-but-huge double (1e300) has an unspecified result and raises a domain
             * error, and narrowing that to int is undefined — so a file could invoke UB on
             * its way to being rejected. Bound it first, then convert.
             *
             * The bound is MAX_GEARS, not the editor's five-ratio ceiling. That ceiling exists
             * to stop the menu creating a gear count it cannot repair; it is not a claim that
             * six- to eight-gear setups are invalid, and applying it here would make a setup
             * that saved cleanly fail to load. The vehicle_setup_is_valid() call below is the
             * real gate: a count whose required ratios are not authored is rejected there. */
            if (!(d >= 1.0 && d <= (double)MAX_GEARS) || (double)(int)d != d) {
                if (error != NULL && errorCap > 0)
                    snprintf(error, errorCap, "line %d: gear_count %s out of range", lineNo,
                             valStr);
                fclose(f);
                return false;
            }
            loaded.gearCount = (int)d;
        } else {
            if (d < (double)ed->items[idx].min - 1e-6 ||
                d > (double)ed->items[idx].max + 1e-6) {
                if (error != NULL && errorCap > 0)
                    snprintf(error, errorCap, "line %d: %s=%s outside [%.4g,%.4g]", lineNo, key,
                             valStr, (double)ed->items[idx].min, (double)ed->items[idx].max);
                fclose(f);
                return false;
            }
            setup_field_set(&loaded, key, (float)d);
        }
    }
    fclose(f);
    if (!vehicle_setup_is_valid(ed->base, &loaded)) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "loaded setup outside vehicle bounds");
        return false;
    }
    const uint32_t loadedHash = vehicle_setup_hash(&loaded);
    if (sawHash && loadedHash != fileHash) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "hash mismatch: file %08x != computed %08x", fileHash,
                     loadedHash);
        return false;
    }
    ed->working = loaded;
    return true;
}

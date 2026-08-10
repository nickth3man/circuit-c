/*
 * setup_editor.c — see setup_editor.h. The dev_params registry is the single source of truth
 * for which setup knobs exist, their bounds, units, and explanations; this file adds only the
 * key -> VehicleSetup field mapping, which mirrors the writes in static vehicle_setup_apply().
 *
 * OFFSETS ARE DELIBERATELY UNUSED. DevParameter::offset indexes VehicleSpec, and VehicleSetup
 * is a different struct with different layout; using it here would write the wrong memory.
 * The explicit if/else on the key string is the same mapping vehicle_setup_apply() performs,
 * so the editor and the physics can never disagree about what a key means.
 */
#include "game/setup_editor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/dev_params.h"

/* Copy a bounded string with a fallback for NULL or empty sources ("-" for unitless). */
static void copy_text(char *dst, size_t cap, const char *src, const char *fallback)
{
    if (src == NULL || src[0] == '\0') src = fallback;
    (void)snprintf(dst, cap, "%.*s", (int)(cap - 1), src);
}

/* key -> VehicleSetup read/write, mirroring vehicle_setup_apply() field by field. Unknown
 * keys read 0.0f and are ignored on write: setup_editor_init only admits registry keys, so
 * this is defensive, not a supported path. */
static float setup_field_value(const VehicleSetup *setup, const char *key)
{
    if (strcmp(key, "drive.gear1") == 0) return setup->gearRatios[0];
    if (strcmp(key, "drive.gear2") == 0) return setup->gearRatios[1];
    if (strcmp(key, "drive.gear3") == 0) return setup->gearRatios[2];
    if (strcmp(key, "drive.gear4") == 0) return setup->gearRatios[3];
    if (strcmp(key, "drive.gear5") == 0) return setup->gearRatios[4];
    if (strcmp(key, "drive.reverse") == 0) return setup->reverseGearRatio;
    if (strcmp(key, "drive.final") == 0) return setup->finalDriveRatio;
    if (strcmp(key, "drive.diff_mode") == 0) return setup->differentialMode;
    if (strcmp(key, "drive.diff_bias_ratio") == 0) return setup->differentialBiasRatio;
    if (strcmp(key, "drive.diff_preload") == 0) return setup->differentialPreloadNm;
    if (strcmp(key, "brake.bias_front") == 0) return setup->brakeBiasFront;
    return 0.0f;
}

static void setup_field_set(VehicleSetup *setup, const char *key, float value)
{
    if (strcmp(key, "drive.gear1") == 0)
        setup->gearRatios[0] = value;
    else if (strcmp(key, "drive.gear2") == 0)
        setup->gearRatios[1] = value;
    else if (strcmp(key, "drive.gear3") == 0)
        setup->gearRatios[2] = value;
    else if (strcmp(key, "drive.gear4") == 0)
        setup->gearRatios[3] = value;
    else if (strcmp(key, "drive.gear5") == 0)
        setup->gearRatios[4] = value;
    else if (strcmp(key, "drive.reverse") == 0)
        setup->reverseGearRatio = value;
    else if (strcmp(key, "drive.final") == 0)
        setup->finalDriveRatio = value;
    else if (strcmp(key, "drive.diff_mode") == 0)
        setup->differentialMode = value;
    else if (strcmp(key, "drive.diff_bias_ratio") == 0)
        setup->differentialBiasRatio = value;
    else if (strcmp(key, "drive.diff_preload") == 0)
        setup->differentialPreloadNm = value;
    else if (strcmp(key, "brake.bias_front") == 0)
        setup->brakeBiasFront = value;
    /* Unknown key: no field to write; the caller's index could only come from init(). */
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
     * so it is appended as the model's own item with the same bounds the physics enforces. */
    if (ed->itemCount < SETUP_EDITOR_MAX_ITEMS) {
        SetupEditorItem *item = &ed->items[ed->itemCount++];
        copy_text(item->key, sizeof(item->key), "drive.gear_count", "");
        copy_text(item->unit, sizeof(item->unit), "", "-");
        copy_text(item->explanation, sizeof(item->explanation),
                  "Active forward gear count; the ratios themselves are the drive.gearN items.",
                  "");
        item->min = 1.0f;
        item->max = (float)MAX_GEARS;
        item->step = 1.0f;
        item->isGearCount = true;
    }
}

bool setup_editor_adjust(SetupEditor *ed, int itemIndex, int direction)
{
    if (ed == NULL || itemIndex < 0 || itemIndex >= ed->itemCount || direction == 0)
        return false;

    SetupEditorItem *item = &ed->items[itemIndex];
    if (item->isGearCount) {
        int next = ed->working.gearCount + (direction > 0 ? 1 : -1);
        if (next < 1) next = 1;
        if (next > MAX_GEARS) next = MAX_GEARS;
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
            unsigned int h = 0;
            if (sscanf(valStr, "%x", &h) == 1) {
                fileHash = (uint32_t)h;
                sawHash = true;
            }
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
            int iv = (int)lrint(d);
            if (iv < 1 || iv > MAX_GEARS || (double)iv != d) {
                if (error != NULL && errorCap > 0)
                    snprintf(error, errorCap, "line %d: gear_count %s out of range", lineNo,
                             valStr);
                fclose(f);
                return false;
            }
            loaded.gearCount = iv;
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

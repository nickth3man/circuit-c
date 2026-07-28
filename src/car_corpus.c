/*
 * car_corpus.c — the demonstration fleet. See the header for the contract.
 *
 * Sweep bounds are read from the DevParameter registry rather than hardcoded, so widening a
 * parameter's range in Phase 2 automatically widens the sweep that demonstrates it, and the
 * two can never disagree.
 *
 * Raylib-free. Lives in DEV_SRCS rather than SHARED_SRCS because it calls dev_preset_apply();
 * the hot-reload harness links SHARED_SRCS without the dev sources and does not need a corpus.
 */
#include "car_corpus.h"

#include <stdio.h>
#include <string.h>

#include "dev_params.h"
#include "dev_presets.h"

/* Steps per sweep axis. Five is enough to read a trend across a contact-sheet row without
 * the row dominating the page. */
#define SWEEP_STEPS 5

/* The registry keys whose visual effect Phase 1 can honestly demonstrate. Each must name a
 * parameter that car_visual.c actually reads AND that visibly changes the rendering; the
 * `car-visual` scenario asserts the latter, which is what catches an axis going dead.
 *
 * Deliberately NOT here: brake.max_torque and wheel.inertia. car_visual.c does derive a brake
 * disc diameter and a spoke count from them, but on a normal car the wheels sit entirely under
 * the body, and at the ~13 px/m the game draws at the wheel interior is a pixel or two wide.
 * Both axes measured as literally zero-pixel changes. Listing them anyway would advertise
 * variety the player cannot see — the same dishonesty as deriving tire width from grip. They
 * become real drivers in Phase 2, when tire width, wheel offset and ride height expose the
 * wheel at the body edge. */
static const char *const kSweepKeys[] = {
    "body.cg_to_rear",         /* wheelbase and weight bias -> length, greenhouse position */
    "body.cg_to_front",        /* the same lever from the other end -> bonnet length */
    "body.track_rear",         /* rear stance -> hips, arch flare, wheels proud of the body */
    "body.track_front",        /* front stance */
    "collision.half_width",    /* body width -> the whole silhouette */
    "wheel.radius",            /* tire diameter */
    "body.mass",               /* bulk latent -> tail fullness, stripped-out cues */
    "body.drag_coefficient",   /* nose taper and whether a wing appears */
};

#define SWEEP_AXES ((int)(sizeof(kSweepKeys) / sizeof(kSweepKeys[0])))

static int archetype_count(void)
{
    return dev_preset_count();
}

static int sweep_count(void)
{
    return SWEEP_AXES * SWEEP_STEPS;
}

int car_corpus_count(void)
{
    return archetype_count() + sweep_count();
}

CarCorpusGroup car_corpus_group(int index)
{
    if (index < archetype_count()) return CAR_CORPUS_ARCHETYPE;
    return CAR_CORPUS_SWEEP;
}

const char *car_corpus_group_name(CarCorpusGroup group)
{
    switch (group) {
        case CAR_CORPUS_ARCHETYPE: return "archetype";
        case CAR_CORPUS_SWEEP:     return "sweep";
        case CAR_CORPUS_SAMPLED:   return "sampled";
        default:                   return "?";
    }
}

/* Decompose a sweep index into its axis and step. Returns false for archetype indices. */
static bool sweep_slot(int index, int *axisOut, int *stepOut)
{
    const int base = archetype_count();
    if (index < base || index >= car_corpus_count()) return false;
    const int offset = index - base;
    if (axisOut != NULL) *axisOut = offset / SWEEP_STEPS;
    if (stepOut != NULL) *stepOut = offset % SWEEP_STEPS;
    return true;
}

const char *car_corpus_sweep_key(int index)
{
    int axis = 0;
    if (!sweep_slot(index, &axis, NULL)) return NULL;
    if (axis < 0 || axis >= SWEEP_AXES) return NULL;
    return kSweepKeys[axis];
}

/* The value this sweep step sets, spread evenly across the registry's declared range. */
static bool sweep_value(int index, const DevParameter **paramOut, float *valueOut)
{
    int axis = 0, step = 0;
    if (!sweep_slot(index, &axis, &step)) return false;

    const DevParameter *param = dev_param_find(kSweepKeys[axis]);
    if (param == NULL) return false;

    const float t = (SWEEP_STEPS > 1) ? ((float)step / (float)(SWEEP_STEPS - 1)) : 0.0f;
    if (paramOut != NULL) *paramOut = param;
    if (valueOut != NULL) *valueOut = param->minimum + (param->maximum - param->minimum) * t;
    return true;
}

bool car_corpus_spec(int index, VehicleSpec *out)
{
    if (out == NULL || index < 0 || index >= car_corpus_count()) return false;

    vehicle_spec_set_default(out);

    if (index < archetype_count()) {
        /* dev_preset_apply resets to stock first, so this is order-independent. */
        (void)dev_preset_apply(out, index);
        return true;
    }

    const DevParameter *param = NULL;
    float value = 0.0f;
    if (!sweep_value(index, &param, &value)) return false;

    /* dev_param_set clamps to the declared range and refreshes derived fields. */
    (void)dev_param_set(out, param, value);
    return true;
}

/* Lower-case, replace anything awkward with '_', so an id is safe as a filename. */
static void slugify(const char *src, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    size_t n = 0;
    for (; src != NULL && *src != '\0' && n + 1 < cap; src++) {
        const char c = *src;
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            buf[n++] = c;
        } else if (c >= 'A' && c <= 'Z') {
            buf[n++] = (char)(c - 'A' + 'a');
        } else if (n > 0 && buf[n - 1] != '_') {
            buf[n++] = '_';
        }
    }
    while (n > 0 && buf[n - 1] == '_') n--;
    buf[n] = '\0';
}

void car_corpus_id(int index, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    buf[0] = '\0';
    if (index < 0 || index >= car_corpus_count()) return;

    if (index < archetype_count()) {
        const DevPreset *preset = dev_preset_at(index);
        char slug[48];
        slugify(preset != NULL ? preset->name : "unknown", slug, sizeof(slug));
        snprintf(buf, cap, "archetype_%02d_%s", index, slug);
        return;
    }

    int axis = 0, step = 0;
    (void)sweep_slot(index, &axis, &step);
    char slug[48];
    slugify(kSweepKeys[axis], slug, sizeof(slug));
    snprintf(buf, cap, "sweep_%s_%d", slug, step);
}

void car_corpus_describe(int index, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    buf[0] = '\0';
    if (index < 0 || index >= car_corpus_count()) return;

    if (index < archetype_count()) {
        const DevPreset *preset = dev_preset_at(index);
        snprintf(buf, cap, "%s — %s",
                 preset != NULL ? preset->name : "?",
                 preset != NULL ? preset->description : "");
        return;
    }

    int step = 0;
    const DevParameter *param = NULL;
    float value = 0.0f;
    (void)sweep_slot(index, NULL, &step);
    if (!sweep_value(index, &param, &value) || param == NULL) return;

    snprintf(buf, cap, "%s = %.3f %s (step %d/%d)",
             param->name, (double)value,
             (param->unit != NULL && param->unit[0] != '\0') ? param->unit : "",
             step + 1, SWEEP_STEPS);
}

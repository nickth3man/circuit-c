/*
 * setup_editor.h — headless setup-editing model (issue #33), shared by a menu screen and the
 * test suite. Raylib-free: the model is pure state + pure functions over VehicleSetup.
 *
 * WHAT IS EDITABLE. The editable items are exactly the registry entries that are both
 * setup-owned (DEV_OWNER_SETUP) and read by a dynamics path (DEV_CLASS_PHYSICS_INPUT):
 * drive.gear1..gear5, drive.reverse, drive.final, drive.diff_mode, drive.diff_bias_ratio,
 * drive.diff_preload, brake.bias_front — plus the typed int drive.gear_count (1..MAX_GEARS),
 * which lives outside the float registry but is a genuine physics input.
 *
 * WHAT IS NOT EDITABLE, AND WHY. The remaining setup-owned keys — tire pressures
 * (DEV_CLASS_INACTIVE) and camber/toe/caster (DEV_CLASS_APPEARANCE or INACTIVE) — are
 * deliberately absent. Editing a dormant value would claim an effect the physics does not
 * have: nothing reads tire pressure or caster, and camber/toe only narrow the drawn contact
 * patch. That would violate issue #12 truthfulness ("a knob may only exist if turning it
 * moves the car") and issue #33's acceptance that unsupported/dormant parameters are not
 * editable. The definition (base) and the baseline setup are borrowed, never mutated; all
 * edits land in `working` and clamp to the registry bounds, so the model can only produce
 * values within the same ranges the Physics Lab enforces.
 */
#ifndef CIRCUIT_SETUP_EDITOR_H
#define CIRCUIT_SETUP_EDITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "physics/vehicle.h"

#define SETUP_EDITOR_MAX_ITEMS 24
#define SETUP_EDITOR_KEY_CHARS 48
#define SETUP_EDITOR_TEXT_CHARS 128

typedef struct {
    char key[SETUP_EDITOR_KEY_CHARS];          /* registry key or "drive.gear_count" */
    char unit[16];                             /* registry unit, "-" for unitless */
    char explanation[SETUP_EDITOR_TEXT_CHARS]; /* registry description */
    float min, max, step;
    bool isGearCount;
} SetupEditorItem;

typedef struct {
    VehicleSetup working;          /* edited copy; the base definition is never mutated */
    VehicleSetup baseline;         /* reset target */
    const VehicleDefinition *base; /* borrowed; validation context */
    SetupEditorItem items[SETUP_EDITOR_MAX_ITEMS];
    int itemCount;
} SetupEditor;

void setup_editor_init(SetupEditor *ed, const VehicleDefinition *def,
                       const VehicleSetup *baseline);
/* ±step clamped to the item's registry bounds; gear_count moves by ±1 within [1, MAX_GEARS].
 * Returns false on a bad index or direction == 0. */
bool setup_editor_adjust(SetupEditor *ed, int itemIndex, int direction);
void setup_editor_reset(SetupEditor *ed);          /* working = baseline */
bool setup_editor_is_valid(const SetupEditor *ed); /* vehicle_setup_is_valid(base, &working) */
uint32_t setup_editor_hash(const SetupEditor *ed); /* vehicle_setup_hash(&working) */
float setup_editor_value(const SetupEditor *ed, int itemIndex); /* current value for display */

#endif /* CIRCUIT_SETUP_EDITOR_H */

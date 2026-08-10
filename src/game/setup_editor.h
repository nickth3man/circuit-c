/*
 * setup_editor.h — headless setup-editing model (issue #33), shared by a menu screen and the
 * test suite. Raylib-free: the model is pure state + pure functions over VehicleSetup.
 *
 * WHAT IS EDITABLE. The editable items are exactly the registry entries that are both
 * setup-owned (DEV_OWNER_SETUP) and read by a dynamics path (DEV_CLASS_PHYSICS_INPUT):
 * drive.gear1..gear5, drive.reverse, drive.final, drive.diff_mode, drive.diff_bias_ratio,
 * drive.diff_preload, brake.bias_front — plus the typed int drive.gear_count, which lives
 * outside the float registry but is a genuine physics input.
 *
 * GEAR COUNT IS CAPPED AT THE EDITABLE RATIOS, NOT AT MAX_GEARS. The registry publishes five
 * forward ratios, so the count tops out at five. Letting it reach MAX_GEARS would let a player
 * commit to a sixth gear whose ratio is whatever the manifest happened to leave in the slot —
 * usually 0, which vehicle_setup_is_valid rejects — with no control in the menu to repair it,
 * i.e. a setup the editor can break and cannot fix. A setup authored with more gears than the
 * editor exposes keeps them and may be lowered; it simply cannot be raised further.
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
    /* Setup-owned physics-input registry keys the editor's field table does not bind, and so
     * did not turn into items. Always 0 in a consistent build; the `setup-editor` scenario
     * asserts it, which is what stops the registry and the mapping drifting apart. */
    int unboundKeyCount;
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
/* Persistence: deterministic text serialization of the working setup. One line per editable
 * item: "key=value" with %.9g for floats, integer for gear_count, sorted by the editor's
 * item order (which itself follows registry order + gear_count). Save writes the file;
 * load parses it, clamps to registry bounds, and leaves `working` untouched on failure.
 * Both are headless and use no raylib. */
bool setup_editor_save(const SetupEditor *ed, const char *path, char *error, size_t errorCap);
bool setup_editor_load(SetupEditor *ed, const char *path, char *error, size_t errorCap);
/* Gate before a session starts: every setup value must be inside vehicle bounds and the
 * derived spec must still validate; optionally also checks class eligibility when a class
 * is supplied. Returns false with a human reason when the setup cannot start. */
bool setup_editor_can_start(const SetupEditor *ed, char *reason, size_t reasonCap);
#endif /* CIRCUIT_SETUP_EDITOR_H */

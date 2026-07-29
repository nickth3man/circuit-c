/*
 * dev_presets.h — hardcoded driving-profile presets for the Physics Lab.
 *
 * Ten presets, each a named bundle of parameter overrides that produce a recognizable
 * driving feel: from a 760 kg kei car to a 900 hp 1988 Formula 1 turbo. Selected from
 * the lab dropdown; applied in place on the live VehicleSpec.
 *
 * Application is deterministic: dev_preset_apply() resets every parameter to its stock
 * default first, then applies the preset's overrides. So switching presets always
 * produces the same car, regardless of what was tuned before.
 *
 * Each preset is grounded in a research lane (real-world pro drift, JDM touge, sim
 * drift, arcade drift, grip racing, rally, and iconic car archetypes). See the comment
 * above each table in dev_presets.c for the source and the defining philosophy.
 *
 * Raylib-free: linked into the headless test executable as well as the game module.
 *
 * RELOAD SAFETY. The preset table is `static const` and lives in the game module's
 * rodata. Nothing here is ever stored in Game, so a module swap cannot leave a stale
 * pointer behind (see the invariant in hotreload.h).
 */
#ifndef DRIFTY_DEV_PRESETS_H
#define DRIFTY_DEV_PRESETS_H

#include "physics/vehicle.h"

typedef struct {
    const char *name;           /* "Touge Hero" */
    const char *description;    /* one-line feel, shown in the lab status line */
} DevPreset;

/* Stable table; index-safe. Index 0 is always "Stock Baseline" (no overrides). */
int              dev_preset_count(void);
const DevPreset *dev_preset_at(int index);

/* Reset spec to stock defaults, then apply every override in the preset.
 * Returns the number of overrides written. Calling with index 0 is equivalent to
 * dev_params_reset_all(). Most presets change geometry (cg_to_front, track_width,
 * half_width are requiresRestart), so the caller should hint "press R to reset". */
int dev_preset_apply(VehicleSpec *spec, int presetIndex);

#endif /* DRIFTY_DEV_PRESETS_H */

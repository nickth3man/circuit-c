/*
 * dev_params.h — the single tunable-parameter registry.
 *
 * ONE registry, five consumers. Every tunable is described exactly once, here, and that one
 * description generates:
 *
 *   - the Physics Lab controls (src/dev_lab.c),
 *   - tuning-profile save and load (dev_params_save / dev_params_load),
 *   - reset-to-default behaviour,
 *   - the telemetry metadata block written beside a run,
 *   - the documentation table in docs/PARAMETERS.md (drifty_tests --dump-params).
 *
 * That is the whole point: defaults, UI limits, saved profiles, and the specification cannot
 * drift apart because there is only one place to change.
 *
 * RELOAD SAFETY. The registry is a `static const` table of byte offsets into VehicleSpec, not
 * of pointers. Nothing here is ever stored in Game, so a module swap cannot leave a stale
 * pointer behind (see the invariant in hotreload.h).
 *
 * This translation unit is raylib-free — it is linked into the headless test executable as
 * well as the game module.
 */
#ifndef DRIFTY_DEV_PARAMS_H
#define DRIFTY_DEV_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "vehicle.h"

/* How prominently a parameter is offered. The Physics Lab shows the essential tier first and
 * hides the deeper ones behind a filter, so the panel stays legible now that the registry
 * describes the whole vehicle rather than only its handling coefficients. */
typedef enum {
    DEV_TIER_ESSENTIAL = 0,   /* the knobs a first-time tuner reaches for */
    DEV_TIER_ADVANCED  = 1,   /* hardware and geometry detail */
    DEV_TIER_EXPERT    = 2    /* numerical model internals and readouts */
} DevParamTier;

/* Description of one tunable float inside VehicleSpec. */
typedef struct {
    const char *name;           /* stable dotted key, e.g. "tire.lat_front.mu" */
    const char *group;          /* UI / documentation grouping */
    const char *unit;           /* SI unit, or "" for a dimensionless ratio */
    size_t      offset;         /* byte offset of the float inside VehicleSpec */
    float       defaultValue;   /* the config.h value; asserted against vehicle_spec_set_default */
    float       minimum;
    float       maximum;
    float       step;
    bool        requiresRestart; /* needs game_reset_sim() to take full effect (wheel layout) */
    /* Computed by vehicle_spec_refresh_derived() from other parameters. A derived entry is a
     * read-only readout: dev_param_set refuses it, the Lab draws a value instead of a slider,
     * and a profile or preset that names one is migrated onto the primaries that produce it
     * (see the alias table in dev_params.c). */
    bool        derived;
    int         tier;           /* DevParamTier */
    const char *description;
} DevParameter;

/* Registry access. The table is immutable and its order is stable. */
int                 dev_params_count(void);
const DevParameter *dev_params_all(void);
const DevParameter *dev_param_at(int index);
const DevParameter *dev_param_find(const char *name);

/* Distinct group names, in first-appearance order. */
int         dev_params_group_count(void);
const char *dev_params_group_name(int groupIndex);

/* Value access. dev_param_set clamps to [minimum, maximum], rejects non-finite input, and
 * refreshes every derived spec field afterwards. It refuses a derived parameter outright:
 * writing a readout would be undone by the next refresh, so a silent no-op would be worse
 * than a refusal. Use dev_params_apply_assignments() to write one by migration. */
float dev_param_get(const VehicleSpec *spec, const DevParameter *param);
bool  dev_param_set(VehicleSpec *spec, const DevParameter *param, float value);
bool  dev_param_is_default(const VehicleSpec *spec, const DevParameter *param);

void dev_param_reset(VehicleSpec *spec, const DevParameter *param);
void dev_params_reset_all(VehicleSpec *spec);

/* Number of parameters differing from their default value. Zero means "stock car". */
int dev_params_modified_count(const VehicleSpec *spec);

/* Recompute the spec fields that are derived from other parameters. Called for you by
 * dev_param_set and dev_params_load; exposed for code that writes spec fields directly. */
void dev_params_refresh_derived(VehicleSpec *spec);

/* ------------------------------------------------------------------------------ profiles --
 *
 * Tuning profiles are plain text: a header comment, then `name = value` lines. Unknown keys
 * are counted and skipped rather than being an error, so a profile written by a newer build
 * still loads. Parsing is deliberately defensive: it is a fuzz target (fuzz/fuzz_profile.c).
 */

#define DEV_PROFILE_MAGIC "# drifty tuning profile v1"

/* Write every parameter (name, value, unit, default) to path. Returns false on any I/O
 * failure. Creates nothing but the file itself. */
bool dev_params_save(const VehicleSpec *spec, const char *path);

/* Apply a profile on top of spec. *appliedOut / *unknownOut / *rejectedOut receive counts
 * when non-NULL. Returns false only when the file could not be read or the resulting spec
 * fails vehicle_spec_is_valid() — in which case spec is left untouched. */
bool dev_params_load(VehicleSpec *spec, const char *path,
                     int *appliedOut, int *unknownOut, int *rejectedOut);

/* Same parser over a memory buffer; used by the file loader and by the fuzz harness. */
bool dev_params_apply_text(VehicleSpec *spec, const char *text, size_t length,
                           int *appliedOut, int *unknownOut, int *rejectedOut);

/* One key/value pair, the in-memory form of a profile line. */
typedef struct {
    const char *key;
    float       value;
} DevParamAssignment;

/* Apply a batch of assignments with derived-key migration, then refresh once. This is the
 * shared path behind profile loading and dev_preset_apply, which is why a preset written
 * against the pre-Phase-2 keys (body.mass, body.cg_to_front, wheel.radius, ...) still
 * produces the car it names: each of those keys is translated into the primaries it used
 * to stand for. Returns the number applied. Unlike dev_params_apply_text this writes spec
 * in place and does not validate, because its callers start from a known-good spec. */
int dev_params_apply_assignments(VehicleSpec *spec,
                                 const DevParamAssignment *items, int count,
                                 int *unknownOut, int *rejectedOut);

/* ------------------------------------------------------------------------------- reports -- */

/* Markdown table of every parameter — the source of docs/PARAMETERS.md. */
void dev_params_write_markdown(FILE *out);

/* `# name,value,default,unit` comment lines for a telemetry sidecar file. */
void dev_params_write_metadata(FILE *out, const VehicleSpec *spec);

/* Only the parameters that differ from default, as `name=value` pairs on one line.
 * Returns the number written. Used in failure bundles and run summaries. */
int dev_params_write_overrides(FILE *out, const VehicleSpec *spec);

#endif /* DRIFTY_DEV_PARAMS_H */

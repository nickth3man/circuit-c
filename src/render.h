/*
 * render.h — Phase 1 interpolation and simple vehicle/debug presentation.
 */
#ifndef DRIFTY_RENDER_H
#define DRIFTY_RENDER_H

#include "vehicle.h"

typedef struct {
    Vector2 positionM;
    float headingRad;
    float wheelAngleRad[WHEEL_COUNT];
} VehicleDrawState;

struct Game;

VehicleDrawState render_interpolate_vehicle(const VehicleRenderState *state, float alpha);
void render_draw_game(struct Game *game, float interpolationAlpha);

/* ------------------------------------------------------------- GPU resource lifetime ----
 *
 * The baked vehicle sprites are raylib-tracked GPU textures held in this module's statics,
 * so they must be released before the module is swapped and re-acquired afterwards — the
 * same contract audio.c has for its sounds, and the reason AGENTS.md lists textures among
 * the things a reload cannot carry across. A handle from the old module is a dangling GPU
 * name in the new one.
 *
 * No Game layout changes: the cache is module-static, not persistent state. Losing it across
 * a reload costs one rebake on the next frame and nothing else.
 *
 * All three are no-ops under DRIFTY_HEADLESS. */
void render_pre_reload(void);
void render_post_reload(void);
void render_shutdown(void);

#endif /* DRIFTY_RENDER_H */

/*
 * dev_lab.h — the in-game Physics Lab.
 *
 * A development-only raygui interface over the parameter registry, the scenario table, the
 * scope history, and the replay timeline. It is the fast subjective loop: change a value,
 * watch the car, compare against a baseline, save the profile — without rebuilding.
 *
 * BUILD CONFIGURATION. The lab exists only when DRIFTY_DEV_TOOLS is defined and the build is
 * not headless. Everywhere else the entry points below are no-op inline stubs, so callers
 * never need an #ifdef. The *state* the lab edits (DevState) is always present — see the
 * layout note in dev_state.h.
 *
 * Keyboard, all function keys so nothing collides with driving:
 *
 *   F2  show/hide the lab            F6  single step (hold Shift for ten)
 *   F3  show/hide the replay inspector   F7  cycle time scale
 *   F5  pause / resume               F8  capture the current run as the baseline ghost
 */
#ifndef DRIFTY_DEV_LAB_H
#define DRIFTY_DEV_LAB_H

#include "render.h"

struct Game;

#if defined(DRIFTY_DEV_TOOLS) && !defined(DRIFTY_HEADLESS)

/* Keyboard shortcuts and time control. Call once per render frame, before drawing. */
void dev_lab_update(struct Game *game);

/* World-space overlays: trajectory, ghost, contact points, per-wheel forces, slip and load
 * indicators. Call inside BeginMode2D(). */
void dev_lab_draw_world(const struct Game *game, const VehicleDrawState *draw);

/* Screen-space interface: parameter panel, scenario and time controls, scope, inspector,
 * invariant warning. Call after EndMode2D() and before EndDrawing(). */
void dev_lab_draw_ui(struct Game *game);

/* raygui keeps its style in module-static memory, which a hot reload discards. Called from
 * game_post_reload() so the lab looks identical either side of a swap. */
void dev_lab_reload_style(void);

#else

static inline void dev_lab_update(struct Game *game) { (void)game; }
static inline void dev_lab_draw_world(const struct Game *game, const VehicleDrawState *draw)
{
    (void)game; (void)draw;
}
static inline void dev_lab_draw_ui(struct Game *game) { (void)game; }
static inline void dev_lab_reload_style(void) { }

#endif

#endif /* DRIFTY_DEV_LAB_H */

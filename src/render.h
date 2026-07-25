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

#endif /* DRIFTY_RENDER_H */

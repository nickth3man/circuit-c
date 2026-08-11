/*
 * ai_driver_internal.h — internals shared by the two AI driver architectures.
 *
 * ai_driver.c (the baseline driver, AI_DRIVER_ARCH_LEGACY) and ai_driver_v2.c (the #79
 * limit-aware clean-slate driver, AI_DRIVER_ARCH_LIMIT) share one planner and one set of
 * geometry helpers, so the clean-slate controller drives the SAME minimum-curvature planned
 * line the baseline searches, and the tests that inspect the plan through
 * ai_driver_plan_point() keep working whichever architecture is active.
 *
 * NOT public API: only the two driver translation units include this header. The names are
 * deliberately short because they are internal to this pair.
 */
#ifndef CIRCUIT_AI_DRIVER_INTERNAL_H
#define CIRCUIT_AI_DRIVER_INTERNAL_H

#include "game/ai_driver.h"

/* Vector helpers. */
Vector2 vec_sub(Vector2 a, Vector2 b);
float vec_len(Vector2 v);

/* Closest point on segment [a,b] to p; writes the clamped parameter and point. */
float closest_on_segment(Vector2 a, Vector2 b, Vector2 p, float *tOut, Vector2 *pointOut);

/* Menger curvature of the triangle abc, 1/m. */
float menger_curvature(Vector2 a, Vector2 b, Vector2 c);

/* Unit normal to the centreline at node i, left of travel. */
Vector2 node_normal(const TrackDefinition *track, int i);

/* Half-width usable for planning at node i after the edge margin. */
float usable_half_width(const TrackDefinition *track, int i, float marginM);

/* The planned lateral offset at a centreline node, or 0 outside the window. */
float plan_offset_at(const AiDriverState *state, const TrackDefinition *track, int nodeIndex);

/* Walk `distanceM` forward along the planned line from (segment, t). */
Vector2 plan_point_ahead(const AiDriverState *state, const TrackDefinition *track, int segment,
                         float t, float distanceM);

/* Minimum-curvature lattice search; stores the plan in state. */
void plan_path(const AiDriverConfig *cfg, AiDriverState *state, const TrackDefinition *track,
               int baseSegment, float carOffsetM);

/* Move an axis toward its demand no faster than the control can physically travel. */
float slew_axis(float current, float target, float pressRatePerS, float releaseRatePerS,
                float deadband, float dt);

/* Lateral/longitudinal mu a driver could expect at a position, composed like physics.c. */
void available_grip(const VehicleSpec *spec, const TrackDefinition *track,
                    const TrackRuntime *runtime, Vector2 positionM, float *muLatOut,
                    float *muLongOut);

#endif /* CIRCUIT_AI_DRIVER_INTERNAL_H */

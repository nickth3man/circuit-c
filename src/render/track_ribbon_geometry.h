/*
 * track_ribbon_geometry.h — pure geometry shared by the GPU track pass and headless tests.
 *
 * Raylib draws each thick line segment with a flat end. At a centreline node where direction
 * changes, the incoming and outgoing end sections therefore differ. These four edge points
 * describe the two bevel triangles that close the exposed wedges between those sections.
 */
#ifndef CIRCUIT_TRACK_RIBBON_GEOMETRY_H
#define CIRCUIT_TRACK_RIBBON_GEOMETRY_H

#include <stdbool.h>

#include "raylib.h" /* Vector2 */

#include "world/track.h"

typedef struct {
    Vector2 centerM;
    Vector2 previousLeftM;
    Vector2 previousRightM;
    Vector2 nextLeftM;
    Vector2 nextRightM;
} TrackRibbonJoin;

/* Build the bevel join at one node of a closed track ribbon. */
bool track_ribbon_join_build(const TrackDefinition *track, int nodeIndex, TrackRibbonJoin *out);

#endif /* CIRCUIT_TRACK_RIBBON_GEOMETRY_H */

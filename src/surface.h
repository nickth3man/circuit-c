/*
 * surface.h — per-surface friction and resistance parameters.
 *
 * Surfaces are static module data, referenced by id (SurfaceId), and resolved fresh at
 * every use through Surface_Get(). Never cache the returned pointer across hot reloads:
 * the table lives in the game module's static data and its address changes.
 *
 * This translation unit must not call any raylib function.
 */
#ifndef DRIFTY_SURFACE_H
#define DRIFTY_SURFACE_H

#include "vehicle.h"   /* SurfaceId */

typedef struct {
    const char *name;
    float muLongitudinal;                /* dimensionless peak longitudinal friction coefficient */
    float muLateral;                     /* dimensionless peak lateral friction coefficient */
    float tireBScale;                    /* dimensionless; scales tire stiffness B (soft buildup < 1) */
    float rollingResistanceCoefficient;  /* dimensionless */
    float looseSurfaceDragN;             /* extra drag force, newtons, opposing contact velocity */
    float sinkFactor;                    /* dimensionless 0..1, reserved for terrain resistance */
} SurfaceSpec;

/* Never returns NULL; out-of-range id is clamped to SURFACE_ASPHALT. */
const SurfaceSpec *Surface_Get(SurfaceId id);

#endif /* DRIFTY_SURFACE_H */

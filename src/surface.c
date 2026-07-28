/*
 * surface.c — per-surface friction and resistance parameters.
 *
 * The static kSurfaces table holds the baseline surfaces defined in docs/SPEC.md.
 * Surface_Get() resolves an id into a const pointer; callers resolve fresh each use
 * and never cache the pointer across hot reloads.
 *
 * No raylib calls. No allocations. No global mutable state.
 */
#include "surface.h"

static const SurfaceSpec kSurfaces[SURFACE_COUNT] = {
    [SURFACE_ASPHALT] = {
        .name                        = "Asphalt",
        .muLongitudinal              = 1.35f,
        .muLateral                   = 1.30f,
        .tireBScale                  = 1.00f,
        .rollingResistanceCoefficient = 0.015f,
        .looseSurfaceDragN           = 0.0f,
        .sinkFactor                  = 0.0f,
    },
    [SURFACE_GRAVEL] = {
        .name                        = "Gravel",
        .muLongitudinal              = 0.85f,
        .muLateral                   = 0.80f,
        .tireBScale                  = 0.65f,
        .rollingResistanceCoefficient = 0.045f,
        .looseSurfaceDragN           = 250.0f,
        .sinkFactor                  = 0.0f,
    },
    [SURFACE_GRASS] = {
        .name                        = "Grass",
        .muLongitudinal              = 0.65f,
        .muLateral                   = 0.60f,
        .tireBScale                  = 0.70f,
        .rollingResistanceCoefficient = 0.080f,
        .looseSurfaceDragN           = 600.0f,
        .sinkFactor                  = 0.0f,
    },
    [SURFACE_SNOW] = {
        .name                        = "Snow",
        .muLongitudinal              = 0.40f,
        .muLateral                   = 0.38f,
        .tireBScale                  = 0.50f,
        .rollingResistanceCoefficient = 0.050f,
        .looseSurfaceDragN           = 200.0f,
        .sinkFactor                  = 0.0f,
    },
};

const SurfaceSpec *Surface_Get(SurfaceId id)
{
    if (id >= SURFACE_COUNT) {
        id = SURFACE_ASPHALT;
    }
    return &kSurfaces[id];
}

/*
 * particle.h — smoke-particle pool and spawn logic.
 *
 * Presentation-layer data (not physics). Uses raylib's Vector2 and Color types from the
 * header, but no raylib function calls. The pool update is pure math and works in headless
 * builds.
 *
 * Phase 6 chunk [6c-1]: particle pool + spawn logic for the presentation backbone.
 */
#ifndef DRIFTY_PARTICLE_H
#define DRIFTY_PARTICLE_H

#include <stdbool.h>

#include "raylib.h"    /* Vector2, Color */
#include "config.h"    /* MAX_PARTICLES, PARTICLE_LIFE_S */

typedef struct {
    Vector2 positionM;
    Vector2 velocityMps;
    float   lifeS;
    float   maxLifeS;
    float   sizeM;
    Color   color;
    bool    active;
} Particle;

typedef struct {
    Particle particles[MAX_PARTICLES];
    int      cursor;           /* round-robin spawn index */
} ParticlePool;

/* Zero the pool: all particles inactive, cursor at 0. */
void particle_pool_init(ParticlePool *pool);

/* Integrate velocity, decay life, deactivate particles whose life reaches zero.
 * dt must be positive; zero or negative is a no-op. */
void particle_pool_update(ParticlePool *pool, float dt);

/* Spawn one particle at worldPosM, overwriting the oldest entry (round-robin cursor).
 * Advances the cursor; wraps at MAX_PARTICLES. */
void particle_spawn(ParticlePool *pool, Vector2 worldPosM, Vector2 worldVelMps,
                    float sizeM, Color color);

#endif /* DRIFTY_PARTICLE_H */

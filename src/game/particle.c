/*
 * particle.c — smoke-particle pool implementation.
 *
 * Pure data operations: no raylib function calls, no allocation. Uses raylib header types
 * (Vector2, Color) only.
 */
#include "game/particle.h"

#include <string.h>

void particle_pool_init(ParticlePool *pool)
{
    if (pool == NULL) return;
    memset(pool, 0, sizeof(*pool));
}

void particle_pool_update(ParticlePool *pool, float dt)
{
    if (pool == NULL || dt <= 0.0f) return;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle *p = &pool->particles[i];
        if (!p->active) continue;

        p->lifeS -= dt;
        if (p->lifeS <= 0.0f) {
            p->active = false;
            continue;
        }

        p->positionM.x += p->velocityMps.x * dt;
        p->positionM.y += p->velocityMps.y * dt;
    }
}

void particle_spawn(ParticlePool *pool, Vector2 worldPosM, Vector2 worldVelMps, float sizeM,
                    Color color)
{
    if (pool == NULL) return;

    const int idx = pool->cursor;
    Particle *p = &pool->particles[idx];

    p->positionM = worldPosM;
    p->velocityMps = worldVelMps;
    p->lifeS = PARTICLE_LIFE_S;
    p->maxLifeS = PARTICLE_LIFE_S;
    p->sizeM = sizeM;
    p->color = color;
    p->active = true;

    pool->cursor = (idx + 1) % MAX_PARTICLES;
}

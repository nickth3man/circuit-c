/*
 * collision.h — vehicle-track collision narrowphase and impulse response, driven by the
 * deterministic CollisionWorld.
 *
 * Baseline capsule model: two circles (front + rear axle) with the connecting body
 * implicitly covered by the swept paths. Swept substeps between prev and curr transform
 * prevent tunneling. The world (collision_world.h) owns the static barrier shapes and the
 * dynamic body proxies; the narrowphase here consumes the world's candidate feed, resolves
 * penetration (reflects normal velocity with restitution, applies Coulomb friction at the
 * contact point, applies the resulting torque so a glancing hit spins the car), and records
 * each physical contact into the world's per-tick event feed for presentation to consume.
 *
 * CANDIDATE ORDER IS THE CONTRACT. Candidates arrive ascending by stable shape id, and the
 * swept pass walks them in that order, re-querying strictly ahead of its cursor after a
 * contact pushes the body — so the resolution sequence is bit-identical to a single
 * brute-force scan over every barrier, independent of pointer values and of whether the
 * world answered from its uniform grid or its brute scan.
 *
 * This translation unit must not call any raylib function. It uses <math.h> and the
 * Vector2 type from raylib.h.
 */
#ifndef CIRCUIT_COLLISION_H
#define CIRCUIT_COLLISION_H

#include "world/collision_world.h"
#include "world/track.h"
#include "physics/vehicle.h"

/* Number of swept substeps between the previous and current vehicle transform.
 * At 120 Hz and ~120 m/s top speed, one tick covers ~1 m.  6 substeps give ~0.17 m each,
 * smaller than the default circle radius (0.85 m), so no tunneling. */
#define COLLISION_SUBSTEPS 6

/* Normal impact speed threshold above which the crash lockout timer is set and a contact is
 * "significant" for presentation. Trivial barrier kisses (< 2 m/s) do not trigger either. */
#define COLLISION_LOCKOUT_THRESHOLD_MPS 2.0f

/*
 * Per-entrant response context for one registered body. `crashLockoutTimerS` is entrant
 * state: set to CRASH_LOCKOUT_S on a significant impact, and gated by later contacts, so it
 * must be the same pointer the entrant's collision stage owns.
 */
typedef struct {
    CollisionBodyId id;
    const VehicleSpec *spec;
    VehicleState *state;
    VehicleRenderState *renderState;
    float *crashLockoutTimerS;
} CollisionBodyContext;

/*
 * Resolve every registered body against the world's static shapes, in ascending body id
 * order, using the matching context. Returns the total contacts resolved, or -1 when a
 * registered body has no matching context (a caller contract violation that must not be
 * silently skipped).
 */
int collision_world_resolve_bodies(CollisionWorld *world, CollisionBodyContext *contexts,
                                   int contextCount);

/*
 * Resolve vehicle-vs-vehicle contacts between registered bodies (issue #27). Pairs are
 * processed in ascending (idA, idB) order — independent of registration order — using the
 * swept capsule poses; the first penetrating substep yields one two-body impulse contact per
 * pair. Requires a CollisionBodyContext per registered body (same contract as
 * collision_world_resolve_bodies). Returns the total contact events recorded, or -1 on a
 * contract violation.
 */
int collision_resolve_body_pairs(CollisionWorld *world, CollisionBodyContext *contexts,
                                 int contextCount);

/*
 * The one-entrant path: begin a tick, register one vehicle body from its spec and render
 * pose, and resolve it. Returns the contacts resolved (0 if none). This is what the single-
 * car game path and the direct collision tests use; a multi-entrant caller drives the
 * world API directly so all bodies resolve in one ordered pass.
 */
int collision_resolve_track(CollisionWorld *world, CollisionBodyId bodyId,
                            const VehicleSpec *spec, VehicleState *state,
                            VehicleRenderState *renderState, float *crashLockoutTimerS);

#endif /* CIRCUIT_COLLISION_H */

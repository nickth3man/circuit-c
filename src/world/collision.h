/*
 * collision.h — vehicle-track collision detection and impulse response.
 *
 * Baseline capsule model: two circles (front + rear axle) with the connecting body
 * implicitly covered by the swept paths. Swept substeps between prev and curr transform
 * prevent tunneling. Impulse response resolves penetration, reflects normal velocity with
 * restitution, applies Coulomb friction at the contact point, and applies the resulting
 * torque so a glancing hit spins the car.
 *
 * This translation unit must not call any raylib function. It uses <math.h> and the
 * Vector2 type from raylib.h.
 */
#ifndef DRIFTY_COLLISION_H
#define DRIFTY_COLLISION_H

#include "world/track.h"
#include "physics/vehicle.h"

/* Number of swept substeps between the previous and current vehicle transform.
 * At 120 Hz and ~120 m/s top speed, one tick covers ~1 m.  6 substeps give ~0.17 m each,
 * smaller than the default circle radius (0.85 m), so no tunneling. */
#define COLLISION_SUBSTEPS 6

/* Normal impact speed threshold above which the crash lockout timer is set.
 * Trivial barrier kisses (< 2 m/s) do not trigger the scoring suspension. */
#define COLLISION_LOCKOUT_THRESHOLD_MPS 2.0f

/*
 * Resolve collisions between the vehicle capsule and the track boundary.
 *
 * Mutates state (position, velocity, yaw) and renderState->curr*. Sets *crashLockoutTimerS
 * to CRASH_LOCKOUT_S on significant impacts.
 *
 * Returns the number of contacts resolved (0 if none).
 */
int collision_resolve_track(const VehicleSpec *spec, VehicleState *state,
                            VehicleRenderState *renderState, const Track *track,
                            float *crashLockoutTimerS);

#endif /* DRIFTY_COLLISION_H */

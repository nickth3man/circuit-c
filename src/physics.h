/*
 * physics.h — pure vehicle calculations and fixed-update ownership.
 *
 * Phase 3 adds the load-transfer stage (a first-order filter on the previous step's solved
 * body-longitudinal acceleration, then static-plus-dynamic axle loads) and the two separated
 * resistance forces. The individual stages are exposed so the headless suite can assert each
 * equation on its own rather than only through a whole-vehicle run.
 */
#ifndef DRIFTY_PHYSICS_H
#define DRIFTY_PHYSICS_H

#include <stdbool.h>

#include "input.h"
#include "vehicle.h"

typedef struct {
    float velocityLongitudinalDerivativeMps2;
    float velocityLateralDerivativeMps2;
    float yawRateDerivativeRadS2;
} VehicleDerivatives;

/* One axle-load solution: the static split, the transfer it was shifted by, the unclamped
 * result (which always sums to mass * g), and the clamped loads the tires actually see. */
typedef struct {
    float staticFrontN;
    float staticRearN;
    float transferN;         /* positive moves load rearward */
    float unclampedFrontN;
    float unclampedRearN;
    float frontN;            /* floored at MIN_NORMAL_LOAD_N */
    float rearN;
} AxleLoads;

Vector2 physics_contact_point_velocity_body(const VehicleState *state, Vector2 pointM);
void physics_static_axle_loads(const VehicleSpec *spec, float *frontLoadN, float *rearLoadN);

/* One step of the first-order load-transfer filter, in isolation.
 *   filtered += (previous - filtered) * (1 - exp(-rateHz * dt))
 * Returns `filtered` unchanged for a non-positive dt or a non-finite input. */
float physics_filter_long_accel(float filteredMps2, float previousMps2,
                                float rateHz, float dt);

/* Static-plus-dynamic axle loads for a given filtered longitudinal acceleration. */
AxleLoads physics_axle_loads(const VehicleSpec *spec, float filteredLongAccelMps2);

/* Aerodynamic drag in the body frame, opposing the full velocity vector. *magnitudeN
 * receives the scalar 0.5*rho*Cd*A*v^2 when non-NULL. */
Vector2 physics_aero_drag_body_n(const VehicleSpec *spec,
                                 float velocityLongitudinalMps, float velocityLateralMps,
                                 float *magnitudeN);

/* Rolling resistance for one wheel, opposing its contact-point velocity. Returns the body-
 * frame force; *magnitudeN receives its length. Zero at rest, with no direction invented. */
Vector2 physics_rolling_resistance_body_n(float rollingResistanceCoefficient,
                                          float normalLoadN, Vector2 contactVelocityMps,
                                          float *magnitudeN);
void physics_update_steering(const VehicleSpec *spec, VehicleState *state,
                             float steerInput, float dt);
void physics_axle_slip_angles(const VehicleSpec *spec, const VehicleState *state,
                              float *frontSlipAngleRad, float *rearSlipAngleRad);
Vector2 physics_rotate_wheel_force_to_body(Vector2 wheelForceN, float steerAngleRad);
float physics_low_speed_blend(float velocityLongitudinalMps);
bool physics_state_is_valid(const VehicleSpec *spec, const VehicleState *state,
                            const VehicleDerived *derived);
void physics_fixed_update(const VehicleSpec *spec,
                          VehicleState *state,
                          VehicleDerived *derived,
                          VehicleRenderState *renderState,
                          const Input *input,
                          float dt);

#endif /* DRIFTY_PHYSICS_H */

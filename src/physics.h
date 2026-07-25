/*
 * physics.h — pure Phase 1 rigid-body vehicle calculations and fixed update ownership.
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

Vector2 physics_contact_point_velocity_body(const VehicleState *state, Vector2 pointM);
void physics_static_axle_loads(const VehicleSpec *spec, float *frontLoadN, float *rearLoadN);
void physics_update_steering(const VehicleSpec *spec, VehicleState *state,
                             float steerInput, float dt);
void physics_axle_slip_angles(const VehicleSpec *spec, const VehicleState *state,
                              float *frontSlipAngleRad, float *rearSlipAngleRad);
float physics_linear_lateral_force(float slipAngleRad, float corneringStiffnessNPerRad,
                                   float frictionCoefficient, float normalLoadN);
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

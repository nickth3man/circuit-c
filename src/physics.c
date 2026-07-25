#include "physics.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

#include "math_utils.h"

Vector2 physics_contact_point_velocity_body(const VehicleState *state, Vector2 pointM)
{
    if (state == NULL) return (Vector2){ 0.0f, 0.0f };
    return (Vector2){
        state->velocityLongitudinalMps - state->yawRateRadS * pointM.y,
        state->velocityLateralMps + state->yawRateRadS * pointM.x
    };
}

void physics_static_axle_loads(const VehicleSpec *spec, float *frontLoadN, float *rearLoadN)
{
    if (frontLoadN != NULL) *frontLoadN = 0.0f;
    if (rearLoadN != NULL) *rearLoadN = 0.0f;
    if (!vehicle_spec_is_valid(spec)) return;
    if (frontLoadN != NULL) {
        *frontLoadN = spec->massKg * GRAVITY_MPS2 * spec->cgToRearM / spec->wheelbaseM;
    }
    if (rearLoadN != NULL) {
        *rearLoadN = spec->massKg * GRAVITY_MPS2 * spec->cgToFrontM / spec->wheelbaseM;
    }
}

void physics_update_steering(const VehicleSpec *spec, VehicleState *state,
                             float steerInput, float dt)
{
    if (spec == NULL || state == NULL || !(dt > 0.0f)) return;
    const float target = clampf(steerInput, -1.0f, 1.0f) * spec->maxRoadWheelAngleRad;
    const float error = target - state->frontRoadWheelAngleRad;
    const float rate = (fabsf(target) < fabsf(state->frontRoadWheelAngleRad))
        ? spec->steerReturnRateRadS : spec->maxSteerRateRadS;
    const float maxChange = rate * dt;
    state->frontRoadWheelAngleRad = clampf(
        state->frontRoadWheelAngleRad + clampf(error, -maxChange, maxChange),
        -spec->maxRoadWheelAngleRad, spec->maxRoadWheelAngleRad);
    state->wheels[WHEEL_FRONT_LEFT].steerAngleRad = state->frontRoadWheelAngleRad;
    state->wheels[WHEEL_FRONT_RIGHT].steerAngleRad = state->frontRoadWheelAngleRad;
    state->wheels[WHEEL_REAR_LEFT].steerAngleRad = 0.0f;
    state->wheels[WHEEL_REAR_RIGHT].steerAngleRad = 0.0f;
}

void physics_axle_slip_angles(const VehicleSpec *spec, const VehicleState *state,
                              float *frontSlipAngleRad, float *rearSlipAngleRad)
{
    if (frontSlipAngleRad != NULL) *frontSlipAngleRad = 0.0f;
    if (rearSlipAngleRad != NULL) *rearSlipAngleRad = 0.0f;
    if (spec == NULL || state == NULL) return;
    const float vxSafe = fmaxf(fabsf(state->velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS);
    const float frontVy = state->velocityLateralMps + spec->cgToFrontM * state->yawRateRadS;
    const float rearVy = state->velocityLateralMps - spec->cgToRearM * state->yawRateRadS;
    if (frontSlipAngleRad != NULL) {
        *frontSlipAngleRad = atan2f(frontVy, vxSafe) - state->frontRoadWheelAngleRad;
    }
    if (rearSlipAngleRad != NULL) *rearSlipAngleRad = atan2f(rearVy, vxSafe);
}

float physics_linear_lateral_force(float slipAngleRad, float corneringStiffnessNPerRad,
                                   float frictionCoefficient, float normalLoadN)
{
    if (!(isfinite(slipAngleRad) && corneringStiffnessNPerRad > 0.0f &&
          frictionCoefficient > 0.0f && normalLoadN > 0.0f)) return 0.0f;
    const float limitN = frictionCoefficient * normalLoadN;
    return clampf(-corneringStiffnessNPerRad * slipAngleRad, -limitN, limitN);
}

Vector2 physics_rotate_wheel_force_to_body(Vector2 wheelForceN, float steerAngleRad)
{
    const float c = cosf(steerAngleRad);
    const float s = sinf(steerAngleRad);
    return (Vector2){
        wheelForceN.x * c - wheelForceN.y * s,
        wheelForceN.x * s + wheelForceN.y * c
    };
}

float physics_low_speed_blend(float velocityLongitudinalMps)
{
    return smoothstep(LOW_SPEED_BEGIN_MPS, LOW_SPEED_END_MPS,
                      fabsf(velocityLongitudinalMps));
}

static float phase1_longitudinal_force(const VehicleSpec *spec, const VehicleState *state,
                                       const Input *input, float dt)
{
    const float throttle = clampf(input->throttle, 0.0f, 1.0f);
    const float brake = clampf(input->brake, 0.0f, 1.0f);
    const float driveDirection = (state->selectedGear < 0) ? -1.0f : 1.0f;
    const float maxDriveN = (driveDirection < 0.0f)
        ? PHASE1_MAX_REVERSE_FORCE_N : PHASE1_MAX_DRIVE_FORCE_N;
    const float driveN = driveDirection * throttle * maxDriveN;

    float resistanceN = 0.0f;
    if (fabsf(state->velocityLongitudinalMps) > 0.0f) {
        const float direction = (state->velocityLongitudinalMps > 0.0f) ? 1.0f : -1.0f;
        float requestedN = brake * PHASE1_MAX_BRAKE_FORCE_N +
                           PHASE1_LINEAR_DRAG_N_PER_MPS *
                           fabsf(state->velocityLongitudinalMps);
        /* Braking and linear drag may stop the vehicle, but cannot push it through zero. */
        const float stopForceN = spec->massKg * fabsf(state->velocityLongitudinalMps) / dt;
        requestedN = fminf(requestedN, stopForceN);
        resistanceN = -direction * requestedN;
    }
    return driveN + resistanceN;
}

static VehicleDerivatives dynamic_derivatives(const VehicleSpec *spec,
                                               const VehicleState *state,
                                               Vector2 totalBodyForceN,
                                               float yawTorqueNm)
{
    VehicleDerivatives out;
    const float axBody = totalBodyForceN.x / spec->massKg;
    const float ayBody = totalBodyForceN.y / spec->massKg;
    out.velocityLongitudinalDerivativeMps2 =
        axBody + state->yawRateRadS * state->velocityLateralMps;
    out.velocityLateralDerivativeMps2 =
        ayBody - state->yawRateRadS * state->velocityLongitudinalMps;
    out.yawRateDerivativeRadS2 = yawTorqueNm / spec->yawInertiaKgM2;
    return out;
}

static VehicleDerivatives kinematic_derivatives(const VehicleSpec *spec,
                                                 const VehicleState *state,
                                                 float longitudinalCommandN)
{
    const float tanDelta = tanf(state->frontRoadWheelAngleRad);
    const float yawTargetRadS =
        state->velocityLongitudinalMps * tanDelta / spec->wheelbaseM;
    const float betaRad = atan2f(spec->cgToRearM * tanDelta, spec->wheelbaseM);
    const float lateralTargetMps =
        state->velocityLongitudinalMps * tanf(betaRad);
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        longitudinalCommandN / spec->massKg +
        state->yawRateRadS * state->velocityLateralMps;
    out.velocityLateralDerivativeMps2 =
        (lateralTargetMps - state->velocityLateralMps) *
        PHASE1_KINEMATIC_RESPONSE_RATE_HZ;
    out.yawRateDerivativeRadS2 =
        (yawTargetRadS - state->yawRateRadS) * PHASE1_KINEMATIC_RESPONSE_RATE_HZ;
    return out;
}

static VehicleDerivatives derivatives_lerp(VehicleDerivatives a, VehicleDerivatives b, float t)
{
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        lerpf(a.velocityLongitudinalDerivativeMps2,
              b.velocityLongitudinalDerivativeMps2, t);
    out.velocityLateralDerivativeMps2 =
        lerpf(a.velocityLateralDerivativeMps2,
              b.velocityLateralDerivativeMps2, t);
    out.yawRateDerivativeRadS2 =
        lerpf(a.yawRateDerivativeRadS2, b.yawRateDerivativeRadS2, t);
    return out;
}

bool physics_state_is_valid(const VehicleSpec *spec, const VehicleState *state,
                            const VehicleDerived *derived)
{
    if (!vehicle_spec_is_valid(spec) || state == NULL || derived == NULL) return false;
#define FINITE_VALUE(v) if (!isfinite(v)) return false
    FINITE_VALUE(state->positionM.x);
    FINITE_VALUE(state->positionM.y);
    FINITE_VALUE(state->headingRad);
    FINITE_VALUE(state->velocityLongitudinalMps);
    FINITE_VALUE(state->velocityLateralMps);
    FINITE_VALUE(state->yawRateRadS);
    FINITE_VALUE(state->frontRoadWheelAngleRad);
    FINITE_VALUE(state->engineRpm);
    FINITE_VALUE(state->filteredLongAccelMps2);
    FINITE_VALUE(state->prevLongAccelMps2);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &state->wheels[i];
        FINITE_VALUE(wheel->localPositionM.x);
        FINITE_VALUE(wheel->localPositionM.y);
        FINITE_VALUE(wheel->steerAngleRad);
        FINITE_VALUE(wheel->angularVelocityRadS);
        FINITE_VALUE(wheel->normalLoadN);
        FINITE_VALUE(wheel->slipAngleRad);
        FINITE_VALUE(wheel->slipRatio);
        FINITE_VALUE(wheel->forceLongitudinalN);
        FINITE_VALUE(wheel->forceLateralN);
        FINITE_VALUE(wheel->frictionUsage);
        if (wheel->normalLoadN <= 0.0f || wheel->frictionUsage < 0.0f ||
            wheel->frictionUsage > 1.001f) return false;
    }
    FINITE_VALUE(derived->bodySideslipRad);
    FINITE_VALUE(derived->longitudinalAccelerationMps2);
    FINITE_VALUE(derived->lateralAccelerationMps2);
    FINITE_VALUE(derived->speedMps);
    FINITE_VALUE(derived->normalLoadFrontN);
    FINITE_VALUE(derived->normalLoadRearN);
    FINITE_VALUE(derived->totalBodyForceN.x);
    FINITE_VALUE(derived->totalBodyForceN.y);
    FINITE_VALUE(derived->totalYawTorqueNm);
    FINITE_VALUE(derived->maxFrictionUsage);
    FINITE_VALUE(derived->lowSpeedBlend);
    FINITE_VALUE(derived->frontAxleContactVelocityBodyMps.x);
    FINITE_VALUE(derived->frontAxleContactVelocityBodyMps.y);
    FINITE_VALUE(derived->rearAxleContactVelocityBodyMps.x);
    FINITE_VALUE(derived->rearAxleContactVelocityBodyMps.y);
    FINITE_VALUE(derived->frontSlipAngleRad);
    FINITE_VALUE(derived->rearSlipAngleRad);
    FINITE_VALUE(derived->frontLateralForceN);
    FINITE_VALUE(derived->rearLateralForceN);
    FINITE_VALUE(derived->frontBodyForceN.x);
    FINITE_VALUE(derived->frontBodyForceN.y);
    FINITE_VALUE(derived->rearBodyForceN.x);
    FINITE_VALUE(derived->rearBodyForceN.y);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        FINITE_VALUE(derived->wheelContactVelocityBodyMps[i].x);
        FINITE_VALUE(derived->wheelContactVelocityBodyMps[i].y);
    }
#undef FINITE_VALUE
    if (derived->normalLoadFrontN <= 0.0f || derived->normalLoadRearN <= 0.0f) return false;
    if (fabsf((derived->normalLoadFrontN + derived->normalLoadRearN) -
              spec->massKg * GRAVITY_MPS2) > 1.0f) return false;
    if (derived->speedMps >= MAX_SAFE_SPEED_MPS) return false;
    if (fabsf(state->yawRateRadS) >= MAX_SAFE_YAW_RATE_RADS) return false;
    if (derived->lowSpeedBlend < 0.0f || derived->lowSpeedBlend > 1.0f) return false;
    if (fabsf(state->frontRoadWheelAngleRad) > spec->maxRoadWheelAngleRad + 1e-5f) return false;
    return true;
}

void physics_fixed_update(const VehicleSpec *spec,
                          VehicleState *state,
                          VehicleDerived *derived,
                          VehicleRenderState *renderState,
                          const Input *input,
                          float dt)
{
    if (!vehicle_spec_is_valid(spec) || state == NULL || derived == NULL ||
        renderState == NULL || input == NULL || !(dt > 0.0f)) return;

    const VehicleState lastGoodState = *state;
    const VehicleDerived lastGoodDerived = *derived;
    const VehicleRenderState lastGoodRenderState = *renderState;

    renderState->prevPositionM = renderState->currPositionM;
    renderState->prevHeadingRad = renderState->currHeadingRad;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        renderState->prevWheelAngleRad[i] = renderState->currWheelAngleRad[i];
    }

    physics_update_steering(spec, state, input->steer, dt);

    float frontLoadN;
    float rearLoadN;
    physics_static_axle_loads(spec, &frontLoadN, &rearLoadN);
    derived->normalLoadFrontN = frontLoadN;
    derived->normalLoadRearN = rearLoadN;
    state->wheels[WHEEL_FRONT_LEFT].normalLoadN = frontLoadN * 0.5f;
    state->wheels[WHEEL_FRONT_RIGHT].normalLoadN = frontLoadN * 0.5f;
    state->wheels[WHEEL_REAR_LEFT].normalLoadN = rearLoadN * 0.5f;
    state->wheels[WHEEL_REAR_RIGHT].normalLoadN = rearLoadN * 0.5f;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->wheelContactVelocityBodyMps[i] =
            physics_contact_point_velocity_body(state, state->wheels[i].localPositionM);
    }
    derived->frontAxleContactVelocityBodyMps = (Vector2){
        state->velocityLongitudinalMps,
        state->velocityLateralMps + spec->cgToFrontM * state->yawRateRadS
    };
    derived->rearAxleContactVelocityBodyMps = (Vector2){
        state->velocityLongitudinalMps,
        state->velocityLateralMps - spec->cgToRearM * state->yawRateRadS
    };

    physics_axle_slip_angles(spec, state, &derived->frontSlipAngleRad,
                             &derived->rearSlipAngleRad);
    const float frontFyN = physics_linear_lateral_force(
        derived->frontSlipAngleRad, TIRE_CORNERING_STIFFNESS_FRONT_N_PER_RAD,
        TIRE_MU_LINEAR_FRONT, frontLoadN);
    const float rearFyN = physics_linear_lateral_force(
        derived->rearSlipAngleRad, TIRE_CORNERING_STIFFNESS_REAR_N_PER_RAD,
        TIRE_MU_LINEAR_REAR, rearLoadN);
    derived->frontLateralForceN = frontFyN;
    derived->rearLateralForceN = rearFyN;

    for (int i = WHEEL_FRONT_LEFT; i <= WHEEL_FRONT_RIGHT; i++) {
        state->wheels[i].slipAngleRad = derived->frontSlipAngleRad;
        state->wheels[i].forceLateralN = frontFyN * 0.5f;
        state->wheels[i].forceLongitudinalN = 0.0f;
        state->wheels[i].frictionUsage =
            fabsf(frontFyN) / (TIRE_MU_LINEAR_FRONT * frontLoadN);
    }
    for (int i = WHEEL_REAR_LEFT; i <= WHEEL_REAR_RIGHT; i++) {
        state->wheels[i].slipAngleRad = derived->rearSlipAngleRad;
        state->wheels[i].forceLateralN = rearFyN * 0.5f;
        state->wheels[i].forceLongitudinalN = 0.0f;
        state->wheels[i].frictionUsage =
            fabsf(rearFyN) / (TIRE_MU_LINEAR_REAR * rearLoadN);
    }

    const float longitudinalCommandN =
        phase1_longitudinal_force(spec, state, input, dt);
    derived->frontBodyForceN = physics_rotate_wheel_force_to_body(
        (Vector2){ 0.0f, frontFyN }, state->frontRoadWheelAngleRad);
    derived->rearBodyForceN = (Vector2){ 0.0f, rearFyN };
    derived->frontBodyForceN.x += longitudinalCommandN;
    derived->totalBodyForceN = (Vector2){
        derived->frontBodyForceN.x + derived->rearBodyForceN.x,
        derived->frontBodyForceN.y + derived->rearBodyForceN.y
    };
    derived->totalYawTorqueNm =
        spec->cgToFrontM * derived->frontBodyForceN.y -
        spec->cgToRearM * derived->rearBodyForceN.y;

    const VehicleDerivatives dynamic = dynamic_derivatives(
        spec, state, derived->totalBodyForceN, derived->totalYawTorqueNm);
    const VehicleDerivatives kinematic = kinematic_derivatives(
        spec, state, longitudinalCommandN);
    derived->lowSpeedBlend = physics_low_speed_blend(state->velocityLongitudinalMps);
    const VehicleDerivatives solved = derivatives_lerp(
        kinematic, dynamic, derived->lowSpeedBlend);

    state->velocityLongitudinalMps +=
        solved.velocityLongitudinalDerivativeMps2 * dt;
    state->velocityLateralMps += solved.velocityLateralDerivativeMps2 * dt;
    state->yawRateRadS += solved.yawRateDerivativeRadS2 * dt;
    state->headingRad = wrap_angle(state->headingRad + state->yawRateRadS * dt);

    const float c = cosf(state->headingRad);
    const float s = sinf(state->headingRad);
    const float worldVxMps = state->velocityLongitudinalMps * c -
                             state->velocityLateralMps * s;
    const float worldVyMps = state->velocityLongitudinalMps * s +
                             state->velocityLateralMps * c;
    state->positionM.x += worldVxMps * dt;
    state->positionM.y += worldVyMps * dt;

    /* Report the blended model's actual body acceleration, not the unused full-dynamic
     * force acceleration at low speed. This keeps stationary steering at zero acceleration. */
    derived->longitudinalAccelerationMps2 =
        solved.velocityLongitudinalDerivativeMps2 -
        state->yawRateRadS * state->velocityLateralMps;
    derived->lateralAccelerationMps2 =
        solved.velocityLateralDerivativeMps2 +
        state->yawRateRadS * state->velocityLongitudinalMps;
    state->prevLongAccelMps2 = derived->longitudinalAccelerationMps2;
    derived->speedMps = sqrtf(state->velocityLongitudinalMps *
                              state->velocityLongitudinalMps +
                              state->velocityLateralMps *
                              state->velocityLateralMps);
    derived->bodySideslipRad = (derived->speedMps < LOW_SPEED_EPSILON_MPS)
        ? 0.0f
        : atan2f(state->velocityLateralMps,
                 fmaxf(fabsf(state->velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS));
    derived->maxFrictionUsage = derived->lowSpeedBlend * fmaxf(
        state->wheels[WHEEL_FRONT_LEFT].frictionUsage,
        state->wheels[WHEEL_REAR_LEFT].frictionUsage);
    derived->physicallySliding = derived->maxFrictionUsage >= 0.98f;
    derived->scoringDrift = false;

    renderState->currPositionM = state->positionM;
    renderState->currHeadingRad = state->headingRad;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        renderState->currWheelAngleRad[i] = state->wheels[i].steerAngleRad;
    }

    const bool valid = physics_state_is_valid(spec, state, derived);
    assert(valid);
    if (!valid) {
        *state = lastGoodState;
        *derived = lastGoodDerived;
        *renderState = lastGoodRenderState;
    }
}

#include "physics.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "drivetrain.h"
#include "math_utils.h"
#include "tire.h"

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

/* --------------------------------------------------------------------- Phase 3: loads -- */

float physics_filter_long_accel(float filteredMps2, float previousMps2,
                                float rateHz, float dt)
{
    if (!(isfinite(filteredMps2) && isfinite(previousMps2))) return 0.0f;
    if (!(isfinite(rateHz) && rateHz > 0.0f && isfinite(dt) && dt > 0.0f)) return filteredMps2;
    /* Exponential form rather than `rate * dt`: the response is then a property of the
     * suspension, not of the timestep, and the filter cannot overshoot at any dt. */
    const float alpha = 1.0f - expf(-rateHz * dt);
    const float next = filteredMps2 + (previousMps2 - filteredMps2) * alpha;
    return isfinite(next) ? next : filteredMps2;
}

AxleLoads physics_axle_loads(const VehicleSpec *spec, float filteredLongAccelMps2)
{
    AxleLoads out;
    memset(&out, 0, sizeof(out));
    if (!vehicle_spec_is_valid(spec)) return out;

    physics_static_axle_loads(spec, &out.staticFrontN, &out.staticRearN);

    const float ax = isfinite(filteredLongAccelMps2) ? filteredLongAccelMps2 : 0.0f;
    out.transferN = spec->massKg * ax * spec->cgHeightM / spec->wheelbaseM;

    /* Accelerating forward (positive ax) unloads the front and loads the rear; the two
     * halves are equal and opposite, so the unclamped pair still sums to mass * g. */
    out.unclampedFrontN = out.staticFrontN - out.transferN;
    out.unclampedRearN = out.staticRearN + out.transferN;

    /* A wheel may be unloaded but never generates negative grip. The floor is applied
     * without renormalising the other axle: pretending the lost load went somewhere would
     * hide exactly the condition worth seeing. */
    out.frontN = fmaxf(out.unclampedFrontN, MIN_NORMAL_LOAD_N);
    out.rearN = fmaxf(out.unclampedRearN, MIN_NORMAL_LOAD_N);
    return out;
}

/* ---------------------------------------------------------------- Phase 3: resistance -- */

Vector2 physics_aero_drag_body_n(const VehicleSpec *spec,
                                 float velocityLongitudinalMps, float velocityLateralMps,
                                 float *magnitudeN)
{
    if (magnitudeN != NULL) *magnitudeN = 0.0f;
    if (spec == NULL || !isfinite(velocityLongitudinalMps) || !isfinite(velocityLateralMps)) {
        return (Vector2){ 0.0f, 0.0f };
    }
    if (!(isfinite(spec->dragCoefficient) && spec->dragCoefficient >= 0.0f &&
          isfinite(spec->frontalAreaM2) && spec->frontalAreaM2 >= 0.0f)) {
        return (Vector2){ 0.0f, 0.0f };
    }

    const float speedMps = sqrtf(velocityLongitudinalMps * velocityLongitudinalMps +
                                 velocityLateralMps * velocityLateralMps);
    if (!(speedMps > RESISTANCE_EPSILON_MPS)) return (Vector2){ 0.0f, 0.0f };

    const float dragN = 0.5f * AIR_DENSITY_KGM3 * spec->dragCoefficient *
                        spec->frontalAreaM2 * speedMps * speedMps;
    if (!isfinite(dragN)) return (Vector2){ 0.0f, 0.0f };
    if (magnitudeN != NULL) *magnitudeN = dragN;

    /* Opposite the velocity VECTOR, not the body X axis: a car travelling sideways is
     * pushing air with its flank and must feel it. */
    return (Vector2){ -dragN * velocityLongitudinalMps / speedMps,
                      -dragN * velocityLateralMps / speedMps };
}

Vector2 physics_rolling_resistance_body_n(float rollingResistanceCoefficient,
                                          float normalLoadN, Vector2 contactVelocityMps,
                                          float *magnitudeN)
{
    if (magnitudeN != NULL) *magnitudeN = 0.0f;
    if (!(isfinite(rollingResistanceCoefficient) && rollingResistanceCoefficient > 0.0f &&
          isfinite(normalLoadN) && normalLoadN > 0.0f &&
          isfinite(contactVelocityMps.x) && isfinite(contactVelocityMps.y))) {
        return (Vector2){ 0.0f, 0.0f };
    }

    const float speedMps = sqrtf(contactVelocityMps.x * contactVelocityMps.x +
                                 contactVelocityMps.y * contactVelocityMps.y);
    if (!(speedMps > RESISTANCE_EPSILON_MPS)) return (Vector2){ 0.0f, 0.0f };

    /* Coulomb-style magnitude, exactly c_rr * Fz, so it does NOT vanish with speed the way
     * drag does. The fixed-update aggregate is limited to the impulse needed to stop the
     * body in this tick; that guard prevents a zero crossing without changing this physical
     * force law away from rest. */
    const float rollingN = rollingResistanceCoefficient * normalLoadN;
    if (!isfinite(rollingN)) return (Vector2){ 0.0f, 0.0f };
    if (magnitudeN != NULL) *magnitudeN = rollingN;

    return (Vector2){ -rollingN * contactVelocityMps.x / speedMps,
                      -rollingN * contactVelocityMps.y / speedMps };
}

/* Per-axis stop-force limit: a resistance force may bring a component of body velocity to
 * zero within one fixed step, never past it. Applied component-wise because the two axes
 * integrate independently, and only ever reduces a magnitude. */
static Vector2 limit_resistance_to_stop(Vector2 forceN, const VehicleSpec *spec,
                                        const VehicleState *state, float dt)
{
    const float stopXN = spec->massKg * fabsf(state->velocityLongitudinalMps) / dt;
    const float stopYN = spec->massKg * fabsf(state->velocityLateralMps) / dt;
    return (Vector2){
        copysignf(fminf(fabsf(forceN.x), stopXN), forceN.x),
        copysignf(fminf(fabsf(forceN.y), stopYN), forceN.y)
    };
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
                                                 float wheelLongitudinalForceN)
{
    const float tanDelta = tanf(state->frontRoadWheelAngleRad);
    const float yawTargetRadS =
        state->velocityLongitudinalMps * tanDelta / spec->wheelbaseM;
    const float betaRad = atan2f(spec->cgToRearM * tanDelta, spec->wheelbaseM);
    const float lateralTargetMps =
        state->velocityLongitudinalMps * tanf(betaRad);
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        wheelLongitudinalForceN / spec->massKg +
        state->yawRateRadS * state->velocityLateralMps;
    out.velocityLateralDerivativeMps2 =
        (lateralTargetMps - state->velocityLateralMps) *
        LOW_SPEED_RESPONSE_RATE_HZ;
    out.yawRateDerivativeRadS2 =
        (yawTargetRadS - state->yawRateRadS) * LOW_SPEED_RESPONSE_RATE_HZ;
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
            wheel->frictionUsage > 1.0f + FRICTION_TOLERANCE) return false;
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
        FINITE_VALUE(derived->pureLongitudinalForceN[i]);
        FINITE_VALUE(derived->pureLateralForceN[i]);
        FINITE_VALUE(derived->driveTorqueNm[i]);
        FINITE_VALUE(derived->serviceBrakeTorqueNm[i]);
        FINITE_VALUE(derived->handbrakeTorqueNm[i]);
        const float longLimitN = spec->tireMuLongScale * state->wheels[i].normalLoadN;
        const float lateralMu = (i <= WHEEL_FRONT_RIGHT)
            ? spec->tireMuLatFront : spec->tireMuLatRear;
        const float lateralLimitN = lateralMu * state->wheels[i].normalLoadN;
        const float nx = state->wheels[i].forceLongitudinalN / longLimitN;
        const float ny = state->wheels[i].forceLateralN / lateralLimitN;
        if (sqrtf(nx * nx + ny * ny) > 1.0f + FRICTION_TOLERANCE) return false;
    }
    FINITE_VALUE(derived->engineTorqueNm);
    FINITE_VALUE(derived->totalGearRatio);
    FINITE_VALUE(derived->drivelineTorqueNm);
    FINITE_VALUE(derived->staticFrontLoadN);
    FINITE_VALUE(derived->staticRearLoadN);
    FINITE_VALUE(derived->unclampedFrontLoadN);
    FINITE_VALUE(derived->unclampedRearLoadN);
    FINITE_VALUE(derived->loadTransferN);
    FINITE_VALUE(derived->previousLongAccelMps2);
    FINITE_VALUE(derived->filteredLongAccelMps2);
    FINITE_VALUE(derived->solvedLongAccelMps2);
    FINITE_VALUE(derived->aeroDragMagnitudeN);
    FINITE_VALUE(derived->aeroDragBodyN.x);
    FINITE_VALUE(derived->aeroDragBodyN.y);
    FINITE_VALUE(derived->rollingResistanceMagnitudeN);
    FINITE_VALUE(derived->rollingResistanceBodyN.x);
    FINITE_VALUE(derived->rollingResistanceBodyN.y);
    for (int i = 0; i < WHEEL_COUNT; i++) FINITE_VALUE(derived->wheelRollingResistanceN[i]);
#undef FINITE_VALUE

    /* ------------------------------------------------------------- Phase 3 load transfer -- */

    const float weightN = spec->massKg * GRAVITY_MPS2;
    if (derived->staticFrontLoadN <= 0.0f || derived->staticRearLoadN <= 0.0f) return false;
    if (fabsf((derived->staticFrontLoadN + derived->staticRearLoadN) - weightN) > 1.0f) {
        return false;
    }
    /* Transfer only MOVES load: what leaves one axle arrives at the other, so the unclamped
     * pair still weighs the car. The clamped pair may legitimately exceed m*g once the floor
     * has caught an unloaded axle, which is why the sum is asserted here and not there. */
    if (fabsf((derived->unclampedFrontLoadN + derived->unclampedRearLoadN) - weightN) > 1.0f) {
        return false;
    }
    if (fabsf(derived->loadTransferN) > MAX_LOAD_TRANSFER_FRACTION * weightN) return false;
    if (derived->normalLoadFrontN < MIN_NORMAL_LOAD_N - 1e-3f ||
        derived->normalLoadRearN < MIN_NORMAL_LOAD_N - 1e-3f) return false;

    /* Each axle load is mirrored equally across its two wheel entries in the bicycle model. */
    if (fabsf((state->wheels[WHEEL_FRONT_LEFT].normalLoadN +
               state->wheels[WHEEL_FRONT_RIGHT].normalLoadN) -
              derived->normalLoadFrontN) > 1e-2f) return false;
    if (fabsf((state->wheels[WHEEL_REAR_LEFT].normalLoadN +
               state->wheels[WHEEL_REAR_RIGHT].normalLoadN) -
              derived->normalLoadRearN) > 1e-2f) return false;

    /* ---------------------------------------------------------------- Phase 3 resistance -- */

    /* Resistance removes energy or does nothing; it never adds any. A positive dot product
     * with the velocity it opposes would mean drag is propelling the car. The per-wheel
     * direction property is asserted against the pure function in the `resistance` unit
     * scenario; here the stored aggregates are checked so validation stays cheap enough to
     * run every tick. */
    if (derived->aeroDragBodyN.x * state->velocityLongitudinalMps +
        derived->aeroDragBodyN.y * state->velocityLateralMps >
        RESISTANCE_POWER_TOLERANCE_W) return false;
    if (derived->rollingResistanceBodyN.x * state->velocityLongitudinalMps +
        derived->rollingResistanceBodyN.y * state->velocityLateralMps >
        RESISTANCE_POWER_TOLERANCE_W) return false;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (derived->wheelRollingResistanceN[i] < 0.0f) return false;
    }

    if (derived->speedMps >= MAX_SAFE_SPEED_MPS) return false;
    if (fabsf(state->yawRateRadS) >= MAX_SAFE_YAW_RATE_RADS) return false;
    if (derived->lowSpeedBlend < 0.0f || derived->lowSpeedBlend > 1.0f) return false;
    if (fabsf(state->frontRoadWheelAngleRad) > spec->maxRoadWheelAngleRad + 1e-5f) return false;
    if (state->selectedGear < -1 || state->selectedGear > spec->gearCount) return false;
    if (state->engineRpm < spec->engineIdleRpm - 1e-3f ||
        state->engineRpm > spec->engineRedlineRpm + 1e-3f) return false;
    if (fabsf(state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS -
              state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS) > 1e-5f) return false;
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

    if (fabsf(state->velocityLongitudinalMps) < 0.1f &&
        input->throttle <= 0.0f) {
        if (input->brake > 0.0f) {
            state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS = 0.0f;
            state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS = 0.0f;
        }
        if (input->brake > 0.0f || input->handbrake > 0.0f) {
            state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = 0.0f;
            state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = 0.0f;
        }
    }

    /* Phase 2's bicycle model uses a locked rear axle. Enforce a single deterministic
     * angular state before it is used for RPM, torque, and slip calculations. */
    const float rearAngularVelocityRadS =
        0.5f * (state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS +
                state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS);
    state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = rearAngularVelocityRadS;
    state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = rearAngularVelocityRadS;
    state->engineRpm = drivetrain_engine_rpm(
        spec, state->selectedGear, rearAngularVelocityRadS);
    const DrivetrainTorques torques = drivetrain_calculate_torques(
        spec, state->selectedGear, rearAngularVelocityRadS,
        input->throttle, input->brake, input->handbrake);
    derived->engineTorqueNm = torques.engineTorqueNm;
    derived->totalGearRatio = torques.totalGearRatio;
    derived->drivelineTorqueNm = torques.drivelineTorqueNm;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->driveTorqueNm[i] = torques.driveTorqueNm[i];
        derived->serviceBrakeTorqueNm[i] = torques.serviceBrakeTorqueNm[i];
        derived->handbrakeTorqueNm[i] = torques.handbrakeTorqueNm[i];
    }

    /* --- 5. contact-point velocities ---------------------------------------------------- */

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

    /* --- 6. slip angles and slip ratios -------------------------------------------------- */

    physics_axle_slip_angles(spec, state, &derived->frontSlipAngleRad,
                             &derived->rearSlipAngleRad);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &state->wheels[i];
        const bool front = i <= WHEEL_FRONT_RIGHT;
        wheel->slipAngleRad = front ? derived->frontSlipAngleRad : derived->rearSlipAngleRad;
        wheel->slipRatio = tire_slip_ratio(
            wheel->angularVelocityRadS, spec->wheelRadiusM,
            front ? derived->frontAxleContactVelocityBodyMps.x
                  : derived->rearAxleContactVelocityBodyMps.x,
            SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP);
    }

    /* --- 7. filtered previous-step longitudinal acceleration ----------------------------- */

    /* The load filter consumes the PREVIOUS step's solved body acceleration. Feeding it this
     * step's value would make loads depend on forces that depend on loads — an algebraic
     * loop with no solution — and a velocity finite difference would smuggle in the r*vy
     * transport term, which pitches nothing. */
    derived->previousLongAccelMps2 = state->prevLongAccelMps2;
    state->filteredLongAccelMps2 = physics_filter_long_accel(
        state->filteredLongAccelMps2, state->prevLongAccelMps2, spec->loadFilterRateHz, dt);
    derived->filteredLongAccelMps2 = state->filteredLongAccelMps2;

    /* --- 8. static and dynamic axle loads ------------------------------------------------ */

    const AxleLoads loads = physics_axle_loads(spec, state->filteredLongAccelMps2);
    derived->staticFrontLoadN = loads.staticFrontN;
    derived->staticRearLoadN = loads.staticRearN;
    derived->unclampedFrontLoadN = loads.unclampedFrontN;
    derived->unclampedRearLoadN = loads.unclampedRearN;
    derived->loadTransferN = loads.transferN;
    derived->normalLoadFrontN = loads.frontN;
    derived->normalLoadRearN = loads.rearN;

    /* --- 9. wheel loads ------------------------------------------------------------------ */

    /* Phase 3 is axle-based: each axle load is mirrored equally across its two contact
     * points. Lateral load transfer, which would break that symmetry, is Phase 4. */
    state->wheels[WHEEL_FRONT_LEFT].normalLoadN = loads.frontN * 0.5f;
    state->wheels[WHEEL_FRONT_RIGHT].normalLoadN = loads.frontN * 0.5f;
    state->wheels[WHEEL_REAR_LEFT].normalLoadN = loads.rearN * 0.5f;
    state->wheels[WHEEL_REAR_RIGHT].normalLoadN = loads.rearN * 0.5f;

    /* --- 10/11. pure tire forces and the combined-friction limit ------------------------- */

    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &state->wheels[i];
        const bool front = i <= WHEEL_FRONT_RIGHT;
        const float lateralB = front ? spec->tireBLatFront : spec->tireBLatRear;
        const float lateralC = front ? spec->tireCLatFront : spec->tireCLatRear;
        const float lateralMu = front ? spec->tireMuLatFront : spec->tireMuLatRear;

        derived->pureLongitudinalForceN[i] = tire_longitudinal_force_n(
            wheel->slipRatio, wheel->normalLoadN,
            spec->tireBLong, spec->tireCLong, spec->tireMuLongScale);
        derived->pureLateralForceN[i] = tire_lateral_force_n(
            wheel->slipAngleRad, wheel->normalLoadN, lateralB, lateralC, lateralMu);
        tire_apply_combined_limit(
            derived->pureLongitudinalForceN[i],
            derived->pureLateralForceN[i],
            spec->tireMuLongScale * wheel->normalLoadN,
            lateralMu * wheel->normalLoadN,
            &wheel->forceLongitudinalN,
            &wheel->forceLateralN,
            &wheel->frictionUsage);
    }

    /* --- 14/15. aerodynamic drag and rolling resistance ---------------------------------- */

    Vector2 aeroDragN = physics_aero_drag_body_n(
        spec, state->velocityLongitudinalMps, state->velocityLateralMps,
        &derived->aeroDragMagnitudeN);

    Vector2 rollingN = { 0.0f, 0.0f };
    derived->rollingResistanceMagnitudeN = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Vector2 wheelRollingN = physics_rolling_resistance_body_n(
            spec->rollingResistanceCoefficient, state->wheels[i].normalLoadN,
            derived->wheelContactVelocityBodyMps[i], &derived->wheelRollingResistanceN[i]);
        rollingN.x += wheelRollingN.x;
        rollingN.y += wheelRollingN.y;
        derived->rollingResistanceMagnitudeN += derived->wheelRollingResistanceN[i];
    }

    aeroDragN = limit_resistance_to_stop(aeroDragN, spec, state, dt);
    rollingN = limit_resistance_to_stop(rollingN, spec, state, dt);
    derived->aeroDragBodyN = aeroDragN;
    derived->rollingResistanceBodyN = rollingN;

    const Vector2 resistanceBodyN = { aeroDragN.x + rollingN.x, aeroDragN.y + rollingN.y };
    const float longitudinalResistanceN = resistanceBodyN.x;

    if ((input->brake > 0.0f || input->handbrake > 0.0f) &&
        fabsf(state->velocityLongitudinalMps) < LOW_SPEED_BEGIN_MPS &&
        fabsf(state->velocityLongitudinalMps) > 0.0f) {
        const float frontFxBodyN = cosf(state->frontRoadWheelAngleRad) *
            (state->wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
             state->wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN);
        const float rearFxBodyN =
            state->wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
            state->wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
        const float wheelFxBodyN = frontFxBodyN + rearFxBodyN;
        const float currentNetN = wheelFxBodyN + longitudinalResistanceN;
        const float stopNetN = -copysignf(
            spec->massKg * fabsf(state->velocityLongitudinalMps) / dt,
            state->velocityLongitudinalMps);
        if (currentNetN * state->velocityLongitudinalMps < 0.0f &&
            fabsf(currentNetN) > fabsf(stopNetN) &&
            fabsf(wheelFxBodyN) > 1e-6f) {
            const float targetWheelFxBodyN = stopNetN - longitudinalResistanceN;
            const float scale = clampf(targetWheelFxBodyN / wheelFxBodyN, 0.0f, 1.0f);
            for (int i = 0; i < WHEEL_COUNT; i++) {
                WheelState *wheel = &state->wheels[i];
                wheel->forceLongitudinalN *= scale;
                const float lateralMu = (i <= WHEEL_FRONT_RIGHT)
                    ? spec->tireMuLatFront : spec->tireMuLatRear;
                const float nx = wheel->forceLongitudinalN /
                    (spec->tireMuLongScale * wheel->normalLoadN);
                const float ny = wheel->forceLateralN /
                    (lateralMu * wheel->normalLoadN);
                wheel->frictionUsage = fminf(sqrtf(nx * nx + ny * ny), 1.0f);
            }
        }
    }

    derived->frontLateralForceN =
        state->wheels[WHEEL_FRONT_LEFT].forceLateralN +
        state->wheels[WHEEL_FRONT_RIGHT].forceLateralN;
    derived->rearLateralForceN =
        state->wheels[WHEEL_REAR_LEFT].forceLateralN +
        state->wheels[WHEEL_REAR_RIGHT].forceLateralN;
    derived->frontBodyForceN = (Vector2){ 0.0f, 0.0f };
    derived->rearBodyForceN = (Vector2){ 0.0f, 0.0f };
    for (int i = WHEEL_FRONT_LEFT; i <= WHEEL_FRONT_RIGHT; i++) {
        const Vector2 bodyForceN = physics_rotate_wheel_force_to_body(
            (Vector2){ state->wheels[i].forceLongitudinalN,
                       state->wheels[i].forceLateralN },
            state->frontRoadWheelAngleRad);
        derived->frontBodyForceN.x += bodyForceN.x;
        derived->frontBodyForceN.y += bodyForceN.y;
    }
    for (int i = WHEEL_REAR_LEFT; i <= WHEEL_REAR_RIGHT; i++) {
        derived->rearBodyForceN.x += state->wheels[i].forceLongitudinalN;
        derived->rearBodyForceN.y += state->wheels[i].forceLateralN;
    }
    /* --- 13/14/15. sum tire forces, then add the two resistance forces ------------------- */

    derived->totalBodyForceN = (Vector2){
        derived->frontBodyForceN.x + derived->rearBodyForceN.x + resistanceBodyN.x,
        derived->frontBodyForceN.y + derived->rearBodyForceN.y + resistanceBodyN.y
    };
    /* Yaw torque comes from the tire forces alone. Both bicycle-model contact points sit on
     * the centreline, and the spec's yaw equation for Phases 1-3 is written over the axle
     * lateral forces only; giving resistance its own yaw term here would be a Phase 4
     * four-contact-patch change smuggled in early. */
    derived->totalYawTorqueNm =
        spec->cgToFrontM * derived->frontBodyForceN.y -
        spec->cgToRearM * derived->rearBodyForceN.y;

    const VehicleDerivatives dynamic = dynamic_derivatives(
        spec, state, derived->totalBodyForceN, derived->totalYawTorqueNm);
    /* At kinematic speeds, steering geometry owns lateral/yaw motion. Longitudinal
     * acceleration still comes only from limited tire Fx plus the two resistance forces,
     * so steering a stationary car cannot create propulsion from a rotated lateral force. */
    const float kinematicLongitudinalForceN =
        cosf(state->frontRoadWheelAngleRad) *
            (state->wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
             state->wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN) +
        state->wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
        state->wheels[WHEEL_REAR_RIGHT].forceLongitudinalN +
        longitudinalResistanceN;
    const VehicleDerivatives kinematic = kinematic_derivatives(
        spec, state, kinematicLongitudinalForceN);
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

    /* Constant across the four wheels: the slip at which the longitudinal curve peaks. */
    const float peakSlip = tanf(DRIFTY_PI / (2.0f * spec->tireCLong)) / spec->tireBLong;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        /* Body integration precedes wheel integration, so the brake/free-roll crossing
         * guard uses the updated contact speed rather than lagging one fixed tick behind. */
        const float wheelVxMps = state->velocityLongitudinalMps;
        const float previousOmegaRadS = state->wheels[i].angularVelocityRadS;
        float nextOmegaRadS = drivetrain_integrate_wheel(
            previousOmegaRadS, wheelVxMps,
            torques.driveTorqueNm[i], torques.serviceBrakeTorqueNm[i],
            torques.handbrakeTorqueNm[i], state->wheels[i].forceLongitudinalN,
            spec->wheelRadiusM, spec->wheelInertiaKgM2, dt,
            &state->wheels[i].locked);

        /* Stiff-limit equilibrium treatment for the unbraked wheel.
         *
         * The wheel-plus-tire subsystem's time constant is well under a millisecond —
         * far below the fixed step — so under any steady sub-peak torque the physical
         * wheel reaches its balance slip within the tick. The explicit integrator cannot:
         * it either overshoots the balance point (a tick-scale limit cycle between engine
         * braking and tire reaction) or, when the start-of-step slip was zero, produces no
         * torque at all and freezes the wheel while the body moves beneath it, which
         * returns as a phantom multi-kilonewton force one tick later.
         *
         * So when a stable equilibrium exists and the wheel is inside the pre-peak band
         * around it — the region where the linearized dynamics converge within one tick —
         * land on the equilibrium directly. Outside the band (deep wheelspin shedding
         * speed through a saturated curve) the multi-tick explicit dynamics are the right
         * answer and are kept. Torques at or beyond the curve's peak have no equilibrium
         * and always stay explicit; braking keeps its own lock/release dynamics. The
         * pure-curve balance ignores ellipse scaling, so under heavy combined slip the
         * landing is slightly conservative; that error is bounded by the ellipse, while
         * the artifacts this removes are an order of magnitude larger. */
        if (torques.serviceBrakeTorqueNm[i] <= 0.0f &&
            torques.handbrakeTorqueNm[i] <= 0.0f) {
            float equilibriumOmegaRadS;
            if (drivetrain_wheel_equilibrium_omega(
                    torques.driveTorqueNm[i], wheelVxMps, spec->wheelRadiusM,
                    spec->tireMuLongScale * state->wheels[i].normalLoadN,
                    spec->tireBLong, spec->tireCLong, SLIP_SPEED_EPSILON_MPS,
                    &equilibriumOmegaRadS)) {
                const float vxSafeMps = fmaxf(fabsf(wheelVxMps), SLIP_SPEED_EPSILON_MPS);
                const float bandRadS = peakSlip * vxSafeMps / spec->wheelRadiusM;
                const bool crossed =
                    (previousOmegaRadS - equilibriumOmegaRadS) *
                    (nextOmegaRadS - equilibriumOmegaRadS) < 0.0f;
                if (crossed || fabsf(previousOmegaRadS - equilibriumOmegaRadS) < bandRadS) {
                    nextOmegaRadS = equilibriumOmegaRadS;
                    state->wheels[i].locked = false;
                }
            }
        }
        state->wheels[i].angularVelocityRadS = nextOmegaRadS;
    }
    /* Identical rear inputs should already produce identical results. Assigning the
     * shared value makes the locked-axle invariant explicit and immune to later rounding
     * differences in diagnostic-only branches. */
    state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS =
        state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    state->wheels[WHEEL_REAR_RIGHT].locked =
        state->wheels[WHEEL_REAR_LEFT].locked;
    if (torques.totalGearRatio != 0.0f) {
        const float redlineWheelOmegaRadS =
            spec->engineRedlineRpm * DRIFTY_TWO_PI /
            (60.0f * fabsf(torques.totalGearRatio));
        const float rearOmega = state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
        const float omegaSign = (rearOmega > 0.0f) ? 1.0f :
                                (rearOmega < 0.0f) ? -1.0f : 0.0f;
        const float limitedRearOmega = omegaSign *
            fminf(fabsf(rearOmega), redlineWheelOmegaRadS);
        state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = limitedRearOmega;
        state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = limitedRearOmega;
    }
    state->engineRpm = drivetrain_engine_rpm(
        spec, state->selectedGear,
        state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS);

    /* Body acceleration is force divided by mass. Do not reconstruct it from the integrated
     * derivative: dvx/dt also contains the rotating-frame transport term r*vy, and using the
     * post-integration state would add a small same-step error. The low-speed blend changes
     * state derivatives, not the force solution used by load transfer and telemetry. */
    derived->longitudinalAccelerationMps2 =
        derived->totalBodyForceN.x / spec->massKg;
    /* The kinematic low-speed model deliberately suppresses tire-model lateral force at
     * rest, so its reported lateral acceleration follows the solved blended derivative. */
    derived->lateralAccelerationMps2 =
        solved.velocityLateralDerivativeMps2 +
        state->yawRateRadS * state->velocityLongitudinalMps;

    /* --- 23. store the solved body-longitudinal acceleration for the next step ----------- */

    /* ax_body, not dvx_dt: load transfer responds to the longitudinal force on the body, and
     * dvx_dt carries the rotational transport term r*vy, which pitches nothing. */
    derived->solvedLongAccelMps2 = derived->longitudinalAccelerationMps2;
    state->prevLongAccelMps2 = derived->longitudinalAccelerationMps2;
    derived->speedMps = sqrtf(state->velocityLongitudinalMps *
                              state->velocityLongitudinalMps +
                              state->velocityLateralMps *
                              state->velocityLateralMps);
    derived->bodySideslipRad = (derived->speedMps < LOW_SPEED_EPSILON_MPS)
        ? 0.0f
        : atan2f(state->velocityLateralMps,
                 fmaxf(fabsf(state->velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS));
    derived->maxFrictionUsage = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->maxFrictionUsage =
            fmaxf(derived->maxFrictionUsage, state->wheels[i].frictionUsage);
    }
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

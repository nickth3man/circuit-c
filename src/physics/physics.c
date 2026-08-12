#include "physics/physics.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "physics/drivetrain.h"
#include "core/math_utils.h"
#include "physics/surface.h"
#include "physics/tire.h"

Vector2 physics_contact_point_velocity_body(const VehicleState *state, Vector2 pointM)
{
    if (state == NULL) return (Vector2){ 0.0f, 0.0f };
    return (Vector2){ state->velocityLongitudinalMps - state->yawRateRadS * pointM.y,
                      state->velocityLateralMps + state->yawRateRadS * pointM.x };
}

Vector2 physics_wheel_world_position(const VehicleState *state, WheelId wheelId)
{
    if (state == NULL || wheelId < 0 || wheelId >= WHEEL_COUNT) {
        return (Vector2){ 0.0f, 0.0f };
    }
    const Vector2 local = state->wheels[wheelId].localPositionM;
    const float c = cosf(state->headingRad);
    const float s = sinf(state->headingRad);
    return (Vector2){ state->positionM.x + local.x * c - local.y * s,
                      state->positionM.y + local.x * s + local.y * c };
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

/* Static toe as a per-wheel heading offset, in the body frame where +angle is a left (counter-
 * clockwise) rotation and +y is the left of the car.
 *
 * Positive toe is toe-IN: both wheels of the axle point at the centreline, so the LEFT wheel is
 * rotated clockwise (negative) and the RIGHT wheel counter-clockwise (positive). The offset is
 * a property of the suspension, not of the rack, so it is ADDED to whatever the steering and
 * Ackermann geometry produced rather than mixed into that calculation. */
float physics_wheel_toe_offset_rad(float toeRad, float wheelLateralPositionM)
{
    if (!(isfinite(toeRad) && isfinite(wheelLateralPositionM))) return 0.0f;
    return (wheelLateralPositionM > 0.0f) ? -toeRad : toeRad;
}

/* TODO(vehicle-audit #10): maxRoadWheelAngleRad is authored per car, and the two "grip" cars
 * (awd_gt, rwd_grip) get only 25.8 deg — less than a real road car (~35-40 deg), so they cannot
 * tighten their line in a hairpin — while rwd_power gets 64.7 deg (drift territory) and awd_rally
 * 55 deg. Re-author per-car steer max angle against real benchmarks. */

void physics_update_steering(const VehicleSpec *spec, VehicleState *state, float steerInput,
                             float dt)
{
    if (spec == NULL || state == NULL || !(dt > 0.0f)) return;
    const float target = clampf(steerInput, -1.0f, 1.0f) * spec->maxRoadWheelAngleRad;
    const float error = target - state->frontRoadWheelAngleRad;
    /* Speed-sensitive feel: steering rate decreases at higher speed. */
    const float speedFrac =
        clampf(spec->steerSpeedRefMps / fmaxf(fabsf(state->velocityLongitudinalMps), 1.0f),
               0.0f, 1.0f);
    const float speedFactor =
        spec->steerSpeedMinFactor + (1.0f - spec->steerSpeedMinFactor) * speedFrac;
    const float rate = (fabsf(target) < fabsf(state->frontRoadWheelAngleRad))
                           ? spec->steerReturnRateRadS * speedFactor
                           : spec->maxSteerRateRadS * speedFactor;
    const float maxChange = rate * dt;
    state->frontRoadWheelAngleRad =
        clampf(state->frontRoadWheelAngleRad + clampf(error, -maxChange, maxChange),
               -spec->maxRoadWheelAngleRad, spec->maxRoadWheelAngleRad);
    /* Ackermann steering: inner wheel steers more than outer. With ackermannPercent
     * at the default 0, steerFL == steerFR == deltaC (parallel, neutral). */
    const float deltaC = state->frontRoadWheelAngleRad;
    float steerFL = deltaC, steerFR = deltaC;
    if (fabsf(deltaC) > 1e-4f) {
        const float R = spec->wheelbaseM / tanf(deltaC);
        const float trackHalf = spec->trackWidthFrontM * 0.5f;
        const float sign = (deltaC > 0.0f) ? 1.0f : -1.0f;
        const float deltaInner = atanf(spec->wheelbaseM / (R - sign * trackHalf));
        const float deltaOuter = atanf(spec->wheelbaseM / (R + sign * trackHalf));
        float deltaL, deltaR;
        if (deltaC > 0.0f) {
            deltaL = deltaInner;
            deltaR = deltaOuter;
        } else {
            deltaL = deltaOuter;
            deltaR = deltaInner;
        }
        steerFL = deltaC + spec->ackermannPercent * (deltaL - deltaC);
        steerFR = deltaC + spec->ackermannPercent * (deltaR - deltaC);
    }
    /* Static alignment closes the chain: steerAngleRad is now each wheel's EFFECTIVE heading in
     * the body frame — rack angle, plus Ackermann, plus static toe — which is exactly what the
     * slip-angle and force-rotation stages read. The rear wheels have no rack, so their heading
     * is their toe alone. */
    const float toeFrontRad = spec->suspToeFrontRad;
    const float toeRearRad = spec->suspToeRearRad;
    state->wheels[WHEEL_FRONT_LEFT].steerAngleRad =
        steerFL + physics_wheel_toe_offset_rad(
                      toeFrontRad, state->wheels[WHEEL_FRONT_LEFT].localPositionM.y);
    state->wheels[WHEEL_FRONT_RIGHT].steerAngleRad =
        steerFR + physics_wheel_toe_offset_rad(
                      toeFrontRad, state->wheels[WHEEL_FRONT_RIGHT].localPositionM.y);
    state->wheels[WHEEL_REAR_LEFT].steerAngleRad = physics_wheel_toe_offset_rad(
        toeRearRad, state->wheels[WHEEL_REAR_LEFT].localPositionM.y);
    state->wheels[WHEEL_REAR_RIGHT].steerAngleRad = physics_wheel_toe_offset_rad(
        toeRearRad, state->wheels[WHEEL_REAR_RIGHT].localPositionM.y);
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
    const float vxSign = (state->velocityLongitudinalMps >= 0.0f) ? 1.0f : -1.0f;
    if (frontSlipAngleRad != NULL) {
        *frontSlipAngleRad = atan2f(frontVy, vxSafe) - vxSign * state->frontRoadWheelAngleRad;
    }
    if (rearSlipAngleRad != NULL) *rearSlipAngleRad = atan2f(rearVy, vxSafe);
}

Vector2 physics_rotate_wheel_force_to_body(Vector2 wheelForceN, float steerAngleRad)
{
    const float c = cosf(steerAngleRad);
    const float s = sinf(steerAngleRad);
    return (Vector2){ wheelForceN.x * c - wheelForceN.y * s,
                      wheelForceN.x * s + wheelForceN.y * c };
}

float physics_low_speed_blend(float velocityLongitudinalMps)
{
    return smoothstep(LOW_SPEED_BEGIN_MPS, LOW_SPEED_END_MPS, fabsf(velocityLongitudinalMps));
}

/* The body's speed through the air, before this step integrates anything. Both the drag model
 * and the aerodynamic vertical loads are functions of it, and they must not disagree about
 * which velocity the car is flying at. */
static float body_speed_mps(const VehicleState *state)
{
    return sqrtf(state->velocityLongitudinalMps * state->velocityLongitudinalMps +
                 state->velocityLateralMps * state->velocityLateralMps);
}

/* --------------------------------------------------------------------- Phase 3: loads -- */

float physics_filter_long_accel(float filteredMps2, float previousMps2, float rateHz, float dt)
{
    if (!(isfinite(filteredMps2) && isfinite(previousMps2))) return 0.0f;
    if (!(isfinite(rateHz) && rateHz > 0.0f && isfinite(dt) && dt > 0.0f)) return filteredMps2;
    /* Exponential form rather than `rate * dt`: the response is then a property of the
     * suspension, not of the timestep, and the filter cannot overshoot at any dt. */
    const float alpha = 1.0f - expf(-rateHz * dt);
    const float next = filteredMps2 + (previousMps2 - filteredMps2) * alpha;
    return isfinite(next) ? next : filteredMps2;
}

/*
 * One axle's aerodynamic vertical load.
 *
 * The product is accumulated in DOUBLE and only then narrowed. Doing it in float lets a large
 * authored coefficient overflow to infinity partway through the multiply, and an infinity here
 * has no honest float answer: returning zero would report "this car has no aerodynamics" for
 * the car with the most, which is the quiet degradation this model must not do. Double carries
 * every product a float coefficient and a float area can make, so the narrowing below is the
 * only place representability is in question, and it saturates — preserving the direction and
 * the ordering of the load — rather than discarding it.
 *
 * vehicle_spec_is_valid() rejects any spec whose load at MAX_SAFE_SPEED_MPS is not
 * representable, so the saturation is unreachable through the solver. It exists because this
 * helper is public and must be total for any inputs a caller hands it.
 */
static float aero_axle_vertical_n(float liftCoefficient, float refAreaM2, float speedMps)
{
    if (!(isfinite(liftCoefficient) && isfinite(refAreaM2) && refAreaM2 >= 0.0f)) return 0.0f;
    if (!(isfinite(speedMps) && speedMps > RESISTANCE_EPSILON_MPS)) return 0.0f;

    const double loadN = -0.5 * (double)AIR_DENSITY_KGM3 * (double)liftCoefficient *
                         (double)refAreaM2 * (double)speedMps * (double)speedMps;
    if (!(loadN > -(double)FLT_MAX)) return -FLT_MAX;
    if (!(loadN < (double)FLT_MAX)) return FLT_MAX;
    return (float)loadN;
}

/* TODO(vehicle-audit #2): aerodynamic vertical loads are fully implemented (#17) but every
 * shipped car has aeroLiftCoef 0, so no car produces downforce or lift. The fast "race" cars
 * (awd_gt/rwd_grip at 277 km/h, rwd_power at 295 km/h) are therefore grip-only and under-corner
 * at speed versus real counterparts, which run wings/splitters. Author per-car negative lift
 * coefficients and reference areas. */

void physics_aero_vertical_loads(const VehicleSpec *spec, float speedMps, float *frontN,
                                 float *rearN)
{
    if (frontN != NULL) *frontN = 0.0f;
    if (rearN != NULL) *rearN = 0.0f;
    if (spec == NULL) return;
    if (frontN != NULL) {
        *frontN =
            aero_axle_vertical_n(spec->aeroLiftCoefFront, spec->aeroRefAreaFrontM2, speedMps);
    }
    if (rearN != NULL) {
        *rearN =
            aero_axle_vertical_n(spec->aeroLiftCoefRear, spec->aeroRefAreaRearM2, speedMps);
    }
}

AxleLoads physics_axle_loads(const VehicleSpec *spec, float filteredLongAccelMps2,
                             float speedMps)
{
    AxleLoads out;
    memset(&out, 0, sizeof(out));
    if (!vehicle_spec_is_valid(spec)) return out;

    physics_static_axle_loads(spec, &out.staticFrontN, &out.staticRearN);
    physics_aero_vertical_loads(spec, speedMps, &out.aeroFrontN, &out.aeroRearN);

    const float ax = isfinite(filteredLongAccelMps2) ? filteredLongAccelMps2 : 0.0f;
    out.transferN = spec->massKg * ax * spec->cgHeightM / spec->wheelbaseM;

    /* Accelerating forward (positive ax) unloads the front and loads the rear; the two
     * halves are equal and opposite, so the transfer moves load without creating any. The
     * aerodynamic terms DO create load — they are an external force on the body, not a
     * couple — so the unclamped pair sums to mass * g plus the two of them. */
    out.unclampedFrontN = out.staticFrontN + out.aeroFrontN - out.transferN;
    out.unclampedRearN = out.staticRearN + out.aeroRearN + out.transferN;

    /* A wheel may be unloaded but never generates negative grip. The floor is applied
     * without renormalising the other axle: pretending the lost load went somewhere would
     * hide exactly the condition worth seeing. */
    out.frontN = fmaxf(out.unclampedFrontN, MIN_NORMAL_LOAD_N);
    out.rearN = fmaxf(out.unclampedRearN, MIN_NORMAL_LOAD_N);
    return out;
}

/* ---------------------------------------------------------------- Phase 3: resistance -- */

Vector2 physics_aero_drag_body_n(const VehicleSpec *spec, float velocityLongitudinalMps,
                                 float velocityLateralMps, float *magnitudeN)
{
    if (magnitudeN != NULL) *magnitudeN = 0.0f;
    if (spec == NULL || !isfinite(velocityLongitudinalMps) || !isfinite(velocityLateralMps)) {
        return (Vector2){ 0.0f, 0.0f };
    }
    if (!(isfinite(spec->frontalAreaM2) && spec->frontalAreaM2 >= 0.0f)) {
        return (Vector2){ 0.0f, 0.0f };
    }
    const float effectiveCd = vehicle_effective_drag_coefficient(spec);
    if (!(isfinite(effectiveCd) && effectiveCd >= 0.0f)) {
        return (Vector2){ 0.0f, 0.0f };
    }

    const float speedMps = sqrtf(velocityLongitudinalMps * velocityLongitudinalMps +
                                 velocityLateralMps * velocityLateralMps);
    if (!(speedMps > RESISTANCE_EPSILON_MPS)) return (Vector2){ 0.0f, 0.0f };

    const float dragN =
        0.5f * AIR_DENSITY_KGM3 * effectiveCd * spec->frontalAreaM2 * speedMps * speedMps;
    if (!isfinite(dragN)) return (Vector2){ 0.0f, 0.0f };
    if (magnitudeN != NULL) *magnitudeN = dragN;

    /* Opposite the velocity VECTOR, not the body X axis: a car travelling sideways is
     * pushing air with its flank and must feel it. */
    return (Vector2){ -dragN * velocityLongitudinalMps / speedMps,
                      -dragN * velocityLateralMps / speedMps };
}

Vector2 physics_rolling_resistance_body_n(float rollingResistanceCoefficient, float normalLoadN,
                                          Vector2 contactVelocityMps, float *magnitudeN)
{
    if (magnitudeN != NULL) *magnitudeN = 0.0f;
    if (!(isfinite(rollingResistanceCoefficient) && rollingResistanceCoefficient > 0.0f &&
          isfinite(normalLoadN) && normalLoadN > 0.0f && isfinite(contactVelocityMps.x) &&
          isfinite(contactVelocityMps.y))) {
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
    return (Vector2){ copysignf(fminf(fabsf(forceN.x), stopXN), forceN.x),
                      copysignf(fminf(fabsf(forceN.y), stopYN), forceN.y) };
}

static VehicleDerivatives dynamic_derivatives(const VehicleSpec *spec,
                                              const VehicleState *state,
                                              Vector2 totalBodyForceN, float yawTorqueNm)
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
    const float yawTargetRadS = state->velocityLongitudinalMps * tanDelta / spec->wheelbaseM;
    const float betaRad = atan2f(spec->cgToRearM * tanDelta, spec->wheelbaseM);
    const float lateralTargetMps = state->velocityLongitudinalMps * tanf(betaRad);
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        wheelLongitudinalForceN / spec->massKg + state->yawRateRadS * state->velocityLateralMps;
    out.velocityLateralDerivativeMps2 =
        (lateralTargetMps - state->velocityLateralMps) * LOW_SPEED_RESPONSE_RATE_HZ;
    out.yawRateDerivativeRadS2 =
        (yawTargetRadS - state->yawRateRadS) * LOW_SPEED_RESPONSE_RATE_HZ;
    return out;
}

static VehicleDerivatives derivatives_lerp(VehicleDerivatives a, VehicleDerivatives b, float t)
{
    VehicleDerivatives out;
    out.velocityLongitudinalDerivativeMps2 =
        lerpf(a.velocityLongitudinalDerivativeMps2, b.velocityLongitudinalDerivativeMps2, t);
    out.velocityLateralDerivativeMps2 =
        lerpf(a.velocityLateralDerivativeMps2, b.velocityLateralDerivativeMps2, t);
    out.yawRateDerivativeRadS2 = lerpf(a.yawRateDerivativeRadS2, b.yawRateDerivativeRadS2, t);
    return out;
}

bool physics_state_is_valid(const VehicleSpec *spec, const VehicleState *state,
                            const VehicleDerived *derived)
{
    if (!vehicle_spec_is_valid(spec) || state == NULL || derived == NULL) return false;
#define FINITE_VALUE(v) \
    if (!isfinite(v)) return false
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
    FINITE_VALUE(state->filteredLatAccelMps2);
    FINITE_VALUE(state->prevLatAccelMps2);
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
        FINITE_VALUE(wheel->forceLateralRelaxedN);
        FINITE_VALUE(wheel->forceLongitudinalRelaxedN);
        FINITE_VALUE(wheel->frictionUsage);
        if (wheel->normalLoadN <= 0.0f || wheel->frictionUsage < 0.0f ||
            wheel->frictionUsage > 1.0f + FRICTION_TOLERANCE)
            return false;
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
        const SurfaceSpec *sv = Surface_Get(state->wheels[i].surfaceId);
        const float muScaleVal = derived->tireLoadSensitivityMuScale[i];
        const float longLimitN = spec->tireMuLongScale * sv->muLongitudinal * muScaleVal *
                                 state->wheels[i].normalLoadN;
        const float lateralMuAxle =
            (i <= WHEEL_FRONT_RIGHT) ? spec->tireMuLatFront : spec->tireMuLatRear;
        const float lateralLimitN = lateralMuAxle * (sv->muLateral / SURFACE_REFERENCE_MU_LAT) *
                                    muScaleVal * state->wheels[i].normalLoadN;
        const float nx = state->wheels[i].forceLongitudinalN / longLimitN;
        const float ny = state->wheels[i].forceLateralN / lateralLimitN;
        if (sqrtf(nx * nx + ny * ny) > 1.0f + FRICTION_TOLERANCE) return false;
    }
    FINITE_VALUE(derived->engineTorqueNm);
    FINITE_VALUE(derived->totalGearRatio);
    FINITE_VALUE(derived->drivelineTorqueNm);
    FINITE_VALUE(derived->staticFrontLoadN);
    FINITE_VALUE(derived->staticRearLoadN);
    FINITE_VALUE(derived->aeroVerticalFrontN);
    FINITE_VALUE(derived->aeroVerticalRearN);
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

    /* Phase 4 scaffolding: finite checks on the new derived diagnostics. Range/logic
     * assertions are added when the logic is wired in sub-steps 2-8. */
    FINITE_VALUE(derived->lateralLoadTransferFrontN);
    FINITE_VALUE(derived->lateralLoadTransferRearN);
    FINITE_VALUE(derived->rollMomentNm);
    for (int i = 0; i < WHEEL_COUNT; i++) FINITE_VALUE(derived->tireLoadSensitivityMuScale[i]);
    FINITE_VALUE(derived->differentialOmegaRadS[0]);
    FINITE_VALUE(derived->differentialOmegaRadS[1]);
    FINITE_VALUE(derived->differentialTorqueNm[0]);
    FINITE_VALUE(derived->differentialTorqueNm[1]);
    FINITE_VALUE(derived->looseSurfaceDragBodyN.x);
    FINITE_VALUE(derived->looseSurfaceDragBodyN.y);

#undef FINITE_VALUE

    /* ------------------------------------------------------------- Phase 3 load transfer -- */

    const float weightN = spec->massKg * GRAVITY_MPS2;
    if (derived->staticFrontLoadN <= 0.0f || derived->staticRearLoadN <= 0.0f) return false;
    if (fabsf((derived->staticFrontLoadN + derived->staticRearLoadN) - weightN) > 1.0f) {
        return false;
    }
    /* Transfer only MOVES load: what leaves one axle arrives at the other, so the unclamped
     * pair weighs the car plus whatever the air is pressing down with. The clamped pair may
     * legitimately exceed that once the floor has caught an unloaded axle, which is why the sum
     * is asserted here and not there.
     *
     * The aero pair is NOT recomputed from the state to compare against: the loads were solved
     * from the speed at the START of the step and the state has since been integrated, so a
     * fresh recompute would legitimately differ. What is checkable without that velocity is the
     * direction — a lift coefficient must never load its axle, and a wing must never unload it.
     */
    const float verticalN = weightN + derived->aeroVerticalFrontN + derived->aeroVerticalRearN;
    if (fabsf((derived->unclampedFrontLoadN + derived->unclampedRearLoadN) - verticalN) >
        1.0f) {
        return false;
    }
    if (derived->aeroVerticalFrontN * spec->aeroLiftCoefFront > 0.0f) return false;
    if (derived->aeroVerticalRearN * spec->aeroLiftCoefRear > 0.0f) return false;
    if (fabsf(derived->loadTransferN) > MAX_LOAD_TRANSFER_FRACTION * weightN) return false;
    if (derived->normalLoadFrontN < MIN_NORMAL_LOAD_N - 1e-3f ||
        derived->normalLoadRearN < MIN_NORMAL_LOAD_N - 1e-3f)
        return false;

    /* Lateral load transfer asymmetrizes the per-axle split. The simple sum check is only
     * valid when neither wheel of an axle has hit the MIN_NORMAL_LOAD_N floor. Once the
     * floor breaks load conservation, the sum is expected to deviate from the axle total. */
    {
        const float frontSum = state->wheels[WHEEL_FRONT_LEFT].normalLoadN +
                               state->wheels[WHEEL_FRONT_RIGHT].normalLoadN;
        const bool frontFloored =
            state->wheels[WHEEL_FRONT_LEFT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[WHEEL_FRONT_RIGHT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        if (!frontFloored && fabsf(frontSum - derived->normalLoadFrontN) > 1e-2f) return false;
    }
    {
        const float rearSum = state->wheels[WHEEL_REAR_LEFT].normalLoadN +
                              state->wheels[WHEEL_REAR_RIGHT].normalLoadN;
        const bool rearFloored =
            state->wheels[WHEEL_REAR_LEFT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[WHEEL_REAR_RIGHT].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        if (!rearFloored && fabsf(rearSum - derived->normalLoadRearN) > 1e-2f) return false;
    }

    /* Lateral transfer direction check: when lateral acceleration is significant and neither
     * wheel is on the floor, the outside wheels must carry at least as much load as the
     * inside wheels. Catches sign errors in the lateral-transfer formula. */
    if (spec->lateralLoadTransferEnabled && fabsf(state->filteredLatAccelMps2) > 0.5f) {
        const bool ayPositive = state->filteredLatAccelMps2 > 0.0f;
        const int outerFront = ayPositive ? WHEEL_FRONT_RIGHT : WHEEL_FRONT_LEFT;
        const int innerFront = ayPositive ? WHEEL_FRONT_LEFT : WHEEL_FRONT_RIGHT;
        const int outerRear = ayPositive ? WHEEL_REAR_RIGHT : WHEEL_REAR_LEFT;
        const int innerRear = ayPositive ? WHEEL_REAR_LEFT : WHEEL_REAR_RIGHT;
        const bool frontFloored =
            state->wheels[innerFront].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[outerFront].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        const bool rearFloored =
            state->wheels[innerRear].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f ||
            state->wheels[outerRear].normalLoadN <= MIN_NORMAL_LOAD_N + 1e-3f;
        if (!frontFloored &&
            state->wheels[outerFront].normalLoadN < state->wheels[innerFront].normalLoadN)
            return false;
        if (!rearFloored &&
            state->wheels[outerRear].normalLoadN < state->wheels[innerRear].normalLoadN)
            return false;
    }

    /* ---------------------------------------------------------------- Phase 3 resistance -- */

    /* Resistance removes energy or does nothing; it never adds any. Aero drag is computed
     * from body velocity, so the stored aggregate dotted with body velocity is the right
     * check. Rolling resistance is computed per wheel from contact velocity (which differs
     * from CG velocity under yaw), then optionally clamped by limit_resistance_to_stop
     * against body axes - so aggregate RR . body_velocity is not a valid dissipation check
     * (it fails near-zero CG speed with residual yaw). Validate stored per-wheel magnitudes
     * against a fresh pure-helper recompute instead of checking a freshly generated force
     * against the velocity that produced it. */
    if (derived->aeroDragBodyN.x * state->velocityLongitudinalMps +
            derived->aeroDragBodyN.y * state->velocityLateralMps >
        RESISTANCE_POWER_TOLERANCE_W)
        return false;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (derived->wheelRollingResistanceN[i] < 0.0f) return false;
        float expectedMagnitudeN = 0.0f;
        (void)physics_rolling_resistance_body_n(
            Surface_Get(state->wheels[i].surfaceId)->rollingResistanceCoefficient,
            state->wheels[i].normalLoadN, derived->wheelContactVelocityBodyMps[i],
            &expectedMagnitudeN);
        if (fabsf(derived->wheelRollingResistanceN[i] - expectedMagnitudeN) >
            RESISTANCE_FORCE_TOLERANCE_N) {
            return false;
        }
    }

    if (derived->speedMps >= MAX_SAFE_SPEED_MPS) return false;
    if (fabsf(state->yawRateRadS) >= MAX_SAFE_YAW_RATE_RADS) return false;
    if (derived->lowSpeedBlend < 0.0f || derived->lowSpeedBlend > 1.0f) return false;
    if (fabsf(state->frontRoadWheelAngleRad) > spec->maxRoadWheelAngleRad + 1e-5f) return false;
    if (state->selectedGear < -1 || state->selectedGear > spec->gearCount) return false;
    if (state->engineRpm < spec->engineIdleRpm - 1e-3f ||
        state->engineRpm > spec->engineRedlineRpm + 1e-3f)
        return false;
    if ((DifferentialMode)(int)spec->differentialMode == DIFF_LOCKED) {
        /* Locked means locked on each DRIVEN axle. An undriven axle's wheels legitimately
         * rotate at different speeds (turns, asymmetric surfaces), so requiring lockstep
         * there would reject valid states and roll the step back. */
        const float frontShare = drivetrain_front_torque_share(spec);
        if (frontShare > 0.0f &&
            fabsf(state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS -
                  state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS) > 1e-5f)
            return false;
        if (frontShare < 1.0f &&
            fabsf(state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS -
                  state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS) > 1e-5f)
            return false;
    }
    return true;
}

/* Locked axle, post-integration: one wheel pair, same treatment front and rear. Both wheels
 * take the mean of their independently integrated speeds — the projection that conserves the
 * pair's angular momentum for equal inertias — plus the shared lock flag. One omega per
 * driven axle is the whole point of a locked differential. */
static void locked_axle_equalize(VehicleState *state, WheelId left, WheelId right)
{
    const float meanOmegaRadS = 0.5f * (state->wheels[left].angularVelocityRadS +
                                        state->wheels[right].angularVelocityRadS);
    state->wheels[left].angularVelocityRadS = meanOmegaRadS;
    state->wheels[right].angularVelocityRadS = meanOmegaRadS;
    state->wheels[right].locked = state->wheels[left].locked;
}

/* Locked axle redline clamp: pin both wheels of the pair to the signed magnitude the
 * engine's redline permits through the current gear. */
static void locked_axle_clamp_redline(VehicleState *state, WheelId left, WheelId right,
                                      float maxOmegaRadS)
{
    const float omega = state->wheels[left].angularVelocityRadS;
    const float omegaSign = (omega > 0.0f) ? 1.0f : (omega < 0.0f) ? -1.0f : 0.0f;
    const float limitedOmega = omegaSign * fminf(fabsf(omega), maxOmegaRadS);
    state->wheels[left].angularVelocityRadS = limitedOmega;
    state->wheels[right].angularVelocityRadS = limitedOmega;
}

/* ------------------------------------------------------------------------ solver stages --
 *
 * The step below is the same arithmetic, in the same order, that used to live in one
 * five-hundred-line function. Nothing was reordered: the stage boundaries were cut where the
 * data already stopped flowing backwards, which is why the recorded traces do not move.
 *
 * Each stage documents what it reads and what it may write. Values that cross a boundary live
 * in the PhysicsStep scratch; values that must survive the step live in VehicleState; values
 * that are only reported live in VehicleDerived.
 */

const char *physics_stage_name(PhysicsStage stage)
{
    switch (stage) {
        case PHYSICS_STAGE_NONE: return "none";
        case PHYSICS_STAGE_BEGIN: return "begin";
        case PHYSICS_STAGE_STEERING: return "steering";
        case PHYSICS_STAGE_POWERTRAIN: return "powertrain";
        case PHYSICS_STAGE_WHEEL_KINEMATICS: return "wheel-kinematics";
        case PHYSICS_STAGE_NORMAL_LOADS: return "normal-loads";
        case PHYSICS_STAGE_TIRE_FORCES: return "tire-forces";
        case PHYSICS_STAGE_TIRE_THERMAL: return "tire-thermal";
        case PHYSICS_STAGE_RESISTANCE: return "resistance";
        case PHYSICS_STAGE_ACCUMULATE: return "accumulate";
        case PHYSICS_STAGE_INTEGRATE_BODY: return "integrate-body";
        case PHYSICS_STAGE_INTEGRATE_WHEELS: return "integrate-wheels";
        case PHYSICS_STAGE_DIAGNOSTICS: return "diagnostics";
        default: return "unknown";
    }
}

bool physics_step_init(PhysicsStep *step, const VehicleSpec *spec, VehicleState *state,
                       VehicleDerived *derived, VehicleRenderState *renderState,
                       VehicleTireState *tireState, const ControllerOutput *input, float dt)
{
    if (step == NULL) return false;
    memset(step, 0, sizeof(*step));
    if (!vehicle_spec_is_valid(spec) || state == NULL || derived == NULL ||
        renderState == NULL || input == NULL || !(dt > 0.0f))
        return false;

    step->spec = spec;
    step->state = state;
    step->derived = derived;
    step->renderState = renderState;
    step->tireState = tireState;
    step->input = *input; /* copied: the step's demands cannot change halfway through it */
    step->dt = dt;
    step->completedStage = PHYSICS_STAGE_NONE;
    return true;
}

/*
 * Stage: begin.
 * Reads: the incoming state. Writes: the rollback snapshot and the render history shift.
 */
static void stage_begin(PhysicsStep *step)
{
    step->lastGoodState = *step->state;
    step->lastGoodDerived = *step->derived;
    step->lastGoodRenderState = *step->renderState;

    VehicleRenderState *renderState = step->renderState;
    renderState->prevPositionM = renderState->currPositionM;
    renderState->prevHeadingRad = renderState->currHeadingRad;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        renderState->prevWheelAngleRad[i] = renderState->currWheelAngleRad[i];
    }
}

/*
 * Stage: steering.
 * Reads: steer demand, brake and handbrake demand, differential hardware.
 * Writes: road-wheel angle, per-wheel steer angles, and the wheel speeds that the holds and
 * the locked-axle pre-equalization force.
 *
 * The locked-axle equalization happens HERE rather than with the other differential work
 * because the drivetrain stage below must see one omega per driven axle to split torque from.
 */
static void stage_steering(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    const ControllerOutput *input = &step->input;

    physics_update_steering(spec, state, input->steer, step->dt);

    if (fabsf(state->velocityLongitudinalMps) < 0.1f && input->throttle <= 0.0f) {
        if (input->brake > 0.0f) {
            state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS = 0.0f;
            state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS = 0.0f;
        }
        if (input->brake > 0.0f || input->handbrake > 0.0f) {
            state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = 0.0f;
            state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = 0.0f;
        }
    }

    /* Locked-axle enforcement, pre-integration: each DRIVEN axle is equalized. Undriven wheels
     * stay independent, and OPEN/LSD perform no equalization at all. */
    step->diffMode = (DifferentialMode)(int)spec->differentialMode;
    step->frontShare = drivetrain_front_torque_share(spec);
    if (step->diffMode == DIFF_LOCKED && step->frontShare > 0.0f) {
        const float frontAngularVelocityRadS =
            0.5f * (state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS +
                    state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS);
        state->wheels[WHEEL_FRONT_LEFT].angularVelocityRadS = frontAngularVelocityRadS;
        state->wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS = frontAngularVelocityRadS;
    }
    if (step->diffMode == DIFF_LOCKED && step->frontShare < 1.0f) {
        const float rearAngularVelocityRadS =
            0.5f * (state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS +
                    state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS);
        state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS = rearAngularVelocityRadS;
        state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = rearAngularVelocityRadS;
    }
}

/*
 * Stage: powertrain.
 * Reads: gear, wheel speeds, last step's longitudinal tire forces, pedal demands, and the
 * filtered longitudinal acceleration for the brake bias.
 * Writes: engine speed, the step's DrivetrainTorques, and the drivetrain diagnostics.
 *
 * Tire reaction torque (Fx * R) comes from the PREVIOUS step, which is what an LSD's grip
 * limit needs and is zero on the first step after a reset — conservatively limiting it to
 * preload alone.
 */
static void stage_powertrain(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;
    const ControllerOutput *input = &step->input;

    /* Brake torque follows forward load transfer, never moving rearward from the configured
     * static bias. This keeps the rear axle from locking first when hard braking unloads it. */
    const AxleLoads brakeLoads =
        physics_axle_loads(spec, state->filteredLongAccelMps2, body_speed_mps(state));
    const float rearOmegaLeftRadS = state->wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    const float rearOmegaRightRadS = state->wheels[WHEEL_REAR_RIGHT].angularVelocityRadS;

    float wheelOmegaRadS[WHEEL_COUNT];
    float wheelTireReactionNm[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; i++) {
        wheelOmegaRadS[i] = state->wheels[i].angularVelocityRadS;
        wheelTireReactionNm[i] =
            state->wheels[i].forceLongitudinalN * vehicle_wheel_radius_m(spec, i);
    }

    const float drivenOmegaRadS =
        drivetrain_driven_mean_omega(wheelOmegaRadS, step->frontShare);
    state->engineRpm = drivetrain_engine_rpm(spec, state->selectedGear, drivenOmegaRadS);

    step->torques = drivetrain_calculate_torques(spec, state->selectedGear, wheelOmegaRadS,
                                                 wheelTireReactionNm, input->throttle,
                                                 input->brake, input->handbrake);
    if (input->brake >= 0.60f && brakeLoads.frontN + brakeLoads.rearN > 0.0f) {
        const float loadBias = brakeLoads.frontN / (brakeLoads.frontN + brakeLoads.rearN);
        /* Combined braking and cornering consumes rear lateral capacity too. Keep a small
         * forward reserve above pure longitudinal load sharing so the unloaded rear axle does
         * not lock first when it is already spending friction on cornering. */
        const float targetBias = loadBias + 0.15f;
        const float effectiveBias =
            clampf(fmaxf(spec->brakeBiasFront, targetBias), spec->brakeBiasFront, 0.85f);
        const float frontServiceTotalNm = input->brake * spec->maxBrakeTorqueNm * effectiveBias;
        const float rearServiceTotalNm =
            input->brake * spec->maxBrakeTorqueNm * (1.0f - effectiveBias);
        step->torques.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] = frontServiceTotalNm * 0.5f;
        step->torques.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT] = frontServiceTotalNm * 0.5f;
        step->torques.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] = rearServiceTotalNm * 0.5f;
        step->torques.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT] = rearServiceTotalNm * 0.5f;
    }
    derived->differentialOmegaRadS[0] = rearOmegaLeftRadS;
    derived->differentialOmegaRadS[1] = rearOmegaRightRadS;
    derived->differentialTorqueNm[0] = step->torques.driveTorqueNm[WHEEL_REAR_LEFT];
    derived->differentialTorqueNm[1] = step->torques.driveTorqueNm[WHEEL_REAR_RIGHT];
    derived->engineTorqueNm = step->torques.engineTorqueNm;
    derived->totalGearRatio = step->torques.totalGearRatio;
    derived->drivelineTorqueNm = step->torques.drivelineTorqueNm;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->driveTorqueNm[i] = step->torques.driveTorqueNm[i];
        derived->serviceBrakeTorqueNm[i] = step->torques.serviceBrakeTorqueNm[i];
        derived->handbrakeTorqueNm[i] = step->torques.handbrakeTorqueNm[i];
    }
}

/*
 * Stage: wheel kinematics.
 * Reads: body velocity, yaw rate, wheel positions, steer angles, wheel speeds.
 * Writes: contact-point velocities and each wheel's slip angle and slip ratio.
 *
 * Per-wheel rather than per-axle: an inside rear wheel on a different surface, or turning
 * through a different radius, gets its own slip. The axle aggregates stay for telemetry and
 * for the kinematic low-speed branch.
 */
static void stage_wheel_kinematics(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->wheelContactVelocityBodyMps[i] =
            physics_contact_point_velocity_body(state, state->wheels[i].localPositionM);
    }
    derived->frontAxleContactVelocityBodyMps =
        (Vector2){ state->velocityLongitudinalMps,
                   state->velocityLateralMps + spec->cgToFrontM * state->yawRateRadS };
    derived->rearAxleContactVelocityBodyMps =
        (Vector2){ state->velocityLongitudinalMps,
                   state->velocityLateralMps - spec->cgToRearM * state->yawRateRadS };

    physics_axle_slip_angles(spec, state, &derived->frontSlipAngleRad,
                             &derived->rearSlipAngleRad);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &state->wheels[i];
        const float vxSafe =
            fmaxf(fabsf(derived->wheelContactVelocityBodyMps[i].x), LOW_SPEED_EPSILON_MPS);
        const float vxSign = (state->velocityLongitudinalMps >= 0.0f) ? 1.0f : -1.0f;
        wheel->slipAngleRad = atan2f(derived->wheelContactVelocityBodyMps[i].y, vxSafe) -
                              vxSign * wheel->steerAngleRad;
        wheel->slipRatio =
            tire_slip_ratio(wheel->angularVelocityRadS, vehicle_wheel_radius_m(spec, i),
                            derived->wheelContactVelocityBodyMps[i].x, SLIP_SPEED_EPSILON_MPS,
                            SLIP_RATIO_CLAMP);
    }
}

/*
 * Stage: normal loads.
 * Reads: the PREVIOUS step's solved body accelerations, mass properties, roll stiffness.
 * Writes: the acceleration filters, the axle-load solution, and each wheel's normal load.
 *
 * THIS IS WHERE THE FEEDBACK LOOP IS BROKEN. The load filter consumes the previous step's
 * solved body acceleration. Feeding it this step's value would make loads depend on forces
 * that depend on loads — an algebraic loop with no solution — and a velocity finite difference
 * would smuggle in the r*vy transport term, which pitches nothing.
 */
static void stage_normal_loads(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;

    derived->previousLongAccelMps2 = state->prevLongAccelMps2;
    state->filteredLongAccelMps2 =
        physics_filter_long_accel(state->filteredLongAccelMps2, state->prevLongAccelMps2,
                                  spec->loadFilterRateHz, step->dt);
    derived->filteredLongAccelMps2 = state->filteredLongAccelMps2;

    /* Lateral acceleration filter, symmetric with the longitudinal one, for the lateral load
     * transfer below. Guarded by the master switch so it is a no-op with zero cost until the
     * feature is enabled. */
    if (spec->lateralLoadTransferEnabled) {
        state->filteredLatAccelMps2 =
            physics_filter_long_accel(state->filteredLatAccelMps2, state->prevLatAccelMps2,
                                      spec->loadFilterRateHz, step->dt);
    }

    step->loads = physics_axle_loads(spec, state->filteredLongAccelMps2, body_speed_mps(state));
    const AxleLoads loads = step->loads;

    derived->staticFrontLoadN = loads.staticFrontN;
    derived->staticRearLoadN = loads.staticRearN;
    derived->aeroVerticalFrontN = loads.aeroFrontN;
    derived->aeroVerticalRearN = loads.aeroRearN;
    derived->unclampedFrontLoadN = loads.unclampedFrontN;
    derived->unclampedRearLoadN = loads.unclampedRearN;
    derived->loadTransferN = loads.transferN;
    derived->normalLoadFrontN = loads.frontN;
    derived->normalLoadRearN = loads.rearN;

    /* Lateral load transfer: the roll moment caused by lateral acceleration is distributed
     * between the front and rear axles according to rollStiffnessFrontFraction. The per-axle
     * lateral transfer delta is then layered on top of the already-long-transferred axle load,
     * unloading the inside wheel and loading the outside. */
    const float rollMoment = spec->massKg * state->filteredLatAccelMps2 * spec->cgHeightM;
    float dFzFront = 0.0f, dFzRear = 0.0f;
    if (spec->lateralLoadTransferEnabled) {
        dFzFront = spec->rollStiffnessFrontFraction * rollMoment / spec->trackWidthFrontM;
        dFzRear =
            (1.0f - spec->rollStiffnessFrontFraction) * rollMoment / spec->trackWidthRearM;
    }
    derived->rollMomentNm = rollMoment;
    derived->lateralLoadTransferFrontN = dFzFront;
    derived->lateralLoadTransferRearN = dFzRear;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool front = (i <= WHEEL_FRONT_RIGHT);
        const float axleTotalN = front ? loads.frontN : loads.rearN;
        const float dFzAxle = front ? dFzFront : dFzRear;
        const float py = state->wheels[i].localPositionM.y;
        /* dFzAxle carries the sign of filteredLatAccelMps2. For ay>0 (left turn), the right
         * wheels (py<0) are outside and receive +dFz; the left wheels (py>0) are inside and
         * receive -dFz. The sign of dFzAxle inverts this for right turns automatically, so the
         * single formula covers both directions. */
        float Fz = axleTotalN * 0.5f + (py > 0.0f ? -dFzAxle : dFzAxle);
        Fz = fmaxf(Fz, MIN_NORMAL_LOAD_N);
        state->wheels[i].normalLoadN = Fz;
    }
}

/*
 * Stage: tire forces.
 * Reads: slip angles and ratios, normal loads, surface, tire coefficients.
 * Writes: the pure per-wheel forces, the relaxed lateral force that persists across steps, and
 * the combined-limited forces with their friction usage.
 */
static void stage_tire_forces(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &state->wheels[i];
        const bool front = i <= WHEEL_FRONT_RIGHT;
        const float lateralB = front ? spec->tireBLatFront : spec->tireBLatRear;
        const float lateralC = front ? spec->tireCLatFront : spec->tireCLatRear;
        const float lateralMuAxle = front ? spec->tireMuLatFront : spec->tireMuLatRear;

        /* Tire load sensitivity: higher normal load reduces effective mu. With
         * tireLoadSensitivityK at the default 0, muScale = 1.0 (neutral). */
        const float Fz = wheel->normalLoadN;
        const float muScale =
            (spec->tireLoadSensitivityK > 0.0f)
                ? clampf(powf(Fz / spec->tireLoadRefPerWheelN, -spec->tireLoadSensitivityK),
                         0.5f, 1.5f)
                : 1.0f;
        derived->tireLoadSensitivityMuScale[i] = muScale;

        /* Tire dimension/pressure stiffness modifiers (issue #16). Width scales lateral
         * stiffness — a wider tire has a larger contact patch and stiffer cornering response.
         * Pressure scales both lateral and longitudinal stiffness — higher inflation makes
         * the carcass less compliant. The exponent model is derived from contact-patch
         * mechanics: cornering stiffness ∝ sqrt(patch_area / patch_length) ∝ sqrt(width),
         * and inflation pressure saturates rather than scaling linearly. At nominal width
         * (225 mm) and pressure (220 kPa) both scales are 1.0, reproducing the baseline. */
        const float sectionWidthMm =
            front ? spec->tireSectionWidthFrontMm : spec->tireSectionWidthRearMm;
        /* Live pressure when the thermal model is active (#21); otherwise the authored
         * nominal pressure, which is what keeps the baseline bit-identical. */
        const float nominalPressureKpa =
            front ? spec->tirePressureFrontKpa : spec->tirePressureRearKpa;
        const float tirePressureKpa =
            (step->tireState != NULL && spec->tireThermalEnabled > 0.0f)
                ? step->tireState[i].pressureKpa
                : nominalPressureKpa;
        const float widthScale =
            powf(sectionWidthMm / TIRE_SECTION_WIDTH_MM, TIRE_WIDTH_STIFFNESS_EXP);
        const float pressureScale =
            powf(tirePressureKpa / TIRE_PRESSURE_KPA, TIRE_PRESSURE_STIFFNESS_EXP);

        /* Bulk temperature grip multiplier (#21): Gaussian falloff around the optimal
         * temperature, 1.0 exactly at the optimum and when the thermal model is off. */
        float tempMult = 1.0f;
        if (step->tireState != NULL && spec->tireThermalEnabled > 0.0f) {
            const float d =
                (step->tireState[i].temperatureC - TIRE_OPTIMAL_TEMP_C) / TIRE_TEMP_BAND_C;
            float m = 1.0f - TIRE_TEMP_FALLOFF * d * d;
            if (m < TIRE_TEMP_MIN_MULT) m = TIRE_TEMP_MIN_MULT;
            if (m > TIRE_TEMP_MAX_MULT) m = TIRE_TEMP_MAX_MULT;
            tempMult = m;
        }
        derived->tireTemperatureGripMultiplier[i] = tempMult;

        /* Wear grip degradation (#22): continuous power-law loss to a bounded floor at
         * complete wear; 1.0 when the model is off. */
        float wearMult = 1.0f;
        if (step->tireState != NULL && spec->tireWearEnabled > 0.0f) {
            wearMult = 1.0f - TIRE_WEAR_FULL_DEGRADE *
                                  powf(step->tireState[i].wear, TIRE_WEAR_DEGRADE_EXP);
            if (wearMult < 1.0f - TIRE_WEAR_FULL_DEGRADE)
                wearMult = 1.0f - TIRE_WEAR_FULL_DEGRADE;
        }
        derived->tireWearGripMultiplier[i] = wearMult;

        /* Surface-relative friction and stiffness. On asphalt the surface ratios equal 1.0
         * and tireBScale is 1.0, so this block is a complete no-op for the surface factor.
         * The width/pressure scales are independent of surface and multiply through here. */
        /* TODO(vehicle-audit #3): tire friction multiplies surface friction here, so a low
         * per-car tireMu reads as a permanent surface condition rather than a dry tire.
         * awd_rally's tireMu of 0.55-0.58 is wet-gravel grip baked into the rubber; real dry
         * tire μ ≈ 0.9-1.1 (street) to 1.4-1.7 (semi-slick). Decide whether awd_rally's low grip
         * should come from the track SURFACE (gravel) instead of the tire compound. */

        /* Surface-relative friction and stiffness. On asphalt the surface ratios equal 1.0 and
         * tireBScale is 1.0, so this block is a complete no-op. */
        const SurfaceSpec *s = Surface_Get(wheel->surfaceId);
        const float muLateralEff = lateralMuAxle * (s->muLateral / SURFACE_REFERENCE_MU_LAT) *
                                   muScale * tempMult * wearMult;
        const float muLongitudinalEff =
            spec->tireMuLongScale * s->muLongitudinal * muScale * tempMult * wearMult;
        const float BlatEff = lateralB * widthScale * pressureScale * s->tireBScale;
        const float BlongEff = spec->tireBLong * pressureScale;

        derived->pureLongitudinalForceN[i] = tire_longitudinal_force_n(
            wheel->slipRatio, wheel->normalLoadN, BlongEff, spec->tireCLong, muLongitudinalEff);
        const float fySlipOnly = tire_lateral_force_n(wheel->slipAngleRad, wheel->normalLoadN,
                                                      BlatEff, lateralC, muLateralEff);
        derived->pureLateralForceN[i] = fySlipOnly;

        /* Camber/caster thrust (issue #15). A cambered tire generates lateral force even at
         * zero slip angle because the contact patch deforms asymmetrically. The reduced-order
         * model folds camber into the slip angle as a Pacejka horizontal shift, so the effect
         * naturally saturates near the friction limit (where the tire-curve slope → 0) without
         * needing the combined-slip limiter to intervene separately.
         *
         * Effective camber combines static camber with the camber gain from caster and
         * steering. Positive caster (steering axis tilted backward) causes both front wheels
         * to tilt the same direction when steered: in a left turn their tops move left, so
         * the outside (right) wheel gains negative SAE camber and generates thrust toward
         * the turn centre. Rear wheels do not steer, so caster has no meaningful effect.
         *
         * The side_sign converts between the SAE camber convention (positive = outward lean)
         * and the wheel-frame force direction (+y = left of the wheel heading): the left
         * wheel's outward is +y, the right wheel's outward is -y. */
        const float camberStatic = front ? spec->suspCamberFrontRad : spec->suspCamberRearRad;
        const float casterRad = front ? spec->suspCasterFrontRad : spec->suspCasterRearRad;
        const float sideSign = (wheel->localPositionM.y > 0.0f) ? 1.0f : -1.0f;
        const float gammaCaster = sideSign * sinf(casterRad) * sinf(wheel->steerAngleRad);
        const float gammaEff = camberStatic + gammaCaster;
        derived->effectiveCamberRad[i] = gammaEff;
        /* Convert camber to an equivalent slip-angle shift. At zero slip this produces
         * F_camber = coeff * mu * gamma * Fz: the camber thrust scales with friction because
         * it comes from the same rubber-road interface. This tapers to nothing near saturation
         * because the tire-curve slope → 0 there, and it is naturally weaker on low-grip tires. */
        const float slopeScale = fmaxf(BlatEff * lateralC, 1e-6f);
        const float slipShift = sideSign * CAMBER_THRUST_COEFF * gammaEff / slopeScale;
        if (isfinite(slipShift) && fabsf(slipShift) < 0.5f) {
            derived->pureLateralForceN[i] =
                tire_lateral_force_n(wheel->slipAngleRad - slipShift, wheel->normalLoadN,
                                     BlatEff, lateralC, muLateralEff);
        }
        derived->camberThrustN[i] = derived->pureLateralForceN[i] - fySlipOnly;

        /* Tire relaxation: first-order lag on force buildup. With tireRelaxationLengthM at the
         * default 0, relaxed = steady (bit-identical). Both lateral and longitudinal forces are
         * relaxed before the combined-slip limiter so transients stay inside the friction
         * ellipse (#20). */
        const float fySteady = derived->pureLateralForceN[i];
        float fyRelaxed;
        if (spec->tireRelaxationLengthM > 0.0f) {
            const float vxEff = fmaxf(fabsf(derived->wheelContactVelocityBodyMps[i].x),
                                      RELAXATION_VX_FLOOR_MPS);
            const float coeff =
                clampf(vxEff * step->dt / spec->tireRelaxationLengthM, 0.0f, 1.0f);
            fyRelaxed =
                wheel->forceLateralRelaxedN + (fySteady - wheel->forceLateralRelaxedN) * coeff;
        } else {
            fyRelaxed = fySteady;
        }
        wheel->forceLateralRelaxedN = fyRelaxed;

        /* Longitudinal relaxation: same first-order lag on longitudinal force (#20). */
        const float fxSteady = derived->pureLongitudinalForceN[i];
        float fxRelaxed;
        if (spec->tireLongRelaxationLengthM > 0.0f) {
            const float vxEff = fmaxf(fabsf(derived->wheelContactVelocityBodyMps[i].x),
                                      RELAXATION_VX_FLOOR_MPS);
            const float coeff =
                clampf(vxEff * step->dt / spec->tireLongRelaxationLengthM, 0.0f, 1.0f);
            fxRelaxed = wheel->forceLongitudinalRelaxedN +
                        (fxSteady - wheel->forceLongitudinalRelaxedN) * coeff;
        } else {
            fxRelaxed = fxSteady;
        }
        wheel->forceLongitudinalRelaxedN = fxRelaxed;

        /* Self-aligning moment from pneumatic trail (#20 diagnostic). The trail is maximum at
         * zero slip and decays toward saturation (slip^2 model), producing a restoring moment
         * that opposes slip: Mz = -trail * Fy. Diagnostic only — excluded from the checksum. */
        {
            const float slipMag = fabsf(wheel->slipAngleRad);
            const float trailScale = 1.0f / (1.0f + slipMag * slipMag * 4.0f);
            const float pneumaticTrailM = 0.03f * trailScale;
            derived->aligningMomentNm[i] = -pneumaticTrailM * fySlipOnly;
        }

        tire_apply_combined_limit(fxRelaxed, fyRelaxed, muLongitudinalEff * wheel->normalLoadN,
                                  muLateralEff * wheel->normalLoadN, &wheel->forceLongitudinalN,
                                  &wheel->forceLateralN, &wheel->frictionUsage);
    }
}

/*
 * Stage: tire thermal (issue #21).
 * Reads: this step's tire forces/slips, wheel speeds and loads. Writes: per-wheel bulk
 * temperature and live pressure in step->tireState.
 *
 * Reduced-order bulk model: slip work (|Fx*slipRatio| + |Fy*slipAngle|) and rolling
 * deformation heat the carcass; convection to ambient cools it, with more airflow at speed.
 * Live pressure follows the ideal-gas law linearisation: p = p_nominal * (1 + k*(T - T_ref)).
 * Bounded by hard clamps so long runs cannot diverge or produce negative pressure. The stage
 * is a no-op unless BOTH step->tireState is present AND spec->tireThermalEnabled > 0; the
 * default keeps the model off and the baseline bit-identical.
 */
static void stage_tire_thermal(PhysicsStep *step)
{
    if (step->tireState == NULL) return;
    const VehicleSpec *spec = step->spec;
    const VehicleState *state = step->state;
    const VehicleDerived *derived = step->derived;
    const float dt = step->dt;
    const float speedMps = derived->speedMps;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &state->wheels[i];
        VehicleTireState *ts = &step->tireState[i];
        const bool front = i <= WHEEL_FRONT_RIGHT;
        const float nominalKpa = front ? spec->tirePressureFrontKpa : spec->tirePressureRearKpa;

        /* Heating: slip work (signed slip, work is absolute) plus rolling deformation loss
         * (proportional to load * speed, using the surface's rolling coefficient). */
        const float slipPowerW = fabsf(wheel->forceLongitudinalN * wheel->slipRatio) +
                                 fabsf(wheel->forceLateralN * wheel->slipAngleRad);
        if (spec->tireThermalEnabled > 0.0f) {
            const float rollingPowerW =
                Surface_Get(wheel->surfaceId)->rollingResistanceCoefficient *
                wheel->normalLoadN *
                fmaxf(fabsf(derived->wheelContactVelocityBodyMps[i].x), 0.0f);
            const float heatW =
                slipPowerW * TIRE_HEAT_SLIP_GAIN + rollingPowerW * TIRE_HEAT_ROLLING_GAIN;

            /* Cooling: convection to ambient, stronger with road speed. */
            const float coolingW =
                TIRE_COOLING_RATE_PER_S * (1.0f + TIRE_COOLING_SPEED_FACTOR * speedMps) *
                (ts->temperatureC - TIRE_AMBIENT_TEMP_C) * TIRE_THERMAL_CAPACITY_J_PER_C;

            float tempC =
                ts->temperatureC + (heatW - coolingW) * dt / TIRE_THERMAL_CAPACITY_J_PER_C;
            if (tempC < TIRE_MIN_TEMP_C) tempC = TIRE_MIN_TEMP_C;
            if (tempC > TIRE_MAX_TEMP_C) tempC = TIRE_MAX_TEMP_C;
            ts->temperatureC = tempC;

            /* Live pressure from the bulk temperature; clamped positive. */
            float pressureKpa =
                nominalKpa * (1.0f + TIRE_PRESSURE_TEMP_COEFF * (tempC - TIRE_OPTIMAL_TEMP_C));
            if (pressureKpa < TIRE_MIN_PRESSURE_KPA) pressureKpa = TIRE_MIN_PRESSURE_KPA;
            ts->pressureKpa = pressureKpa;
        }

        /* Tire wear (issue #22): monotonic accumulation from slip energy, boosted while the
         * wheel is locked, with a surface-abrasion factor. Only ever grows inside this stage;
         * the only reset is an explicit service/replace call. */
        if (spec->tireWearEnabled > 0.0f) {
            const float abrasion =
                Surface_Get(wheel->surfaceId)->rollingResistanceCoefficient >= 0.02f ? 1.6f
                                                                                     : 1.0f;
            const float lockMult = wheel->locked ? TIRE_WEAR_LOCKUP_MULT : 1.0f;
            const float wearPerS = slipPowerW * TIRE_WEAR_SLIP_GAIN * lockMult * abrasion;
            float wear = ts->wear + wearPerS * dt;
            if (wear > 1.0f) wear = 1.0f;
            ts->wear = wear;
        }
    }
}

/*
 * Stage: resistance.
 * Reads: body velocity, contact velocities, normal loads, surfaces, and this step's tire
 * forces. Writes: the aero and rolling resistance diagnostics, the summed body resistance in
 * the scratch, and — in the low-speed braking case only — a scale-down of the tire forces.
 *
 * The scale-down exists because a brake solution strong enough to reverse the car within one
 * step is a numeric overshoot, not a physical event.
 */
static void stage_resistance(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;
    const ControllerOutput *input = &step->input;
    const float dt = step->dt;

    Vector2 aeroDragN =
        physics_aero_drag_body_n(spec, state->velocityLongitudinalMps,
                                 state->velocityLateralMps, &derived->aeroDragMagnitudeN);

    Vector2 rollingN = { 0.0f, 0.0f };
    derived->rollingResistanceMagnitudeN = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Vector2 wheelRollingN = physics_rolling_resistance_body_n(
            Surface_Get(state->wheels[i].surfaceId)->rollingResistanceCoefficient,
            state->wheels[i].normalLoadN, derived->wheelContactVelocityBodyMps[i],
            &derived->wheelRollingResistanceN[i]);
        rollingN.x += wheelRollingN.x;
        rollingN.y += wheelRollingN.y;
        derived->rollingResistanceMagnitudeN += derived->wheelRollingResistanceN[i];
    }

    aeroDragN = limit_resistance_to_stop(aeroDragN, spec, state, dt);
    rollingN = limit_resistance_to_stop(rollingN, spec, state, dt);
    derived->aeroDragBodyN = aeroDragN;
    derived->rollingResistanceBodyN = rollingN;

    step->resistanceBodyN = (Vector2){ aeroDragN.x + rollingN.x, aeroDragN.y + rollingN.y };
    step->longitudinalResistanceN = step->resistanceBodyN.x;

    if ((input->brake > 0.0f || input->handbrake > 0.0f) &&
        fabsf(state->velocityLongitudinalMps) < LOW_SPEED_BEGIN_MPS &&
        fabsf(state->velocityLongitudinalMps) > 0.0f) {
        const float frontFxBodyN = cosf(state->frontRoadWheelAngleRad) *
                                   (state->wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                                    state->wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN);
        const float rearFxBodyN = state->wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                                  state->wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
        const float wheelFxBodyN = frontFxBodyN + rearFxBodyN;
        const float currentNetN = wheelFxBodyN + step->longitudinalResistanceN;
        const float stopNetN =
            -copysignf(spec->massKg * fabsf(state->velocityLongitudinalMps) / dt,
                       state->velocityLongitudinalMps);
        if (currentNetN * state->velocityLongitudinalMps < 0.0f &&
            fabsf(currentNetN) > fabsf(stopNetN) && fabsf(wheelFxBodyN) > 1e-6f) {
            const float targetWheelFxBodyN = stopNetN - step->longitudinalResistanceN;
            const float scale = clampf(targetWheelFxBodyN / wheelFxBodyN, 0.0f, 1.0f);
            for (int i = 0; i < WHEEL_COUNT; i++) {
                WheelState *wheel = &state->wheels[i];
                wheel->forceLongitudinalN *= scale;
                const SurfaceSpec *sv2 = Surface_Get(wheel->surfaceId);
                const float lateralMuAxle2 =
                    (i <= WHEEL_FRONT_RIGHT) ? spec->tireMuLatFront : spec->tireMuLatRear;
                const float nx = wheel->forceLongitudinalN /
                                 (spec->tireMuLongScale * sv2->muLongitudinal *
                                  derived->tireLoadSensitivityMuScale[i] * wheel->normalLoadN);
                const float ny = wheel->forceLateralN /
                                 (lateralMuAxle2 * (sv2->muLateral / SURFACE_REFERENCE_MU_LAT) *
                                  derived->tireLoadSensitivityMuScale[i] * wheel->normalLoadN);
                wheel->frictionUsage = fminf(sqrtf(nx * nx + ny * ny), 1.0f);
            }
        }
    }
}

/*
 * Stage: accumulate.
 * Reads: the limited per-wheel forces, steer angles, wheel positions, and the step's
 * resistance. Writes: axle force sums, the body-frame total, and the yaw moment.
 */
static void stage_accumulate(PhysicsStep *step)
{
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;

    derived->frontLateralForceN = 0.0f;
    derived->rearLateralForceN = 0.0f;
    derived->frontBodyForceN = (Vector2){ 0.0f, 0.0f };
    derived->rearBodyForceN = (Vector2){ 0.0f, 0.0f };
    derived->totalYawTorqueNm = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool front = (i <= WHEEL_FRONT_RIGHT);
        /* Wheel-frame lateral force sum (for telemetry; before rotation). */
        if (front)
            derived->frontLateralForceN += state->wheels[i].forceLateralN;
        else
            derived->rearLateralForceN += state->wheels[i].forceLateralN;

        /* Rotate to body frame by each wheel's own effective heading. The rear pair used to be
         * copied through unrotated on the grounds that it never steers; with static toe active
         * it has a heading of its own, and skipping the rotation would silently discard the
         * drag component that rear toe exists to produce. At zero rear toe the rotation is the
         * identity, so this is not a change for a car with no rear alignment. */
        const Vector2 bodyForceN = physics_rotate_wheel_force_to_body(
            (Vector2){ state->wheels[i].forceLongitudinalN, state->wheels[i].forceLateralN },
            state->wheels[i].steerAngleRad);

        if (front) {
            derived->frontBodyForceN.x += bodyForceN.x;
            derived->frontBodyForceN.y += bodyForceN.y;
        } else {
            derived->rearBodyForceN.x += bodyForceN.x;
            derived->rearBodyForceN.y += bodyForceN.y;
        }

        /* Four-wheel yaw torque: M_z = sum (px.Fy_body - py.Fx_body). The px.Fy_body term
         * reproduces the axle yaw moment (front: +l_f.Fy, rear: -l_r.Fy); the -py.Fx_body term
         * captures the longitudinal force's lever arm through the track width, which the
         * centreline bicycle model omits. */
        derived->totalYawTorqueNm += state->wheels[i].localPositionM.x * bodyForceN.y -
                                     state->wheels[i].localPositionM.y * bodyForceN.x;
    }

    derived->totalBodyForceN = (Vector2){
        derived->frontBodyForceN.x + derived->rearBodyForceN.x + step->resistanceBodyN.x,
        derived->frontBodyForceN.y + derived->rearBodyForceN.y + step->resistanceBodyN.y
    };
}

/*
 * Stage: integrate body.
 * Reads: the total body force and yaw moment, plus the wheel forces the kinematic branch uses.
 * Writes: body velocities, yaw rate, heading, position, the low-speed blend, and the solved
 * derivatives in the scratch.
 */
static void stage_integrate_body(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;
    const ControllerOutput *input = &step->input;
    const float dt = step->dt;

    const VehicleDerivatives dynamic =
        dynamic_derivatives(spec, state, derived->totalBodyForceN, derived->totalYawTorqueNm);
    /* At kinematic speeds, steering geometry owns lateral/yaw motion. Longitudinal
     * acceleration still comes only from limited tire Fx plus the two resistance forces, so
     * steering a stationary car cannot create propulsion from a rotated lateral force. */
    const float kinematicLongitudinalForceN =
        cosf(state->frontRoadWheelAngleRad) *
            (state->wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
             state->wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN) +
        state->wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
        state->wheels[WHEEL_REAR_RIGHT].forceLongitudinalN + step->longitudinalResistanceN;
    const VehicleDerivatives kinematic =
        kinematic_derivatives(spec, state, kinematicLongitudinalForceN);
    derived->lowSpeedBlend = physics_low_speed_blend(state->velocityLongitudinalMps);
    step->solved = derivatives_lerp(kinematic, dynamic, derived->lowSpeedBlend);
    const VehicleDerivatives solved = step->solved;

    state->velocityLongitudinalMps += solved.velocityLongitudinalDerivativeMps2 * dt;
    state->velocityLateralMps += solved.velocityLateralDerivativeMps2 * dt;
    state->yawRateRadS += solved.yawRateDerivativeRadS2 * dt;
    state->headingRad = wrap_angle(state->headingRad + state->yawRateRadS * dt);

    const float c = cosf(state->headingRad);
    const float s = sinf(state->headingRad);
    const float worldVxMps = state->velocityLongitudinalMps * c - state->velocityLateralMps * s;
    const float worldVyMps = state->velocityLongitudinalMps * s + state->velocityLateralMps * c;
    state->positionM.x += worldVxMps * dt;
    state->positionM.y += worldVyMps * dt;

    /* Braking-reversal guard.
     *
     * Braking cannot propel the car, only slow it. With the +35% longitudinal grip of the
     * Phase 4 surface correction (asphalt muLongitudinal == 1.35), the tire reaction torque on
     * locked wheels can briefly exceed the brake-torque capacity, causing the wheel integrator
     * to un-lock the wheel to rolling speed. That eliminates braking force from the tire; when
     * the wheel lands fractionally above rolling speed on the following tick, the residual
     * positive slip produces a small propelling impulse.
     *
     * The guard is a physical invariant, not a hack: braking torque can only oppose the
     * direction of travel, never reverse it, and never increase the speed in the direction of
     * travel. At the zero boundary it brings the body to rest rather than crossing through. */
    if ((input->brake > 0.0f || input->handbrake > 0.0f) && input->throttle <= 0.0f) {
        /* Reconstruct the velocity before integration so we can detect sign flips and speed
         * increases caused by this step's force solution. */
        const float vxPrev =
            state->velocityLongitudinalMps - solved.velocityLongitudinalDerivativeMps2 * dt;
        /* A braking-only input may not increase speed in the direction of travel. */
        if (vxPrev > 0.0f && state->velocityLongitudinalMps > vxPrev) {
            state->velocityLongitudinalMps = vxPrev;
        } else if (vxPrev < 0.0f && state->velocityLongitudinalMps < vxPrev) {
            state->velocityLongitudinalMps = vxPrev;
        }
        /* Braking can never push the body past rest: the wheels stop turning, lock, and the
         * body coasts to a stop via residual resistance. A sign reversal is a numeric
         * overshoot, not a physical event. */
        if (vxPrev * state->velocityLongitudinalMps < 0.0f) {
            state->velocityLongitudinalMps = 0.0f;
        }
    }
}

/*
 * Stage: integrate wheels.
 * Reads: the step's torques, this step's tire forces, and the freshly integrated body speed.
 * Writes: wheel speeds, lock flags, the locked-axle post-conditions, and the engine speed the
 * next step's gearing will see.
 *
 * Body integration precedes wheel integration deliberately, so the brake/free-roll crossing
 * guard uses the updated contact speed rather than lagging one fixed tick behind.
 */
static void stage_integrate_wheels(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    /* Const on purpose: this stage reads the load-sensitivity scale the tire stage produced and
     * writes no diagnostics of its own. */
    const VehicleDerived *derived = step->derived;
    const DrivetrainTorques *torques = &step->torques;
    const float dt = step->dt;

    /* Constant across the four wheels: the slip at which the longitudinal curve peaks. */
    const float peakSlip = tanf(CIRCUIT_PI / (2.0f * spec->tireCLong)) / spec->tireBLong;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const float wheelVxMps = state->velocityLongitudinalMps;
        const float previousOmegaRadS = state->wheels[i].angularVelocityRadS;
        const float radiusM = vehicle_wheel_radius_m(spec, i);
        float nextOmegaRadS = drivetrain_integrate_wheel(
            previousOmegaRadS, wheelVxMps, torques->driveTorqueNm[i],
            torques->serviceBrakeTorqueNm[i], torques->handbrakeTorqueNm[i],
            state->wheels[i].forceLongitudinalN, radiusM, spec->wheelInertiaKgM2, dt,
            &state->wheels[i].locked);

        /* Stiff-limit equilibrium treatment for the unbraked wheel.
         *
         * The wheel-plus-tire subsystem's time constant is well under a millisecond - far
         * below the fixed step - so under any steady sub-peak torque the physical wheel reaches
         * its balance slip within the tick. The explicit integrator cannot: it either
         * overshoots the balance point (a tick-scale limit cycle between engine braking and
         * tire reaction) or, when the start-of-step slip was zero, produces no torque at all
         * and freezes the wheel while the body moves beneath it, which returns as a phantom
         * multi-kilonewton force one tick later.
         *
         * So when a stable equilibrium exists and the wheel is inside the pre-peak band around
         * it - the region where the linearized dynamics converge within one tick - land on the
         * equilibrium directly. Outside the band (deep wheelspin shedding speed through a
         * saturated curve) the multi-tick explicit dynamics are the right answer and are kept.
         * Torques at or beyond the curve's peak have no equilibrium and always stay explicit;
         * braking keeps its own lock/release dynamics. The pure-curve balance ignores ellipse
         * scaling, so under heavy combined slip the landing is slightly conservative; that
         * error is bounded by the ellipse, while the artifacts this removes are an order of
         * magnitude larger. */
        if (torques->serviceBrakeTorqueNm[i] <= 0.0f && torques->handbrakeTorqueNm[i] <= 0.0f) {
            float equilibriumOmegaRadS;
            if (drivetrain_wheel_equilibrium_omega(
                    torques->driveTorqueNm[i], wheelVxMps, radiusM,
                    spec->tireMuLongScale *
                        Surface_Get(state->wheels[i].surfaceId)->muLongitudinal *
                        derived->tireLoadSensitivityMuScale[i] * state->wheels[i].normalLoadN,
                    spec->tireBLong, spec->tireCLong, SLIP_SPEED_EPSILON_MPS,
                    &equilibriumOmegaRadS)) {
                const float vxSafeMps = fmaxf(fabsf(wheelVxMps), SLIP_SPEED_EPSILON_MPS);
                const float bandRadS = peakSlip * vxSafeMps / radiusM;
                const bool crossed = (previousOmegaRadS - equilibriumOmegaRadS) *
                                         (nextOmegaRadS - equilibriumOmegaRadS) <
                                     0.0f;
                if (crossed || fabsf(previousOmegaRadS - equilibriumOmegaRadS) < bandRadS) {
                    nextOmegaRadS = equilibriumOmegaRadS;
                    state->wheels[i].locked = false;
                }
            }
        }
        state->wheels[i].angularVelocityRadS = nextOmegaRadS;
    }

    /* Locked-axle enforcement: only for LOCKED mode, applied to each DRIVEN axle. OPEN and LSD
     * let wheels rotate independently, so no equalization is performed. */
    if (step->diffMode == DIFF_LOCKED && step->frontShare > 0.0f)
        locked_axle_equalize(state, WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT);
    if (step->diffMode == DIFF_LOCKED && step->frontShare < 1.0f)
        locked_axle_equalize(state, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT);
    if (step->diffMode == DIFF_LOCKED && torques->totalGearRatio != 0.0f) {
        const float redlineWheelOmegaRadS =
            spec->engineRedlineRpm * CIRCUIT_TWO_PI / (60.0f * fabsf(torques->totalGearRatio));
        if (step->frontShare > 0.0f)
            locked_axle_clamp_redline(state, WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT,
                                      redlineWheelOmegaRadS);
        if (step->frontShare < 1.0f)
            locked_axle_clamp_redline(state, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT,
                                      redlineWheelOmegaRadS);
    }
    {
        float postOmega[WHEEL_COUNT];
        for (int i = 0; i < WHEEL_COUNT; i++)
            postOmega[i] = state->wheels[i].angularVelocityRadS;
        state->engineRpm =
            drivetrain_engine_rpm(spec, state->selectedGear,
                                  drivetrain_driven_mean_omega(postOmega, step->frontShare));
    }
}

/*
 * Stage: diagnostics.
 * Reads: the solved forces and the integrated state. Writes: reported accelerations, the
 * accelerations the NEXT step's load filter consumes, speed, sideslip, friction usage, and the
 * render pose.
 *
 * The two prevAccel values are the only thing here a later step reads; everything else is a
 * report.
 */
static void stage_diagnostics(PhysicsStep *step)
{
    const VehicleSpec *spec = step->spec;
    VehicleState *state = step->state;
    VehicleDerived *derived = step->derived;
    VehicleRenderState *renderState = step->renderState;

    /* Body acceleration is force divided by mass. Do not reconstruct it from the integrated
     * derivative: dvx/dt also contains the rotating-frame transport term r*vy, and using the
     * post-integration state would add a small same-step error. The low-speed blend changes
     * state derivatives, not the force solution used by load transfer and telemetry. */
    derived->longitudinalAccelerationMps2 = derived->totalBodyForceN.x / spec->massKg;
    /* The kinematic low-speed model deliberately suppresses tire-model lateral force at rest,
     * so its reported lateral acceleration follows the solved blended derivative. */
    derived->lateralAccelerationMps2 = step->solved.velocityLateralDerivativeMps2 +
                                       state->yawRateRadS * state->velocityLongitudinalMps;

    /* ax_body, not dvx_dt: load transfer responds to the longitudinal force on the body, and
     * dvx_dt carries the rotational transport term r*vy, which pitches nothing. */
    derived->solvedLongAccelMps2 = derived->longitudinalAccelerationMps2;
    state->prevLongAccelMps2 = derived->longitudinalAccelerationMps2;
    state->prevLatAccelMps2 = derived->totalBodyForceN.y / spec->massKg;
    derived->speedMps = sqrtf(state->velocityLongitudinalMps * state->velocityLongitudinalMps +
                              state->velocityLateralMps * state->velocityLateralMps);
    derived->bodySideslipRad =
        (derived->speedMps < LOW_SPEED_EPSILON_MPS)
            ? 0.0f
            : atan2f(state->velocityLateralMps,
                     fmaxf(fabsf(state->velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS));
    derived->maxFrictionUsage = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        derived->maxFrictionUsage =
            fmaxf(derived->maxFrictionUsage, state->wheels[i].frictionUsage);
    }
    derived->physicallySliding = derived->maxFrictionUsage >= 0.98f;

    renderState->currPositionM = state->positionM;
    renderState->currHeadingRad = state->headingRad;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        renderState->currWheelAngleRad[i] = state->wheels[i].steerAngleRad;
    }
}

void physics_step_run(PhysicsStep *step, PhysicsStage throughStage)
{
    if (step == NULL || step->spec == NULL) return;
    if (throughStage >= PHYSICS_STAGE_COUNT) throughStage = PHYSICS_STAGE_COUNT - 1;

    /* A plain ordered walk. No stage is skipped, no stage runs twice, and the sequence does
     * not depend on the vehicle's state — which is what makes the operation count of a step
     * identical for every state the solver can be in. */
    for (PhysicsStage s = (PhysicsStage)(step->completedStage + 1); s <= throughStage;
         s = (PhysicsStage)(s + 1)) {
        switch (s) {
            case PHYSICS_STAGE_BEGIN: stage_begin(step); break;
            case PHYSICS_STAGE_STEERING: stage_steering(step); break;
            case PHYSICS_STAGE_POWERTRAIN: stage_powertrain(step); break;
            case PHYSICS_STAGE_WHEEL_KINEMATICS: stage_wheel_kinematics(step); break;
            case PHYSICS_STAGE_NORMAL_LOADS: stage_normal_loads(step); break;
            case PHYSICS_STAGE_TIRE_FORCES: stage_tire_forces(step); break;
            case PHYSICS_STAGE_TIRE_THERMAL: stage_tire_thermal(step); break;
            case PHYSICS_STAGE_RESISTANCE: stage_resistance(step); break;
            case PHYSICS_STAGE_ACCUMULATE: stage_accumulate(step); break;
            case PHYSICS_STAGE_INTEGRATE_BODY: stage_integrate_body(step); break;
            case PHYSICS_STAGE_INTEGRATE_WHEELS: stage_integrate_wheels(step); break;
            case PHYSICS_STAGE_DIAGNOSTICS: stage_diagnostics(step); break;
            default: break;
        }
        step->completedStage = s;
    }
}

/*
 * Is every value the solver integrates still a number?
 *
 * This is deliberately NOT physics_state_is_valid(). That predicate is a whole-STEP invariant:
 * besides finiteness it cross-checks values that different stages produce against each other —
 * rolling resistance against contact velocity, engine speed against gearing — and those are
 * inconsistent halfway through a step by construction. Asking it after each stage would report
 * a failure in wheel kinematics for every healthy moving car, because the resistance stage that
 * would make its own numbers agree has not run yet.
 *
 * Attribution needs the narrower question, and this is it.
 */
static bool state_is_finite(const VehicleState *state)
{
    if (!isfinite(state->positionM.x) || !isfinite(state->positionM.y) ||
        !isfinite(state->headingRad) || !isfinite(state->velocityLongitudinalMps) ||
        !isfinite(state->velocityLateralMps) || !isfinite(state->yawRateRadS) ||
        !isfinite(state->frontRoadWheelAngleRad) || !isfinite(state->engineRpm) ||
        !isfinite(state->filteredLongAccelMps2) || !isfinite(state->prevLongAccelMps2) ||
        !isfinite(state->filteredLatAccelMps2) || !isfinite(state->prevLatAccelMps2))
        return false;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &state->wheels[i];
        if (!isfinite(wheel->steerAngleRad) || !isfinite(wheel->angularVelocityRadS) ||
            !isfinite(wheel->normalLoadN) || !isfinite(wheel->slipAngleRad) ||
            !isfinite(wheel->slipRatio) || !isfinite(wheel->forceLongitudinalN) ||
            !isfinite(wheel->forceLateralRelaxedN) ||
            !isfinite(wheel->forceLongitudinalRelaxedN) || !isfinite(wheel->frictionUsage))
            return false;
    }
    return true;
}

/*
 * Which stage first drove the vehicle state non-finite.
 *
 * Only ever called after a step has already failed, so its cost never lands on a healthy
 * frame. It replays the same step from the snapshot one stage at a time; the first stage after
 * which the state stops being finite is the one that broke it. The replay writes to its own
 * copies, so the caller's rollback is unaffected.
 *
 * Returns PHYSICS_STAGE_NONE when the replay stays finite throughout — which means the step
 * failed one of physics_state_is_valid()'s consistency or bounds checks rather than by
 * producing a non-number, and no single stage owns that.
 */
static PhysicsStage diagnose_failing_stage(const PhysicsStep *failed)
{
    PhysicsStep probe;
    VehicleState state = failed->lastGoodState;
    VehicleDerived derived = failed->lastGoodDerived;
    VehicleRenderState renderState = failed->lastGoodRenderState;
    if (!physics_step_init(&probe, failed->spec, &state, &derived, &renderState,
                           failed->tireState, &failed->input, failed->dt))
        return PHYSICS_STAGE_NONE;

    for (PhysicsStage s = PHYSICS_STAGE_BEGIN; s < PHYSICS_STAGE_COUNT;
         s = (PhysicsStage)(s + 1)) {
        physics_step_run(&probe, s);
        if (!state_is_finite(&state)) return s;
    }
    return PHYSICS_STAGE_NONE;
}

void physics_fixed_update(const VehicleSpec *spec, VehicleState *state, VehicleDerived *derived,
                          VehicleRenderState *renderState, VehicleTireState *tireState,
                          const ControllerOutput *input, float dt)
{
    PhysicsStep step;
    if (!physics_step_init(&step, spec, state, derived, renderState, tireState, input, dt))
        return;

    physics_step_run(&step, PHYSICS_STAGE_COUNT - 1);

    const bool valid = physics_state_is_valid(spec, state, derived);
    const PhysicsStage failedStage = valid ? PHYSICS_STAGE_NONE : diagnose_failing_stage(&step);
    assert(valid);
    if (!valid) {
        *state = step.lastGoodState;
        *derived = step.lastGoodDerived;
        *renderState = step.lastGoodRenderState;
    }
    /* Written after the rollback, which would otherwise restore the snapshot's own report. */
    derived->solverFailedStage = (int)failedStage;
}

#include "vehicle.h"

#include <math.h>
#include <string.h>

static void set_wheel_positions(const VehicleSpec *spec, VehicleState *state)
{
    const float halfFrontTrackM = 0.5f * spec->trackWidthFrontM;
    const float halfRearTrackM = 0.5f * spec->trackWidthRearM;

    state->wheels[WHEEL_FRONT_LEFT].localPositionM =
        (Vector2){ spec->cgToFrontM, halfFrontTrackM };
    state->wheels[WHEEL_FRONT_RIGHT].localPositionM =
        (Vector2){ spec->cgToFrontM, -halfFrontTrackM };
    state->wheels[WHEEL_REAR_LEFT].localPositionM =
        (Vector2){ -spec->cgToRearM, halfRearTrackM };
    state->wheels[WHEEL_REAR_RIGHT].localPositionM =
        (Vector2){ -spec->cgToRearM, -halfRearTrackM };
}

void vehicle_spec_set_default(VehicleSpec *spec)
{
    if (spec == NULL) return;
    memset(spec, 0, sizeof(*spec));

    spec->massKg = VEH_MASS_KG;
    spec->yawInertiaKgM2 = VEH_YAW_INERTIA_KGM2;
    spec->cgToFrontM = VEH_CG_TO_FRONT_M;
    spec->cgToRearM = VEH_CG_TO_REAR_M;
    spec->wheelbaseM = VEH_CG_TO_FRONT_M + VEH_CG_TO_REAR_M;
    spec->cgHeightM = VEH_CG_HEIGHT_M;
    spec->trackWidthFrontM = VEH_TRACK_FRONT_M;
    spec->trackWidthRearM = VEH_TRACK_REAR_M;
    spec->wheelRadiusM = WHEEL_RADIUS_M;
    spec->wheelInertiaKgM2 = WHEEL_INERTIA_KGM2;
    spec->maxRoadWheelAngleRad = STEER_MAX_RAD;
    spec->maxSteerRateRadS = STEER_RATE_RAD_S;
    spec->steerReturnRateRadS = STEER_RETURN_RATE_RAD_S;
    spec->dragCoefficient = DRAG_COEFFICIENT;
    spec->frontalAreaM2 = FRONTAL_AREA_M2;
    spec->rollingResistanceCoefficient = ROLLING_RESISTANCE_COEF;
    spec->loadFilterRateHz = LOAD_FILTER_RATE_HZ;
    spec->tireBLatFront = TIRE_B_LAT_FRONT;
    spec->tireCLatFront = TIRE_C_LAT_FRONT;
    spec->tireMuLatFront = TIRE_MU_LAT_FRONT;
    spec->tireBLatRear = TIRE_B_LAT_REAR;
    spec->tireCLatRear = TIRE_C_LAT_REAR;
    spec->tireMuLatRear = TIRE_MU_LAT_REAR;
    spec->tireBLong = TIRE_B_LONG;
    spec->tireCLong = TIRE_C_LONG;
    spec->tireMuLongScale = TIRE_MU_LONG_SCALE;
    {
        const float ratios[MAX_GEARS] = GEAR_RATIOS;
        memcpy(spec->gearRatios, ratios, sizeof(ratios));
    }
    spec->gearCount = GEAR_COUNT;
    spec->reverseGearRatio = REVERSE_GEAR_RATIO;
    spec->finalDriveRatio = FINAL_DRIVE_RATIO;
    spec->drivetrainEfficiency = DRIVETRAIN_EFFICIENCY;
    spec->engineIdleRpm = ENGINE_IDLE_RPM;
    spec->engineRedlineRpm = ENGINE_REDLINE_RPM;
    {
        const float torqueCurve[ENGINE_CURVE_POINTS] = ENGINE_TORQUE_CURVE_NM;
        memcpy(spec->engineTorqueCurveNm, torqueCurve, sizeof(torqueCurve));
    }
    spec->engineBrakingTorqueNm = ENGINE_BRAKING_TORQUE_NM;
    spec->maxBrakeTorqueNm = MAX_BRAKE_TORQUE_NM;
    spec->brakeBiasFront = BRAKE_BIAS_FRONT;
    spec->handbrakeTorqueNm = HANDBRAKE_TORQUE_NM;
}

bool vehicle_spec_is_valid(const VehicleSpec *spec)
{
    if (spec == NULL) return false;
    if (!(isfinite(spec->massKg) && spec->massKg > 0.0f)) return false;
    if (!(isfinite(spec->yawInertiaKgM2) && spec->yawInertiaKgM2 > 0.0f)) return false;
    if (!(isfinite(spec->cgToFrontM) && spec->cgToFrontM > 0.0f)) return false;
    if (!(isfinite(spec->cgToRearM) && spec->cgToRearM > 0.0f)) return false;
    if (!(isfinite(spec->wheelbaseM) &&
          fabsf(spec->wheelbaseM - (spec->cgToFrontM + spec->cgToRearM)) < 1e-5f)) return false;
    if (!(isfinite(spec->trackWidthFrontM) && spec->trackWidthFrontM > 0.0f)) return false;
    if (!(isfinite(spec->trackWidthRearM) && spec->trackWidthRearM > 0.0f)) return false;
    if (!(isfinite(spec->cgHeightM) && spec->cgHeightM > 0.0f)) return false;
    if (!(isfinite(spec->wheelRadiusM) && spec->wheelRadiusM > 0.0f)) return false;
    if (!(isfinite(spec->wheelInertiaKgM2) && spec->wheelInertiaKgM2 > 0.0f)) return false;
    if (!(isfinite(spec->dragCoefficient) && spec->dragCoefficient >= 0.0f)) return false;
    if (!(isfinite(spec->frontalAreaM2) && spec->frontalAreaM2 >= 0.0f)) return false;
    if (!(isfinite(spec->rollingResistanceCoefficient) &&
          spec->rollingResistanceCoefficient >= 0.0f)) return false;
    if (!(isfinite(spec->loadFilterRateHz) && spec->loadFilterRateHz > 0.0f)) return false;
    if (!(isfinite(spec->tireBLatFront) && spec->tireBLatFront > 0.0f &&
          isfinite(spec->tireCLatFront) && spec->tireCLatFront > 0.0f &&
          isfinite(spec->tireMuLatFront) && spec->tireMuLatFront > 0.0f &&
          isfinite(spec->tireBLatRear) && spec->tireBLatRear > 0.0f &&
          isfinite(spec->tireCLatRear) && spec->tireCLatRear > 0.0f &&
          isfinite(spec->tireMuLatRear) && spec->tireMuLatRear > 0.0f &&
          isfinite(spec->tireBLong) && spec->tireBLong > 0.0f &&
          isfinite(spec->tireCLong) && spec->tireCLong > 0.0f &&
          isfinite(spec->tireMuLongScale) && spec->tireMuLongScale > 0.0f)) return false;
    if (spec->gearCount <= 0 || spec->gearCount > MAX_GEARS) return false;
    for (int i = 0; i < spec->gearCount; i++) {
        if (!(isfinite(spec->gearRatios[i]) && spec->gearRatios[i] > 0.0f)) return false;
    }
    if (!(isfinite(spec->reverseGearRatio) && spec->reverseGearRatio > 0.0f)) return false;
    if (!(isfinite(spec->finalDriveRatio) && spec->finalDriveRatio > 0.0f)) return false;
    if (!(isfinite(spec->drivetrainEfficiency) &&
          spec->drivetrainEfficiency >= 0.0f && spec->drivetrainEfficiency <= 1.0f)) return false;
    if (!(isfinite(spec->engineIdleRpm) && isfinite(spec->engineRedlineRpm) &&
          spec->engineIdleRpm > 0.0f && spec->engineIdleRpm < spec->engineRedlineRpm)) return false;
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++) {
        if (!(isfinite(spec->engineTorqueCurveNm[i]) &&
              spec->engineTorqueCurveNm[i] >= 0.0f)) return false;
    }
    if (!(isfinite(spec->engineBrakingTorqueNm) &&
          spec->engineBrakingTorqueNm >= 0.0f)) return false;
    if (!(isfinite(spec->maxBrakeTorqueNm) && spec->maxBrakeTorqueNm >= 0.0f)) return false;
    if (!(isfinite(spec->brakeBiasFront) &&
          spec->brakeBiasFront >= 0.0f && spec->brakeBiasFront <= 1.0f)) return false;
    if (!(isfinite(spec->handbrakeTorqueNm) && spec->handbrakeTorqueNm >= 0.0f)) return false;
    return true;
}

void vehicle_state_reset(const VehicleSpec *spec,
                         VehicleState *state,
                         VehicleDerived *derived,
                         VehicleRenderState *renderState)
{
    if (spec == NULL || state == NULL || derived == NULL || renderState == NULL) return;
    memset(state, 0, sizeof(*state));
    memset(derived, 0, sizeof(*derived));
    memset(renderState, 0, sizeof(*renderState));

    state->selectedGear = 1;
    state->engineRpm = spec->engineIdleRpm;
    set_wheel_positions(spec, state);

    const float frontLoadN = spec->massKg * GRAVITY_MPS2 * spec->cgToRearM / spec->wheelbaseM;
    const float rearLoadN = spec->massKg * GRAVITY_MPS2 * spec->cgToFrontM / spec->wheelbaseM;
    state->wheels[WHEEL_FRONT_LEFT].normalLoadN = frontLoadN * 0.5f;
    state->wheels[WHEEL_FRONT_RIGHT].normalLoadN = frontLoadN * 0.5f;
    state->wheels[WHEEL_REAR_LEFT].normalLoadN = rearLoadN * 0.5f;
    state->wheels[WHEEL_REAR_RIGHT].normalLoadN = rearLoadN * 0.5f;
    for (int i = 0; i < WHEEL_COUNT; i++) state->wheels[i].surfaceId = SURFACE_ASPHALT;

    /* memset already zeroed prevLongAccelMps2 and filteredLongAccelMps2, which is what a
     * reset means for the load filter: no history, so the first step sees the static split. */
    derived->normalLoadFrontN = frontLoadN;
    derived->normalLoadRearN = rearLoadN;
    derived->staticFrontLoadN = frontLoadN;
    derived->staticRearLoadN = rearLoadN;
    derived->unclampedFrontLoadN = frontLoadN;
    derived->unclampedRearLoadN = rearLoadN;

    renderState->prevPositionM = state->positionM;
    renderState->currPositionM = state->positionM;
    renderState->prevHeadingRad = state->headingRad;
    renderState->currHeadingRad = state->headingRad;
}

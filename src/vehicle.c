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

static float tire_unloaded_radius_m(float sectionWidthMm, float aspectPct, float rimDiameterIn)
{
    return rimDiameterIn * 0.0254f * 0.5f + (sectionWidthMm * 0.001f) * (aspectPct * 0.01f);
}

void vehicle_spec_refresh_derived(VehicleSpec *spec)
{
    if (spec == NULL) return;

    /* Stage 1 — dimensions -------------------------------------------------------------- */
    spec->lengthOverallM = spec->wheelbaseM + spec->frontOverhangM + spec->rearOverhangM;
    spec->bodyHalfWidthM = 0.5f * spec->widthOverallM;
    spec->frontalAreaM2 = spec->widthOverallM * spec->heightOverallM * VEH_FRONTAL_AREA_FILL;

    /* Stage 2 — mass particles → mass, CG, yaw inertia ----------------------------------- */
    const float masses[5] = {
        spec->massEngineKg, spec->massGearboxKg, spec->massFuelKg,
        spec->massDriverKg, spec->massChassisKg
    };
    const float xs[5] = {
        spec->massEngineXM, spec->massGearboxXM, spec->massFuelXM,
        spec->massDriverXM, spec->massChassisXM
    };
    const float zs[5] = {
        spec->massEngineZM, spec->massGearboxZM, spec->massFuelZM,
        spec->massDriverZM, spec->massChassisZM
    };

    float massKg = 0.0f;
    float momentX = 0.0f;
    float momentZ = 0.0f;
    for (int i = 0; i < 5; i++) {
        massKg += masses[i];
        momentX += masses[i] * xs[i];
        momentZ += masses[i] * zs[i];
    }
    spec->massKg = massKg;
    if (massKg > 0.0f) {
        const float xCg = momentX / massKg;
        const float zCg = momentZ / massKg;
        spec->cgToFrontM = 0.5f * spec->wheelbaseM - xCg;
        spec->cgToRearM = 0.5f * spec->wheelbaseM + xCg;
        spec->cgHeightM = zCg;

        float izz = 0.0f;
        for (int i = 0; i < 5; i++) {
            const float dx = xs[i] - xCg;
            izz += masses[i] * dx * dx;
        }
        const float yawRadiusM = VEH_YAW_RADIUS_FACTOR * spec->wheelbaseM;
        izz += massKg * yawRadiusM * yawRadiusM;
        spec->yawInertiaKgM2 = izz;
    }

    /* Stage 3 — tires → radii, inertia, relaxation, load reference ----------------------- */
    const float unloadedF = tire_unloaded_radius_m(
        spec->tireSectionWidthFrontMm, spec->tireAspectFrontPct, spec->tireRimDiameterFrontIn);
    const float unloadedR = tire_unloaded_radius_m(
        spec->tireSectionWidthRearMm, spec->tireAspectRearPct, spec->tireRimDiameterRearIn);
    spec->wheelRadiusFrontM = unloadedF * TIRE_LOAD_RADIUS_FACTOR;
    spec->wheelRadiusRearM = unloadedR * TIRE_LOAD_RADIUS_FACTOR;
    spec->wheelRadiusM = spec->wheelRadiusRearM;

    if (massKg > 0.0f) {
        spec->tireLoadRefPerWheelN = massKg * GRAVITY_MPS2 * 0.25f;
    }

    /* Wheel inertia, tire relaxation, roll stiffness, and max brake torque stay primary
     * handling tunables. Tire designation and suspension/brake hardware feed the visual
     * grammar; a later phase can derive the handling fields once presets migrate fully. */
}

void vehicle_spec_set_default(VehicleSpec *spec)
{
    if (spec == NULL) return;
    memset(spec, 0, sizeof(*spec));

    spec->wheelbaseM = VEH_WHEELBASE_M;
    spec->trackWidthFrontM = VEH_TRACK_FRONT_M;
    spec->trackWidthRearM = VEH_TRACK_REAR_M;
    spec->frontOverhangM = VEH_FRONT_OVERHANG_M;
    spec->rearOverhangM = VEH_REAR_OVERHANG_M;
    spec->widthOverallM = VEH_WIDTH_OVERALL_M;
    spec->heightOverallM = VEH_HEIGHT_OVERALL_M;
    spec->rideHeightFrontM = VEH_RIDE_HEIGHT_FRONT_M;
    spec->rideHeightRearM = VEH_RIDE_HEIGHT_REAR_M;
    spec->cowlXM = VEH_COWL_X_M;
    spec->backlightXM = VEH_BACKLIGHT_X_M;

    spec->massEngineKg = MASS_ENGINE_KG;
    spec->massEngineXM = MASS_ENGINE_X_M;
    spec->massEngineZM = MASS_ENGINE_Z_M;
    spec->massGearboxKg = MASS_GEARBOX_KG;
    spec->massGearboxXM = MASS_GEARBOX_X_M;
    spec->massGearboxZM = MASS_GEARBOX_Z_M;
    spec->massFuelKg = MASS_FUEL_KG;
    spec->massFuelXM = MASS_FUEL_X_M;
    spec->massFuelZM = MASS_FUEL_Z_M;
    spec->massDriverKg = MASS_DRIVER_KG;
    spec->massDriverXM = MASS_DRIVER_X_M;
    spec->massDriverZM = MASS_DRIVER_Z_M;
    spec->massChassisKg = MASS_CHASSIS_KG;
    spec->massChassisXM = MASS_CHASSIS_X_M;
    spec->massChassisZM = MASS_CHASSIS_Z_M;

    spec->tireSectionWidthFrontMm = TIRE_SECTION_WIDTH_MM;
    spec->tireSectionWidthRearMm = TIRE_SECTION_WIDTH_MM;
    spec->tireAspectFrontPct = TIRE_ASPECT_RATIO_PCT;
    spec->tireAspectRearPct = TIRE_ASPECT_RATIO_PCT;
    spec->tireRimDiameterFrontIn = TIRE_RIM_DIAMETER_IN;
    spec->tireRimDiameterRearIn = TIRE_RIM_DIAMETER_IN;
    spec->tireRimWidthFrontIn = TIRE_RIM_WIDTH_IN;
    spec->tireRimWidthRearIn = TIRE_RIM_WIDTH_IN;
    spec->tirePressureFrontKpa = TIRE_PRESSURE_KPA;
    spec->tirePressureRearKpa = TIRE_PRESSURE_KPA;

    spec->wheelInertiaKgM2 = WHEEL_INERTIA_KGM2;

    spec->suspCamberFrontRad = SUSP_CAMBER_FRONT_RAD;
    spec->suspCamberRearRad = SUSP_CAMBER_REAR_RAD;
    spec->suspToeFrontRad = SUSP_TOE_FRONT_RAD;
    spec->suspToeRearRad = SUSP_TOE_REAR_RAD;
    spec->suspCasterFrontRad = SUSP_CASTER_FRONT_RAD;
    spec->suspCasterRearRad = SUSP_CASTER_REAR_RAD;
    spec->suspWheelRateFrontNpm = SUSP_WHEEL_RATE_FRONT_NPM;
    spec->suspWheelRateRearNpm = SUSP_WHEEL_RATE_REAR_NPM;
    spec->suspAntiRollFrontNpm = SUSP_ANTI_ROLL_FRONT_NPM;
    spec->suspAntiRollRearNpm = SUSP_ANTI_ROLL_REAR_NPM;
    spec->suspTravelFrontM = SUSP_TRAVEL_FRONT_M;
    spec->suspTravelRearM = SUSP_TRAVEL_REAR_M;
    spec->suspRollCentreFrontM = SUSP_ROLL_CENTRE_FRONT_M;
    spec->suspRollCentreRearM = SUSP_ROLL_CENTRE_REAR_M;

    spec->wheelOffsetEtFrontMm = WHEEL_OFFSET_ET_FRONT_MM;
    spec->wheelOffsetEtRearMm = WHEEL_OFFSET_ET_REAR_MM;
    spec->brakeDiscRadiusFrontM = BRAKE_DISC_RADIUS_FRONT_M;
    spec->brakeDiscRadiusRearM = BRAKE_DISC_RADIUS_REAR_M;
    spec->brakePadFriction = BRAKE_PAD_FRICTION;

    spec->aeroLiftCoefFront = AERO_LIFT_COEF_FRONT;
    spec->aeroLiftCoefRear = AERO_LIFT_COEF_REAR;
    spec->aeroRefAreaFrontM2 = AERO_REF_AREA_FRONT_M2;
    spec->aeroRefAreaRearM2 = AERO_REF_AREA_REAR_M2;
    spec->aeroCentreOfPressureXM = AERO_COP_X_M;

    spec->drivetrainLayout = DRIVETRAIN_LAYOUT_DEFAULT;
    spec->frontTorqueSplit = DRIVETRAIN_FRONT_TORQUE_SPLIT;
    spec->engineCylinders = ENGINE_CYLINDERS;
    spec->engineDisplacementL = ENGINE_DISPLACEMENT_L;

    spec->maxRoadWheelAngleRad = STEER_MAX_RAD;
    spec->maxSteerRateRadS = STEER_RATE_RAD_S;
    spec->steerReturnRateRadS = STEER_RETURN_RATE_RAD_S;
    spec->dragCoefficient = DRAG_COEFFICIENT;
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
    spec->tireLoadSensitivityK = TIRE_LOAD_SENSITIVITY_K;
    spec->tireRelaxationLengthM = TIRE_RELAXATION_LENGTH_M;
    spec->ackermannPercent = ACKERMANN_PERCENT;
    spec->differentialMode = (float)DIFFERENTIAL_MODE_DEFAULT;
    spec->differentialBiasRatio = DIFFERENTIAL_BIAS_RATIO;
    spec->differentialPreloadNm = DIFFERENTIAL_PRELOAD_NM;
    spec->rollStiffnessFrontFraction = ROLL_STIFFNESS_FRONT_FRACTION;
    spec->lateralLoadTransferEnabled = false;
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
    spec->collisionRestitution = COLLISION_RESTITUTION;
    spec->collisionFriction = COLLISION_FRICTION;

    vehicle_spec_refresh_derived(spec);
}

bool vehicle_spec_is_valid(const VehicleSpec *spec)
{
    if (spec == NULL) return false;
    if (!(isfinite(spec->wheelbaseM) && spec->wheelbaseM > 0.0f)) return false;
    if (!(isfinite(spec->massKg) && spec->massKg > 0.0f)) return false;
    if (!(isfinite(spec->yawInertiaKgM2) && spec->yawInertiaKgM2 > 0.0f)) return false;
    if (!(isfinite(spec->cgToFrontM) && spec->cgToFrontM > 0.0f)) return false;
    if (!(isfinite(spec->cgToRearM) && spec->cgToRearM > 0.0f)) return false;
    if (!(isfinite(spec->wheelbaseM) &&
          fabsf(spec->wheelbaseM - (spec->cgToFrontM + spec->cgToRearM)) < 1e-4f)) return false;
    if (!(isfinite(spec->trackWidthFrontM) && spec->trackWidthFrontM > 0.0f)) return false;
    if (!(isfinite(spec->trackWidthRearM) && spec->trackWidthRearM > 0.0f)) return false;
    if (!(isfinite(spec->cgHeightM) && spec->cgHeightM > 0.0f)) return false;
    if (!(isfinite(spec->frontOverhangM) && spec->frontOverhangM >= 0.0f)) return false;
    if (!(isfinite(spec->rearOverhangM) && spec->rearOverhangM >= 0.0f)) return false;
    if (!(isfinite(spec->widthOverallM) && spec->widthOverallM > 0.0f)) return false;
    if (!(isfinite(spec->heightOverallM) && spec->heightOverallM > 0.0f)) return false;
    if (!(isfinite(spec->wheelRadiusFrontM) && spec->wheelRadiusFrontM > 0.0f)) return false;
    if (!(isfinite(spec->wheelRadiusRearM) && spec->wheelRadiusRearM > 0.0f)) return false;
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

    if (!(isfinite(spec->bodyHalfWidthM) && spec->bodyHalfWidthM > 0.0f)) return false;
    if (!(isfinite(spec->collisionRestitution) &&
          spec->collisionRestitution >= 0.0f && spec->collisionRestitution <= 1.0f)) return false;
    if (!(isfinite(spec->collisionFriction) && spec->collisionFriction >= 0.0f)) return false;

    if (!(isfinite(spec->tireRelaxationLengthM) &&
          spec->tireRelaxationLengthM >= 0.0f && spec->tireRelaxationLengthM <= 1.0f)) return false;
    if (!(isfinite(spec->tireLoadSensitivityK) &&
          spec->tireLoadSensitivityK >= 0.0f && spec->tireLoadSensitivityK <= 0.05f)) return false;
    if (!(isfinite(spec->tireLoadRefPerWheelN) &&
          spec->tireLoadRefPerWheelN > 0.0f && spec->tireLoadRefPerWheelN <= 20000.0f)) return false;
    if (!(isfinite(spec->ackermannPercent) &&
          spec->ackermannPercent >= 0.0f && spec->ackermannPercent <= 1.0f)) return false;
    if (!(isfinite(spec->differentialMode) &&
          spec->differentialMode >= 0.0f && spec->differentialMode <= 2.0f)) return false;
    if (!(isfinite(spec->differentialBiasRatio) &&
          spec->differentialBiasRatio >= 1.0f && spec->differentialBiasRatio <= 5.0f)) return false;
    if (!(isfinite(spec->differentialPreloadNm) &&
          spec->differentialPreloadNm >= 0.0f && spec->differentialPreloadNm <= 400.0f)) return false;
    if (!(isfinite(spec->rollStiffnessFrontFraction) &&
          spec->rollStiffnessFrontFraction >= 0.0f && spec->rollStiffnessFrontFraction <= 1.0f)) return false;

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

#include "physics/vehicle.h"

#include <math.h>
#include <stdio.h>
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

static bool integer_in_range(float value, int minimum, int maximum)
{
    return isfinite(value) && value >= (float)minimum && value <= (float)maximum &&
           floorf(value) == value;
}

static bool zero_or_range(float value, float minimum, float maximum)
{
    return isfinite(value) && (value == 0.0f || (value >= minimum && value <= maximum));
}

float vehicle_effective_drag_coefficient(const VehicleSpec *spec)
{
    if (spec == NULL || !isfinite(spec->dragCoefficient) || spec->dragCoefficient < 0.0f)
        return 0.0f;
    if (!isfinite(spec->windscreenRakeRad) ||
        spec->windscreenRakeRad < VEH_WINDSCREEN_RAKE_MIN_RAD ||
        spec->windscreenRakeRad > VEH_WINDSCREEN_RAKE_MAX_RAD)
        return spec->dragCoefficient;

    /* Bayindirli & Celik measured roughly 0.4% Cd reduction per added degree of windscreen
     * inclination on a bus (IJAET 9(3), 2020). 0.004/deg = 0.229183/rad. Keep the published
     * slope as a fixed derivation, not a hidden tunable, and bound extrapolation to +/-10%. */
    const float cdPerRad = 0.2291831181f;
    const float factor = fminf(
        fmaxf(1.0f - cdPerRad * (spec->windscreenRakeRad - VEH_WINDSCREEN_RAKE_RAD), 0.90f),
        1.10f);
    return spec->dragCoefficient * factor;
}

void vehicle_spec_refresh_derived(VehicleSpec *spec)
{
    if (spec == NULL) return;

    /* Stage 1 — dimensions -------------------------------------------------------------- */
    spec->lengthOverallM = spec->wheelbaseM + spec->frontOverhangM + spec->rearOverhangM;
    spec->bodyHalfWidthM = 0.5f * spec->widthOverallM;
    spec->frontalAreaM2 = spec->widthOverallM * spec->heightOverallM * VEH_FRONTAL_AREA_FILL;

    /* Stage 2 — mass particles → mass, CG, yaw inertia ----------------------------------- */
    const float masses[5] = { spec->massEngineKg, spec->massGearboxKg, spec->massFuelKg,
                              spec->massDriverKg, spec->massChassisKg };
    const float xs[5] = { spec->massEngineXM, spec->massGearboxXM, spec->massFuelXM,
                          spec->massDriverXM, spec->massChassisXM };
    const float zs[5] = { spec->massEngineZM, spec->massGearboxZM, spec->massFuelZM,
                          spec->massDriverZM, spec->massChassisZM };

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
    spec->bedLengthM = VEH_BED_LENGTH_M;
    spec->noseWidthM = VEH_NOSE_WIDTH_M;
    spec->tailWidthM = VEH_TAIL_WIDTH_M;
    spec->shoulderXM = VEH_SHOULDER_X_M;
    spec->fenderFlareFrontM = VEH_FENDER_FLARE_FRONT_M;
    spec->fenderFlareRearM = VEH_FENDER_FLARE_REAR_M;
    spec->roofStartXM = VEH_ROOF_START_X_M;
    spec->roofEndXM = VEH_ROOF_END_X_M;
    spec->roofWidthM = VEH_ROOF_WIDTH_M;
    spec->windscreenRakeRad = VEH_WINDSCREEN_RAKE_RAD;
    spec->backlightRakeRad = VEH_BACKLIGHT_RAKE_RAD;
    spec->sideWindowCount = VEH_SIDE_WINDOW_COUNT;
    spec->quarterWindowLengthM = VEH_QUARTER_WINDOW_LENGTH_M;
    spec->sunroofLengthM = VEH_SUNROOF_LENGTH_M;
    spec->doorCount = VEH_DOOR_COUNT;
    spec->cabinRows = VEH_CABIN_ROWS;
    spec->roofType = VEH_ROOF_TYPE;

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
    spec->steerSpeedRefMps = STEER_SPEED_REF_MPS;
    spec->steerSpeedMinFactor = STEER_SPEED_MIN_FACTOR;
    spec->dragCoefficient = DRAG_COEFFICIENT;
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
    spec->lateralLoadTransferEnabled = true;
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
          fabsf(spec->wheelbaseM - (spec->cgToFrontM + spec->cgToRearM)) < 1e-4f))
        return false;
    if (!(isfinite(spec->trackWidthFrontM) && spec->trackWidthFrontM > 0.0f)) return false;
    if (!(isfinite(spec->trackWidthRearM) && spec->trackWidthRearM > 0.0f)) return false;
    if (!(isfinite(spec->cgHeightM) && spec->cgHeightM > 0.0f)) return false;
    if (!(isfinite(spec->frontOverhangM) && spec->frontOverhangM >= 0.0f)) return false;
    if (!(isfinite(spec->rearOverhangM) && spec->rearOverhangM >= 0.0f)) return false;
    if (!(isfinite(spec->widthOverallM) && spec->widthOverallM > 0.0f)) return false;
    if (!(isfinite(spec->heightOverallM) && spec->heightOverallM > 0.0f)) return false;
    /* Zero is the common case — most vehicles have no bed — so this is >= rather than >. */
    if (!(isfinite(spec->bedLengthM) && spec->bedLengthM >= 0.0f)) return false;
    /* Phase B silhouette endpoints are physical [identity] hull stations. An endpoint wider
     * than the body it caps is incompatible geometry: widthOverallM is the maximum body width
     * and the shoulder sits at it, so an endpoint past it would falsify the identity (the
     * grammar would have to clamp) and make the shoulder's max-width claim ambiguous. Reject
     * it here rather than silently clamp in car_visual.c. */
    if (!(isfinite(spec->noseWidthM) && spec->noseWidthM > 0.0f &&
          spec->noseWidthM <= spec->widthOverallM))
        return false;
    if (!(isfinite(spec->tailWidthM) && spec->tailWidthM > 0.0f &&
          spec->tailWidthM <= spec->widthOverallM))
        return false;
    /* A longitudinal station; finiteness only, like cowl_x / backlight_x. */
    if (!(isfinite(spec->shoulderXM))) return false;
    if (!(isfinite(spec->fenderFlareFrontM) && spec->fenderFlareFrontM >= 0.0f)) return false;
    if (!(isfinite(spec->fenderFlareRearM) && spec->fenderFlareRearM >= 0.0f)) return false;
    if (!(isfinite(spec->roofStartXM) && isfinite(spec->roofEndXM) &&
          spec->roofStartXM > spec->roofEndXM))
        return false;
    if (!(isfinite(spec->roofWidthM) && spec->roofWidthM > 0.0f &&
          spec->roofWidthM <= spec->widthOverallM))
        return false;
    if (!(isfinite(spec->windscreenRakeRad) &&
          spec->windscreenRakeRad >= VEH_WINDSCREEN_RAKE_MIN_RAD &&
          spec->windscreenRakeRad <= VEH_WINDSCREEN_RAKE_MAX_RAD))
        return false;
    if (!(isfinite(spec->backlightRakeRad) &&
          spec->backlightRakeRad >= VEH_BACKLIGHT_RAKE_MIN_RAD &&
          spec->backlightRakeRad <= VEH_BACKLIGHT_RAKE_MAX_RAD))
        return false;
    if (!integer_in_range(spec->sideWindowCount, 2, 6)) return false;
    if (!zero_or_range(spec->quarterWindowLengthM, 0.2f, 0.4f)) return false;
    if (!zero_or_range(spec->sunroofLengthM, 0.4f, 1.0f)) return false;
    if (!(spec->doorCount == 2.0f || spec->doorCount == 4.0f || spec->doorCount == 5.0f))
        return false;
    if (!integer_in_range(spec->cabinRows, 1, 3)) return false;
    if (!integer_in_range(spec->roofType, VEH_ROOF_FIXED, VEH_ROOF_CONVERTIBLE)) return false;
    if (!(isfinite(spec->wheelRadiusFrontM) && spec->wheelRadiusFrontM > 0.0f)) return false;
    if (!(isfinite(spec->wheelRadiusRearM) && spec->wheelRadiusRearM > 0.0f)) return false;
    if (!(isfinite(spec->wheelInertiaKgM2) && spec->wheelInertiaKgM2 > 0.0f)) return false;
    if (!(isfinite(spec->dragCoefficient) && spec->dragCoefficient >= 0.0f)) return false;
    if (!(isfinite(spec->frontalAreaM2) && spec->frontalAreaM2 >= 0.0f)) return false;
    if (!(isfinite(spec->loadFilterRateHz) && spec->loadFilterRateHz > 0.0f)) return false;
    if (!(isfinite(spec->tireBLatFront) && spec->tireBLatFront > 0.0f &&
          isfinite(spec->tireCLatFront) && spec->tireCLatFront > 0.0f &&
          isfinite(spec->tireMuLatFront) && spec->tireMuLatFront > 0.0f &&
          isfinite(spec->tireBLatRear) && spec->tireBLatRear > 0.0f &&
          isfinite(spec->tireCLatRear) && spec->tireCLatRear > 0.0f &&
          isfinite(spec->tireMuLatRear) && spec->tireMuLatRear > 0.0f &&
          isfinite(spec->tireBLong) && spec->tireBLong > 0.0f && isfinite(spec->tireCLong) &&
          spec->tireCLong > 0.0f && isfinite(spec->tireMuLongScale) &&
          spec->tireMuLongScale > 0.0f))
        return false;
    if (spec->gearCount <= 0 || spec->gearCount > MAX_GEARS) return false;
    for (int i = 0; i < spec->gearCount; i++) {
        if (!(isfinite(spec->gearRatios[i]) && spec->gearRatios[i] > 0.0f)) return false;
    }
    if (!(isfinite(spec->reverseGearRatio) && spec->reverseGearRatio > 0.0f)) return false;
    if (!(isfinite(spec->finalDriveRatio) && spec->finalDriveRatio > 0.0f)) return false;
    if (!(isfinite(spec->drivetrainEfficiency) && spec->drivetrainEfficiency >= 0.0f &&
          spec->drivetrainEfficiency <= 1.0f))
        return false;
    if (!(isfinite(spec->engineIdleRpm) && isfinite(spec->engineRedlineRpm) &&
          spec->engineIdleRpm > 0.0f && spec->engineIdleRpm < spec->engineRedlineRpm))
        return false;
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++) {
        if (!(isfinite(spec->engineTorqueCurveNm[i]) && spec->engineTorqueCurveNm[i] >= 0.0f))
            return false;
    }
    if (!(isfinite(spec->engineBrakingTorqueNm) && spec->engineBrakingTorqueNm >= 0.0f))
        return false;
    if (!(isfinite(spec->maxBrakeTorqueNm) && spec->maxBrakeTorqueNm >= 0.0f)) return false;
    if (!(isfinite(spec->brakeBiasFront) && spec->brakeBiasFront >= 0.0f &&
          spec->brakeBiasFront <= 1.0f))
        return false;
    if (!(isfinite(spec->handbrakeTorqueNm) && spec->handbrakeTorqueNm >= 0.0f)) return false;
    if (!(isfinite(spec->steerSpeedRefMps) && spec->steerSpeedRefMps > 0.0f)) return false;
    if (!(isfinite(spec->steerSpeedMinFactor) && spec->steerSpeedMinFactor >= 0.0f &&
          spec->steerSpeedMinFactor <= 1.0f))
        return false;
    if (!(isfinite(spec->collisionRestitution) && spec->collisionRestitution >= 0.0f &&
          spec->collisionRestitution <= 1.0f))
        return false;
    if (!(isfinite(spec->collisionFriction) && spec->collisionFriction >= 0.0f)) return false;

    if (!(isfinite(spec->tireRelaxationLengthM) && spec->tireRelaxationLengthM >= 0.0f &&
          spec->tireRelaxationLengthM <= 1.0f))
        return false;
    if (!(isfinite(spec->tireLoadSensitivityK) && spec->tireLoadSensitivityK >= 0.0f &&
          spec->tireLoadSensitivityK <= 0.05f))
        return false;
    if (!(isfinite(spec->tireLoadRefPerWheelN) && spec->tireLoadRefPerWheelN > 0.0f &&
          spec->tireLoadRefPerWheelN <= 20000.0f))
        return false;
    if (!(isfinite(spec->ackermannPercent) && spec->ackermannPercent >= 0.0f &&
          spec->ackermannPercent <= 1.0f))
        return false;
    if (!(isfinite(spec->differentialMode) && spec->differentialMode >= 0.0f &&
          spec->differentialMode <= 2.0f))
        return false;
    if (!(isfinite(spec->differentialBiasRatio) && spec->differentialBiasRatio >= 1.0f &&
          spec->differentialBiasRatio <= 5.0f))
        return false;
    if (!(isfinite(spec->differentialPreloadNm) && spec->differentialPreloadNm >= 0.0f &&
          spec->differentialPreloadNm <= 400.0f))
        return false;
    if (!(isfinite(spec->rollStiffnessFrontFraction) &&
          spec->rollStiffnessFrontFraction >= 0.0f && spec->rollStiffnessFrontFraction <= 1.0f))
        return false;

    return true;
}

void vehicle_state_reset(const VehicleSpec *spec, VehicleState *state, VehicleDerived *derived,
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

static uint32_t vehicle_hash_u32(uint32_t hash, uint32_t value)
{
    for (int shift = 0; shift < 32; shift += 8) {
        hash ^= (value >> shift) & 0xffu;
        hash *= 0x01000193u;
    }
    return hash;
}

static uint32_t vehicle_hash_f32(uint32_t hash, float value)
{
    uint32_t bits = 0u;
    memcpy(&bits, &value, sizeof(bits));
    return vehicle_hash_u32(hash, bits);
}

static uint32_t vehicle_content_hash(const VehicleSpec *spec)
{
    uint32_t hash = 0x811c9dc5u;
#define HASH_FLOAT(field) hash = vehicle_hash_f32(hash, spec->field)

    HASH_FLOAT(wheelbaseM);
    HASH_FLOAT(trackWidthFrontM);
    HASH_FLOAT(trackWidthRearM);
    HASH_FLOAT(frontOverhangM);
    HASH_FLOAT(rearOverhangM);
    HASH_FLOAT(widthOverallM);
    HASH_FLOAT(heightOverallM);
    HASH_FLOAT(rideHeightFrontM);
    HASH_FLOAT(rideHeightRearM);
    HASH_FLOAT(cowlXM);
    HASH_FLOAT(backlightXM);
    HASH_FLOAT(bedLengthM);
    HASH_FLOAT(noseWidthM);
    HASH_FLOAT(tailWidthM);
    HASH_FLOAT(shoulderXM);
    HASH_FLOAT(fenderFlareFrontM);
    HASH_FLOAT(fenderFlareRearM);
    HASH_FLOAT(roofStartXM);
    HASH_FLOAT(roofEndXM);
    HASH_FLOAT(roofWidthM);
    HASH_FLOAT(windscreenRakeRad);
    HASH_FLOAT(backlightRakeRad);
    HASH_FLOAT(sideWindowCount);
    HASH_FLOAT(quarterWindowLengthM);
    HASH_FLOAT(sunroofLengthM);
    HASH_FLOAT(doorCount);
    HASH_FLOAT(cabinRows);
    HASH_FLOAT(roofType);
    HASH_FLOAT(massEngineKg);
    HASH_FLOAT(massEngineXM);
    HASH_FLOAT(massEngineZM);
    HASH_FLOAT(massGearboxKg);
    HASH_FLOAT(massGearboxXM);
    HASH_FLOAT(massGearboxZM);
    HASH_FLOAT(massFuelKg);
    HASH_FLOAT(massFuelXM);
    HASH_FLOAT(massFuelZM);
    HASH_FLOAT(massDriverKg);
    HASH_FLOAT(massDriverXM);
    HASH_FLOAT(massDriverZM);
    HASH_FLOAT(massChassisKg);
    HASH_FLOAT(massChassisXM);
    HASH_FLOAT(massChassisZM);
    HASH_FLOAT(tireSectionWidthFrontMm);
    HASH_FLOAT(tireSectionWidthRearMm);
    HASH_FLOAT(tireAspectFrontPct);
    HASH_FLOAT(tireAspectRearPct);
    HASH_FLOAT(tireRimDiameterFrontIn);
    HASH_FLOAT(tireRimDiameterRearIn);
    HASH_FLOAT(tireRimWidthFrontIn);
    HASH_FLOAT(tireRimWidthRearIn);
    HASH_FLOAT(tirePressureFrontKpa);
    HASH_FLOAT(tirePressureRearKpa);
    HASH_FLOAT(suspCamberFrontRad);
    HASH_FLOAT(suspCamberRearRad);
    HASH_FLOAT(suspToeFrontRad);
    HASH_FLOAT(suspToeRearRad);
    HASH_FLOAT(suspCasterFrontRad);
    HASH_FLOAT(suspCasterRearRad);
    HASH_FLOAT(suspWheelRateFrontNpm);
    HASH_FLOAT(suspWheelRateRearNpm);
    HASH_FLOAT(suspAntiRollFrontNpm);
    HASH_FLOAT(suspAntiRollRearNpm);
    HASH_FLOAT(suspTravelFrontM);
    HASH_FLOAT(suspTravelRearM);
    HASH_FLOAT(suspRollCentreFrontM);
    HASH_FLOAT(suspRollCentreRearM);
    HASH_FLOAT(wheelOffsetEtFrontMm);
    HASH_FLOAT(wheelOffsetEtRearMm);
    HASH_FLOAT(brakeDiscRadiusFrontM);
    HASH_FLOAT(brakeDiscRadiusRearM);
    HASH_FLOAT(brakePadFriction);
    HASH_FLOAT(aeroLiftCoefFront);
    HASH_FLOAT(aeroLiftCoefRear);
    HASH_FLOAT(aeroRefAreaFrontM2);
    HASH_FLOAT(aeroRefAreaRearM2);
    HASH_FLOAT(aeroCentreOfPressureXM);
    HASH_FLOAT(drivetrainLayout);
    HASH_FLOAT(frontTorqueSplit);
    HASH_FLOAT(engineCylinders);
    HASH_FLOAT(engineDisplacementL);
    HASH_FLOAT(massKg);
    HASH_FLOAT(yawInertiaKgM2);
    HASH_FLOAT(cgToFrontM);
    HASH_FLOAT(cgToRearM);
    HASH_FLOAT(cgHeightM);
    HASH_FLOAT(lengthOverallM);
    HASH_FLOAT(wheelRadiusFrontM);
    HASH_FLOAT(wheelRadiusRearM);
    HASH_FLOAT(wheelRadiusM);
    HASH_FLOAT(wheelInertiaKgM2);
    HASH_FLOAT(frontalAreaM2);
    HASH_FLOAT(bodyHalfWidthM);
    HASH_FLOAT(maxBrakeTorqueNm);
    HASH_FLOAT(rollStiffnessFrontFraction);
    HASH_FLOAT(tireRelaxationLengthM);
    HASH_FLOAT(tireLoadRefPerWheelN);
    HASH_FLOAT(maxRoadWheelAngleRad);
    HASH_FLOAT(maxSteerRateRadS);
    HASH_FLOAT(steerReturnRateRadS);
    HASH_FLOAT(steerSpeedRefMps);
    HASH_FLOAT(steerSpeedMinFactor);
    HASH_FLOAT(dragCoefficient);
    HASH_FLOAT(loadFilterRateHz);
    HASH_FLOAT(tireBLatFront);
    HASH_FLOAT(tireCLatFront);
    HASH_FLOAT(tireMuLatFront);
    HASH_FLOAT(tireBLatRear);
    HASH_FLOAT(tireCLatRear);
    HASH_FLOAT(tireMuLatRear);
    HASH_FLOAT(tireBLong);
    HASH_FLOAT(tireCLong);
    HASH_FLOAT(tireMuLongScale);
    for (int i = 0; i < MAX_GEARS; i++) HASH_FLOAT(gearRatios[i]);
    hash = vehicle_hash_u32(hash, (uint32_t)spec->gearCount);
    HASH_FLOAT(reverseGearRatio);
    HASH_FLOAT(finalDriveRatio);
    HASH_FLOAT(drivetrainEfficiency);
    HASH_FLOAT(engineIdleRpm);
    HASH_FLOAT(engineRedlineRpm);
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++) HASH_FLOAT(engineTorqueCurveNm[i]);
    HASH_FLOAT(engineBrakingTorqueNm);
    HASH_FLOAT(brakeBiasFront);
    HASH_FLOAT(handbrakeTorqueNm);
    HASH_FLOAT(collisionRestitution);
    HASH_FLOAT(collisionFriction);
    HASH_FLOAT(tireLoadSensitivityK);
    HASH_FLOAT(ackermannPercent);
    HASH_FLOAT(differentialMode);
    HASH_FLOAT(differentialBiasRatio);
    HASH_FLOAT(differentialPreloadNm);
    hash = vehicle_hash_u32(hash, spec->lateralLoadTransferEnabled ? 1u : 0u);

#undef HASH_FLOAT
    return hash;
}

bool vehicle_definition_init(VehicleDefinition *definition, const char *id,
                             const char *appearanceId, uint32_t contentVersion,
                             const VehicleSpec *spec)
{
    if (definition == NULL || id == NULL || id[0] == '\0' || appearanceId == NULL ||
        appearanceId[0] == '\0' || contentVersion == 0u || spec == NULL)
        return false;

    VehicleDefinition candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.spec = *spec;
    vehicle_spec_refresh_derived(&candidate.spec);
    if (!vehicle_spec_is_valid(&candidate.spec)) return false;

    if (snprintf(candidate.id, sizeof(candidate.id), "%s", id) >= (int)sizeof(candidate.id) ||
        snprintf(candidate.appearanceId, sizeof(candidate.appearanceId), "%s", appearanceId) >=
            (int)sizeof(candidate.appearanceId))
        return false;
    candidate.contentVersion = contentVersion;
    candidate.contentHash = vehicle_content_hash(&candidate.spec);
    *definition = candidate;
    return true;
}

void vehicle_definition_set_default(VehicleDefinition *definition)
{
    if (definition == NULL) return;
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    (void)vehicle_definition_init(definition, "builtin/default", "builtin/default", 1u, &spec);
}

void vehicle_setup_set_default(const VehicleDefinition *definition, VehicleSetup *setup)
{
    if (definition == NULL || setup == NULL) return;
    const VehicleSpec *spec = &definition->spec;
    memset(setup, 0, sizeof(*setup));
    setup->tirePressureFrontKpa = spec->tirePressureFrontKpa;
    setup->tirePressureRearKpa = spec->tirePressureRearKpa;
    setup->suspCamberFrontRad = spec->suspCamberFrontRad;
    setup->suspCamberRearRad = spec->suspCamberRearRad;
    setup->suspToeFrontRad = spec->suspToeFrontRad;
    setup->suspToeRearRad = spec->suspToeRearRad;
    setup->suspCasterFrontRad = spec->suspCasterFrontRad;
    setup->suspCasterRearRad = spec->suspCasterRearRad;
    memcpy(setup->gearRatios, spec->gearRatios, sizeof(setup->gearRatios));
    setup->gearCount = spec->gearCount;
    setup->reverseGearRatio = spec->reverseGearRatio;
    setup->finalDriveRatio = spec->finalDriveRatio;
    setup->brakeBiasFront = spec->brakeBiasFront;
    setup->differentialMode = spec->differentialMode;
    setup->differentialBiasRatio = spec->differentialBiasRatio;
    setup->differentialPreloadNm = spec->differentialPreloadNm;
}

uint32_t vehicle_setup_hash(const VehicleSetup *setup)
{
    /* Deterministic serialization anchor for saved/replayed setups (issue #33): the field
     * order below mirrors the entrant-setup block of game_state_checksum (game.c), so an
     * untouched setup round-trips to identical bytes and hashes on every platform. A NULL
     * setup hashes as a fully zeroed one: the empty setup is a legal, stable value. */
    VehicleSetup zeroed;
    if (setup == NULL) {
        memset(&zeroed, 0, sizeof(zeroed));
        setup = &zeroed;
    }

    uint32_t hash = 0x811c9dc5u;
    hash = vehicle_hash_f32(hash, setup->tirePressureFrontKpa);
    hash = vehicle_hash_f32(hash, setup->tirePressureRearKpa);
    hash = vehicle_hash_f32(hash, setup->suspCamberFrontRad);
    hash = vehicle_hash_f32(hash, setup->suspCamberRearRad);
    hash = vehicle_hash_f32(hash, setup->suspToeFrontRad);
    hash = vehicle_hash_f32(hash, setup->suspToeRearRad);
    hash = vehicle_hash_f32(hash, setup->suspCasterFrontRad);
    hash = vehicle_hash_f32(hash, setup->suspCasterRearRad);
    hash = vehicle_hash_u32(hash, (uint32_t)setup->gearCount);
    for (int i = 0; i < MAX_GEARS; i++) hash = vehicle_hash_f32(hash, setup->gearRatios[i]);
    hash = vehicle_hash_f32(hash, setup->reverseGearRatio);
    hash = vehicle_hash_f32(hash, setup->finalDriveRatio);
    hash = vehicle_hash_f32(hash, setup->brakeBiasFront);
    hash = vehicle_hash_f32(hash, setup->differentialMode);
    hash = vehicle_hash_f32(hash, setup->differentialBiasRatio);
    hash = vehicle_hash_f32(hash, setup->differentialPreloadNm);
    return hash;
}

static void vehicle_setup_apply(VehicleSpec *spec, const VehicleSetup *setup)
{
    spec->tirePressureFrontKpa = setup->tirePressureFrontKpa;
    spec->tirePressureRearKpa = setup->tirePressureRearKpa;
    spec->suspCamberFrontRad = setup->suspCamberFrontRad;
    spec->suspCamberRearRad = setup->suspCamberRearRad;
    spec->suspToeFrontRad = setup->suspToeFrontRad;
    spec->suspToeRearRad = setup->suspToeRearRad;
    spec->suspCasterFrontRad = setup->suspCasterFrontRad;
    spec->suspCasterRearRad = setup->suspCasterRearRad;
    memcpy(spec->gearRatios, setup->gearRatios, sizeof(spec->gearRatios));
    spec->gearCount = setup->gearCount;
    spec->reverseGearRatio = setup->reverseGearRatio;
    spec->finalDriveRatio = setup->finalDriveRatio;
    spec->brakeBiasFront = setup->brakeBiasFront;
    spec->differentialMode = setup->differentialMode;
    spec->differentialBiasRatio = setup->differentialBiasRatio;
    spec->differentialPreloadNm = setup->differentialPreloadNm;
}

bool vehicle_setup_is_valid(const VehicleDefinition *definition, const VehicleSetup *setup)
{
    if (definition == NULL || setup == NULL) return false;
    VehicleSpec compiled = definition->spec;
    vehicle_setup_apply(&compiled, setup);
    vehicle_spec_refresh_derived(&compiled);
    return vehicle_spec_is_valid(&compiled);
}

bool vehicle_instance_derive(VehicleInstance *instance, const VehicleDefinition *definition,
                             const VehicleSetup *setup)
{
    if (instance == NULL || definition == NULL || setup == NULL ||
        !vehicle_setup_is_valid(definition, setup))
        return false;
    instance->spec = definition->spec;
    vehicle_setup_apply(&instance->spec, setup);
    vehicle_spec_refresh_derived(&instance->spec);
    return true;
}

void vehicle_instance_reset(VehicleInstance *instance)
{
    if (instance == NULL) return;
    const bool automaticTransmissionEnabled = instance->autoTrans.enabled;
    const bool automaticTransmissionForwardOnly = instance->autoTrans.forwardOnly;
    vehicle_state_reset(&instance->spec, &instance->vehicle, &instance->derived,
                        &instance->renderState);
    instance->autoTrans.enabled = automaticTransmissionEnabled;
    instance->autoTrans.forwardOnly = automaticTransmissionForwardOnly;
    instance->autoTrans.driveState = AUTO_DRIVE;
    instance->autoTrans.neutralTimer = 0.0f;
    memset(&instance->vehicleControls, 0, sizeof(instance->vehicleControls));
    instance->fuelKg = instance->spec.massFuelKg;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool front = i == WHEEL_FRONT_LEFT || i == WHEEL_FRONT_RIGHT;
        instance->tireState[i].pressureKpa =
            front ? instance->spec.tirePressureFrontKpa : instance->spec.tirePressureRearKpa;
        instance->tireState[i].temperatureC = 20.0f;
        instance->tireState[i].wear = 0.0f;
    }
    instance->damage = 0.0f;
    instance->crashLockoutTimerS = 0.0f;
}

bool vehicle_instance_init(VehicleInstance *instance, const VehicleDefinition *definition,
                           const VehicleSetup *setup)
{
    if (instance == NULL) return false;
    memset(instance, 0, sizeof(*instance));
    if (!vehicle_instance_derive(instance, definition, setup)) return false;
    instance->autoTrans.enabled = true;
    instance->autoTrans.forwardOnly = false;
    vehicle_instance_reset(instance);
    return true;
}

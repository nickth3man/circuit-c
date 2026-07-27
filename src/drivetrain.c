#include "drivetrain.h"

#include <math.h>
#include <string.h>

#include "math_utils.h"

static float signf_nonzero(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

float drivetrain_engine_torque_at_rpm(const VehicleSpec *spec, float engineRpm)
{
    if (spec == NULL || !(isfinite(spec->engineIdleRpm) &&
                          isfinite(spec->engineRedlineRpm) &&
                          spec->engineRedlineRpm > spec->engineIdleRpm) ||
        !isfinite(engineRpm)) return 0.0f;

    const float clampedRpm = clampf(engineRpm, spec->engineIdleRpm, spec->engineRedlineRpm);
    const float position = (clampedRpm - spec->engineIdleRpm) /
                           (spec->engineRedlineRpm - spec->engineIdleRpm) *
                           (float)(ENGINE_CURVE_POINTS - 1);
    int lower = (int)floorf(position);
    if (lower < 0) lower = 0;
    if (lower >= ENGINE_CURVE_POINTS - 1) {
        return spec->engineTorqueCurveNm[ENGINE_CURVE_POINTS - 1];
    }
    return lerpf(spec->engineTorqueCurveNm[lower],
                 spec->engineTorqueCurveNm[lower + 1],
                 position - (float)lower);
}

float drivetrain_total_gear_ratio(const VehicleSpec *spec, int selectedGear)
{
    if (spec == NULL || spec->gearCount <= 0 || spec->gearCount > MAX_GEARS ||
        !isfinite(spec->finalDriveRatio) || !(spec->finalDriveRatio > 0.0f)) return 0.0f;
    if (selectedGear == 0) return 0.0f;
    if (selectedGear == -1) {
        if (!(isfinite(spec->reverseGearRatio) && spec->reverseGearRatio > 0.0f)) return 0.0f;
        return -spec->reverseGearRatio * spec->finalDriveRatio;
    }
    if (selectedGear < 1 || selectedGear > spec->gearCount) return 0.0f;
    return spec->gearRatios[selectedGear - 1] * spec->finalDriveRatio;
}

float drivetrain_engine_rpm(const VehicleSpec *spec, int selectedGear,
                            float rearAngularVelocityRadS)
{
    if (spec == NULL || !isfinite(rearAngularVelocityRadS)) return 0.0f;
    const float totalGear = drivetrain_total_gear_ratio(spec, selectedGear);
    if (totalGear == 0.0f) return spec->engineIdleRpm;
    const float rpm = fabsf(rearAngularVelocityRadS) * fabsf(totalGear) *
                      60.0f / DRIFTY_TWO_PI;
    return clampf(rpm, spec->engineIdleRpm, spec->engineRedlineRpm);
}

DrivetrainTorques drivetrain_calculate_torques(const VehicleSpec *spec,
                                               int selectedGear,
                                               float rearAngularVelocityRadS,
                                               float throttle,
                                               float brake,
                                               float handbrake)
{
    DrivetrainTorques out;
    memset(&out, 0, sizeof(out));
    if (spec == NULL) return out;

    throttle = clampf(throttle, 0.0f, 1.0f);
    brake = clampf(brake, 0.0f, 1.0f);
    handbrake = clampf(handbrake, 0.0f, 1.0f);
    out.totalGearRatio = drivetrain_total_gear_ratio(spec, selectedGear);
    const float rpm = drivetrain_engine_rpm(spec, selectedGear, rearAngularVelocityRadS);
    const float curveTorqueNm = drivetrain_engine_torque_at_rpm(spec, rpm);

    /* The simple engine model has no clutch state. Suppressing closed-throttle engine
     * braking while the axle is stationary prevents idle torque from launching the car
     * backwards; once rotating, the documented signed gear ratio makes it oppose motion.
     *
     * Engine braking also fades out as the driveline approaches idle speed. An engine
     * held at idle by its own fueling cannot keep extracting energy from the axle — in a
     * real car the clutch slips or the engine stalls — and without the fade the constant
     * torque drags the rear axle below its rolling speed every tick at walking pace,
     * exciting a two-tick limit cycle between engine braking and tire reaction. The fade
     * is linear from zero at idle to full at ENGINE_BRAKING_FADE_RPM_SPAN above it, using
     * the UNCLAMPED driveline rpm so the fade engages exactly where the idle clamp does. */
    #define ENGINE_BRAKING_FADE_RPM_SPAN 300.0f
    float engineBrakingScale = 0.0f;
    if (fabsf(rearAngularVelocityRadS) > 1e-4f && out.totalGearRatio != 0.0f) {
        const float rawRpm = fabsf(rearAngularVelocityRadS) * fabsf(out.totalGearRatio) *
                             60.0f / DRIFTY_TWO_PI;
        engineBrakingScale = clampf((rawRpm - spec->engineIdleRpm) /
                                    ENGINE_BRAKING_FADE_RPM_SPAN, 0.0f, 1.0f);
    }
    const float engineBrakingNm =
        engineBrakingScale * (1.0f - throttle) * spec->engineBrakingTorqueNm;
    out.engineTorqueNm = throttle * curveTorqueNm - engineBrakingNm;
    out.drivelineTorqueNm = out.engineTorqueNm * out.totalGearRatio *
                            spec->drivetrainEfficiency;
    out.driveTorqueNm[WHEEL_REAR_LEFT] = out.drivelineTorqueNm * 0.5f;
    out.driveTorqueNm[WHEEL_REAR_RIGHT] = out.drivelineTorqueNm * 0.5f;

    const float frontServiceTotalNm = brake * spec->maxBrakeTorqueNm *
                                      spec->brakeBiasFront;
    const float rearServiceTotalNm = brake * spec->maxBrakeTorqueNm *
                                     (1.0f - spec->brakeBiasFront);
    out.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] = frontServiceTotalNm * 0.5f;
    out.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT] = frontServiceTotalNm * 0.5f;
    out.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] = rearServiceTotalNm * 0.5f;
    out.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT] = rearServiceTotalNm * 0.5f;
    out.handbrakeTorqueNm[WHEEL_REAR_LEFT] = handbrake * spec->handbrakeTorqueNm * 0.5f;
    out.handbrakeTorqueNm[WHEEL_REAR_RIGHT] = handbrake * spec->handbrakeTorqueNm * 0.5f;
    return out;
}

float drivetrain_integrate_wheel(float angularVelocityRadS,
                                 float wheelLongitudinalVelocityMps,
                                 float driveTorqueNm,
                                 float serviceBrakeTorqueNm,
                                 float handbrakeTorqueNm,
                                 float tireLongitudinalForceN,
                                 float wheelRadiusM,
                                 float wheelInertiaKgM2,
                                 float dt,
                                 bool *locked)
{
    if (locked != NULL) *locked = false;
    if (!(isfinite(angularVelocityRadS) &&
          isfinite(wheelLongitudinalVelocityMps) &&
          isfinite(driveTorqueNm) && isfinite(serviceBrakeTorqueNm) &&
          isfinite(handbrakeTorqueNm) && isfinite(tireLongitudinalForceN) &&
          isfinite(wheelRadiusM) && wheelRadiusM > 0.0f &&
          isfinite(wheelInertiaKgM2) && wheelInertiaKgM2 > 0.0f &&
          isfinite(dt) && dt > 0.0f)) return 0.0f;

    const float brakeMagnitudeNm = fmaxf(serviceBrakeTorqueNm, 0.0f) +
                                   fmaxf(handbrakeTorqueNm, 0.0f);
    if (brakeMagnitudeNm > 0.0f &&
        fabsf(angularVelocityRadS) <= 1e-4f &&
        fabsf(wheelLongitudinalVelocityMps) <= 1e-4f &&
        fabsf(driveTorqueNm) <= 1e-4f &&
        fabsf(tireLongitudinalForceN) <= 1e-4f) {
        if (locked != NULL) *locked = true;
        return 0.0f;
    }
    float brakeDirection = signf_nonzero(angularVelocityRadS);
    if (brakeDirection == 0.0f) brakeDirection = signf_nonzero(wheelLongitudinalVelocityMps);

    const float nonBrakeTorqueNm = driveTorqueNm -
                                   tireLongitudinalForceN * wheelRadiusM;
    const float netTorqueNm = nonBrakeTorqueNm -
                              brakeMagnitudeNm * brakeDirection;
    float next = angularVelocityRadS +
                 netTorqueNm / wheelInertiaKgM2 * dt;

    if (brakeMagnitudeNm > 0.0f && brakeDirection != 0.0f &&
        driveTorqueNm * brakeDirection <= brakeMagnitudeNm &&
        ((angularVelocityRadS * next < 0.0f) ||
         (fabsf(angularVelocityRadS) <= 1e-4f && next * brakeDirection <= 0.0f))) {
        next = 0.0f;
        if (locked != NULL) *locked = true;
    }
    if (brakeMagnitudeNm > 0.0f && brakeDirection != 0.0f) {
        const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
        if ((angularVelocityRadS - rollingOmega) * brakeDirection <= 0.0f &&
            (next - rollingOmega) * brakeDirection > 0.0f) {
            next = rollingOmega;
        }
    }
    if (brakeMagnitudeNm == 0.0f &&
        driveTorqueNm * angularVelocityRadS < 0.0f &&
        angularVelocityRadS * next < 0.0f) {
        next = 0.0f;
    }
    if (driveTorqueNm != 0.0f && brakeMagnitudeNm == 0.0f) {
        const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
        const float driveDirection = signf_nonzero(driveTorqueNm);
        if ((angularVelocityRadS - rollingOmega) * driveDirection >= 0.0f &&
            (next - rollingOmega) * driveDirection < 0.0f) {
            next = rollingOmega;
        }
    }

    /* The explicit wheel equation is stiff around free rolling. A non-driven, unbraked
     * wheel may settle exactly at the rolling speed, but tire reaction may not numerically
     * throw it past that equilibrium in one fixed step. */
    if (driveTorqueNm == 0.0f && brakeMagnitudeNm == 0.0f) {
        const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
        if ((angularVelocityRadS - rollingOmega) * (next - rollingOmega) < 0.0f) {
            next = rollingOmega;
        }
    }

    /*
     * One-sided bound: only DRIVE torque may spin a wheel faster than the road is turning it.
     *
     * The tire force is evaluated from the slip at the start of the step, but the body has
     * already integrated by the time the wheel does, so the contact-patch rolling speed has
     * moved underneath it. Near a stop that stale force is enormous relative to the wheel's
     * inertia — dFx/domega * R * dt / I is around fifty at 120 Hz — and it can throw a braked
     * wheel from just below rolling speed to well above it in a single tick. The wheel then
     * has positive slip it did not earn, the tire pushes forward, and the brakes accelerate
     * the car.
     *
     * The road can only ever spin a wheel TOWARD its rolling speed, so bounding the result
     * on that side alone costs nothing physical: braking may still drag a wheel below rolling
     * speed and lock it, and drive torque may still break it loose into wheelspin.
     */
    const float rollingOmega = wheelLongitudinalVelocityMps / wheelRadiusM;
    const bool driveCanSpinUp = (rollingOmega >= 0.0f) ? (driveTorqueNm > 0.0f)
                                                       : (driveTorqueNm < 0.0f);
    if (!driveCanSpinUp) {
        if (rollingOmega >= 0.0f) {
            next = fminf(next, fmaxf(angularVelocityRadS, rollingOmega));
        } else {
            next = fmaxf(next, fminf(angularVelocityRadS, rollingOmega));
        }
    }
    return isfinite(next) ? next : 0.0f;
}

bool drivetrain_wheel_equilibrium_omega(float driveTorqueNm,
                                        float wheelLongitudinalVelocityMps,
                                        float wheelRadiusM,
                                        float longitudinalLimitN,
                                        float tireBLong, float tireCLong,
                                        float slipSpeedEpsilonMps,
                                        float *equilibriumOmegaRadS)
{
    if (equilibriumOmegaRadS != NULL) *equilibriumOmegaRadS = 0.0f;
    if (!(isfinite(driveTorqueNm) && isfinite(wheelLongitudinalVelocityMps) &&
          isfinite(wheelRadiusM) && wheelRadiusM > 0.0f &&
          isfinite(longitudinalLimitN) && longitudinalLimitN > 0.0f &&
          isfinite(tireBLong) && tireBLong > 0.0f &&
          isfinite(tireCLong) && tireCLong > 0.0f &&
          isfinite(slipSpeedEpsilonMps) && slipSpeedEpsilonMps > 0.0f)) return false;

    const float normalized = driveTorqueNm / (wheelRadiusM * longitudinalLimitN);
    /* At or beyond the curve's peak there is no stable balance point: the reaction falls as
     * slip grows and the wheel genuinely runs away. Leave that to the explicit dynamics. */
    if (!(fabsf(normalized) < 0.999f)) return false;

    const float slipEq = tanf(asinf(normalized) / tireCLong) / tireBLong;
    const float vxSafe = fmaxf(fabsf(wheelLongitudinalVelocityMps), slipSpeedEpsilonMps);
    const float omegaEq = (wheelLongitudinalVelocityMps + slipEq * vxSafe) / wheelRadiusM;
    if (!isfinite(omegaEq)) return false;

    if (equilibriumOmegaRadS != NULL) *equilibriumOmegaRadS = omegaEq;
    return true;
}

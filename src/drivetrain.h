/*
 * drivetrain.h — pure Phase 2 engine, gearing, torque split, and wheel integration.
 */
#ifndef DRIFTY_DRIVETRAIN_H
#define DRIFTY_DRIVETRAIN_H

#include "vehicle.h"

typedef struct {
    float engineTorqueNm;
    float totalGearRatio;
    float drivelineTorqueNm;
    float driveTorqueNm[WHEEL_COUNT];
    float serviceBrakeTorqueNm[WHEEL_COUNT];
    float handbrakeTorqueNm[WHEEL_COUNT];
} DrivetrainTorques;

float drivetrain_engine_torque_at_rpm(const VehicleSpec *spec, float engineRpm);
float drivetrain_total_gear_ratio(const VehicleSpec *spec, int selectedGear);
float drivetrain_engine_rpm(const VehicleSpec *spec, int selectedGear,
                            float rearAngularVelocityRadS);

DrivetrainTorques drivetrain_calculate_torques(const VehicleSpec *spec,
                                               int selectedGear,
                                               float rearOmegaLeftRadS,
                                               float rearOmegaRightRadS,
                                               float rearTireReactionTorqueLeftNm,
                                               float rearTireReactionTorqueRightNm,
                                               float throttle,
                                               float brake,
                                               float handbrake);

float drivetrain_integrate_wheel(float angularVelocityRadS,
                                 float wheelLongitudinalVelocityMps,
                                 float driveTorqueNm,
                                 float serviceBrakeTorqueNm,
                                 float handbrakeTorqueNm,
                                 float tireLongitudinalForceN,
                                 float wheelRadiusM,
                                 float wheelInertiaKgM2,
                                 float dt,
                                 bool *locked);

/*
 * The wheel speed at which the pure longitudinal tire reaction exactly balances a steady
 * drive torque — the implicit solution of the stiff wheel equation. Returns false when no
 * stable equilibrium exists, i.e. the torque demands a reaction at or beyond the curve's
 * peak (genuine wheelspin or lockup, where the explicit dynamics are the right answer).
 *
 * Inverting force = longLimitN * sin(C * atan(B * k)) for the slip k that reacts
 * driveTorqueNm / wheelRadiusM:
 *
 *     k_eq      = tan(asin(normalized) / C) / B
 *     omega_eq  = (wheel_vx + k_eq * max(|wheel_vx|, slipSpeedEpsilonMps)) / R
 */
bool drivetrain_wheel_equilibrium_omega(float driveTorqueNm,
                                        float wheelLongitudinalVelocityMps,
                                        float wheelRadiusM,
                                        float longitudinalLimitN,
                                        float tireBLong, float tireCLong,
                                        float slipSpeedEpsilonMps,
                                        float *equilibriumOmegaRadS);

#endif /* DRIFTY_DRIVETRAIN_H */

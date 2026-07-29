/*
 * tire.h — pure normalized tire curves and per-wheel combined-friction limiting.
 *
 * This module uses SI forces and dimensionless slips. It calls no raylib function.
 */
#ifndef DRIFTY_TIRE_H
#define DRIFTY_TIRE_H

float tire_normalized_curve(float stiffnessB, float shapeC, float slip);

float tire_lateral_force_n(float slipAngleRad, float normalLoadN,
                           float stiffnessB, float shapeC,
                           float frictionCoefficient);

float tire_longitudinal_force_n(float slipRatio, float normalLoadN,
                                float stiffnessB, float shapeC,
                                float frictionCoefficient);

float tire_slip_ratio(float angularVelocityRadS, float wheelRadiusM,
                      float wheelLongitudinalVelocityMps,
                      float speedEpsilonMps, float slipClamp);

void tire_apply_combined_limit(float requestedLongitudinalForceN,
                               float requestedLateralForceN,
                               float longitudinalLimitN,
                               float lateralLimitN,
                               float *limitedLongitudinalForceN,
                               float *limitedLateralForceN,
                               float *frictionUsage);

#endif /* DRIFTY_TIRE_H */

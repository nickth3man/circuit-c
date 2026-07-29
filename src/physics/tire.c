#include "physics/tire.h"

#include <math.h>
#include <stddef.h>

#include "core/math_utils.h"

float tire_normalized_curve(float stiffnessB, float shapeC, float slip)
{
    if (!(isfinite(stiffnessB) && stiffnessB > 0.0f &&
          isfinite(shapeC) && shapeC > 0.0f && isfinite(slip))) return 0.0f;
    return sinf(shapeC * atanf(stiffnessB * slip));
}

float tire_lateral_force_n(float slipAngleRad, float normalLoadN,
                           float stiffnessB, float shapeC,
                           float frictionCoefficient)
{
    if (!(isfinite(normalLoadN) && normalLoadN > 0.0f &&
          isfinite(frictionCoefficient) && frictionCoefficient > 0.0f)) return 0.0f;
    return -frictionCoefficient * normalLoadN *
           tire_normalized_curve(stiffnessB, shapeC, slipAngleRad);
}

float tire_longitudinal_force_n(float slipRatio, float normalLoadN,
                                float stiffnessB, float shapeC,
                                float frictionCoefficient)
{
    if (!(isfinite(normalLoadN) && normalLoadN > 0.0f &&
          isfinite(frictionCoefficient) && frictionCoefficient > 0.0f)) return 0.0f;
    return frictionCoefficient * normalLoadN *
           tire_normalized_curve(stiffnessB, shapeC, slipRatio);
}

float tire_slip_ratio(float angularVelocityRadS, float wheelRadiusM,
                      float wheelLongitudinalVelocityMps,
                      float speedEpsilonMps, float slipClamp)
{
    if (!(isfinite(angularVelocityRadS) && isfinite(wheelRadiusM) &&
          wheelRadiusM > 0.0f && isfinite(wheelLongitudinalVelocityMps) &&
          isfinite(speedEpsilonMps) && speedEpsilonMps > 0.0f &&
          isfinite(slipClamp) && slipClamp > 0.0f)) return 0.0f;
    const float denominator = fmaxf(fabsf(wheelLongitudinalVelocityMps), speedEpsilonMps);
    const float ratio = (angularVelocityRadS * wheelRadiusM -
                         wheelLongitudinalVelocityMps) / denominator;
    return clampf(ratio, -slipClamp, slipClamp);
}

void tire_apply_combined_limit(float requestedLongitudinalForceN,
                               float requestedLateralForceN,
                               float longitudinalLimitN,
                               float lateralLimitN,
                               float *limitedLongitudinalForceN,
                               float *limitedLateralForceN,
                               float *frictionUsage)
{
    if (limitedLongitudinalForceN != NULL) *limitedLongitudinalForceN = 0.0f;
    if (limitedLateralForceN != NULL) *limitedLateralForceN = 0.0f;
    if (frictionUsage != NULL) *frictionUsage = 0.0f;

    const float longLimit = fabsf(longitudinalLimitN);
    const float latLimit = fabsf(lateralLimitN);
    if (!(isfinite(requestedLongitudinalForceN) &&
          isfinite(requestedLateralForceN) &&
          isfinite(longLimit) && longLimit > 0.0f &&
          isfinite(latLimit) && latLimit > 0.0f)) return;

    const float nx = requestedLongitudinalForceN / longLimit;
    const float ny = requestedLateralForceN / latLimit;
    const float usage = sqrtf(nx * nx + ny * ny);
    if (!isfinite(usage)) return;
    const float scale = (usage > 1.0f) ? 1.0f / usage : 1.0f;

    if (limitedLongitudinalForceN != NULL) {
        *limitedLongitudinalForceN = requestedLongitudinalForceN * scale;
    }
    if (limitedLateralForceN != NULL) {
        *limitedLateralForceN = requestedLateralForceN * scale;
    }
    if (frictionUsage != NULL) *frictionUsage = fminf(usage, 1.0f);
}

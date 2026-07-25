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
    spec->tireBLatFront = TIRE_B_LAT_FRONT;
    spec->tireCLatFront = TIRE_C_LAT_FRONT;
    spec->tireMuLatFront = TIRE_MU_LAT_FRONT;
    spec->tireBLatRear = TIRE_B_LAT_REAR;
    spec->tireCLatRear = TIRE_C_LAT_REAR;
    spec->tireMuLatRear = TIRE_MU_LAT_REAR;
    spec->tireBLong = TIRE_B_LONG;
    spec->tireCLong = TIRE_C_LONG;
    spec->tireMuLongScale = TIRE_MU_LONG_SCALE;

    /* Canonical Phase 2 fields remain deterministic and neutral until Phase 2. */
    spec->gearCount = 0;
    spec->drivetrainEfficiency = 0.0f;
    spec->brakeBiasFront = 0.0f;
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
    if (spec->gearCount < 0 || spec->gearCount > MAX_GEARS) return false;
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

    state->selectedGear = 1; /* Phase 1 direction selector: +1 forward, -1 reverse. */
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
    renderState->prevPositionM = state->positionM;
    renderState->currPositionM = state->positionM;
    renderState->prevHeadingRad = state->headingRad;
    renderState->currHeadingRad = state->headingRad;
}

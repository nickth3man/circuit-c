/*
 * vehicle.h — canonical vehicle structures and deterministic Phase 1 initialization.
 *
 * Physical values use SI units. This header uses raylib's Vector2 type but neither this
 * translation unit nor physics.c calls a raylib function.
 */
#ifndef DRIFTY_VEHICLE_H
#define DRIFTY_VEHICLE_H

#include <stdbool.h>

#include "raylib.h"

#include "config.h"

typedef enum {
    SURFACE_ASPHALT = 0,
    SURFACE_GRAVEL,
    SURFACE_GRASS,
    SURFACE_SNOW,
    SURFACE_COUNT
} SurfaceId;

typedef enum {
    WHEEL_FRONT_LEFT = 0,
    WHEEL_FRONT_RIGHT,
    WHEEL_REAR_LEFT,
    WHEEL_REAR_RIGHT,
    WHEEL_COUNT
} WheelId;

typedef struct {
    Vector2 localPositionM;
    float   steerAngleRad;
    float   angularVelocityRadS;
    float   normalLoadN;
    float   slipAngleRad;
    float   slipRatio;
    float   forceLongitudinalN;
    float   forceLateralN;
    float   frictionUsage;
    bool    locked;
    SurfaceId surfaceId;
} WheelState;

typedef struct {
    float massKg;
    float yawInertiaKgM2;
    float cgToFrontM;
    float cgToRearM;
    float wheelbaseM;
    float cgHeightM;
    float trackWidthFrontM;
    float trackWidthRearM;

    float wheelRadiusM;
    float wheelInertiaKgM2;

    float maxRoadWheelAngleRad;
    float maxSteerRateRadS;
    float steerReturnRateRadS;

    float dragCoefficient;
    float frontalAreaM2;
    float rollingResistanceCoefficient;

    float tireBLatFront, tireCLatFront, tireMuLatFront;
    float tireBLatRear,  tireCLatRear,  tireMuLatRear;
    float tireBLong, tireCLong, tireMuLongScale;
    float tireRelaxationLengthM;

    float gearRatios[MAX_GEARS];
    int   gearCount;
    float finalDriveRatio;
    float drivetrainEfficiency;
    float engineIdleRpm;
    float engineRedlineRpm;
    float engineTorqueCurveNm[ENGINE_CURVE_POINTS];
    float engineBrakingTorqueNm;

    float maxBrakeTorqueNm;
    float brakeBiasFront;
    float handbrakeTorqueNm;
} VehicleSpec;

typedef struct {
    Vector2 positionM;
    float   headingRad;
    float   velocityLongitudinalMps;
    float   velocityLateralMps;
    float   yawRateRadS;
    float   frontRoadWheelAngleRad;
    float   engineRpm;
    int     selectedGear;
    float   filteredLongAccelMps2;
    float   prevLongAccelMps2;
    WheelState wheels[WHEEL_COUNT];
} VehicleState;

typedef struct {
    float   bodySideslipRad;
    float   longitudinalAccelerationMps2;
    float   lateralAccelerationMps2;
    float   speedMps;
    float   normalLoadFrontN;
    float   normalLoadRearN;
    Vector2 totalBodyForceN;
    float   totalYawTorqueNm;
    float   maxFrictionUsage;
    float   lowSpeedBlend;
    bool    physicallySliding;
    bool    scoringDrift;

    /* Phase 1 diagnostics used by tests, telemetry, and the debug vectors. */
    Vector2 wheelContactVelocityBodyMps[WHEEL_COUNT];
    Vector2 frontAxleContactVelocityBodyMps;
    Vector2 rearAxleContactVelocityBodyMps;
    float   frontSlipAngleRad;
    float   rearSlipAngleRad;
    float   frontLateralForceN;
    float   rearLateralForceN;
    Vector2 frontBodyForceN;
    Vector2 rearBodyForceN;
} VehicleDerived;

typedef struct {
    Vector2 prevPositionM;
    float   prevHeadingRad;
    float   prevWheelAngleRad[WHEEL_COUNT];
    Vector2 currPositionM;
    float   currHeadingRad;
    float   currWheelAngleRad[WHEEL_COUNT];
} VehicleRenderState;

void vehicle_spec_set_default(VehicleSpec *spec);
bool vehicle_spec_is_valid(const VehicleSpec *spec);
void vehicle_state_reset(const VehicleSpec *spec,
                         VehicleState *state,
                         VehicleDerived *derived,
                         VehicleRenderState *renderState);

#endif /* DRIFTY_VEHICLE_H */

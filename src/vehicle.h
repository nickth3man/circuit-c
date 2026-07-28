/*
 * vehicle.h — canonical vehicle structures and deterministic Phase 2 initialization.
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

typedef enum {
    DIFF_LOCKED = 0,   /* both rear wheels share one omega; equal torque split */
    DIFF_OPEN,         /* equal torque split; independent rear wheel speeds    */
    DIFF_LSD           /* torque-biasing clutch: biasRatio + preload            */
} DifferentialMode;

typedef struct {
    Vector2 localPositionM;
    float   steerAngleRad;
    float   angularVelocityRadS;
    float   normalLoadN;
    float   slipAngleRad;
    float   slipRatio;
    float   forceLongitudinalN;
    float   forceLateralN;
    float   forceLateralRelaxedN;   /* N; persistent relaxation state (Phase 4 feature 6) */
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
    float loadFilterRateHz;

    float tireBLatFront, tireCLatFront, tireMuLatFront;
    float tireBLatRear,  tireCLatRear,  tireMuLatRear;
    float tireBLong, tireCLong, tireMuLongScale;
    float tireRelaxationLengthM;

    float gearRatios[MAX_GEARS];
    int   gearCount;
    float reverseGearRatio;
    float finalDriveRatio;
    float drivetrainEfficiency;
    float engineIdleRpm;
    float engineRedlineRpm;
    float engineTorqueCurveNm[ENGINE_CURVE_POINTS];
    float engineBrakingTorqueNm;

    float maxBrakeTorqueNm;
    float brakeBiasFront;
    float handbrakeTorqueNm;

    /* ---------------------------------------------------------------- Phase 5 collision -- */
    float bodyHalfWidthM;        /* m; collision capsule circle radius */
    float collisionRestitution;  /* dimensionless; barrier bounce */
    float collisionFriction;     /* dimensionless; Coulomb friction at impact */

    /* ---------------------------------------------------------------- Phase 4 four-wheel -- */
    float tireLoadSensitivityK;        /* dimensionless; exponent mu_eff=mu*(Fz/FzRef)^-k */
    float tireLoadRefPerWheelN;        /* N; reference load for load-sensitivity curve */
    float ackermannPercent;            /* dimensionless 0..1; 0=parallel, 1=true Ackermann */
    float differentialMode;            /* 0=LOCKED, 1=OPEN, 2=LSD; cast to enum at use */
    float differentialBiasRatio;       /* dimensionless; LSD slower/faster torque cap */
    float differentialPreloadNm;       /* N*m; LSD clutch preload */
    float rollStiffnessFrontFraction;  /* dimensionless 0..1; front axle share of roll moment */
    bool  lateralLoadTransferEnabled;  /* master switch for lateral load transfer */
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
    float filteredLongAccelMps2;
    float prevLongAccelMps2;

    /* Phase 4 lateral-acceleration filter state for lateral load transfer */
    float filteredLatAccelMps2;
    float prevLatAccelMps2;

    WheelState wheels[WHEEL_COUNT];
} VehicleState;

typedef struct {
    float   bodySideslipRad;
    float   longitudinalAccelerationMps2;
    float   lateralAccelerationMps2;
    float   speedMps;

    /* The dynamic axle loads that actually fed the tire model this step: static split plus
     * longitudinal transfer, then floored at MIN_NORMAL_LOAD_N. Phase 3 gives these their
     * physical meaning; the unclamped pair below is kept for diagnosis. */
    float   normalLoadFrontN;
    float   normalLoadRearN;

    Vector2 totalBodyForceN;
    float   totalYawTorqueNm;
    float   maxFrictionUsage;
    float   lowSpeedBlend;
    bool    physicallySliding;
    bool    scoringDrift;

    /* Phase 1/2 diagnostics used by tests, telemetry, and the debug overlay. */
    Vector2 wheelContactVelocityBodyMps[WHEEL_COUNT];
    Vector2 frontAxleContactVelocityBodyMps;
    Vector2 rearAxleContactVelocityBodyMps;
    float   frontSlipAngleRad;
    float   rearSlipAngleRad;
    float   frontLateralForceN;
    float   rearLateralForceN;
    Vector2 frontBodyForceN;
    Vector2 rearBodyForceN;
    float   pureLongitudinalForceN[WHEEL_COUNT];
    float   pureLateralForceN[WHEEL_COUNT];
    float   driveTorqueNm[WHEEL_COUNT];
    float   serviceBrakeTorqueNm[WHEEL_COUNT];
    float   handbrakeTorqueNm[WHEEL_COUNT];
    float   engineTorqueNm;
    float   totalGearRatio;
    float   drivelineTorqueNm;

    /* ---------------------------------------------------------------- Phase 3 diagnostics --
     *
     * All recomputed every fixed update and never integrated, so none of them belongs in
     * VehicleState and none of them is in the state checksum. The only Phase 3 values that
     * ARE persistent — and therefore checksummed — are prevLongAccelMps2 and
     * filteredLongAccelMps2 in VehicleState.
     */
    float   staticFrontLoadN;        /* m*g*l_r/L */
    float   staticRearLoadN;         /* m*g*l_f/L */
    float   unclampedFrontLoadN;     /* static -/+ transfer, before MIN_NORMAL_LOAD_N */
    float   unclampedRearLoadN;      /* the pair always sums to m*g */
    float   loadTransferN;           /* m * filtered ax * h / L; positive = rearward */

    float   previousLongAccelMps2;   /* the value the filter consumed this step */
    float   filteredLongAccelMps2;   /* the filtered value load transfer used this step */
    float   solvedLongAccelMps2;     /* this step's solved body ax, stored for the next one */

    float   aeroDragMagnitudeN;
    Vector2 aeroDragBodyN;           /* body frame; opposes the full velocity vector */
    float   rollingResistanceMagnitudeN;   /* sum of the four wheel magnitudes */
    Vector2 rollingResistanceBodyN;        /* body frame; per-wheel sum */
    float   wheelRollingResistanceN[WHEEL_COUNT];

    /* ---------------------------------------------------------------- Phase 4 diagnostics --
     *
     * All recomputed every fixed update and never integrated. None of them is in the state
     * checksum. The logic that fills them is wired in subsequent sub-steps. */
    float   lateralLoadTransferFrontN;            /* N; inner->outer delta on front axle */
    float   lateralLoadTransferRearN;             /* N; inner->outer delta on rear axle  */
    float   rollMomentNm;                         /* N*m; m*ay*h */
    float   tireLoadSensitivityMuScale[WHEEL_COUNT];  /* dimensionless; per-wheel (Fz/FzRef)^-k */
    float   differentialOmegaRadS[2];             /* rad/s; {omega_RL, omega_RR} post-diff */
    float   differentialTorqueNm[2];              /* N*m; {T_RL, T_RR} post-redistribution  */
    Vector2 looseSurfaceDragBodyN;                /* N; summed per-wheel loose-surface drag */
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

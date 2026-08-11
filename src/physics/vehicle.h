/*
 * vehicle.h — canonical vehicle structures and deterministic Phase 2 initialization.
 *
 * Physical values use SI units. This header uses raylib's Vector2 type but neither this
 * translation unit nor physics.c calls a raylib function.
 */
#ifndef CIRCUIT_VEHICLE_H
#define CIRCUIT_VEHICLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "raylib.h"

#include "core/config.h"

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
    DIFF_LOCKED = 0, /* both rear wheels share one omega; equal torque split */
    DIFF_OPEN,       /* equal torque split; independent rear wheel speeds    */
    DIFF_LSD         /* torque-biasing clutch: biasRatio + preload            */
} DifferentialMode;

/* Which axle(s) the driveline connects to. VehicleSpec stores this as the float tunable
 * drivetrainLayout so profiles can set it; cast with (DrivetrainLayout)(int) to compare. */
typedef enum {
    DRIVE_LAYOUT_RWD = 0, /* rear axle driven                                  */
    DRIVE_LAYOUT_FWD,     /* front axle driven                                 */
    DRIVE_LAYOUT_AWD      /* both axles; frontTorqueSplit sets the front share */
} DrivetrainLayout;

typedef enum { VEH_ROOF_FIXED = 0, VEH_ROOF_TARGA, VEH_ROOF_CONVERTIBLE } VehicleRoofType;

typedef struct {
    Vector2 localPositionM;
    float steerAngleRad;
    float angularVelocityRadS;
    float normalLoadN;
    float slipAngleRad;
    float slipRatio;
    float forceLongitudinalN;
    float forceLateralN;
    float forceLateralRelaxedN;      /* N; persistent lateral relaxation state (#20) */
    float forceLongitudinalRelaxedN; /* N; persistent longitudinal relaxation state (#20) */
    float frictionUsage;
    bool locked;
    SurfaceId surfaceId;
} WheelState;

typedef struct {
    /* ---- primary layout ---- */
    float wheelbaseM; /* primary; cg distances derive from this + mass particles */
    float trackWidthFrontM;
    float trackWidthRearM;
    float frontOverhangM;
    float rearOverhangM;
    float widthOverallM;
    float heightOverallM;
    float rideHeightFrontM;
    float rideHeightRearM;
    float cowlXM;            /* layout-frame X of cowl / windscreen foot */
    float backlightXM;       /* layout-frame X of backlight / hatch foot */
    float bedLengthM;        /* open cargo bed forward from the tail; 0 = none */
    float noseWidthM;        /* width at the foremost hull station (full width, m) */
    float tailWidthM;        /* width at the rearmost hull station (full width, m) */
    float shoulderXM;        /* layout-frame station of maximum body width */
    float fenderFlareFrontM; /* arch flare proud of the hull, front axle (m) */
    float fenderFlareRearM;  /* arch flare proud of the hull, rear axle (m) */

    /* ---- greenhouse (primary; Phase C) ---- */
    float roofStartXM;          /* forward roof edge, layout frame */
    float roofEndXM;            /* aft roof edge, layout frame */
    float roofWidthM;           /* full physical roof width */
    float windscreenRakeRad;    /* angle from vertical; also modifies effective Cd */
    float backlightRakeRad;     /* angle from vertical */
    float sideWindowCount;      /* integer 2..6 */
    float quarterWindowLengthM; /* 0 or 0.2..0.4 m */
    float sunroofLengthM;       /* 0 or 0.4..1.0 m */
    float doorCount;            /* exactly 2, 4, or 5 */
    float cabinRows;            /* integer 1..3; packages rearward from massDriverXM */
    float roofType;             /* VehicleRoofType stored as float for the registry */

    /* ---- mass particles (layout frame: axle midpoint origin, +X forward) ---- */
    float massEngineKg, massEngineXM, massEngineZM;
    float massGearboxKg, massGearboxXM, massGearboxZM;
    float massFuelKg, massFuelXM, massFuelZM;
    float massDriverKg, massDriverXM, massDriverZM;
    float massChassisKg, massChassisXM, massChassisZM;

    /* ---- tire designation (primary) ---- */
    float tireSectionWidthFrontMm, tireSectionWidthRearMm;
    float tireAspectFrontPct, tireAspectRearPct;
    float tireRimDiameterFrontIn, tireRimDiameterRearIn;
    float tireRimWidthFrontIn, tireRimWidthRearIn;
    float tirePressureFrontKpa, tirePressureRearKpa;

    /* ---- suspension / stance (primary) ---- */
    float suspCamberFrontRad, suspCamberRearRad;
    float suspToeFrontRad, suspToeRearRad;
    float suspCasterFrontRad, suspCasterRearRad;
    float suspWheelRateFrontNpm, suspWheelRateRearNpm;
    float suspAntiRollFrontNpm, suspAntiRollRearNpm;
    float suspTravelFrontM, suspTravelRearM;
    float suspRollCentreFrontM, suspRollCentreRearM;

    /* ---- wheel offset + brake hardware (primary) ---- */
    float wheelOffsetEtFrontMm, wheelOffsetEtRearMm;
    float brakeDiscRadiusFrontM, brakeDiscRadiusRearM;
    float brakePadFriction;

    /* ---- aero (primary) ---- */
    float aeroLiftCoefFront, aeroLiftCoefRear;
    float aeroRefAreaFrontM2, aeroRefAreaRearM2;
    float aeroCentreOfPressureXM;

    /* ---- layout / engine packaging (primary) ---- */
    float drivetrainLayout; /* DrivetrainLayout: RWD/FWD/AWD */
    float frontTorqueSplit; /* 0..1 front share when AWD */
    float engineCylinders;
    float engineDisplacementL;

    /* ---- derived dynamics (filled by dev_params_refresh_derived) ---- */
    float massKg;
    float yawInertiaKgM2;
    float cgToFrontM;
    float cgToRearM;
    float cgHeightM;
    float lengthOverallM;
    float wheelRadiusFrontM;
    float wheelRadiusRearM;
    float wheelRadiusM; /* legacy alias: equals wheelRadiusRearM after refresh */
    float wheelInertiaKgM2;
    float frontalAreaM2;
    float bodyHalfWidthM;
    float maxBrakeTorqueNm;
    float rollStiffnessFrontFraction;
    float tireRelaxationLengthM;
    float tireLongRelaxationLengthM;
    float tireLoadRefPerWheelN;

    float maxRoadWheelAngleRad;
    float maxSteerRateRadS;
    float steerReturnRateRadS;
    float steerSpeedRefMps;    /* m/s; below this, full-rate steering */
    float steerSpeedMinFactor; /* dimensionless; minimum rate factor at high speed */

    float dragCoefficient;
    float loadFilterRateHz;

    float tireBLatFront, tireCLatFront, tireMuLatFront;
    float tireBLatRear, tireCLatRear, tireMuLatRear;
    float tireBLong, tireCLong, tireMuLongScale;

    float gearRatios[MAX_GEARS];
    int gearCount;
    float reverseGearRatio;
    float finalDriveRatio;
    float drivetrainEfficiency;
    float engineIdleRpm;
    float engineRedlineRpm;
    float engineTorqueCurveNm[ENGINE_CURVE_POINTS];
    float engineBrakingTorqueNm;

    float brakeBiasFront;
    float handbrakeTorqueNm;

    /* ---------------------------------------------------------------- Phase 5 collision -- */
    float collisionRestitution; /* dimensionless; barrier bounce */
    float collisionFriction;    /* dimensionless; Coulomb friction at impact */

    /* ---------------------------------------------------------------- Phase 4 four-wheel -- */
    float tireLoadSensitivityK;      /* dimensionless; exponent mu_eff=mu*(Fz/FzRef)^-k */
    float ackermannPercent;          /* dimensionless 0..1; 0=parallel, 1=true Ackermann */
    float differentialMode;          /* 0=LOCKED, 1=OPEN, 2=LSD; cast to enum at use */
    float differentialBiasRatio;     /* dimensionless; LSD slower/faster torque cap */
    float differentialPreloadNm;     /* N*m; LSD clutch preload */
    bool lateralLoadTransferEnabled; /* master switch for lateral load transfer */
} VehicleSpec;

/* Effective rolling radius for a wheel index. Safe with a NULL spec (returns 0). */
static inline float vehicle_wheel_radius_m(const VehicleSpec *spec, int wheelId)
{
    if (spec == NULL) return 0.0f;
    if (wheelId == WHEEL_FRONT_LEFT || wheelId == WHEEL_FRONT_RIGHT) {
        return spec->wheelRadiusFrontM;
    }
    return spec->wheelRadiusRearM;
}

typedef struct {
    Vector2 positionM;
    float headingRad;
    float velocityLongitudinalMps;
    float velocityLateralMps;
    float yawRateRadS;
    float frontRoadWheelAngleRad;
    float engineRpm;
    int selectedGear;
    float filteredLongAccelMps2;
    float prevLongAccelMps2;

    /* Phase 4 lateral-acceleration filter state for lateral load transfer */
    float filteredLatAccelMps2;
    float prevLatAccelMps2;

    WheelState wheels[WHEEL_COUNT];
} VehicleState;

typedef struct {
    float bodySideslipRad;
    float longitudinalAccelerationMps2;
    float lateralAccelerationMps2;
    float speedMps;

    /* The dynamic axle loads that actually fed the tire model this step: static split plus
     * longitudinal transfer, then floored at MIN_NORMAL_LOAD_N. Phase 3 gives these their
     * physical meaning; the unclamped pair below is kept for diagnosis. */
    float normalLoadFrontN;
    float normalLoadRearN;

    Vector2 totalBodyForceN;
    float totalYawTorqueNm;
    float maxFrictionUsage;
    float lowSpeedBlend;
    bool physicallySliding;

    /* Phase 1/2 diagnostics used by tests, telemetry, and the debug overlay. */
    Vector2 wheelContactVelocityBodyMps[WHEEL_COUNT];
    Vector2 frontAxleContactVelocityBodyMps;
    Vector2 rearAxleContactVelocityBodyMps;
    float frontSlipAngleRad;
    float rearSlipAngleRad;
    float frontLateralForceN;
    float rearLateralForceN;
    Vector2 frontBodyForceN;
    Vector2 rearBodyForceN;
    float pureLongitudinalForceN[WHEEL_COUNT];
    float pureLateralForceN[WHEEL_COUNT];
    float driveTorqueNm[WHEEL_COUNT];
    float serviceBrakeTorqueNm[WHEEL_COUNT];
    float handbrakeTorqueNm[WHEEL_COUNT];
    float engineTorqueNm;
    float totalGearRatio;
    float drivelineTorqueNm;

    /* ---------------------------------------------------------------- Phase 3 diagnostics --
     *
     * All recomputed every fixed update and never integrated, so none of them belongs in
     * VehicleState and none of them is in the state checksum. The only Phase 3 values that
     * ARE persistent — and therefore checksummed — are prevLongAccelMps2 and
     * filteredLongAccelMps2 in VehicleState.
     */
    float staticFrontLoadN;    /* m*g*l_r/L */
    float staticRearLoadN;     /* m*g*l_f/L */
    float aeroVerticalFrontN;  /* N; + = downforce on the front axle, - = lift (issue #17) */
    float aeroVerticalRearN;   /* N; same sign convention on the rear axle */
    float unclampedFrontLoadN; /* static + aero -/+ transfer, before MIN_NORMAL_LOAD_N */
    float unclampedRearLoadN;  /* the pair always sums to m*g + the two aero terms */
    float loadTransferN;       /* m * filtered ax * h / L; positive = rearward */

    float previousLongAccelMps2; /* the value the filter consumed this step */
    float filteredLongAccelMps2; /* the filtered value load transfer used this step */
    float solvedLongAccelMps2;   /* this step's solved body ax, stored for the next one */

    float aeroDragMagnitudeN;
    Vector2 aeroDragBodyN;             /* body frame; opposes the full velocity vector */
    float rollingResistanceMagnitudeN; /* sum of the four wheel magnitudes */
    Vector2 rollingResistanceBodyN;    /* body frame; per-wheel sum */
    float wheelRollingResistanceN[WHEEL_COUNT];

    /* ---------------------------------------------------------------- Phase 4 diagnostics --
     *
     * All recomputed every fixed update and never integrated. None of them is in the state
     * checksum. The logic that fills them is wired in subsequent sub-steps. */
    float lateralLoadTransferFrontN;               /* N; inner->outer delta on front axle */
    float lateralLoadTransferRearN;                /* N; inner->outer delta on rear axle  */
    float rollMomentNm;                            /* N*m; m*ay*h */
    float tireLoadSensitivityMuScale[WHEEL_COUNT]; /* dimensionless; per-wheel (Fz/FzRef)^-k */
    float differentialOmegaRadS[2];                /* rad/s; {omega_RL, omega_RR} post-diff */
    float differentialTorqueNm[2];                 /* N*m; {T_RL, T_RR} post-redistribution  */
    Vector2 looseSurfaceDragBodyN;                 /* N; summed per-wheel loose-surface drag */

    /* ---------------------------------------------------------------- Phase 5 diagnostics --
     *
     * Camber/caster (#15): effective camber per wheel and the resulting lateral force,
     * separated from the slip-angle force. Recomputed every fixed update, never integrated,
     * excluded from the state checksum. */
    float effectiveCamberRad[WHEEL_COUNT]; /* rad; static + caster-induced, SAE convention */
    float aligningMomentNm
        [WHEEL_COUNT];                /* N*m; self-aligning moment from pneumatic trail (#20) */
    float camberThrustN[WHEEL_COUNT]; /* N; camber-derived lateral force per wheel */

    /* Which solver stage first produced a non-finite state, as a PhysicsStage value, or 0 when
     * the last step was healthy. Diagnostic only: it is recomputed every step and, like the
     * rest of VehicleDerived, is excluded from the state checksum. physics_stage_name() turns
     * it into something readable. */
    int solverFailedStage;
} VehicleDerived;

typedef struct {
    Vector2 prevPositionM;
    float prevHeadingRad;
    float prevWheelAngleRad[WHEEL_COUNT];
    Vector2 currPositionM;
    float currHeadingRad;
    float currWheelAngleRad[WHEEL_COUNT];
} VehicleRenderState;

#define VEHICLE_CONTENT_ID_CAPACITY 64

typedef enum { AUTO_DRIVE = 0, AUTO_NEUTRAL = 1, AUTO_REVERSE = 2 } AutoDriveState;

typedef struct {
    bool enabled;
    bool forwardOnly;
    AutoDriveState driveState;
    float neutralTimer;
} AutoTransmission;

/* Immutable, shareable vehicle content. `spec` is never used as mutable runtime storage;
 * callers derive an entrant-local compiled spec at the boundary below. Appearance identity is
 * a reference rather than render state, so deterministic presentation can resolve it without
 * putting textures or module-owned pointers in the simulation. */
typedef struct {
    char id[VEHICLE_CONTENT_ID_CAPACITY];
    uint32_t contentVersion;
    uint32_t contentHash;
    char appearanceId[VEHICLE_CONTENT_ID_CAPACITY];
    VehicleSpec spec;
} VehicleDefinition;

/* The currently supported entrant adjustments. Issue 12 may classify more definition fields
 * as adjustable, but adding speculative setup controls here would make today's validation
 * contract dishonest. */
typedef struct {
    float tirePressureFrontKpa, tirePressureRearKpa;
    float suspCamberFrontRad, suspCamberRearRad;
    float suspToeFrontRad, suspToeRearRad;
    float suspCasterFrontRad, suspCasterRearRad;
    float gearRatios[MAX_GEARS];
    int gearCount;
    float reverseGearRatio;
    float finalDriveRatio;
    float brakeBiasFront;
    float differentialMode;
    float differentialBiasRatio;
    float differentialPreloadNm;
} VehicleSetup;

typedef struct {
    float steer;
    float throttle;
    float brake;
    float handbrake;
} VehicleControlState;

typedef struct {
    float pressureKpa;
    float temperatureC;
    float wear;
} VehicleTireState;

/* One entrant's complete mutable vehicle state. `spec` is an entrant-local compiled cache,
 * never an alias of VehicleDefinition memory. Its field order deliberately supports Game's
 * temporary one-entrant compatibility view while callers migrate subsystem by subsystem. */
typedef struct {
    VehicleSpec spec;
    VehicleState vehicle;
    VehicleDerived derived;
    VehicleRenderState renderState;
    AutoTransmission autoTrans;
    VehicleControlState vehicleControls;
    float fuelKg;
    VehicleTireState tireState[WHEEL_COUNT];
    float damage;
    float crashLockoutTimerS;
} VehicleInstance;

void vehicle_spec_set_default(VehicleSpec *spec);
/* Staged recompute of derived VehicleSpec fields (dimensions → mass → tires →
 * suspension → brakes). Safe to call repeatedly; does nothing on NULL. */
void vehicle_spec_refresh_derived(VehicleSpec *spec);
bool vehicle_spec_is_valid(const VehicleSpec *spec);
/* Base dragCoefficient adjusted by the bounded, default-neutral windscreen-rake factor. */
float vehicle_effective_drag_coefficient(const VehicleSpec *spec);
void vehicle_state_reset(const VehicleSpec *spec, VehicleState *state, VehicleDerived *derived,
                         VehicleRenderState *renderState);

bool vehicle_definition_init(VehicleDefinition *definition, const char *id,
                             const char *appearanceId, uint32_t contentVersion,
                             const VehicleSpec *spec);
void vehicle_definition_set_default(VehicleDefinition *definition);
void vehicle_setup_set_default(const VehicleDefinition *definition, VehicleSetup *setup);
bool vehicle_setup_is_valid(const VehicleDefinition *definition, const VehicleSetup *setup);
/* Deterministic serialization anchor for saved/replayed setups (issue #33): FNV-1a over the
 * setup fields in the same order game_state_checksum hashes them, so the same bytes yield the
 * same hash on every platform. NULL hashes as a zeroed setup. */
uint32_t vehicle_setup_hash(const VehicleSetup *setup);
/* The sole definition/setup -> compiled-runtime recomputation boundary. It changes no mutable
 * simulation state, so callers may choose whether a setup change also resets the entrant. */
bool vehicle_instance_derive(VehicleInstance *instance, const VehicleDefinition *definition,
                             const VehicleSetup *setup);
void vehicle_instance_reset(VehicleInstance *instance);
bool vehicle_instance_init(VehicleInstance *instance, const VehicleDefinition *definition,
                           const VehicleSetup *setup);

#endif /* CIRCUIT_VEHICLE_H */

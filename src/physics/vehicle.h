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
    /* 0/1 master switch for the bulk tire thermal model (#21). 0 (default) keeps the
     * temperature/pressure dynamics off and reproduces the baseline exactly; 1 evolves
     * per-wheel temperature from slip/rolling heat and maps temperature + live pressure to
     * bounded grip/stiffness effects. */
    float tireThermalEnabled;
    /* 0/1 master switch for tire wear (#22). 0 (default) keeps wear pinned and the baseline
     * exact; 1 accumulates per-wheel wear monotonically from slip energy and surface
     * abrasion, degrading grip continuously to a bounded floor. */
    float tireWearEnabled;
    /* Dynamic engine model (#23). 0 (default) keeps the kinematic engine (RPM derived from
     * wheel speed, instantaneous shifts) and the baseline exact; > 0 gives the engine real
     * rotational inertia, a clutch with slip, and phased shifts with torque interruption. */
    float engineInertiaKgM2; /* kg*m^2; 0 = kinematic engine (baseline), >0 = dynamic (#23) */
    float maxClutchTorqueNm; /* N*m; clutch torque capacity while slipping */
    float shiftDurationS;    /* s; total phased-shift window (cut, swap, engage) */
    /* Fuel model (#24). 0 (default) pins fuelKg at its initial load and keeps the baseline
     * exact; 1 consumes fuel proportional to engine work and feeds the live fuel mass back
     * into mass/CG/inertia via vehicle_spec_set_fuel_mass(). */
    float fuelEnabled;
    float fuelTankCapacityL;          /* litres; upper bound for service/refuel operations */
    float fuelConsumptionRateKgPerWS; /* kg per joule of engine work (BSFC-derived) */
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
    /* Bump-stop engagement fractions per axle, 0..1 (issue #19). Derived each tick from the
     * elastic load share's compression against the travel limit, and consumed as the axle's
     * effective roll-stiffness modifier on the NEXT tick — the documented one-tick lag of the
     * reduced-order quasi-static model. Authoritative (checksummed, replayed). */
    float bumpStopFracFront;
    float bumpStopFracRear;
    /* Dynamic-engine/clutch state (#23). Zeroed by default = kinematic engine, which is what
     * keeps every recorded baseline. `shiftPhase`/`shiftTimerS`/`shiftTargetGear` drive the
     * phased-shift machine; clutch engagement is derived from them (1 when not shifting). */
    int shiftPhase;      /* 0 none, 1 cutting (clutch opening), 2 engaging (clutch closing) */
    float shiftTimerS;   /* seconds into the current phase */
    int shiftTargetGear; /* gear applied at the cutting->engaging boundary */
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
    float
        lateralLoadTransferFrontN;  /* N; inner->outer delta on front axle, elastic+geometric */
    float lateralLoadTransferRearN; /* N; inner->outer delta on rear axle, elastic+geometric  */
    /* The two routes the roll moment takes to the contact patch, reported separately so a
     * setup change can be attributed to bars or to linkage geometry (issue #18). */
    float lateralLoadTransferElasticFrontN;
    float lateralLoadTransferElasticRearN;
    float lateralLoadTransferGeometricFrontN;
    float lateralLoadTransferGeometricRearN;
    float rollAxisHeightAtCgM; /* m; roll centres interpolated at the CG */
    float rollMomentNm;        /* N*m; m*ay*h */
    /* Suspension travel diagnostics (issue #19): per-wheel compression from the elastic load
     * share, wheel-contact status, and bump-stop engagement. Recomputed every tick, never
     * integrated, excluded from the checksum. */
    float suspCompressionM[WHEEL_COUNT]; /* m; positive compression, negative droop */
    bool wheelContact[WHEEL_COUNT];      /* false when the wheel has left contact (droop) */
    bool bumpStopEngaged[WHEEL_COUNT];   /* true when the wheel is past full compression */
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
    float tireTemperatureGripMultiplier[WHEEL_COUNT]; /* dimensionless; 1.0 when thermal off */
    float tireWearGripMultiplier[WHEEL_COUNT];        /* dimensionless; 1.0 when wear off */
    bool absActive; /* assist diagnostics (#25): an intervention is being applied this tick */
    bool tcsActive;

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
    /* AI eligibility policy (issue #52): content may declare itself unsuitable for the AI
     * driver (e.g. appearance corpus). `race_roster_spawn` refuses an AI controller on an
     * ineligible definition. Defaults to true for built-ins. Not part of the physics content
     * hash: it is eligibility policy, not physics. */
    bool aiEligible;
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
    /* Driver assists (#25): 0 = off, 1 = light, 2 = strong. Pedal-level deterministic
     * controllers; off preserves the baseline exactly. */
    int absLevel;
    int tcsLevel;
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
/* Deterministic tire service hook (issue #22, consumed by future pit rules): `replace` resets
 * every wheel's wear to 0; when `replace` is false only pressure/temperature are restored to
 * their cold nominal/ambient values. Never touches the vehicle pose or any other state. */
void vehicle_tire_service(VehicleInstance *instance, bool replace);
/* Recompute the spec's mass/CG/yaw-inertia from its mass particles with the fuel particle
 * set to `fuelKg` (issue #24). Deterministic; leaves every other spec field untouched. */
void vehicle_spec_set_fuel_mass(VehicleSpec *spec, float fuelKg);
/* Deterministic refuel service hook (#24, consumed by future pit rules): adds up to `litres`
 * (converted at fuelDensityKgPerL) without exceeding the tank capacity. */
void vehicle_refuel(VehicleInstance *instance, float litres);
bool vehicle_instance_init(VehicleInstance *instance, const VehicleDefinition *definition,
                           const VehicleSetup *setup);

#endif /* CIRCUIT_VEHICLE_H */

/*
 * physics.h — pure vehicle calculations and fixed-update ownership.
 *
 * Phase 3 adds the load-transfer stage (a first-order filter on the previous step's solved
 * body-longitudinal acceleration, then static-plus-dynamic axle loads) and the two separated
 * resistance forces. The individual stages are exposed so the headless suite can assert each
 * equation on its own rather than only through a whole-vehicle run.
 */
#ifndef CIRCUIT_PHYSICS_H
#define CIRCUIT_PHYSICS_H

#include <stdbool.h>

#include "game/controller_output.h"
#include "physics/drivetrain.h" /* DrivetrainTorques, DifferentialMode */
#include "physics/vehicle.h"

typedef struct {
    float velocityLongitudinalDerivativeMps2;
    float velocityLateralDerivativeMps2;
    float yawRateDerivativeRadS2;
} VehicleDerivatives;

/* One axle-load solution: the static split, the aerodynamic load added to each axle, the
 * longitudinal transfer between them, the unclamped result (which always sums to mass * g plus
 * the two aerodynamic terms), and the clamped loads the tires actually see. */
typedef struct {
    float staticFrontN;
    float staticRearN;
    float aeroFrontN; /* positive = downforce; negative = lift unloading the axle */
    float aeroRearN;
    float transferN; /* positive moves load rearward */
    float unclampedFrontN;
    float unclampedRearN;
    float frontN; /* floored at MIN_NORMAL_LOAD_N */
    float rearN;
} AxleLoads;

Vector2 physics_contact_point_velocity_body(const VehicleState *state, Vector2 pointM);
Vector2 physics_wheel_world_position(const VehicleState *state, WheelId wheelId);
void physics_static_axle_loads(const VehicleSpec *spec, float *frontLoadN, float *rearLoadN);

/* One step of the first-order load-transfer filter, in isolation.
 *   filtered += (previous - filtered) * (1 - exp(-rateHz * dt))
 * Returns `filtered` unchanged for a non-positive dt or a non-finite input. */
float physics_filter_long_accel(float filteredMps2, float previousMps2, float rateHz, float dt);

/*
 * Aerodynamic vertical load per axle at a given road speed, in newtons.
 *
 * SIGN. aeroLiftCoef* are LIFT coefficients, so the returned value is
 * -0.5 * rho * Cl * A * v^2: a positive authored coefficient lifts the axle and returns a
 * NEGATIVE load, a negative coefficient (a wing) returns positive downforce. The two axles use
 * their own coefficient and their own reference area, which is what fixes the aerodynamic
 * balance — there is no separate balance fraction to disagree with them.
 *
 * `speedMps` is the magnitude of the road-relative velocity, matching the drag model: a car
 * travelling sideways is still moving through the same air. Below RESISTANCE_EPSILON_MPS, and
 * for any non-finite or negative-area input, both outputs are zero, so a no-aero vehicle and a
 * stationary one are bit-identical to the model without this term.
 */
void physics_aero_vertical_loads(const VehicleSpec *spec, float speedMps, float *frontN,
                                 float *rearN);

/* Static-plus-aerodynamic-plus-dynamic axle loads, for a given filtered longitudinal
 * acceleration and road speed. */
AxleLoads physics_axle_loads(const VehicleSpec *spec, float filteredLongAccelMps2,
                             float speedMps);

/* Aerodynamic drag in the body frame, opposing the full velocity vector. *magnitudeN
 * receives the scalar 0.5*rho*Cd*A*v^2 when non-NULL. */
Vector2 physics_aero_drag_body_n(const VehicleSpec *spec, float velocityLongitudinalMps,
                                 float velocityLateralMps, float *magnitudeN);

/* Rolling resistance for one wheel, opposing its contact-point velocity. Returns the body-
 * frame force; *magnitudeN receives its length. Zero at rest, with no direction invented. */
Vector2 physics_rolling_resistance_body_n(float rollingResistanceCoefficient, float normalLoadN,
                                          Vector2 contactVelocityMps, float *magnitudeN);
/* Static toe as a signed per-wheel heading offset. `toeRad` is positive for toe-IN and
 * `wheelLateralPositionM` is the wheel's body-frame y (positive on the left of the car), so
 * the same authored value produces mirrored offsets on the two sides of an axle. */
float physics_wheel_toe_offset_rad(float toeRad, float wheelLateralPositionM);

void physics_update_steering(const VehicleSpec *spec, VehicleState *state, float steerInput,
                             float dt);
void physics_axle_slip_angles(const VehicleSpec *spec, const VehicleState *state,
                              float *frontSlipAngleRad, float *rearSlipAngleRad);
Vector2 physics_rotate_wheel_force_to_body(Vector2 wheelForceN, float steerAngleRad);
float physics_low_speed_blend(float velocityLongitudinalMps);
bool physics_state_is_valid(const VehicleSpec *spec, const VehicleState *state,
                            const VehicleDerived *derived);

/* ------------------------------------------------------------------------ solver stages --
 *
 * One fixed step is an ordered sequence of named stages. The order is the contract: each
 * stage's inputs are the outputs of the ones before it, and the two places where that is
 * subtle are called out at the stage that depends on them.
 *
 * THE ONE FEEDBACK LOOP. Normal loads depend on longitudinal acceleration, and longitudinal
 * acceleration depends on tire forces, which depend on normal loads. That is an algebraic loop
 * with no solution, so it is broken in time rather than iterated: PHYSICS_STAGE_NORMAL_LOADS
 * consumes the PREVIOUS step's solved body acceleration through a first-order filter. There is
 * no fixed-point iteration anywhere in the solver, and adding one would make the step count
 * depend on the state.
 *
 * DETERMINISM. No stage allocates, calls the clock, or loops over anything but the four
 * wheels. Every loop bound is a compile-time constant, so the operation count of a step is
 * identical for every state the solver can be in.
 */
typedef enum {
    PHYSICS_STAGE_NONE = 0,
    PHYSICS_STAGE_BEGIN,            /* snapshot for rollback; shift render history */
    PHYSICS_STAGE_STEERING,         /* steer actuation, wheel holds, locked-axle pre-equalize */
    PHYSICS_STAGE_POWERTRAIN,       /* engine speed, drivetrain and brake torques */
    PHYSICS_STAGE_WHEEL_KINEMATICS, /* contact velocities, slip angles, slip ratios */
    PHYSICS_STAGE_NORMAL_LOADS,     /* acceleration filters, axle loads, per-wheel Fz */
    PHYSICS_STAGE_TIRE_FORCES,      /* pure forces, relaxation lag, combined-friction limit */
    PHYSICS_STAGE_RESISTANCE,       /* aero drag, rolling resistance, low-speed brake guard */
    PHYSICS_STAGE_ACCUMULATE,       /* body-frame force sum and yaw moment */
    PHYSICS_STAGE_INTEGRATE_BODY,   /* derivatives, blend, integrate pose and velocity */
    PHYSICS_STAGE_INTEGRATE_WHEELS, /* wheel speeds, equilibrium landing, locked-axle post */
    PHYSICS_STAGE_DIAGNOSTICS,      /* accelerations, sideslip, friction usage, render pose */
    PHYSICS_STAGE_COUNT
} PhysicsStage;

/* Human-readable stage name, for failure reports. Never NULL. */
const char *physics_stage_name(PhysicsStage stage);

/*
 * The per-tick scratch. It holds the step's inputs, the rollback snapshot, and every
 * intermediate that crosses a stage boundary — which is precisely the set of values that used
 * to be locals buried in one long function and were therefore impossible to inspect.
 *
 * It lives on the caller's stack for exactly one step. Nothing here is persistent simulation
 * state: a value that must survive the step belongs in VehicleState, and a value that must be
 * reported belongs in VehicleDerived.
 */
typedef struct {
    /* Inputs, fixed for the step. */
    const VehicleSpec *spec;
    VehicleState *state;
    VehicleDerived *derived;
    VehicleRenderState *renderState;
    ControllerOutput input;
    float dt;

    /* Rollback snapshot, taken by PHYSICS_STAGE_BEGIN. */
    VehicleState lastGoodState;
    VehicleDerived lastGoodDerived;
    VehicleRenderState lastGoodRenderState;

    /* Intermediates that cross stages. */
    DifferentialMode diffMode;
    float frontShare;
    DrivetrainTorques torques;
    AxleLoads loads;
    Vector2 resistanceBodyN;
    float longitudinalResistanceN;
    VehicleDerivatives solved;

    PhysicsStage completedStage; /* the last stage that ran to completion */
} PhysicsStep;

/*
 * Prepare a step. Returns false when any input is unusable, in which case no stage may run.
 * `input` is copied, so a caller cannot change the step's demands halfway through it.
 */
bool physics_step_init(PhysicsStep *step, const VehicleSpec *spec, VehicleState *state,
                       VehicleDerived *derived, VehicleRenderState *renderState,
                       const ControllerOutput *input, float dt);

/*
 * Run stages up to and including `throughStage`, continuing from wherever the step last got
 * to. Running a prefix and inspecting the scratch is how a test asserts one stage's contract
 * without the ones after it having overwritten the evidence.
 */
void physics_step_run(PhysicsStep *step, PhysicsStage throughStage);

/*
 * One complete fixed step: every stage, then validation, then rollback to the pre-step state
 * if the result is rejected. On rollback the offending stage is identified by re-running the
 * step from the snapshot one stage at a time, and reported in
 * VehicleDerived.solverFailedStage — which costs nothing on the healthy path and turns "the
 * vehicle went non-finite" into "it went non-finite in tire forces".
 *
 * Note that physics_state_is_valid() is a whole-step invariant, not a per-stage one: it
 * cross-checks values that different stages produce against each other, so it is only
 * meaningful once every stage has run. Attribution therefore asks the narrower question of
 * where the state stopped being finite, and reports PHYSICS_STAGE_NONE when a step is rejected
 * for a consistency or bounds reason that no single stage owns.
 */
void physics_fixed_update(const VehicleSpec *spec, VehicleState *state, VehicleDerived *derived,
                          VehicleRenderState *renderState, const ControllerOutput *input,
                          float dt);

#endif /* CIRCUIT_PHYSICS_H */

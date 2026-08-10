#include "dev/dev_state.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/config.h"
#include "dev/dev_scenario.h"
#include "game/game.h"
#include "physics/physics.h"

void dev_state_init(DevState *dev)
{
    if (dev == NULL) return;
    memset(dev, 0, sizeof(*dev));

    dev->timeScale = 1.0f;
    dev->scenario = 0; /* "free" */
    dev->showForces = true;
    dev->showVelocity = true;
    dev->showSlip = true;
    dev->showLoads = false;
    dev->showContacts = true;
    dev->showPath = true;
    dev->showGhost = true;
    dev->showScope = true;
    dev->showResistance = false;
    dev->inspectorFollowsLive = true;
    dev->activeGroup = 0;
    dev->galleryPage = 0;
    dev->labTierFilter = 2; /* DEV_TIER_EXPERT — show all tiers by default */
    dev->scopePreset = DEV_SCOPE_PRESET_HANDLING;
    dev->markerPeakTransferSlot = -1;
}

void dev_state_clear_history(DevState *dev)
{
    if (dev == NULL) return;
    memset(dev->scope, 0, sizeof(dev->scope));
    dev->scopeHead = 0;
    dev->scopeCount = 0;
    dev->pathCount = 0;
    dev->pathHead = 0;
    dev->pathDecimator = 0;

    dev->markerCount = 0;
    dev->markerPrevThrottle = 0.0f;
    dev->markerPrevBrake = 0.0f;
    dev->markerPrevHandbrake = 0.0f;
    dev->markerPrevSteer = 0.0f;
    dev->markerSaturated = false;
    dev->markerInSlide = false;
    dev->markerPeakTransferN = 0.0f;
    dev->markerPeakTransferSlot = -1;
}

void dev_state_clear_invariants(DevState *dev)
{
    if (dev == NULL) return;
    dev->invariantFailed = false;
    dev->invariantTick = 0;
    dev->invariantCount = 0;
    dev->invariantText[0] = '\0';
}

void dev_state_set_status(DevState *dev, bool isError, const char *format, ...)
{
    if (dev == NULL || format == NULL) return;
    va_list args;
    va_start(args, format);
    vsnprintf(dev->status, sizeof(dev->status), format, args);
    va_end(args);
    dev->statusIsError = isError;
    dev->statusTimerS = 4.0f;
}

/* ------------------------------------------------------------------------------- scope -- */

static const char *const g_scopeNames[DEV_SCOPE_COUNT] = {
    "body sideslip",   "yaw rate",       "steering",      "throttle",    "front slip angle",
    "rear slip angle", "friction usage", "speed",         "filtered ax", "solved ax",
    "front load",      "rear load",      "load transfer", "aero drag",   "rolling resistance",
    "front usage",     "rear usage"
};

static const char *const g_scopeUnits[DEV_SCOPE_COUNT] = {
    "rad",   "rad/s", "",  "",  "rad", "rad", "", "m/s", "m/s^2",
    "m/s^2", "N",     "N", "N", "N",   "N",   "", ""
};

static const DevScopeChannel g_scopePresets[DEV_SCOPE_PRESET_COUNT][DEV_SCOPE_VISIBLE] = {
    [DEV_SCOPE_PRESET_HANDLING] = { DEV_SCOPE_SIDESLIP, DEV_SCOPE_YAW_RATE, DEV_SCOPE_STEER,
                                    DEV_SCOPE_THROTTLE, DEV_SCOPE_FRONT_SLIP,
                                    DEV_SCOPE_REAR_SLIP, DEV_SCOPE_FRICTION, DEV_SCOPE_SPEED },
    [DEV_SCOPE_PRESET_LOAD] = { DEV_SCOPE_SOLVED_ACCEL, DEV_SCOPE_FILTERED_ACCEL,
                                DEV_SCOPE_FRONT_LOAD, DEV_SCOPE_REAR_LOAD,
                                DEV_SCOPE_LOAD_TRANSFER, DEV_SCOPE_AERO_DRAG, DEV_SCOPE_ROLLING,
                                DEV_SCOPE_SPEED },
    [DEV_SCOPE_PRESET_GRIP] = { DEV_SCOPE_FRONT_SLIP, DEV_SCOPE_REAR_SLIP,
                                DEV_SCOPE_FRONT_USAGE, DEV_SCOPE_REAR_USAGE,
                                DEV_SCOPE_FRONT_LOAD, DEV_SCOPE_REAR_LOAD, DEV_SCOPE_SIDESLIP,
                                DEV_SCOPE_YAW_RATE },
};

static const char *const g_scopePresetNames[DEV_SCOPE_PRESET_COUNT] = { "handling", "load",
                                                                        "grip" };

const char *dev_scope_preset_name(DevScopePreset preset)
{
    if (preset < 0 || preset >= DEV_SCOPE_PRESET_COUNT) return "?";
    return g_scopePresetNames[preset];
}

DevScopeChannel dev_scope_preset_channel(DevScopePreset preset, int slot)
{
    if (preset < 0 || preset >= DEV_SCOPE_PRESET_COUNT) preset = DEV_SCOPE_PRESET_HANDLING;
    if (slot < 0 || slot >= DEV_SCOPE_VISIBLE) return DEV_SCOPE_SIDESLIP;
    return g_scopePresets[preset][slot];
}

const char *dev_state_scope_name(DevScopeChannel channel)
{
    if (channel < 0 || channel >= DEV_SCOPE_COUNT) return "?";
    return g_scopeNames[channel];
}

const char *dev_state_scope_unit(DevScopeChannel channel)
{
    if (channel < 0 || channel >= DEV_SCOPE_COUNT) return "";
    return g_scopeUnits[channel];
}

float dev_state_scope_value(const DevState *dev, DevScopeChannel channel, int index)
{
    if (dev == NULL || channel < 0 || channel >= DEV_SCOPE_COUNT) return 0.0f;
    if (index < 0 || index >= dev->scopeCount) return 0.0f;
    const int oldest = (dev->scopeCount == DEV_SCOPE_SAMPLES) ? dev->scopeHead : 0;
    return dev->scope[channel][(oldest + index) % DEV_SCOPE_SAMPLES];
}

Vector2 dev_state_path_point(const DevState *dev, int index)
{
    if (dev == NULL || index < 0 || index >= dev->pathCount) return (Vector2){ 0.0f, 0.0f };
    const int oldest = (dev->pathCount == DEV_PATH_POINTS) ? dev->pathHead : 0;
    return dev->path[(oldest + index) % DEV_PATH_POINTS];
}

void dev_state_capture_ghost(DevState *dev)
{
    if (dev == NULL) return;
    const int count = dev->pathCount;
    for (int i = 0; i < count; i++) dev->ghost[i] = dev_state_path_point(dev, i);
    dev->ghostCount = count;
    dev->ghostValid = (count > 0);

    /* Both halves are captured together so a baseline is always self-consistent. */
    for (int c = 0; c < DEV_SCOPE_COUNT; c++) {
        for (int i = 0; i < dev->scopeCount; i++) {
            dev->ghostScope[c][i] = dev_state_scope_value(dev, (DevScopeChannel)c, i);
        }
    }
    dev->ghostScopeCount = dev->scopeCount;
}

float dev_state_ghost_scope_value(const DevState *dev, DevScopeChannel channel, int index)
{
    if (dev == NULL || channel < 0 || channel >= DEV_SCOPE_COUNT) return 0.0f;
    if (index < 0 || index >= dev->ghostScopeCount) return 0.0f;
    return dev->ghostScope[channel][index]; /* already stored oldest-first */
}

/* -------------------------------------------------------------------------- invariants -- */

/* One physical invariant per check, with the failing value in the message: a red panel that
 * only says "invalid" is worth very little at three in the morning. */
static bool evaluate_invariants(const Game *game, char *text, size_t capacity)
{
    const VehicleState *v = &game->vehicle;
    const VehicleDerived *d = &game->derived;

    if (!physics_state_is_valid(&game->spec, v, d)) {
        snprintf(text, capacity, "physics_state_is_valid() rejected the state");
        return false;
    }
    if (!isfinite(v->positionM.x) || !isfinite(v->positionM.y)) {
        snprintf(text, capacity, "position is not finite (%.3f, %.3f)", (double)v->positionM.x,
                 (double)v->positionM.y);
        return false;
    }
    if (d->speedMps > MAX_SAFE_SPEED_MPS) {
        snprintf(text, capacity, "speed %.1f m/s exceeds MAX_SAFE_SPEED_MPS (%.1f)",
                 (double)d->speedMps, (double)MAX_SAFE_SPEED_MPS);
        return false;
    }
    if (fabsf(v->yawRateRadS) > MAX_SAFE_YAW_RATE_RADS) {
        snprintf(text, capacity, "yaw rate %.2f rad/s exceeds MAX_SAFE_YAW_RATE_RADS (%.1f)",
                 (double)v->yawRateRadS, (double)MAX_SAFE_YAW_RATE_RADS);
        return false;
    }
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &v->wheels[i];
        if (wheel->normalLoadN < 0.0f) {
            snprintf(text, capacity, "wheel %d carries a negative normal load (%.1f N)", i,
                     (double)wheel->normalLoadN);
            return false;
        }
        if (wheel->frictionUsage > 1.0f + FRICTION_TOLERANCE) {
            snprintf(text, capacity,
                     "wheel %d uses %.4f of its friction budget (limit 1 + %.3f)", i,
                     (double)wheel->frictionUsage, (double)FRICTION_TOLERANCE);
            return false;
        }
    }

    /* Phase 3. physics_state_is_valid() already rejects each of these; repeating them here
     * with the numbers attached is the difference between a red panel that says "invalid"
     * and one that says which equation stopped being true. */
    const float weightN = game->spec.massKg * GRAVITY_MPS2;
    if (!isfinite(d->filteredLongAccelMps2) || !isfinite(d->previousLongAccelMps2) ||
        !isfinite(d->solvedLongAccelMps2)) {
        snprintf(text, capacity,
                 "load-filter acceleration is not finite (prev %.3f, "
                 "filtered %.3f, solved %.3f)",
                 (double)d->previousLongAccelMps2, (double)d->filteredLongAccelMps2,
                 (double)d->solvedLongAccelMps2);
        return false;
    }
    /* Weight plus the aerodynamic vertical load (issue #17), not weight alone: downforce is an
     * external force on the body, so it adds to what the four contact patches carry. */
    const float verticalN = weightN + d->aeroVerticalFrontN + d->aeroVerticalRearN;
    if (fabsf((d->unclampedFrontLoadN + d->unclampedRearLoadN) - verticalN) > 1.0f) {
        snprintf(text, capacity,
                 "unclamped axle loads sum to %.1f N, not mass*g + aero = %.1f N",
                 (double)(d->unclampedFrontLoadN + d->unclampedRearLoadN), (double)verticalN);
        return false;
    }
    if (fabsf(d->loadTransferN) > MAX_LOAD_TRANSFER_FRACTION * weightN) {
        snprintf(text, capacity, "load transfer %.1f N exceeds %.1fx mass*g",
                 (double)d->loadTransferN, (double)MAX_LOAD_TRANSFER_FRACTION);
        return false;
    }
    if (d->aeroDragBodyN.x * v->velocityLongitudinalMps +
            d->aeroDragBodyN.y * v->velocityLateralMps >
        RESISTANCE_POWER_TOLERANCE_W) {
        snprintf(text, capacity, "aerodynamic drag is adding energy (%.4f W)",
                 (double)(d->aeroDragBodyN.x * v->velocityLongitudinalMps +
                          d->aeroDragBodyN.y * v->velocityLateralMps));
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------ markers -- */

static void push_marker(DevState *dev, DevMarkerKind kind, uint64_t tick, float value)
{
    if (dev->markerCount >= DEV_MARKER_CAPACITY) return;
    dev->markers[dev->markerCount].tick = tick;
    dev->markers[dev->markerCount].kind = kind;
    dev->markers[dev->markerCount].value = value;
    dev->markerCount++;
}

/* Thresholds are display heuristics, not physics: they decide when a line is drawn on the
 * inspector's timeline, and nothing reads them back into the simulation. */
#define MARKER_CONTROL_THRESHOLD 0.50f
#define MARKER_STEER_THRESHOLD 0.05f
#define MARKER_SATURATION_USAGE 0.99f
#define MARKER_SLIDE_ENTER_RAD 0.35f
#define MARKER_SLIDE_EXIT_RAD 0.10f
#define MARKER_RECOVERY_SPEED_MPS 3.00f

static void record_markers(struct Game *game, const ControllerOutput *appliedInput)
{
    DevState *dev = &game->dev;
    const uint64_t tick = game->sim.tick;

    const float throttle = (appliedInput != NULL) ? appliedInput->throttle : 0.0f;
    const float brake = (appliedInput != NULL) ? appliedInput->brake : 0.0f;
    const float handbrake = (appliedInput != NULL) ? appliedInput->handbrake : 0.0f;
    const float steer = (appliedInput != NULL) ? appliedInput->steer : 0.0f;

    if (throttle >= MARKER_CONTROL_THRESHOLD &&
        dev->markerPrevThrottle < MARKER_CONTROL_THRESHOLD) {
        push_marker(dev, DEV_MARKER_THROTTLE_ON, tick, throttle);
    }
    if (throttle < MARKER_CONTROL_THRESHOLD &&
        dev->markerPrevThrottle >= MARKER_CONTROL_THRESHOLD) {
        push_marker(dev, DEV_MARKER_THROTTLE_LIFT, tick, throttle);
    }
    if (brake >= MARKER_CONTROL_THRESHOLD && dev->markerPrevBrake < MARKER_CONTROL_THRESHOLD) {
        push_marker(dev, DEV_MARKER_BRAKE_ON, tick, brake);
    }
    if (handbrake >= MARKER_CONTROL_THRESHOLD &&
        dev->markerPrevHandbrake < MARKER_CONTROL_THRESHOLD) {
        push_marker(dev, DEV_MARKER_HANDBRAKE_ON, tick, handbrake);
    }
    if ((dev->markerPrevSteer > MARKER_STEER_THRESHOLD && steer < -MARKER_STEER_THRESHOLD) ||
        (dev->markerPrevSteer < -MARKER_STEER_THRESHOLD && steer > MARKER_STEER_THRESHOLD)) {
        push_marker(dev, DEV_MARKER_STEER_REVERSAL, tick, steer);
    }
    dev->markerPrevThrottle = throttle;
    dev->markerPrevBrake = brake;
    dev->markerPrevHandbrake = handbrake;
    dev->markerPrevSteer = steer;

    if (!dev->markerSaturated && game->derived.maxFrictionUsage >= MARKER_SATURATION_USAGE) {
        dev->markerSaturated = true;
        push_marker(dev, DEV_MARKER_SATURATION, tick, game->derived.maxFrictionUsage);
    }

    /* One peak-transfer marker, moved rather than duplicated as the peak grows. */
    const float transferN = fabsf(game->derived.loadTransferN);
    if (transferN > dev->markerPeakTransferN + 1e-3f) {
        dev->markerPeakTransferN = transferN;
        if (dev->markerPeakTransferSlot < 0) {
            dev->markerPeakTransferSlot = dev->markerCount;
            push_marker(dev, DEV_MARKER_PEAK_TRANSFER, tick, game->derived.loadTransferN);
        } else {
            dev->markers[dev->markerPeakTransferSlot].tick = tick;
            dev->markers[dev->markerPeakTransferSlot].value = game->derived.loadTransferN;
        }
    }

    const float sideslip = fabsf(game->derived.bodySideslipRad);
    if (!dev->markerInSlide && sideslip >= MARKER_SLIDE_ENTER_RAD) {
        dev->markerInSlide = true;
    } else if (dev->markerInSlide && sideslip <= MARKER_SLIDE_EXIT_RAD &&
               game->derived.speedMps >= MARKER_RECOVERY_SPEED_MPS &&
               game->vehicle.velocityLongitudinalMps > 0.0f) {
        dev->markerInSlide = false;
        push_marker(dev, DEV_MARKER_RECOVERY, tick, sideslip);
    }
}

/* ----------------------------------------------------------------------------- recording -- */

void dev_state_record(struct Game *game, const ControllerOutput *appliedInput)
{
    if (game == NULL) return;
    DevState *dev = &game->dev;

    if (appliedInput != NULL) dev->appliedInput = *appliedInput;

    const float sample[DEV_SCOPE_COUNT] = {
        game->derived.bodySideslipRad,
        game->vehicle.yawRateRadS,
        game->vehicle.frontRoadWheelAngleRad,
        (appliedInput != NULL) ? appliedInput->throttle : 0.0f,
        game->derived.frontSlipAngleRad,
        game->derived.rearSlipAngleRad,
        game->derived.maxFrictionUsage,
        game->derived.speedMps,
        game->derived.filteredLongAccelMps2,
        game->derived.solvedLongAccelMps2,
        game->derived.normalLoadFrontN,
        game->derived.normalLoadRearN,
        game->derived.loadTransferN,
        game->derived.aeroDragMagnitudeN,
        game->derived.rollingResistanceMagnitudeN,
        fmaxf(game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
              game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage),
        fmaxf(game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
              game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage)
    };
    for (int c = 0; c < DEV_SCOPE_COUNT; c++) dev->scope[c][dev->scopeHead] = sample[c];
    dev->scopeHead = (dev->scopeHead + 1) % DEV_SCOPE_SAMPLES;
    if (dev->scopeCount < DEV_SCOPE_SAMPLES) dev->scopeCount++;

    if (++dev->pathDecimator >= DEV_PATH_DECIMATION) {
        dev->pathDecimator = 0;
        dev->path[dev->pathHead] = game->vehicle.positionM;
        dev->pathHead = (dev->pathHead + 1) % DEV_PATH_POINTS;
        if (dev->pathCount < DEV_PATH_POINTS) dev->pathCount++;
    }

    record_markers(game, appliedInput);

    char text[DEV_INVARIANT_CHARS];
    if (!evaluate_invariants(game, text, sizeof(text))) {
        dev->invariantCount++;
        if (!dev->invariantFailed) {
            /* Latch the FIRST violation: later ticks are usually consequences of it. */
            dev->invariantFailed = true;
            dev->invariantTick = game->sim.tick;
            snprintf(dev->invariantText, sizeof(dev->invariantText), "%s", text);
            push_marker(dev, DEV_MARKER_INVARIANT, game->sim.tick, 0.0f);
        }
    }

    if (dev->scenarioRunning) {
        const DevScenario *scenario = dev_scenario_at(dev->scenario);
        if (scenario != NULL && scenario->durationTicks > 0) {
            const uint64_t elapsed = game->sim.tick - dev->scenarioStartTick;
            if (elapsed + 1u >= (uint64_t)scenario->durationTicks) {
                dev->scenarioRunning = false;
                dev->paused = true;
                dev_state_set_status(dev, false, "scenario '%s' finished at tick %llu",
                                     scenario->name, (unsigned long long)game->sim.tick);
            }
        }
    }
}

/* ----------------------------------------------------------------------------- scenarios -- */

void dev_state_scenario_start(struct Game *game, int scenarioIndex)
{
    if (game == NULL) return;
    const DevScenario *scenario = dev_scenario_at(scenarioIndex);
    if (scenario == NULL) return;

    DevState *dev = &game->dev;

    /* A scenario is only reproducible from a known starting state. */
    game_reset_sim(game);
    dev_state_clear_history(dev);
    dev_state_clear_invariants(dev);
    replay_begin_recording(&game->replay, game->sim.tick);

    dev->scenario = scenarioIndex;
    dev->scenarioStartTick = game->sim.tick;
    dev->scenarioRunning = (scenarioIndex > 0);
    dev->seed = scenario->seed;
    dev->paused = false;
    dev->stepTicks = 0;
    dev->inspectorFollowsLive = true;
    dev_state_set_status(dev, false, "scenario '%s' started", scenario->name);
}

void dev_state_scenario_stop(struct Game *game)
{
    if (game == NULL) return;
    game->dev.scenarioRunning = false;
    dev_state_set_status(&game->dev, false, "scenario stopped; manual control restored");
}

/*
 * pit_state.c — issue #57 implementation.
 */
#include "game/pit_state.h"

#include <string.h>

#include "core/config.h"
#include "physics/vehicle.h"

void pit_state_reset(EntrantPitState *pit)
{
    if (pit == NULL) return;
    memset(pit, 0, sizeof(*pit));
    pit->assignedBox = -1;
}

/*
 * The lane is a continuous corridor from the entry gate, past every service box in authored
 * order, to the exit gate — see track_point_in_pit_lane(). It used to be three disconnected
 * blobs: inside a box, or within ten metres of the entry marker, or within ten metres of the
 * exit marker. A car in the middle of the lane matched none of them, so the speed limiter
 * switched itself off over most of the lane's length (MAP.md known issue #8).
 */
static bool in_pit_lane(const TrackDefinition *track, Vector2 p)
{
    return track_point_in_pit_lane(track, p, PIT_LANE_HALF_WIDTH_M);
}

/*
 * The box this car may use, or -1.
 *
 * A box another entrant is already occupying is not available, which is what makes a pit lane
 * a shared resource rather than a teleport pad: a second car arriving at a taken box drives on
 * rather than servicing on top of the car already there. Every entrant used to be assigned box
 * 0 regardless of where it stopped (MAP.md known issue #7), so two cars pitting together were
 * indistinguishable to everything downstream.
 */
static int available_box_at(const TrackDefinition *track, Vector2 p, uint32_t occupiedMask)
{
    const int idx = track_service_box_at(track, p);
    if (idx < 0 || idx >= 32) return -1;
    if ((occupiedMask & (1u << (unsigned)idx)) != 0u) return -1;
    return idx;
}

bool pit_state_update(EntrantPitState *pit, const TrackDefinition *track, Vector2 positionM,
                      float speedMps, VehicleInstance *instance, float serviceTimeS, float dt,
                      uint32_t occupiedBoxMask)
{
    if (pit == NULL || track == NULL) return false;
    const bool inLane = in_pit_lane(track, positionM);
    const int freeBox = available_box_at(track, positionM, occupiedBoxMask);
    const bool inBox = (freeBox >= 0);

    switch (pit->state) {
        case PIT_STATE_NONE:
            /* Entering: crossing the entry gate, or simply appearing inside the lane (a
             * teleport or spawn mid-lane still counts). */
            if (inLane) {
                pit->state = PIT_STATE_ENTERING;
                if (track->hasPitSpeedLine) {
                    /* The speed line is not a speed figure in the manifest; use a sane
                     * default pit limit, configurable by the session later. */
                    pit->speedLimitMps = 15.0f;
                }
            }
            break;

        case PIT_STATE_ENTERING:
            if (inBox && speedMps < 1.0f) {
                /* Stopped at a box that is free: begin service in THAT box. */
                pit->state = PIT_STATE_AT_BOX;
                pit->assignedBox = freeBox;
                pit->serviceTimerS = serviceTimeS;
                pit->served = false;
            } else if (!inLane) {
                pit->state = PIT_STATE_NONE;
            } else {
                pit->state = PIT_STATE_IN_LANE;
            }
            break;

        case PIT_STATE_IN_LANE:
            if (inBox && speedMps < 1.0f) {
                pit->state = PIT_STATE_AT_BOX;
                pit->assignedBox = freeBox;
                pit->serviceTimerS = serviceTimeS;
                pit->served = false;
            } else if (!inLane) {
                pit->state = PIT_STATE_NONE;
            }
            break;

        case PIT_STATE_AT_BOX:
            if (pit->serviceTimerS > 0.0f) {
                pit->serviceTimerS -= dt;
            }
            if (pit->serviceTimerS <= 0.0f && !pit->served) {
                /* Apply the service exactly once, on the tick the timer expires. */
                if (instance != NULL) {
                    if (pit->requestTires) vehicle_tire_service(instance, true);
                    if (pit->requestFuelL > 0.0f) vehicle_refuel(instance, pit->requestFuelL);
                    if (pit->requestRepair) instance->damage = 0.0f;
                }
                pit->served = true;
                pit->state = PIT_STATE_EXITING;
            }
            break;

        case PIT_STATE_EXITING:
            if (!inLane) pit->state = PIT_STATE_NONE;
            break;

        default: pit->state = PIT_STATE_NONE; break;
    }

    return (pit->state == PIT_STATE_ENTERING || pit->state == PIT_STATE_IN_LANE ||
            pit->state == PIT_STATE_AT_BOX || pit->state == PIT_STATE_EXITING);
}

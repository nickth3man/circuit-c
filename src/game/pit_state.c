/*
 * pit_state.c — issue #57 implementation.
 */
#include "game/pit_state.h"

#include <string.h>

#include "physics/vehicle.h"

void pit_state_reset(EntrantPitState *pit)
{
    if (pit == NULL) return;
    memset(pit, 0, sizeof(*pit));
    pit->assignedBox = -1;
}

/* Does the point lie within the pit lane corridor (entry-to-exit along the lane)? We
 * approximate with the service-box corridor: any point inside a service box, or within a fixed
 * margin of the entry/exit gates, counts as in-lane. This keeps the zone purely geometric. */
static bool in_pit_lane(const TrackDefinition *track, Vector2 p)
{
    if (track == NULL) return false;
    if (track_point_in_service_box(track, p)) return true;
    if (track->hasPitEntry) {
        const float dx = p.x - track->pitEntry.centerM.x;
        const float dy = p.y - track->pitEntry.centerM.y;
        if (dx * dx + dy * dy < 100.0f) return true; /* 10 m around the entry gate */
    }
    if (track->hasPitExit) {
        const float dx = p.x - track->pitExit.centerM.x;
        const float dy = p.y - track->pitExit.centerM.y;
        if (dx * dx + dy * dy < 100.0f) return true; /* 10 m around the exit gate */
    }
    return false;
}

bool pit_state_update(EntrantPitState *pit, const TrackDefinition *track, Vector2 positionM,
                      float speedMps, VehicleInstance *instance, float serviceTimeS, float dt)
{
    if (pit == NULL || track == NULL) return false;
    const bool inLane = in_pit_lane(track, positionM);
    const bool inBox = track_point_in_service_box(track, positionM);

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
                /* Stopped at a box: begin service. */
                pit->state = PIT_STATE_AT_BOX;
                pit->assignedBox = 0;
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
                pit->assignedBox = 0;
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

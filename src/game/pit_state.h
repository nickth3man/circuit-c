/*
 * pit_state.h — issue #57: deterministic pit cycle (entry, limiter, box, service, exit).
 *
 * The pit cycle is a per-entrant state machine driven by the track's authored pit geometry
 * (entry/exit/speed-line gates and service boxes). All service changes (tires, fuel, damage
 * repair) happen only through validated operations on authoritative fixed ticks; nothing reads
 * render state, and a track without pit geometry rejects pit rules at configuration time.
 */
#ifndef CIRCUIT_PIT_STATE_H
#define CIRCUIT_PIT_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "world/track.h"

typedef enum {
    PIT_STATE_NONE = 0, /* not in the pit cycle */
    PIT_STATE_ENTERING, /* crossed the pit-entry gate, limiter engages */
    PIT_STATE_IN_LANE,  /* in the pit lane, speed-limited */
    PIT_STATE_AT_BOX,   /* stopped at the assigned box, service running */
    PIT_STATE_EXITING,  /* service complete, leaving the lane */
    PIT_STATE_COUNT
} PitState;

/* Per-entrant pit cycle state. */
typedef struct {
    PitState state;
    float speedLimitMps; /* the track's pit-lane limit (from the speed line) */
    float serviceTimerS; /* remaining service time; >0 only while AT_BOX */
    int assignedBox;     /* service box index, -1 = unassigned */
    bool requestTires;   /* service request: replace tires */
    float requestFuelL;  /* service request: refuel litres (0 = none) */
    bool requestRepair;  /* service request: repair damage */
    bool served;         /* this stop's requests were applied */
} EntrantPitState;

/* Zeroed state is the legal "not in pits" default. */
void pit_state_reset(EntrantPitState *pit);

/*
 * Per-tick pit cycle stage. Advances the state machine using the entrant's position, the
 * track's pit geometry, and the frozen pit rules. Service (if any) is applied exactly once when
 * the service timer expires, through vehicle_tire_service/vehicle_refuel/damage repair. Returns
 * true while the entrant is inside the pit lane (speed limiter should be engaged).
 */
bool pit_state_update(EntrantPitState *pit, const TrackDefinition *track, Vector2 positionM,
                      float speedMps, VehicleInstance *instance, float serviceTimeS, float dt);

#endif /* CIRCUIT_PIT_STATE_H */

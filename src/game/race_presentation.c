/*
 * race_presentation.c — issue #56 implementation.
 */
#include "game/race_presentation.h"

#include <string.h>

#include <stdio.h>

#include "game/car_roster.h"
#include "physics/vehicle.h"
#include "world/track.h"

/*
 * Turn a distance behind into a time gap, using the trailing car's own speed: "how long until I
 * am where they are". Below a walking pace the quotient stops describing anything a viewer would
 * recognise as a gap — a stopped car is infinitely behind — so it returns 0 and the caller
 * reports the distance instead. Negative distances (a car the sort put behind but which has
 * since edged ahead within a tick) also return 0 rather than a negative gap.
 */
#define PRESENTATION_MIN_GAP_SPEED_MPS 1.5f

static float gap_seconds(float distanceM, float speedMps)
{
    if (distanceM <= 0.0f || speedMps < PRESENTATION_MIN_GAP_SPEED_MPS) return 0.0f;
    return distanceM / speedMps;
}

void race_presentation_snapshot(const RaceSession *session, const TrackDefinition *track,
                                RacePresentationSnapshot *out)
{
    if (session == NULL || out == NULL) return;
    memset(out, 0, sizeof(*out));
    out->phase = session->phase;
    out->countdownTicksRemaining = session->countdownTicksRemaining;
    out->sessionTimeS = session->clockS;
    out->targetLaps = session->rules.targetLaps;

    /* Live order is the authoritative position source. */
    int order[RACE_MAX_ENTRANTS];
    const int n = race_session_live_order(session, order, RACE_MAX_ENTRANTS);
    out->entrantCount = n;

    const EntrantId localId = session->roster.localEntrantId;
    float leaderTime = 0.0f;
    float leaderDistanceM = 0.0f;
    float aheadDistanceM = 0.0f;

    for (int pos = 0; pos < n; pos++) {
        const RaceEntrant *e = &session->roster.entrants[order[pos]];
        PresentationEntrantRow *row = &out->rows[pos];
        row->entrantId = e->id;
        (void)snprintf(row->carId, sizeof(row->carId), "%s", e->definition.id);
        row->livePosition = pos + 1;
        row->lapsCompleted = e->progress.lap;
        row->lastLapTimeS = e->progress.lastLapTimeS;
        row->bestLapTimeS = e->progress.bestLapTimeS;
        row->finished = e->result.finished;
        row->isLocal = (e->id == localId);

        const float distanceM = e->progress.raceDistanceM;
        if (pos == 0) {
            leaderTime = e->result.finishTimeS;
            leaderDistanceM = distanceM;
        } else {
            row->distanceToLeaderM = leaderDistanceM - distanceM;
            row->distanceToAheadM = aheadDistanceM - distanceM;
            if (e->result.finished) {
                /* Both classified: the difference of finish times is exact, and no speed-based
                 * estimate can improve on it. */
                row->gapToLeaderS = e->result.finishTimeS - leaderTime;
            } else {
                const float speedMps = e->instance.derived.speedMps;
                row->gapToLeaderS = gap_seconds(row->distanceToLeaderM, speedMps);
                row->gapToAheadS = gap_seconds(row->distanceToAheadM, speedMps);
            }
        }
        aheadDistanceM = distanceM;
        if (row->isLocal) out->localPosition = pos + 1;
    }

    out->fastestLapTimeS = race_session_fastest_lap(session, &out->fastestLapEntrantId);

    /* Local entrant vehicle state. */
    const RaceEntrant *local = race_roster_find_const(&session->roster, localId);
    if (local != NULL) {
        const VehicleInstance *inst = &local->instance;
        const VehicleSpec *spec = &inst->spec;
        const float tankKg = spec->fuelTankCapacityL * FUEL_DENSITY_KG_PER_L;
        out->fuelPercent = (tankKg > 0.0f) ? (inst->fuelKg / tankKg * 100.0f) : 100.0f;
        float wear = 0.0f;
        for (int w = 0; w < WHEEL_COUNT; w++) wear += inst->tireState[w].wear;
        out->tireWearPercent = wear / (float)WHEEL_COUNT * 100.0f;
        out->damagePercent = inst->damage * 100.0f;
        out->absLevel = local->setup.absLevel;
        out->tcsLevel = local->setup.tcsLevel;
        out->wrongWay = local->progress.wrongWay;
        out->pendingPenalties = race_session_pending_penalties(session, localId);
        out->localLapsCompleted = local->progress.lap;
        out->localLapTimerS = local->progress.lapTimerS;
        out->localLastLapTimeS = local->progress.lastLapTimeS;
        out->localBestLapTimeS = local->progress.bestLapTimeS;
        out->localPitState = local->result.pit.state;
        out->pitServiceRemainingS = local->result.pit.serviceTimerS;
        out->pitAssignedBox = local->result.pit.assignedBox;
    }
    (void)track;
}

bool race_session_retry(RaceSession *session, const TrackDefinition *track)
{
    if (session == NULL || track == NULL) return false;
    /* Preserve the roster (who is racing) and the frozen rules; reset mutable state. */
    for (int i = 0; i < session->roster.count; i++) {
        vehicle_instance_reset(&session->roster.entrants[i].instance);
        controller_reset(&session->roster.entrants[i].controller);
        controller_output_zero(&session->roster.entrants[i].controllerOutput);
        memset(&session->roster.entrants[i].result, 0, sizeof(EntrantResult));
        track_reset_progress_at(&session->roster.entrants[i].progress, track, 0);
    }
    /* Re-place the grid and re-start the session with the same rules. */
    if (!race_session_place_grid(session, track)) return false;
    RaceRules rules = session->rules;
    race_session_start(session, &rules);
    session->firstFinisherClockS = 0.0f;
    return true;
}

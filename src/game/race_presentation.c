/*
 * race_presentation.c — issue #56 implementation.
 */
#include "game/race_presentation.h"

#include <string.h>

#include "game/car_roster.h"
#include "physics/vehicle.h"
#include "world/track.h"

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

    for (int pos = 0; pos < n; pos++) {
        const RaceEntrant *e = &session->roster.entrants[order[pos]];
        PresentationEntrantRow *row = &out->rows[pos];
        row->entrantId = e->id;
        row->livePosition = pos + 1;
        row->lapsCompleted = e->progress.lap;
        row->lastLapTimeS = e->progress.lastLapTimeS;
        row->bestLapTimeS = e->progress.bestLapTimeS;
        row->finished = e->result.finished;
        row->isLocal = (e->id == localId);
        /* Gap: for finished entrants, use their finish time vs the leader. For active, use 0
         * (live gaps require distance/speed interpolation that belongs in a future issue). */
        if (pos == 0) {
            leaderTime = e->result.finishTimeS;
            row->gapToLeaderS = 0.0f;
        } else {
            row->gapToLeaderS =
                e->result.finished ? (e->result.finishTimeS - leaderTime) : 0.0f;
        }
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

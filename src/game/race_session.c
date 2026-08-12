/*
 * race_session.c — race lifecycle, finish rules, deterministic events, and classification.
 *
 * Everything here is plain value manipulation on a caller-owned RaceSession, driven by an
 * explicit `dt`. Nothing allocates, nothing reads the clock, and nothing touches presentation,
 * so a whole race can be run headlessly and hashed.
 */
#include "game/race_session.h"

#include <math.h>
#include <string.h>

#include "core/config.h"

void race_rules_set_default(RaceRules *rules)
{
    if (rules == NULL) return;
    memset(rules, 0, sizeof(*rules));
    rules->mode = RACE_MODE_TIME_TRIAL;
    rules->targetLaps = RESULTS_TARGET_LAPS;
    rules->countdownS = 0.0f;
    rules->damageMode = DAMAGE_OFF;
    rules->stuckRecoveryEnabled = false;
    rules->stuckRecoveryDelayS = STUCK_RECOVERY_DELAY_S;
    rules->falseStartPenaltyS = FALSE_START_PENALTY_S;
    rules->falseStartSpeedMps = FALSE_START_SPEED_MPS;
    rules->finishingWindowS = RACE_FINISHING_WINDOW_S;
}

void race_environment_set_default(RaceEnvironment *env)
{
    if (env == NULL) return;
    memset(env, 0, sizeof(*env));
    env->ambientTempC = 20.0f;
    env->trackTempC = 20.0f;
    env->timeOfDayHours = 12.0f;
}

void race_environment_update(RaceEnvironment *env, float dt)
{
    if (env == NULL || !(dt > 0.0f)) return;
    /* Rain adds wetness to every surface; drainage and evaporation dry it. */
    const float rainAdd = ENV_RAIN_RATE_PER_S * env->precipitation * dt;
    const float dryRem = ENV_DRY_RATE_PER_S * dt;
    for (int s = 0; s < SURFACE_COUNT; s++) {
        float w = env->wetness[s] + rainAdd - dryRem;
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
        env->wetness[s] = w;
    }
    /* Track temperature relaxes toward ambient plus a small rain cooling offset. */
    const float targetC = env->ambientTempC - 5.0f * env->precipitation;
    const float alpha = 1.0f - expf(-ENV_TRACK_TEMP_RATE_PER_S * dt);
    env->trackTempC += (targetC - env->trackTempC) * alpha;
}

void race_session_init(RaceSession *session)
{
    if (session == NULL) return;
    memset(session, 0, sizeof(*session));
    race_roster_init(&session->roster);
    race_rules_set_default(&session->rules);
    race_environment_set_default(&session->environment);
    session->phase = RACE_PHASE_CONFIGURING;
    session->resumePhase = RACE_PHASE_CONFIGURING;
}

/* One place that writes `phase`, so every transition raises exactly one event. */
static void set_phase(RaceSession *session, RacePhase phase)
{
    if (session->phase == phase) return;
    session->phase = phase;
    race_session_log_event(session, RACE_EVENT_PHASE_CHANGED, RACE_ENTRANT_ID_NONE,
                           (int32_t)phase);
}

void race_session_start(RaceSession *session, const RaceRules *rules)
{
    if (session == NULL) return;
    if (rules != NULL) session->rules = *rules;

    session->tick = 0u;
    session->clockS = 0.0f;
    session->classifiedCount = 0;
    /* A fresh race starts with the authored environment defaults; the session config flow
     * (#48) will set precipitation/ambient explicitly. */
    race_environment_set_default(&session->environment);
    /* Resolved once, in fixed steps. The simulation rate is a product constant, so a countdown
     * is a whole number of ticks and lasts the same number of ticks on every machine.
     *
     * Clamped before the conversion, not after: a NaN or an absurd countdown would otherwise
     * make the float-to-int cast undefined, and "undefined" is not a grid procedure. */
    session->countdownTicksRemaining = 0;
    if (session->rules.countdownS > 0.0f) {
        const float ticks = ceilf(session->rules.countdownS / (float)FIXED_DT_S);
        session->countdownTicksRemaining =
            (ticks >= (float)RACE_COUNTDOWN_MAX_TICKS) ? RACE_COUNTDOWN_MAX_TICKS : (int)ticks;
    }
    memset(&session->events, 0, sizeof(session->events));
    memset(&session->results, 0, sizeof(session->results));

    /* Every entrant's result goes with the session, not with the car: restarting a race must
     * not leave last run's finishing position attached to anybody. */
    for (int i = 0; i < session->roster.count; i++) {
        memset(&session->roster.entrants[i].result, 0,
               sizeof(session->roster.entrants[i].result));
    }

    /* Grid first even when the release is immediate, so the phase sequence a listener sees is
     * the same whether or not a countdown was configured. */
    session->phase = RACE_PHASE_CONFIGURING;
    set_phase(session, RACE_PHASE_GRID);
    set_phase(session, (session->countdownTicksRemaining > 0) ? RACE_PHASE_COUNTDOWN
                                                              : RACE_PHASE_RUNNING);
    session->resumePhase = session->phase;
}

bool race_session_is_simulating(const RaceSession *session)
{
    if (session == NULL) return false;
    switch (session->phase) {
        case RACE_PHASE_COUNTDOWN:
        case RACE_PHASE_RUNNING:
        case RACE_PHASE_FINISHING: return true;
        default: return false;
    }
}

bool race_session_is_over(const RaceSession *session)
{
    if (session == NULL) return false;
    return session->phase == RACE_PHASE_CLASSIFIED || session->phase == RACE_PHASE_ABORTED;
}

bool race_session_pause(RaceSession *session)
{
    if (session == NULL || !race_session_is_simulating(session)) return false;
    session->resumePhase = session->phase;
    set_phase(session, RACE_PHASE_PAUSED);
    return true;
}

bool race_session_resume(RaceSession *session)
{
    if (session == NULL || session->phase != RACE_PHASE_PAUSED) return false;
    set_phase(session, session->resumePhase);
    return true;
}

bool race_session_abort(RaceSession *session)
{
    if (session == NULL) return false;
    /* Only a race that could still have finished can be abandoned. A classified session already
     * has an answer, and moving it to ABORTED would leave a finished race claiming it produced
     * no results while `results.valid` says otherwise. */
    if (session->phase == RACE_PHASE_CONFIGURING || race_session_is_over(session)) return false;
    set_phase(session, RACE_PHASE_ABORTED);
    session->resumePhase = RACE_PHASE_ABORTED;
    return true;
}

void race_session_log_event(RaceSession *session, RaceEventKind kind, EntrantId entrantId,
                            int32_t value)
{
    if (session == NULL) return;
    RaceEventLog *log = &session->events;
    const int slot = (log->head + log->count) % RACE_EVENT_CAPACITY;
    if (log->count < RACE_EVENT_CAPACITY) {
        log->count++;
    } else {
        log->head = (log->head + 1) % RACE_EVENT_CAPACITY; /* overwrite the oldest */
    }
    log->items[slot] = (RaceEvent){ .kind = kind,
                                    .tick = session->tick,
                                    .timeS = session->clockS,
                                    .entrantId = entrantId,
                                    .value = value };
    log->totalAppended++;
}

const RaceEvent *race_session_last_event(const RaceSession *session)
{
    if (session == NULL || session->events.count <= 0) return NULL;
    const int slot = (session->events.head + session->events.count - 1) % RACE_EVENT_CAPACITY;
    return &session->events.items[slot];
}

/* The distance is complete for this entrant. Kept as `>=` against the raw target so the test
 * is character-for-character the comparison that used to sit in game_fixed_update(). */
static bool entrant_has_finished_distance(const RaceEntrant *entrant, const RaceRules *rules)
{
    return entrant->progress.lap >= rules->targetLaps;
}

static bool every_entrant_finished(const RaceSession *session)
{
    if (session->roster.count <= 0) return false;
    for (int i = 0; i < session->roster.count; i++) {
        if (!session->roster.entrants[i].result.finished) return false;
    }
    return true;
}

/* Written once, on entry to CLASSIFIED, in ascending finishing position with the entrants that
 * never finished appended in EntrantId order behind them. */
static void capture_results(RaceSession *session)
{
    RaceResults *results = &session->results;
    float winnerTimeS = 0.0f;

    for (int position = 1; position <= session->roster.count; position++) {
        for (int i = 0; i < session->roster.count; i++) {
            const RaceEntrant *entrant = &session->roster.entrants[i];
            if (!entrant->result.finished || entrant->result.finishPosition != position)
                continue;
            results->rows[results->count++] = (RaceResultRow){
                .entrantId = entrant->id,
                .finishPosition = entrant->result.finishPosition,
                .lapsCompleted = entrant->progress.lap,
                .finishTimeS = entrant->result.finishTimeS,
                .lastLapTimeS = entrant->progress.lastLapTimeS,
                .bestLapTimeS = entrant->progress.bestLapTimeS,
                .gapToLeaderS =
                    (position == 1) ? 0.0f : (entrant->result.finishTimeS - winnerTimeS),
                .finished = true
            };
            if (position == 1) winnerTimeS = entrant->result.finishTimeS;
        }
    }
    for (int i = 0; i < session->roster.count; i++) {
        const RaceEntrant *entrant = &session->roster.entrants[i];
        if (entrant->result.finished) continue;
        results->rows[results->count++] =
            (RaceResultRow){ .entrantId = entrant->id,
                             .finishPosition = 0,
                             .lapsCompleted = entrant->progress.lap,
                             .finishTimeS = 0.0f,
                             .lastLapTimeS = entrant->progress.lastLapTimeS,
                             .bestLapTimeS = entrant->progress.bestLapTimeS,
                             .finished = false };
    }
    results->fastestLapTimeS = race_session_fastest_lap(session, &results->fastestLapEntrantId);
    results->valid = true;
}

void race_session_begin_tick(RaceSession *session, float dt)
{
    if (session == NULL || !race_session_is_simulating(session)) return;

    /* Established once, at the top of the tick, so that every event any stage raises during it
     * carries the same tick and clock. Advancing them halfway through would let a lap crossing
     * and the finish it triggers disagree about when they happened. */
    session->tick++;

    if (session->phase == RACE_PHASE_COUNTDOWN) {
        /* Counted in fixed steps, never in seconds and never in frames: the grid releases on
         * the same tick every run, and `dt` is deliberately not consulted here at all.
         *
         * Checked before it is decremented, not after. Releasing on the tick that reaches zero
         * would hand that tick's controls straight to the driver, because the gating in the
         * pre-physics stage reads the phase this function just changed — so an N-tick countdown
         * would hold the grid for N-1 ticks and a one-tick countdown would hold it for none.
         * A countdown of N ticks now holds the car for N ticks and goes green on the next. */
        if (session->countdownTicksRemaining > 0) {
            session->countdownTicksRemaining--;
            return; /* still held, and the race clock does not run before green */
        }
        set_phase(session, RACE_PHASE_RUNNING);
    }

    session->clockS += dt;
}

void race_session_update_rules(RaceSession *session)
{
    if (session == NULL) return;
    /* Nothing finishes before green, and a countdown tick has already returned above. */
    if (session->phase != RACE_PHASE_RUNNING && session->phase != RACE_PHASE_FINISHING) return;

    /* Award finishing positions in ascending EntrantId, so two entrants completing the distance
     * on the same tick are ordered by identity rather than by storage accident. */
    for (int i = 0; i < session->roster.count; i++) {
        RaceEntrant *entrant = &session->roster.entrants[i];
        if (entrant->result.finished) continue;
        if (!entrant_has_finished_distance(entrant, &session->rules)) continue;

        entrant->result.finished = true;
        entrant->result.finishPosition = ++session->classifiedCount;
        entrant->result.finishTimeS = session->clockS + entrant->result.penaltyTimeS;
        if (session->classifiedCount == 1) session->firstFinisherClockS = session->clockS;
        race_session_log_event(session, RACE_EVENT_ENTRANT_FINISHED, entrant->id,
                               entrant->result.finishPosition);
    }

    if (session->classifiedCount <= 0) return;

    /* The mode's whole job: a time trial ends with the first finisher, a race waits for the
     * field. Neither reads a global, and adding a third mode adds a case here and nowhere
     * else. */
    const bool allDone = every_entrant_finished(session);
    const bool windowExpired =
        (session->rules.finishingWindowS > 0.0f &&
         session->clockS - session->firstFinisherClockS > session->rules.finishingWindowS);
    const bool classified =
        (session->rules.mode == RACE_MODE_TIME_TRIAL) ? true : (allDone || windowExpired);
    if (classified) {
        /* Finishing window closed: award remaining active entrants their live-order positions
         * as DNF, so a stuck or lapped car still gets a deterministic result. */
        if (!allDone) {
            int order[RACE_MAX_ENTRANTS];
            const int n = race_session_live_order(session, order, RACE_MAX_ENTRANTS);
            for (int k = 0; k < n; k++) {
                RaceEntrant *e = &session->roster.entrants[order[k]];
                if (e->result.finished) continue;
                e->result.finished = true;
                e->result.finishPosition = ++session->classifiedCount;
                e->result.finishTimeS = 0.0f; /* DNF: no finish time */
            }
        }
        capture_results(session);
        set_phase(session, RACE_PHASE_CLASSIFIED);
    } else {
        set_phase(session, RACE_PHASE_FINISHING);
    }
}

bool race_session_place_grid(RaceSession *session, const TrackDefinition *track)
{
    if (session == NULL || track == NULL) return false;
    for (int i = 0; i < session->roster.count; i++) {
        RaceEntrant *entrant = &session->roster.entrants[i];
        Vector2 pos = { 0.0f, 0.0f };
        float heading = 0.0f;
        const int slot = (entrant->gridSlot >= 0) ? entrant->gridSlot : i;
        if (!track_grid_pose_at(track, slot, &pos, &heading)) return false;
        vehicle_instance_reset(&entrant->instance);
        controller_reset(&entrant->controller);
        entrant->instance.vehicle.positionM = pos;
        entrant->instance.vehicle.headingRad = heading;
        entrant->instance.renderState.prevPositionM = pos;
        entrant->instance.renderState.currPositionM = pos;
        entrant->instance.renderState.prevHeadingRad = heading;
        entrant->instance.renderState.currHeadingRad = heading;
        track_reset_progress_at(&entrant->progress, track, 0);
        entrant->result.falseStarted = false;
    }
    return true;
}

void race_session_record_false_start(RaceSession *session, const RaceRules *rules,
                                     EntrantId entrantId)
{
    if (session == NULL || entrantId == RACE_ENTRANT_ID_NONE) return;
    RaceEntrant *entrant = race_roster_find(&session->roster, entrantId);
    if (entrant == NULL || entrant->result.falseStarted) return;
    entrant->result.falseStarted = true;
    float penalty = 0.0f;
    if (rules != NULL) penalty = rules->falseStartPenaltyS;
    if (penalty > 0.0f) entrant->result.penaltyTimeS += penalty;
    race_session_log_event(session, RACE_EVENT_FALSE_START, entrantId,
                           (int32_t)(penalty * 100.0f + 0.5f));
}

int race_session_live_order(const RaceSession *session, int *entrantIndices, int maxCount)
{
    if (session == NULL || entrantIndices == NULL || maxCount <= 0) return 0;
    const int n = session->roster.count;
    int order[RACE_MAX_ENTRANTS];
    for (int i = 0; i < n; i++) order[i] = i;
    /* Insertion sort by race distance descending; finished entrants first; tie-break ascending
     * EntrantId (which is roster storage order — ascending id). Stable and small-N. */
    for (int i = 1; i < n; i++) {
        const int key = order[i];
        int j = i - 1;
        while (j >= 0) {
            const RaceEntrant *a = &session->roster.entrants[order[j]];
            const RaceEntrant *b = &session->roster.entrants[key];
            const float da = a->progress.raceDistanceM;
            const float db = b->progress.raceDistanceM;
            const bool aAhead = a->result.finished || (da > db);
            const bool tie = (da == db);
            if (aAhead && !tie) break;
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }
    const int written = (n < maxCount) ? n : maxCount;
    for (int i = 0; i < written; i++) entrantIndices[i] = order[i];
    return written;
}

float race_session_fastest_lap(const RaceSession *session, EntrantId *outEntrantId)
{
    if (session == NULL) {
        if (outEntrantId != NULL) *outEntrantId = RACE_ENTRANT_ID_NONE;
        return 0.0f;
    }
    float best = 0.0f;
    EntrantId bestId = RACE_ENTRANT_ID_NONE;
    for (int i = 0; i < session->roster.count; i++) {
        const RaceEntrant *e = &session->roster.entrants[i];
        const float t = e->progress.bestLapTimeS;
        if (t > 0.0f && (best == 0.0f || t < best)) {
            best = t;
            bestId = e->id;
        }
    }
    if (outEntrantId != NULL) *outEntrantId = bestId;
    return best;
}

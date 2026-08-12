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
}

void race_session_init(RaceSession *session)
{
    if (session == NULL) return;
    memset(session, 0, sizeof(*session));
    race_roster_init(&session->roster);
    race_rules_set_default(&session->rules);
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
    memset(results, 0, sizeof(*results));

    for (int position = 1; position <= session->roster.count; position++) {
        for (int i = 0; i < session->roster.count; i++) {
            const RaceEntrant *entrant = &session->roster.entrants[i];
            if (!entrant->result.finished || entrant->result.finishPosition != position)
                continue;
            results->rows[results->count++] =
                (RaceResultRow){ .entrantId = entrant->id,
                                 .finishPosition = entrant->result.finishPosition,
                                 .lapsCompleted = entrant->progress.lap,
                                 .finishTimeS = entrant->result.finishTimeS,
                                 .lastLapTimeS = entrant->progress.lastLapTimeS,
                                 .finished = true };
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
                             .finished = false };
    }
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
        /* Recovery penalties (issue #28) land on the entrant's classified time. */
        entrant->result.finishTimeS = session->clockS + entrant->result.penaltyTimeS;
        race_session_log_event(session, RACE_EVENT_ENTRANT_FINISHED, entrant->id,
                               entrant->result.finishPosition);
    }

    if (session->classifiedCount <= 0) return;

    /* The mode's whole job: a time trial ends with the first finisher, a race waits for the
     * field. Neither reads a global, and adding a third mode adds a case here and nowhere
     * else. */
    const bool classified =
        (session->rules.mode == RACE_MODE_TIME_TRIAL) ? true : every_entrant_finished(session);
    if (classified) {
        capture_results(session);
        set_phase(session, RACE_PHASE_CLASSIFIED);
    } else {
        set_phase(session, RACE_PHASE_FINISHING);
    }
}

/*
 * race_session.c — race lifecycle, finish rules, deterministic events, and classification.
 *
 * Everything here is plain value manipulation on a caller-owned RaceSession, driven by an
 * explicit `dt`. Nothing allocates, nothing reads the clock, and nothing touches presentation,
 * so a whole race can be run headlessly and hashed.
 */
#include "game/race_session.h"

#include <math.h>
#include <stdio.h>
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
    rules->pitLaneEnabled = false;
    rules->pitServiceTimeS = PIT_SERVICE_TIME_S;
    rules->pitSpeedLimitMps = PIT_SPEED_LIMIT_MPS;
    rules->pitMandatory = false;
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

const char *race_status_name(RaceFinalStatus status)
{
    switch (status) {
        case RACE_STATUS_RUNNING: return "RUNNING";
        case RACE_STATUS_FINISHED: return "FINISHED";
        case RACE_STATUS_DNF: return "DNF";
        case RACE_STATUS_DNS: return "DNS";
        case RACE_STATUS_DSQ: return "DSQ";
        case RACE_STATUS_RETIRED: return "RETIRED";
        case RACE_STATUS_COUNT:
        default: return "?";
    }
}

/*
 * A car that never took the start has covered essentially no distance. One car length is the
 * threshold: less than that and nothing that could be called a race happened, more and the
 * entrant raced and did not finish. Rolling forward on the grid is not taking the start.
 */
#define RACE_DNS_DISTANCE_M 5.0f

/*
 * Give every entrant a final status, once, immediately before the results are captured.
 *
 * Statuses already decided are left alone — a completed distance, an exclusion, a withdrawal.
 * What is left to decide is the difference between a car that raced and did not finish and one
 * that never took the start, which is the only place that distinction can be drawn: the session
 * is the authority on how far each entrant got. Nothing may reach a results screen still saying
 * RUNNING, which is what makes RACE_STATUS_RUNNING safe to use as the zero value.
 */
static void finalize_statuses(RaceSession *session)
{
    for (int i = 0; i < session->roster.count; i++) {
        RaceEntrant *e = &session->roster.entrants[i];
        if (e->result.finalStatus != RACE_STATUS_RUNNING) continue;
        const bool tookTheStart =
            (e->progress.lap > 0) || (e->progress.raceDistanceM > RACE_DNS_DISTANCE_M);
        e->result.finalStatus = tookTheStart ? RACE_STATUS_DNF : RACE_STATUS_DNS;
    }
}

/* Fill one row from an entrant. Everything a results screen needs is copied, so a row survives
 * the roster being rebuilt for the next session. */
static RaceResultRow result_row_from(const RaceSession *session, const RaceEntrant *entrant)
{
    RaceResultRow row;
    memset(&row, 0, sizeof(row));
    row.entrantId = entrant->id;
    (void)snprintf(row.carId, sizeof(row.carId), "%s", entrant->definition.id);
    row.finishPosition = entrant->result.finishPosition;
    row.lapsCompleted = entrant->progress.lap;
    row.finishTimeS = entrant->result.finishTimeS;
    row.lastLapTimeS = entrant->progress.lastLapTimeS;
    row.bestLapTimeS = entrant->progress.bestLapTimeS;
    row.finished = entrant->result.finished;
    row.status = (RaceFinalStatus)entrant->result.finalStatus;
    row.penaltyTimeS = entrant->result.penaltyTimeS;
    row.penaltyCount = 0;
    for (int p = 0; p < session->penalties.count; p++) {
        const int idx =
            (session->penalties.head + RACE_PENALTY_CAPACITY - session->penalties.count + p) %
            RACE_PENALTY_CAPACITY;
        if (session->penalties.items[idx].entrantId == entrant->id) row.penaltyCount++;
    }
    return row;
}

/*
 * Written once, on entry to CLASSIFIED.
 *
 * Order is the classification order a results table shows top to bottom: entrants holding a
 * finishing position first, in that order, then everyone who holds none. The second group is the
 * excluded and the withdrawn — a DSQ has no position by definition, so appending it below the
 * classified field is the only placement that does not imply one.
 */
static void capture_results(RaceSession *session)
{
    RaceResults *results = &session->results;
    float winnerTimeS = 0.0f;

    for (int position = 1; position <= session->roster.count; position++) {
        for (int i = 0; i < session->roster.count; i++) {
            const RaceEntrant *entrant = &session->roster.entrants[i];
            if (!entrant->result.finished || entrant->result.finishPosition != position)
                continue;
            RaceResultRow row = result_row_from(session, entrant);
            row.gapToLeaderS =
                (position == 1) ? 0.0f : (entrant->result.finishTimeS - winnerTimeS);
            /* A gap only means something between two entrants that both completed the
             * distance. A DNF has no finish time, so reporting one would be arithmetic on a
             * zero. */
            if (row.status != RACE_STATUS_FINISHED) row.gapToLeaderS = 0.0f;
            results->rows[results->count++] = row;
            if (position == 1) winnerTimeS = entrant->result.finishTimeS;
        }
    }
    for (int i = 0; i < session->roster.count; i++) {
        const RaceEntrant *entrant = &session->roster.entrants[i];
        if (entrant->result.finished && entrant->result.finishPosition > 0) continue;
        results->rows[results->count++] = result_row_from(session, entrant);
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
        entrant->result.finalStatus = RACE_STATUS_FINISHED;
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
        finalize_statuses(session);
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

/*
 * Where an entrant belongs in the running order, before distance is considered.
 *
 *   0  completed the distance — ahead of everyone still on track
 *   1  still racing            — ordered among themselves by distance
 *   2  out of the race         — excluded, withdrawn, or classified without finishing
 */
static int live_order_tier(const RaceEntrant *entrant)
{
    switch ((RaceFinalStatus)entrant->result.finalStatus) {
        case RACE_STATUS_FINISHED: return 0;
        case RACE_STATUS_RUNNING: return 1;
        default: return 2;
    }
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
            /* Three tiers, then distance inside a tier. Distance alone is not enough once an
             * entrant can be excluded: the car with the most distance covered may be the one
             * the stewards just threw out, and it would otherwise be shown leading the race it
             * is no longer in. With no status set — every entrant still running — this is the
             * distance comparison it has always been. */
            const int ta = live_order_tier(a);
            const int tb = live_order_tier(b);
            const bool aAhead = (ta < tb) || ((ta == tb) && (da > db));
            const bool tie = (ta == tb) && (da == db);
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

bool race_session_add_penalty(RaceSession *session, RacePenaltyRule rule,
                              RacePenaltyConsequence consequence, EntrantId entrantId,
                              float penaltySeconds, int32_t evidence)
{
    if (session == NULL || rule < 0 || rule >= PENALTY_RULE_COUNT || consequence < 0 ||
        consequence >= PENALTY_CONSEQUENCE_COUNT)
        return false;
    RaceEntrant *entrant = race_roster_find(&session->roster, entrantId);
    if (entrant == NULL) return false;

    RacePenaltyLog *log = &session->penalties;
    RacePenalty *p = &log->items[log->head];
    p->rule = rule;
    p->consequence = consequence;
    p->entrantId = entrantId;
    p->tick = session->tick;
    p->timeS = session->clockS;
    p->penaltySeconds = penaltySeconds;
    p->evidence = evidence;
    p->served = false;
    log->head = (log->head + 1) % RACE_PENALTY_CAPACITY;
    if (log->count < RACE_PENALTY_CAPACITY) log->count++;
    log->totalAppended++;

    /* Apply the consequence immediately. */
    if (consequence == PENALTY_CONSEQUENCE_TIME && penaltySeconds > 0.0f) {
        entrant->result.penaltyTimeS += penaltySeconds;
        p->served = true;
    } else if (consequence == PENALTY_CONSEQUENCE_LAP_INVALID) {
        entrant->progress.lapInvalid = true;
        entrant->progress.lastLapInvalidReason = 1;
        p->served = true;
    } else if (consequence == PENALTY_CONSEQUENCE_DISQUALIFICATION) {
        /* Exclusion ends the entrant's race here. It is classified so the field can complete,
         * but holds no finishing position and no finish time: a disqualified car did not place,
         * and giving it a position would push a legitimate finisher down the order. A second
         * disqualification for an already-excluded entrant records the decision and changes
         * nothing else. */
        if (entrant->result.finalStatus == RACE_STATUS_RUNNING) {
            entrant->result.finalStatus = RACE_STATUS_DSQ;
            entrant->result.finished = true;
            entrant->result.finishPosition = 0;
            entrant->result.finishTimeS = 0.0f;
        }
        p->served = true;
    }
    return true;
}

int race_session_penalty_count(const RaceSession *session, EntrantId entrantId,
                               RacePenaltyRule rule)
{
    if (session == NULL) return 0;
    int count = 0;
    for (int i = 0; i < session->penalties.count; i++) {
        const int idx =
            (session->penalties.head + RACE_PENALTY_CAPACITY - session->penalties.count + i) %
            RACE_PENALTY_CAPACITY;
        const RacePenalty *p = &session->penalties.items[idx];
        if (p->entrantId == entrantId && p->rule == rule) count++;
    }
    return count;
}

/*
 * The escalation ladder.
 *
 * One warning, then two time penalties, then exclusion. The shape matters more than the exact
 * numbers: a first offence is told to the driver and costs nothing, a repeat costs time, and a
 * driver who keeps doing it is removed. Expressed as a count-to-consequence mapping in one place
 * so that every rule escalates the same way and a detector never has to decide severity.
 */
#define RACE_PENALTY_WARNINGS 1
#define RACE_PENALTY_TIMED 3 /* offences 2 and 3 cost time; the 4th excludes */

RacePenaltyConsequence race_session_report_infringement(RaceSession *session,
                                                        RacePenaltyRule rule,
                                                        EntrantId entrantId,
                                                        float penaltySeconds, int32_t evidence)
{
    if (session == NULL) return PENALTY_CONSEQUENCE_COUNT;
    const RaceEntrant *entrant = race_roster_find_const(&session->roster, entrantId);
    if (entrant == NULL) return PENALTY_CONSEQUENCE_COUNT;
    /* An entrant whose race has already ended cannot be penalised further: the result is
     * final, and a second exclusion would only add rows nobody can act on. */
    if (entrant->result.finalStatus != RACE_STATUS_RUNNING) return PENALTY_CONSEQUENCE_COUNT;

    const int prior = race_session_penalty_count(session, entrantId, rule);
    RacePenaltyConsequence consequence;
    if (prior < RACE_PENALTY_WARNINGS) {
        consequence = PENALTY_CONSEQUENCE_WARNING;
    } else if (prior < RACE_PENALTY_TIMED) {
        consequence = PENALTY_CONSEQUENCE_TIME;
    } else {
        consequence = PENALTY_CONSEQUENCE_DISQUALIFICATION;
    }
    if (!race_session_add_penalty(session, rule, consequence, entrantId, penaltySeconds,
                                  evidence)) {
        return PENALTY_CONSEQUENCE_COUNT;
    }
    return consequence;
}

bool race_session_retire(RaceSession *session, EntrantId entrantId)
{
    if (session == NULL) return false;
    RaceEntrant *entrant = race_roster_find(&session->roster, entrantId);
    if (entrant == NULL) return false;
    if (entrant->result.finalStatus != RACE_STATUS_RUNNING) return false;
    entrant->result.finalStatus = RACE_STATUS_RETIRED;
    entrant->result.finished = true;
    entrant->result.finishPosition = 0;
    entrant->result.finishTimeS = 0.0f;
    return true;
}

int race_session_pending_penalties(const RaceSession *session, EntrantId entrantId)
{
    if (session == NULL) return 0;
    int pending = 0;
    for (int i = 0; i < session->penalties.count; i++) {
        const int idx =
            (session->penalties.head + RACE_PENALTY_CAPACITY - session->penalties.count + i) %
            RACE_PENALTY_CAPACITY;
        if (!session->penalties.items[idx].served &&
            session->penalties.items[idx].entrantId == entrantId)
            pending++;
    }
    return pending;
}

/*
 * race_session.h — the authority for one race: who is in it, what the rules are, what phase it
 * is in, and when it is over.
 *
 * WHY THIS EXISTS. Race lifecycle used to be four screen enums and a lap comparison buried in
 * the middle of game_fixed_update(). `GameStateId` answered "which screen is up", and the same
 * value was also asked "is the car being simulated" and "is the run finished". Those are three
 * different questions, and only the first one belongs to the UI. `RaceSession` owns the other
 * two, so a headless test can drive a race from configuration to classified results without a
 * window, and a second race mode does not need a new global.
 *
 * SCREENS ARE NOT PHASES. `GameStateId` still selects what the HUD draws (see
 * render_hud_draw_state_overlay). `RacePhase` says what the race is doing. They move together
 * for the single-screen layouts that exist today — a classified session shows the results
 * overlay — but they are separate axes and the simulation gate reads both: a session only steps
 * when its screen is up AND its phase is one that runs.
 *
 * TIME. `tick` counts the ticks this session actually simulated, countdown included; `clockS`
 * counts green-flag seconds and so does not start until the grid is released. Neither advances
 * while the session is paused, which is what makes "pause freezes race time" a property rather
 * than a hope. Both are established at the top of a tick by race_session_begin_tick(), so every
 * event raised anywhere in that tick carries the same stamp. The platform's own fixed-tick
 * counter (`Game.sim.tick`) keeps counting regardless, because it times the application, not
 * the race — a restarted session rewinds its clock while the application tick does not.
 *
 * HOT RELOAD. Plain value data with no function pointers and no pointers into the reloadable
 * module, like everything else in the persistent Game block. The event log is a fixed ring, so
 * a session never allocates.
 *
 * See docs/SIMULATION_OWNERSHIP.md for the ownership table and the fixed-step stage contract.
 */
#ifndef CIRCUIT_RACE_SESSION_H
#define CIRCUIT_RACE_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include "game/race_entrant.h"
#include "world/track.h"

/*
 * Where a race is in its life.
 *
 * CONFIGURING is the only phase that does not simulate and is not an ending: it is the state a
 * zeroed session starts in, before anyone has said what is being raced.
 */
typedef enum {
    RACE_PHASE_CONFIGURING = 0, /* content and rules being chosen; nothing runs */
    RACE_PHASE_GRID,            /* entrants placed, waiting to be released */
    RACE_PHASE_COUNTDOWN,       /* holding on the grid; simulates, controls are gated off */
    RACE_PHASE_RUNNING,         /* green; the normal racing phase */
    RACE_PHASE_FINISHING,       /* someone has finished, others are still running */
    RACE_PHASE_CLASSIFIED,      /* everyone who can finish has; results are final */
    RACE_PHASE_PAUSED,          /* simulation and race clock frozen; resumes to resumePhase */
    RACE_PHASE_ABORTED,         /* abandoned before classification; no results */
    RACE_PHASE_COUNT
} RacePhase;

/*
 * What finishing means.
 *
 * The difference is real, not decorative: a time trial is over for everybody the moment the
 * driver being timed is done, whereas a race keeps running until the last entrant who can
 * finish has. That is the whole of the mode's behaviour — it selects a finish condition, it
 * does not gate physics, content, or scoring.
 */
typedef enum {
    RACE_MODE_TIME_TRIAL = 0, /* classify as soon as the first entrant completes the distance */
    RACE_MODE_RACE,           /* classify when every entrant has finished */
    RACE_MODE_COUNT
} RaceMode;

/* Frozen for the duration of a session. Copied in at start; changing a caller's copy later
 * cannot reach a running race. */
typedef struct {
    RaceMode mode;
    /* Laps that complete the distance. The finish test is `lap >= targetLaps`, unchanged from
     * the comparison this replaced, so a session configured with a non-positive target
     * classifies on its first tick exactly as it always did. */
    int targetLaps;
    /* Seconds held on the grid before green. 0 releases immediately, which is what every
     * existing caller gets and why no recorded trace moves. Resolved to whole fixed steps at
     * start and clamped to RACE_COUNTDOWN_MAX_TICKS. */
    float countdownS;
    /* Collision damage policy (issue #28): 0 = off (contact physics unchanged), 1 = cosmetic
     * (damage accumulates for presentation only), 2 = mechanical (damage also degrades
     * engine/aero/grip in the solver). Default 0 preserves every recorded baseline. */
    int damageMode;
    /* Deterministic stuck recovery (issue #28): when true, an entrant stalled for
     * stuckRecoveryDelayS is repositioned onto its localized route pose with a time penalty
     * and a session event. Default false: no trace changes. */
    bool stuckRecoveryEnabled;
    float stuckRecoveryDelayS;
    float falseStartPenaltyS; /* added to finish time when an entrant jumps the start (#49) */
    float falseStartSpeedMps; /* speed above which a held car is flagged for a jump (0 = off) */
    float
        finishingWindowS; /* race: seconds after the first finisher before DNF'ing the rest (#54) */
    bool trackLimitsEnabled; /* when true, off-track entries produce steward penalties (#55) */
} RaceRules;

typedef enum {
    DAMAGE_OFF = 0,   /* contact physics untouched; damage not accumulated */
    DAMAGE_COSMETIC,  /* damage accumulated for HUD/presentation, no physics effect */
    DAMAGE_MECHANICAL /* damage accumulated AND fed to the solver (engine/aero/grip) */
} RaceDamageMode;

/* Deterministic session environment (issue #41): weather/wetness/temperature state that
 * evolves per tick and feeds physical grip and tire thermal behavior. All-zero/dry defaults
 * reproduce the approved baseline. Purely visual fields (time of day, scenery region) are
 * presentation hints that must never enter the checksum; the physical fields are
 * authoritative. */
typedef struct {
    float precipitation;          /* 0..1 rain intensity */
    float ambientTempC;           /* ambient air temperature */
    float trackTempC;             /* track surface temperature, evolves toward ambient */
    float wetness[SURFACE_COUNT]; /* 0..1 water film per surface id */
    float timeOfDayHours;         /* presentation-only: 0..24 (excluded from the checksum) */
    char region[32];              /* presentation-only: scenery/audio region id */
} RaceEnvironment;

/* Defaults: dry, 20 C ambient, no rain. */
void race_environment_set_default(RaceEnvironment *env);
/* Deterministic per-tick evolution: rain adds wetness, drainage and evaporation dry it, and
 * the track temperature relaxes toward ambient. Bounded. */
void race_environment_update(RaceEnvironment *env, float dt);

/* Upper bound on a resolved countdown, in fixed steps — ten minutes at the simulation rate.
 * It exists so that converting a nonsense or non-finite countdown to a tick count is defined
 * behaviour rather than whatever the cast happens to produce. */
#define RACE_COUNTDOWN_MAX_TICKS 72000

typedef enum {
    RACE_EVENT_PHASE_CHANGED = 0, /* value is the new RacePhase */
    RACE_EVENT_LAP_COMPLETED,     /* value is the lap number just completed */
    RACE_EVENT_SECTOR_COMPLETED,  /* value is the sector index just crossed */
    RACE_EVENT_ENTRANT_FINISHED,  /* value is the finishing position awarded */
    RACE_EVENT_STUCK_RECOVERED,   /* value is the running penalty count for that entrant */
    RACE_EVENT_FALSE_START,       /* value is the penalty seconds x100 (rounded) */
    RACE_EVENT_KIND_COUNT
} RaceEventKind;

/*
 * One thing that happened, ordered. Events are materialised in the rules stage and consumed
 * afterwards by presentation and tests; nothing reads an event back into the simulation, which
 * is what keeps the log a report rather than a channel.
 */
typedef struct {
    RaceEventKind kind;
    uint64_t tick;       /* session tick it was raised on */
    float timeS;         /* session clock at that tick */
    EntrantId entrantId; /* RACE_ENTRANT_ID_NONE for session-wide events */
    int32_t value;       /* meaning depends on kind; see above */
} RaceEvent;

#define RACE_EVENT_CAPACITY 64

/* Fixed ring. `totalAppended` keeps counting past the capacity, so a consumer can tell that it
 * missed events instead of silently seeing a short list. */
typedef struct {
    RaceEvent items[RACE_EVENT_CAPACITY];
    int head;
    int count;
    uint32_t totalAppended;
} RaceEventLog;

/* ---- Penalty system (issue #55) ---- */

typedef enum {
    PENALTY_RULE_TRACK_LIMITS = 0,  /* all four wheels beyond the racing surface */
    PENALTY_RULE_CUT,               /* gained position by leaving and rejoining ahead */
    PENALTY_RULE_WRONG_WAY,         /* driving against the route direction */
    PENALTY_RULE_FALSE_START,       /* moved before green (#49) */
    PENALTY_RULE_AVOIDABLE_CONTACT, /* contact deemed the entrant's fault */
    PENALTY_RULE_PIT_SPEED,         /* exceeded the pit-lane speed limit */
    PENALTY_RULE_COUNT
} RacePenaltyRule;

typedef enum {
    PENALTY_CONSEQUENCE_WARNING = 0,
    PENALTY_CONSEQUENCE_TIME,        /* seconds added to finish time */
    PENALTY_CONSEQUENCE_LAP_INVALID, /* this lap does not count for records */
    PENALTY_CONSEQUENCE_COUNT
} RacePenaltyConsequence;

/* One steward decision. Evidence fields are rule-specific (e.g. off-track ticks for track
 * limits, speed for pit speeding). `served` flips when the consequence has been applied. */
typedef struct {
    RacePenaltyRule rule;
    RacePenaltyConsequence consequence;
    EntrantId entrantId;
    uint64_t tick;
    float timeS;
    float penaltySeconds; /* >0 for time penalties */
    int32_t evidence;     /* rule-specific (e.g. consecutive off-track ticks) */
    bool served;
} RacePenalty;

#define RACE_PENALTY_CAPACITY 64

typedef struct {
    RacePenalty items[RACE_PENALTY_CAPACITY];
    int head;
    int count;
    uint32_t totalAppended;
} RacePenaltyLog;

/* One entrant's line in the results, ordered by finishing position. */
typedef struct {
    EntrantId entrantId;
    int finishPosition; /* 1-based; 0 when the entrant never finished */
    int lapsCompleted;
    float finishTimeS; /* session clock when this entrant finished; 0 when it did not */
    float lastLapTimeS;
    bool finished;
    float bestLapTimeS; /* this entrant's best valid lap across the session (#54) */
    float gapToLeaderS; /* time behind the leader at classification (0 for the winner) (#54) */
} RaceResultRow;

/* Filled once, when the session reaches RACE_PHASE_CLASSIFIED. */

typedef struct {
    RaceResultRow rows[RACE_MAX_ENTRANTS];
    int count;
    bool valid;
    float fastestLapTimeS;         /* the fastest valid lap across the field (0 = none) (#54) */
    EntrantId fastestLapEntrantId; /* who set it (NONE = no valid lap) (#54) */
} RaceResults;

typedef struct {
    /* MUST be first: Game overlays its one-entrant compatibility view onto entrants[0], which
     * means onto the head of this struct. game.h asserts it. */
    RaceRoster roster;

    RacePhase phase;
    /* What a resume returns to. Only meaningful while phase is RACE_PHASE_PAUSED. */
    RacePhase resumePhase;
    RaceRules rules;

    /* Selected content, by stable id. The TrackDefinition itself still lives on Game (trackDef)
     * and the session records which id that definition came from; empty means no track selected
     * yet (the initial configuring phase before any configure call). */
    char trackId[TRACK_ID_CHARS];

    /* Session time. `tick` counts simulated ticks including the countdown; `clockS` counts
     * green-flag seconds only. Neither advances while paused, finished, or configuring. */
    uint64_t tick;
    float clockS;
    /* Fixed steps left before green, counted as an integer rather than a shrinking float.
     * Subtracting dt from seconds leaves a residue that can keep a three-step countdown alive
     * for a fourth step, which would make the grid release depend on floating-point luck. */
    int countdownTicksRemaining;

    int classifiedCount; /* finishing positions awarded so far */
    float
        firstFinisherClockS; /* session clock when the first entrant finished (0 = none yet) (#54) */
    RaceEventLog events;
    RaceResults results;
    RacePenaltyLog penalties;
    /* Deterministic session environment (issue #41): wetness, temperatures, precipitation.
     * Physical fields are authoritative (checksummed); presentation fields are excluded. */
    RaceEnvironment environment;
} RaceSession;

/* Zero the session into RACE_PHASE_CONFIGURING with an empty roster and default rules. */
void race_session_init(RaceSession *session);

/* Default rules: a time trial over RESULTS_TARGET_LAPS with no countdown. */
void race_rules_set_default(RaceRules *rules);

/*
 * Freeze `rules` and put the session on the grid, then release it: the phase becomes
 * RACE_PHASE_COUNTDOWN when the rules ask for one and RACE_PHASE_RUNNING when they do not.
 * Clock, tick, countdown, classification, events and results all reset; the roster and its
 * entrants are left alone, because who is racing is not what starting a session decides.
 * Passing NULL rules keeps whatever the session already had.
 */
void race_session_start(RaceSession *session, const RaceRules *rules);

/* True while the fixed update should advance the simulation: countdown, running, or finishing.
 * Configuring, grid, paused, classified and aborted all stand still. */
bool race_session_is_simulating(const RaceSession *session);

/*
 * Phase transitions. Each is a no-op when the requested move is not legal from the current
 * phase, and each returns whether it changed anything, so a caller can assert transitions
 * rather than assume them.
 */
bool race_session_pause(RaceSession *session);
bool race_session_resume(RaceSession *session);
bool race_session_abort(RaceSession *session);

/* Append an event. Silently drops nothing: the ring overwrites its oldest entry and
 * `totalAppended` records the truth. */
void race_session_log_event(RaceSession *session, RaceEventKind kind, EntrantId entrantId,
                            int32_t value);

/* The most recent event of any kind, or NULL when none has been raised. */
const RaceEvent *race_session_last_event(const RaceSession *session);

/*
 * Open a simulating tick: advance the session tick, count down the grid release, and advance
 * the race clock once green. Call it once, before the tick's other stages, so that every event
 * they raise is stamped with the same tick and clock. Does nothing when the session is not in a
 * simulating phase.
 */
void race_session_begin_tick(RaceSession *session, float dt);

/*
 * The rules stage, run once per simulating tick after progress has been updated.
 *
 * Marks entrants that have completed the distance, awards finishing positions in the order they
 * finish, and moves the phase to FINISHING and then CLASSIFIED as the mode's finish condition
 * is met. Writes the results snapshot exactly once, on entry to CLASSIFIED. Takes no `dt`: time
 * belongs to race_session_begin_tick(), and a rule that needed the step would be reading the
 * clock twice.
 */
void race_session_update_rules(RaceSession *session);

/*
 * Place every occupied entrant on its validated grid slot (issue #49). Each entrant is reset to
 * a clean vehicle state and positioned at the staggered grid pose for its slot, so no two cars
 * overlap and nobody carries hidden drivetrain state into the launch. Call after
 * race_session_start() has built the grid. Returns false when any entrant's slot has no pose.
 */
bool race_session_place_grid(RaceSession *session, const TrackDefinition *track);

/*
 * Record a jump start (issue #49): adds the configured penalty to the entrant's accumulated
 * penalty time and logs a FALSE_START event. Idempotent per entrant per session — a second
 * report for the same entrant is ignored, so a held car that creeps for several ticks is
 * penalized once.
 */
void race_session_record_false_start(RaceSession *session, const RaceRules *rules,
                                     EntrantId entrantId);

/*
 * Authoritative live running order (issue #54). Fills `entrantIndices` with roster indices
 * sorted by race distance (descending), tie-broken by ascending EntrantId. Finished entrants
 * sort ahead of active ones. Returns the number of indices written (min(count, maxCount)).
 * This is the one source of truth for "who is P1" — the HUD reads it and computes nothing.
 */
int race_session_live_order(const RaceSession *session, int *entrantIndices, int maxCount);

/*
 * The fastest valid lap across the field so far (issue #54). Writes 0 / RACE_ENTRANT_ID_NONE
 * when no entrant has recorded a valid lap. Read-only; updated inside the rules stage.
 */
float race_session_fastest_lap(const RaceSession *session, EntrantId *outEntrantId);

/* True once the session has reached a phase from which it will not simulate again. */
bool race_session_is_over(const RaceSession *session);

/*
 * Record a steward penalty (issue #55). Appends to the penalty log and, for time consequences,
 * adds the penalty seconds to the entrant's accumulated result penalty immediately. For lap-
 * invalidating consequences, sets the entrant's current lap invalid flag. Returns false when the
 * entrant is not found or the rule/consequence are out of range.
 */
bool race_session_add_penalty(RaceSession *session, RacePenaltyRule rule,
                              RacePenaltyConsequence consequence, EntrantId entrantId,
                              float penaltySeconds, int32_t evidence);

/* How many unserved penalties an entrant has (issue #55). */
int race_session_pending_penalties(const RaceSession *session, EntrantId entrantId);

#endif /* CIRCUIT_RACE_SESSION_H */

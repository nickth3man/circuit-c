/*
 * session_config.h — issue #48: the player-facing pre-race configuration bundle.
 *
 * A SessionConfig is the answer to "what are we racing?": the track, the player's car and
 * assists, the AI field, the rules (mode/distance/countdown/damage/recovery), and the
 * environment. It is validated before allocation and applied by game_configure_session(),
 * which turns a config into a live session in one call — the one non-dev path to "start a
 * spawn, and the apply step is infallible given a validated config.
 *
 * Serialization is deterministic (fixed field order) and widget-order independent, so a
 * replay's metadata describes what was raced, not how it was chosen.
 */
#ifndef CIRCUIT_SESSION_CONFIG_H
#define CIRCUIT_SESSION_CONFIG_H

#include <stdio.h>

#include "game/race_session.h" /* RaceMode, RaceEnvironment, RACE_MAX_ENTRANTS */

#define SESSION_CONFIG_ID_CHARS 32

/* AI difficulty tiers, named for the HUD. */
#define SESSION_DIFFICULTY_EASY 0
#define SESSION_DIFFICULTY_MEDIUM 1
#define SESSION_DIFFICULTY_HARD 2
#define SESSION_DIFFICULTY_COUNT 3

/* Assist bounds, matching VehicleSetup's ABS/TCS levels. */
#define SESSION_ASSIST_MAX 3

typedef struct {
    char trackId[SESSION_CONFIG_ID_CHARS];
    char carId[SESSION_CONFIG_ID_CHARS];
    int aiCount;      /* 0..RACE_MAX_ENTRANTS-1 AI opponents */
    int aiDifficulty; /* SESSION_DIFFICULTY_* */
    RaceMode mode;    /* time trial (solo) or race */
    int targetLaps;   /* >= 1 */
    float countdownS; /* >= 0; 0 = immediate green */
    int damageMode;   /* DAMAGE_* */
    bool stuckRecoveryEnabled;
    int absLevel; /* player assists: 0..3 */
    int tcsLevel;
    RaceEnvironment environment;
    uint64_t seed; /* deterministic session seed */
} SessionConfig;

/* Defaults: chicane + the reference rwd_grip, solo time trial, 3 laps, dry weather, ABS 2 /
 * TCS 1 — a valid session on a clean install. */
void session_config_set_default(SessionConfig *cfg);

/*
 * Reject invalid combinations with an exact, player-facing reason in `reason`.
 * Checks: track and car exist and are selectable, mode is in range, aiCount is in
 * [0, RACE_MAX_ENTRANTS-1] and only nonzero in race mode, targetLaps >= 1, countdown >= 0,
 * damage mode in range, assists in [0,3], difficulty in range, and the environment's
 * physical fields are finite and bounded. Returns false (and writes a reason) on the first
 * violation.
 */
bool session_config_validate(const SessionConfig *cfg, char *reason, size_t reasonCap);

/* Deterministic JSON serialization in a fixed field order — widget-order independent.
 * Writes a compact, newline-terminated document. Returns false on a NULL argument. */
bool session_config_serialize(const SessionConfig *cfg, FILE *out);
/* Re-parse a serialized document into `cfg`. Returns false on a NULL argument, a missing
 * required field, or a value outside the validated ranges. A round-trip is byte-identical
 * for any config that passes session_config_validate(). */
bool session_config_deserialize(SessionConfig *cfg, const char *text);

#endif /* CIRCUIT_SESSION_CONFIG_H */

/*
 * player_profile.h — versioned player profile: last selections, settings, bindings,
 * accessibility, and best-lap records (issue #47).
 *
 * The profile is plain value data serialized as JSON through the existing parser plus a small
 * hand-rolled emitter (the parser has no writer). All persistence is atomic (tmp + rename),
 * migrations are explicit per version, and a corrupt file is backed up rather than silently
 * destroyed. Headless tests use player_profile_load_memory() and never touch user directories.
 *
 * Stable content IDs only — never file paths — so a profile survives content renames and can
 * report a missing car/track without crashing.
 */
#ifndef CIRCUIT_PLAYER_PROFILE_H
#define CIRCUIT_PLAYER_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "content/vehicle_manifest.h" /* VEHICLE_CONTENT_ID_CAPACITY */
#include "world/track.h"              /* TRACK_ID_CHARS */

#define PLAYER_PROFILE_VERSION 2u
#define PLAYER_PROFILE_ACTION_CHARS 32
/* Wide enough for the longest key id the game can bind — "KEY_RIGHT_CONTROL" is 17 characters
 * plus its terminator. The persisted names are unchanged; only the buffer that holds them
 * grew, because a modifier key silently truncating to "KEY_RIGHT_CONTRO" would be a binding
 * that never loads back. */
#define PLAYER_PROFILE_KEY_CHARS 24
#define PLAYER_PROFILE_RECORD_KEY_CHARS 64
#define PLAYER_PROFILE_MAX_BINDINGS 24
#define PLAYER_PROFILE_MAX_RECORDS 16
#define PLAYER_PROFILE_JSON_CAP 16384

typedef struct {
    uint32_t version;
    /* Last selections, as stable content ids. */
    char lastCarId[VEHICLE_CONTENT_ID_CAPACITY];
    char lastTrackId[TRACK_ID_CHARS];
    /* Audio settings, 0..1. */
    float masterVolume;
    float sfxVolume;
    float musicVolume;
    /* Video settings. */
    bool vsyncEnabled;
    float uiScale; /* 0.5 .. 2.0 */
    /* Accessibility. */
    bool reducedCameraShake;
    bool reducedFlashes;
    bool audioCountdownCue;
    /* Key bindings: action id -> key name. The last write to a key wins; a conflict is
     * reported by player_profile_rebind instead of silently clobbering. */
    struct {
        char action[PLAYER_PROFILE_ACTION_CHARS];
        char key[PLAYER_PROFILE_KEY_CHARS];
    } bindings[PLAYER_PROFILE_MAX_BINDINGS];
    int bindingCount;
    /* Best laps keyed by track + car content ids and a compatibility key (issue #50): the
     * key encodes the track geometry hash + car content hash + rules/assists policy, so a
     * record never compares across materially different physics. */
    struct {
        char trackId[TRACK_ID_CHARS];
        char carId[VEHICLE_CONTENT_ID_CAPACITY];
        char compatibilityKey[PLAYER_PROFILE_RECORD_KEY_CHARS];
        float bestLapTimeS;
    } records[PLAYER_PROFILE_MAX_RECORDS];
    int recordCount;
} PlayerProfile;

/* A fresh profile: current version, empty selections, default settings. */
void player_profile_default(PlayerProfile *profile);

/* Serialize to `out` (NUL-terminated). Returns the length written (excluding the NUL), or -1
 * when the profile cannot be represented in `cap` bytes. Deterministic field order. */
int player_profile_serialize(const PlayerProfile *profile, char *out, size_t cap);

/* Deserialize from a NUL-terminated buffer, migrating older versions. Returns false (with an
 * error in `error` when non-NULL) on malformed input; the profile is left as defaults on
 * failure. */
bool player_profile_deserialize(PlayerProfile *profile, const char *text, char *error,
                                size_t errorCap);

/* In-memory default profile: no filesystem access. All headless tests use this. */
void player_profile_load_memory(PlayerProfile *profile);

/* Atomic save: write `path.tmp` then rename over `path`, so an interrupted write leaves the
 * old file intact. Returns false with `error` set on failure. */
bool player_profile_save(const PlayerProfile *profile, const char *path, char *error,
                         size_t errorCap);

/* Load, migrating older versions. A corrupt file is moved to `path.corrupt` and defaults are
 * returned (the corrupt original is never destroyed). A missing file is NOT an error: it
 * returns defaults. Returns false only on I/O failure that prevents reading. */
bool player_profile_load(PlayerProfile *profile, const char *path, char *error,
                         size_t errorCap);

/* Bind `action` to `key`. Returns false (and leaves the profile untouched) when the key is
 * already bound to a DIFFERENT action, or when the binding tables are full. */
bool player_profile_rebind(PlayerProfile *profile, const char *action, const char *key);

/* The key currently bound to `action`, or NULL. */
const char *player_profile_bound_key(const PlayerProfile *profile, const char *action);

/* Record a best lap for a track/car pair under a compatibility key (issue #50: track
 * geometry hash + car content hash + rules/assists digest). Returns true when the record was
 * stored (new or improved); the existing record wins on a slower time, and a record with a
 * DIFFERENT key is never replaced or compared (the caller decides what to do with a new key —
 * the old record simply stays). Unknown content ids are still stored: they are stable
 * identifiers, and the game decides what to do with a missing one. */
bool player_profile_record_lap(PlayerProfile *profile, const char *trackId, const char *carId,
                               const char *compatibilityKey, float bestLapTimeS);

/* The stored best lap for a track/car pair under exactly this compatibility key, or 0.0 when
 * none exists. */
float player_profile_best_lap(const PlayerProfile *profile, const char *trackId,
                              const char *carId, const char *compatibilityKey);

#endif /* CIRCUIT_PLAYER_PROFILE_H */

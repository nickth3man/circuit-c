/*
 * player_profile.c — versioned player profile persistence (issue #47).
 *
 * Serialization is a tiny deterministic JSON emitter (the project's json.c is a parser only);
 * loading goes through json_parse + per-version migration. Writes are atomic
 * (tmp + rename). A corrupt file is renamed to `.corrupt` before defaults are returned, so
 * recoverable data is never destroyed.
 */
#include "game/player_profile.h"

#include <stdio.h>
#include <string.h>

#include "core/json.h"

static void profile_defaults(PlayerProfile *profile)
{
    memset(profile, 0, sizeof(*profile));
    profile->version = PLAYER_PROFILE_VERSION;
    profile->masterVolume = 1.0f;
    profile->sfxVolume = 1.0f;
    profile->musicVolume = 0.8f;
    profile->vsyncEnabled = true;
    profile->uiScale = 1.0f;
    profile->reducedCameraShake = false;
    profile->reducedFlashes = false;
    profile->audioCountdownCue = true;
}

void player_profile_default(PlayerProfile *profile)
{
    if (profile == NULL) return;
    profile_defaults(profile);
}

void player_profile_load_memory(PlayerProfile *profile)
{
    player_profile_default(profile);
}

int player_profile_serialize(const PlayerProfile *profile, char *out, size_t cap)
{
    if (profile == NULL || out == NULL || cap == 0) return -1;
    size_t used = 0;

#define EMIT(...)                                                    \
    do {                                                             \
        const int n = snprintf(out + used, cap - used, __VA_ARGS__); \
        if (n < 0 || (size_t)n >= cap - used) return -1;             \
        used += (size_t)n;                                           \
    } while (0)

    EMIT("{\"version\":%u,", profile->version);
    EMIT("\"lastCarId\":\"%s\",", profile->lastCarId);
    EMIT("\"lastTrackId\":\"%s\",", profile->lastTrackId);
    EMIT("\"masterVolume\":%.6f,", (double)profile->masterVolume);
    EMIT("\"sfxVolume\":%.6f,", (double)profile->sfxVolume);
    EMIT("\"musicVolume\":%.6f,", (double)profile->musicVolume);
    EMIT("\"vsyncEnabled\":%s,", profile->vsyncEnabled ? "true" : "false");
    EMIT("\"uiScale\":%.6f,", (double)profile->uiScale);
    EMIT("\"reducedCameraShake\":%s,", profile->reducedCameraShake ? "true" : "false");
    EMIT("\"reducedFlashes\":%s,", profile->reducedFlashes ? "true" : "false");
    EMIT("\"audioCountdownCue\":%s,", profile->audioCountdownCue ? "true" : "false");

    EMIT("\"bindings\":[");
    for (int i = 0; i < profile->bindingCount; i++) {
        if (i > 0) EMIT(",");
        EMIT("{\"action\":\"%s\",\"key\":\"%s\"}", profile->bindings[i].action,
             profile->bindings[i].key);
    }
    EMIT("],");

    EMIT("\"records\":[");
    for (int i = 0; i < profile->recordCount; i++) {
        if (i > 0) EMIT(",");
        EMIT("{\"trackId\":\"%s\",\"carId\":\"%s\",\"compatibilityKey\":\"%s\","
             "\"bestLapTimeS\":%.6f}",
             profile->records[i].trackId, profile->records[i].carId,
             profile->records[i].compatibilityKey, (double)profile->records[i].bestLapTimeS);
    }
    EMIT("]}");
#undef EMIT

    return (int)used;
}

/* Copy a bounded string field from JSON, rejecting overlong values (defensive: the emitter
 * only writes in-bounds strings, but a hand-edited profile must not overflow buffers). */
static bool copy_field(char *dst, size_t dstCap, const JsonValue *value, const char *name,
                       char *error, size_t errorCap)
{
    if (value == NULL) return true; /* absent: leave the default */
    const char *text = json_as_string(value);
    if (text == NULL) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "profile field '%s' must be a string", name);
        return false;
    }
    if (strlen(text) >= dstCap) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "profile field '%s' is too long", name);
        return false;
    }
    snprintf(dst, dstCap, "%s", text);
    return true;
}

bool player_profile_deserialize(PlayerProfile *profile, const char *text, char *error,
                                size_t errorCap)
{
    if (profile == NULL || text == NULL) return false;
    profile_defaults(profile);

    char parseError[128] = "";
    JsonDocument *doc = json_parse(text, 0, parseError, sizeof(parseError));
    if (doc == NULL) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "profile JSON: %s", parseError);
        return false;
    }
    const JsonValue *root = json_document_root(doc);
    if (root == NULL || !json_is_object(root)) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "profile root must be an object");
        json_document_free(doc);
        return false;
    }

    /* Version, then migrate. Today there is only version 1. */
    const JsonValue *versionVal = json_object_get(root, "version");
    uint32_t version = 1u;
    if (versionVal != NULL && json_is_number(versionVal)) {
        const double v = json_as_number(versionVal);
        if (v >= 1.0 && v <= 100.0) version = (uint32_t)v;
    }
    profile->version = PLAYER_PROFILE_VERSION;

    copy_field(profile->lastCarId, sizeof(profile->lastCarId),
               json_object_get(root, "lastCarId"), "lastCarId", error, errorCap);
    copy_field(profile->lastTrackId, sizeof(profile->lastTrackId),
               json_object_get(root, "lastTrackId"), "lastTrackId", error, errorCap);

    const JsonValue *num = NULL;
    num = json_object_get(root, "masterVolume");
    if (num != NULL && json_is_number(num)) profile->masterVolume = (float)json_as_number(num);
    num = json_object_get(root, "sfxVolume");
    if (num != NULL && json_is_number(num)) profile->sfxVolume = (float)json_as_number(num);
    num = json_object_get(root, "musicVolume");
    if (num != NULL && json_is_number(num)) profile->musicVolume = (float)json_as_number(num);
    num = json_object_get(root, "uiScale");
    if (num != NULL && json_is_number(num)) profile->uiScale = (float)json_as_number(num);

    const JsonValue *b = json_object_get(root, "vsyncEnabled");
    if (b != NULL && json_is_bool(b)) profile->vsyncEnabled = json_as_bool(b);
    b = json_object_get(root, "reducedCameraShake");
    if (b != NULL && json_is_bool(b)) profile->reducedCameraShake = json_as_bool(b);
    b = json_object_get(root, "reducedFlashes");
    if (b != NULL && json_is_bool(b)) profile->reducedFlashes = json_as_bool(b);
    b = json_object_get(root, "audioCountdownCue");
    if (b != NULL && json_is_bool(b)) profile->audioCountdownCue = json_as_bool(b);

    /* Bindings. */
    const JsonValue *bindings = json_object_get(root, "bindings");
    if (bindings != NULL && json_is_array(bindings)) {
        const int count = json_array_count(bindings);
        if (count > PLAYER_PROFILE_MAX_BINDINGS) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "profile has too many bindings (%d)", count);
            json_document_free(doc);
            return false;
        }
        for (int i = 0; i < count; i++) {
            const JsonValue *entry = json_array_at(bindings, i);
            if (entry == NULL || !json_is_object(entry)) continue;
            if (!copy_field(profile->bindings[i].action, sizeof(profile->bindings[i].action),
                            json_object_get(entry, "action"), "bindings[].action", error,
                            errorCap) ||
                !copy_field(profile->bindings[i].key, sizeof(profile->bindings[i].key),
                            json_object_get(entry, "key"), "bindings[].key", error, errorCap)) {
                json_document_free(doc);
                return false;
            }
            profile->bindingCount = i + 1;
        }
    }

    /* Records. */
    const JsonValue *records = json_object_get(root, "records");
    if (records != NULL && json_is_array(records)) {
        const int count = json_array_count(records);
        if (count > PLAYER_PROFILE_MAX_RECORDS) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "profile has too many records (%d)", count);
            json_document_free(doc);
            return false;
        }
        for (int i = 0; i < count; i++) {
            const JsonValue *entry = json_array_at(records, i);
            if (entry == NULL || !json_is_object(entry)) continue;
            if (!copy_field(profile->records[i].trackId, sizeof(profile->records[i].trackId),
                            json_object_get(entry, "trackId"), "records[].trackId", error,
                            errorCap) ||
                !copy_field(profile->records[i].carId, sizeof(profile->records[i].carId),
                            json_object_get(entry, "carId"), "records[].carId", error,
                            errorCap)) {
                json_document_free(doc);
                return false;
            }
            if (!copy_field(profile->records[i].compatibilityKey,
                            sizeof(profile->records[i].compatibilityKey),
                            json_object_get(entry, "compatibilityKey"),
                            "records[].compatibilityKey", error, errorCap)) {
                json_document_free(doc);
                return false;
            }
            const JsonValue *timeVal = json_object_get(entry, "bestLapTimeS");
            if (timeVal != NULL && json_is_number(timeVal))
                profile->records[i].bestLapTimeS = (float)json_as_number(timeVal);
            profile->recordCount = i + 1;
        }
    }

    json_document_free(doc);
    (void)version; /* migrations append here as new versions land */
    return true;
}

bool player_profile_save(const PlayerProfile *profile, const char *path, char *error,
                         size_t errorCap)
{
    if (profile == NULL || path == NULL) return false;
    char json[PLAYER_PROFILE_JSON_CAP];
    if (player_profile_serialize(profile, json, sizeof(json)) < 0) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "profile too large to serialize");
        return false;
    }

    char tmpPath[512];
    if (snprintf(tmpPath, sizeof(tmpPath), "%s.tmp", path) >= (int)sizeof(tmpPath)) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "profile path too long");
        return false;
    }

    FILE *f = fopen(tmpPath, "wb");
    if (f == NULL) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "cannot open %s for writing", tmpPath);
        return false;
    }
    const size_t len = strlen(json);
    const bool wrote = fwrite(json, 1, len, f) == len;
    if (fclose(f) != 0 || !wrote) {
        remove(tmpPath);
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "write to %s failed", tmpPath);
        return false;
    }
    if (rename(tmpPath, path) != 0) {
        remove(tmpPath);
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "cannot move %s into place", tmpPath);
        return false;
    }
    return true;
}

bool player_profile_load(PlayerProfile *profile, const char *path, char *error, size_t errorCap)
{
    if (profile == NULL || path == NULL) return false;
    player_profile_default(profile);

    FILE *f = fopen(path, "rb");
    if (f == NULL) return true; /* missing file: clean defaults, not an error */

    char text[PLAYER_PROFILE_JSON_CAP];
    const size_t got = fread(text, 1, sizeof(text) - 1, f);
    const bool readOk = (fclose(f) == 0);
    if (!readOk || got == 0) {
        /* Unreadable or empty: back it up and fall back to defaults. */
        if (got > 0 || !readOk) {
            char corrupt[512];
            snprintf(corrupt, sizeof(corrupt), "%s.corrupt", path);
            rename(path, corrupt);
        }
        return true;
    }
    text[got] = '\0';

    if (!player_profile_deserialize(profile, text, error, errorCap)) {
        /* Corrupt: preserve the original under `.corrupt`, return defaults. */
        char corrupt[512];
        snprintf(corrupt, sizeof(corrupt), "%s.corrupt", path);
        rename(path, corrupt);
        player_profile_default(profile);
        return true;
    }
    return true;
}

bool player_profile_rebind(PlayerProfile *profile, const char *action, const char *key)
{
    if (profile == NULL || action == NULL || key == NULL || action[0] == '\0' || key[0] == '\0')
        return false;

    /* The same key bound to a different action is a conflict; binding it to the SAME action
     * again is a no-op success. */
    for (int i = 0; i < profile->bindingCount; i++) {
        if (strcmp(profile->bindings[i].key, key) == 0) {
            return strcmp(profile->bindings[i].action, action) == 0;
        }
    }
    /* Update an existing action's key. */
    for (int i = 0; i < profile->bindingCount; i++) {
        if (strcmp(profile->bindings[i].action, action) == 0) {
            snprintf(profile->bindings[i].key, sizeof(profile->bindings[i].key), "%s", key);
            return true;
        }
    }
    if (profile->bindingCount >= PLAYER_PROFILE_MAX_BINDINGS) return false;
    snprintf(profile->bindings[profile->bindingCount].action,
             sizeof(profile->bindings[profile->bindingCount].action), "%s", action);
    snprintf(profile->bindings[profile->bindingCount].key,
             sizeof(profile->bindings[profile->bindingCount].key), "%s", key);
    profile->bindingCount++;
    return true;
}

const char *player_profile_bound_key(const PlayerProfile *profile, const char *action)
{
    if (profile == NULL || action == NULL) return NULL;
    for (int i = 0; i < profile->bindingCount; i++) {
        if (strcmp(profile->bindings[i].action, action) == 0) return profile->bindings[i].key;
    }
    return NULL;
}

bool player_profile_record_lap(PlayerProfile *profile, const char *trackId, const char *carId,
                               const char *compatibilityKey, float bestLapTimeS)
{
    if (profile == NULL || trackId == NULL || carId == NULL || compatibilityKey == NULL ||
        !(bestLapTimeS > 0.0f))
        return false;
    for (int i = 0; i < profile->recordCount; i++) {
        if (strcmp(profile->records[i].trackId, trackId) == 0 &&
            strcmp(profile->records[i].carId, carId) == 0 &&
            strcmp(profile->records[i].compatibilityKey, compatibilityKey) == 0) {
            if (bestLapTimeS < profile->records[i].bestLapTimeS) {
                profile->records[i].bestLapTimeS = bestLapTimeS;
                return true;
            }
            return false; /* existing record stands */
        }
    }
    if (profile->recordCount >= PLAYER_PROFILE_MAX_RECORDS) return false;
    snprintf(profile->records[profile->recordCount].trackId,
             sizeof(profile->records[profile->recordCount].trackId), "%s", trackId);
    snprintf(profile->records[profile->recordCount].carId,
             sizeof(profile->records[profile->recordCount].carId), "%s", carId);
    snprintf(profile->records[profile->recordCount].compatibilityKey,
             sizeof(profile->records[profile->recordCount].compatibilityKey), "%s",
             compatibilityKey);
    profile->records[profile->recordCount].bestLapTimeS = bestLapTimeS;
    profile->recordCount++;
    return true;
}

float player_profile_best_lap(const PlayerProfile *profile, const char *trackId,
                              const char *carId, const char *compatibilityKey)
{
    if (profile == NULL || trackId == NULL || carId == NULL || compatibilityKey == NULL)
        return 0.0f;
    for (int i = 0; i < profile->recordCount; i++) {
        if (strcmp(profile->records[i].trackId, trackId) == 0 &&
            strcmp(profile->records[i].carId, carId) == 0 &&
            strcmp(profile->records[i].compatibilityKey, compatibilityKey) == 0)
            return profile->records[i].bestLapTimeS;
    }
    return 0.0f;
}

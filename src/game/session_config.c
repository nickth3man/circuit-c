/*
 * session_config.c — issue #48 implementation.
 */
#include "game/session_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "content/track_manifest.h" /* track_manifest_id_is_valid */
#include "content/vehicle_manifest.h"
#include "core/math_utils.h" /* isfinite */
#include "game/car_roster.h"

/* ---- validation helpers ---- */

static bool track_known(const char *id)
{
    if (id == NULL || id[0] == '\0' || !track_manifest_id_is_valid(id)) return false;
    /* Format-valid is not enough: the id must resolve to a shipped track. */
    TrackCatalog catalog;
    memset(&catalog, 0, sizeof(catalog));
    char error[128] = "";
    if (!track_catalog_load(NULL, &catalog, error, sizeof(error))) return false;
    const bool found = (track_catalog_find(&catalog, id) >= 0);
    track_catalog_free(&catalog);
    return found;
}

static bool car_selectable(const char *id)
{
    if (id == NULL || id[0] == '\0') return false;
    return car_roster_find(id) >= 0;
}

static void set_reason(char *reason, size_t cap, const char *msg)
{
    if (reason != NULL && cap > 0) {
        (void)snprintf(reason, cap, "%s", msg);
    }
}

void session_config_set_default(SessionConfig *cfg)
{
    if (cfg == NULL) return;
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->trackId, sizeof(cfg->trackId), "%s", "chicane");
    snprintf(cfg->carId, sizeof(cfg->carId), "%s", "rwd_grip");
    cfg->aiCount = 0;
    cfg->aiDifficulty = SESSION_DIFFICULTY_MEDIUM;
    cfg->mode = RACE_MODE_TIME_TRIAL;
    cfg->targetLaps = 3;
    cfg->countdownS = 0.0f;
    cfg->damageMode = DAMAGE_OFF;
    cfg->stuckRecoveryEnabled = false;
    cfg->trackLimitsEnabled = false;
    cfg->absLevel = 2;
    cfg->tcsLevel = 1;
    race_environment_set_default(&cfg->environment);
    cfg->pitLaneEnabled = false;
    cfg->pitServiceTimeS = 0.0f; /* 0 = rules default */
    cfg->seed = 0xC1C0C1C0ULL;   /* fixed default seed for reproducible sessions */
}

bool session_config_validate(const SessionConfig *cfg, char *reason, size_t reasonCap)
{
    if (cfg == NULL) {
        set_reason(reason, reasonCap, "no session config");
        return false;
    }
    if (!track_known(cfg->trackId)) {
        set_reason(reason, reasonCap, "unknown track");
        return false;
    }
    if (!car_selectable(cfg->carId)) {
        set_reason(reason, reasonCap, "unknown or unselectable car");
        return false;
    }
    if (cfg->mode < 0 || cfg->mode >= RACE_MODE_COUNT) {
        set_reason(reason, reasonCap, "unsupported race mode");
        return false;
    }
    if (cfg->aiCount < 0 || cfg->aiCount > RACE_MAX_ENTRANTS - 1) {
        set_reason(reason, reasonCap, "AI field size out of range");
        return false;
    }
    /* A time trial is solo by definition; AI opponents are a race-mode concept. */
    if (cfg->mode == RACE_MODE_TIME_TRIAL && cfg->aiCount > 0) {
        set_reason(reason, reasonCap, "time trial cannot have AI opponents");
        return false;
    }
    if (cfg->aiDifficulty < 0 || cfg->aiDifficulty >= SESSION_DIFFICULTY_COUNT) {
        set_reason(reason, reasonCap, "AI difficulty out of range");
        return false;
    }
    if (cfg->targetLaps < 1) {
        set_reason(reason, reasonCap, "target laps must be at least 1");
        return false;
    }
    if (!isfinite(cfg->countdownS) || cfg->countdownS < 0.0f) {
        set_reason(reason, reasonCap, "countdown must be non-negative");
        return false;
    }
    if (cfg->damageMode < 0 || cfg->damageMode > DAMAGE_MECHANICAL) {
        set_reason(reason, reasonCap, "damage mode out of range");
        return false;
    }
    if (cfg->absLevel < 0 || cfg->absLevel > SESSION_ASSIST_MAX || cfg->tcsLevel < 0 ||
        cfg->tcsLevel > SESSION_ASSIST_MAX) {
        set_reason(reason, reasonCap, "assist level out of range");
        return false;
    }
    /* Environment physical bounds: precipitation in [0,1], finite temps. The presentation-only
     * fields (timeOfDay, region) are not validated for gameplay. */
    const RaceEnvironment *e = &cfg->environment;
    if (!isfinite(e->precipitation) || e->precipitation < 0.0f || e->precipitation > 1.0f) {
        set_reason(reason, reasonCap, "precipitation must be in [0,1]");
        return false;
    }
    if (!isfinite(e->ambientTempC) || !isfinite(e->trackTempC)) {
        set_reason(reason, reasonCap, "environment temperature is not finite");
        return false;
    }
    /* Pit rules need a track that authors pit geometry (issue #57). */
    if (cfg->pitLaneEnabled) {
        TrackCatalog catalog;
        memset(&catalog, 0, sizeof(catalog));
        char err[128] = "";
        if (!track_catalog_load(NULL, &catalog, err, sizeof(err))) {
            set_reason(reason, reasonCap, "track catalog unavailable");
            return false;
        }
        const int idx = track_catalog_find(&catalog, cfg->trackId);
        bool hasPits = (idx >= 0 && track_pit_has_geometry(&catalog.entries[idx].definition));
        track_catalog_free(&catalog);
        if (!hasPits) {
            set_reason(reason, reasonCap, "track has no pit geometry for pit rules");
            return false;
        }
    }
    return true;
}

/* ---- serialization ---- */

bool session_config_serialize(const SessionConfig *cfg, FILE *out)
{
    if (cfg == NULL || out == NULL) return false;
    /* Fixed field order: the document is the same no matter which menu widget changed last. */
    fprintf(out,
            "{\"schema\":\"circuit/session-config\",\"version\":1,"
            "\"trackId\":\"%s\",\"carId\":\"%s\","
            "\"aiCount\":%d,\"aiDifficulty\":%d,\"mode\":%d,\"targetLaps\":%d,"
            "\"countdownS\":%.6f,\"damageMode\":%d,\"stuckRecoveryEnabled\":%s,"
            "\"trackLimitsEnabled\":%s,"
            "\"pitLaneEnabled\":%s,\"pitServiceTimeS\":%.6f,"
            "\"absLevel\":%d,\"tcsLevel\":%d,\"seed\":%llu,"
            "\"environment\":{\"precipitation\":%.6f,\"ambientTempC\":%.6f,"
            "\"trackTempC\":%.6f,\"timeOfDayHours\":%.6f,\"region\":\"%s\"}}\n",
            cfg->trackId, cfg->carId, cfg->aiCount, cfg->aiDifficulty, (int)cfg->mode,
            cfg->targetLaps, (double)cfg->countdownS, cfg->damageMode,
            cfg->stuckRecoveryEnabled ? "true" : "false",
            cfg->trackLimitsEnabled ? "true" : "false", cfg->pitLaneEnabled ? "true" : "false",
            (double)cfg->pitServiceTimeS, cfg->absLevel, cfg->tcsLevel,
            (unsigned long long)cfg->seed, (double)cfg->environment.precipitation,
            (double)cfg->environment.ambientTempC, (double)cfg->environment.trackTempC,
            (double)cfg->environment.timeOfDayHours, cfg->environment.region);
    return true;
}

/* Minimal field extractor: finds "key":value and parses int/float/bool/string into the dest.
 * Returns true when the key was found and parsed. Deliberately tiny — the schema is fixed and
 * we validate ranges after parsing, so a full JSON parser is not warranted here. */
static bool find_int(const char *text, const char *key, int *out)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(text, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    *out = (int)strtol(p, NULL, 10);
    return true;
}

static bool find_float(const char *text, const char *key, float *out)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(text, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    *out = (float)strtod(p, NULL);
    return true;
}

static bool find_bool(const char *text, const char *key, bool *out)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(text, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    if (strncmp(p, "true", 4) == 0) {
        *out = true;
        return true;
    }
    if (strncmp(p, "false", 5) == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool find_string(const char *text, const char *key, char *out, size_t cap)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *p = strstr(text, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    const char *end = strchr(p, '"');
    if (end == NULL) return false;
    size_t len = (size_t)(end - p);
    if (len >= cap) len = cap - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool find_u64(const char *text, const char *key, uint64_t *out)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(text, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    *out = strtoull(p, NULL, 10);
    return true;
}

bool session_config_deserialize(SessionConfig *cfg, const char *text)
{
    if (cfg == NULL || text == NULL) return false;
    SessionConfig parsed;
    memset(&parsed, 0, sizeof(parsed));
    race_environment_set_default(&parsed.environment);

    if (!find_string(text, "trackId", parsed.trackId, sizeof(parsed.trackId))) return false;
    if (!find_string(text, "carId", parsed.carId, sizeof(parsed.carId))) return false;
    if (!find_int(text, "aiCount", &parsed.aiCount)) return false;
    if (!find_int(text, "aiDifficulty", &parsed.aiDifficulty)) return false;
    int mode = 0;
    if (!find_int(text, "mode", &mode)) return false;
    parsed.mode = (RaceMode)mode;
    if (!find_int(text, "targetLaps", &parsed.targetLaps)) return false;
    if (!find_float(text, "countdownS", &parsed.countdownS)) return false;
    if (!find_int(text, "damageMode", &parsed.damageMode)) return false;
    if (!find_bool(text, "stuckRecoveryEnabled", &parsed.stuckRecoveryEnabled)) return false;
    if (!find_bool(text, "trackLimitsEnabled", &parsed.trackLimitsEnabled)) return false;
    if (!find_bool(text, "pitLaneEnabled", &parsed.pitLaneEnabled)) return false;
    if (!find_float(text, "pitServiceTimeS", &parsed.pitServiceTimeS)) return false;
    if (!find_int(text, "absLevel", &parsed.absLevel)) return false;
    if (!find_int(text, "tcsLevel", &parsed.tcsLevel)) return false;
    if (!find_u64(text, "seed", &parsed.seed)) return false;
    if (!find_float(text, "precipitation", &parsed.environment.precipitation)) return false;
    if (!find_float(text, "ambientTempC", &parsed.environment.ambientTempC)) return false;
    if (!find_float(text, "trackTempC", &parsed.environment.trackTempC)) return false;
    (void)find_float(text, "timeOfDayHours", &parsed.environment.timeOfDayHours);
    (void)find_string(text, "region", parsed.environment.region,
                      sizeof(parsed.environment.region));

    *cfg = parsed;
    return true;
}

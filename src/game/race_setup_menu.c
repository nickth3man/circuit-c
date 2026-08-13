/*
 * race_setup_menu.c — the headless race-setup model.
 */
#include "game/race_setup_menu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "content/track_manifest.h"
#include "content/vehicle_manifest.h"
#include "game/car_roster.h"

/* Bounds the screen enforces. The validator's own limits are wider (any lap count >= 1 is
 * legal); these are what a control can reach by being held down, which is a different
 * question. */
#define RACE_SETUP_MAX_LAPS 50
#define RACE_SETUP_MAX_COUNTDOWN_S 10.0f
#define RACE_SETUP_COUNTDOWN_STEP_S 1.0f

/* Weather presets, as precipitation. The environment's wetness evolves from this per tick
 * (race_environment_update), so these are physical inputs, not labels. */
#define RACE_SETUP_WEATHER_COUNT 3
static const float kWeatherPrecipitation[RACE_SETUP_WEATHER_COUNT] = { 0.0f, 0.35f, 0.80f };
static const char *const kWeatherName[RACE_SETUP_WEATHER_COUNT] = { "dry", "damp", "wet" };

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void copy_text(char *out, size_t cap, const char *text)
{
    if (out == NULL || cap == 0) return;
    (void)snprintf(out, cap, "%s", text);
}

/* Which preset the config's precipitation is nearest. A config written by hand or by an older
 * build need not sit exactly on a preset, so the screen names the closest one rather than
 * refusing to describe it. */
static int weather_index(const SessionConfig *cfg)
{
    int best = 0;
    float bestDelta = fabsf(cfg->environment.precipitation - kWeatherPrecipitation[0]);
    for (int i = 1; i < RACE_SETUP_WEATHER_COUNT; i++) {
        const float delta = fabsf(cfg->environment.precipitation - kWeatherPrecipitation[i]);
        if (delta < bestDelta) {
            bestDelta = delta;
            best = i;
        }
    }
    return best;
}

static int track_index(const RaceSetupMenu *menu, const char *id)
{
    for (int i = 0; i < menu->trackCount; i++) {
        if (strcmp(menu->tracks[i].id, id) == 0) return i;
    }
    return -1;
}

static bool selected_track_has_pits(const RaceSetupMenu *menu, const SessionConfig *cfg)
{
    const int idx = track_index(menu, cfg->trackId);
    return (idx >= 0) && menu->tracks[idx].hasPits;
}

void race_setup_menu_init(RaceSetupMenu *menu)
{
    if (menu == NULL) return;
    memset(menu, 0, sizeof(*menu));

    TrackCatalog catalog;
    memset(&catalog, 0, sizeof(catalog));
    char error[128] = "";
    if (track_catalog_load(NULL, &catalog, error, sizeof(error))) {
        const int count =
            (catalog.count < RACE_SETUP_MAX_TRACKS) ? catalog.count : RACE_SETUP_MAX_TRACKS;
        for (int i = 0; i < count; i++) {
            const TrackDefinition *def = &catalog.entries[i].definition;
            copy_text(menu->tracks[i].id, sizeof(menu->tracks[i].id), def->id);
            menu->tracks[i].hasPits = track_pit_has_geometry(def);
        }
        menu->trackCount = count;
        track_catalog_free(&catalog);
    }

    /* The AI field is staffed from roster cars other than the player's, so the ceiling is one
     * less than the eligible count in the worst case. Counting eligibility here rather than at
     * every keypress keeps the control cheap; race_setup_menu_init() is what picks up content
     * edited under the open menu. */
    const int rosterCount = car_roster_count();
    for (int i = 0; i < rosterCount; i++) {
        const VehicleManifest *m = car_roster_manifest(i);
        if (m != NULL && m->definition.aiEligible) menu->aiEligibleCars++;
    }
    menu->cursor = 0;
}

bool race_setup_menu_move_cursor(RaceSetupMenu *menu, int direction)
{
    if (menu == NULL || direction == 0) return false;
    int cursor = menu->cursor + ((direction > 0) ? 1 : -1);
    if (cursor < 0) cursor = RACE_SETUP_ROW_COUNT - 1;
    if (cursor >= RACE_SETUP_ROW_COUNT) cursor = 0;
    menu->cursor = cursor;
    return true;
}

/* The AI field's ceiling: whichever is smaller, the grid's spare slots or the AI-eligible cars
 * the roster can staff them with, minus the one the player may be driving. */
static int ai_count_max(const RaceSetupMenu *menu)
{
    int staffable = menu->aiEligibleCars - 1;
    if (staffable < 0) staffable = 0;
    return clamp_int(staffable, 0, RACE_MAX_ENTRANTS - 1);
}

bool race_setup_menu_row_is_active(const RaceSetupMenu *menu, const SessionConfig *cfg, int row)
{
    if (menu == NULL || cfg == NULL || row < 0 || row >= RACE_SETUP_ROW_COUNT) return false;
    switch (row) {
        case RACE_SETUP_ROW_TRACK: return menu->trackCount > 1;
        case RACE_SETUP_ROW_AI_COUNT:
            return (cfg->mode == RACE_MODE_RACE) && (ai_count_max(menu) > 0);
        case RACE_SETUP_ROW_AI_DIFFICULTY:
            return (cfg->mode == RACE_MODE_RACE) && (cfg->aiCount > 0);
        case RACE_SETUP_ROW_PIT_LANE: return selected_track_has_pits(menu, cfg);
        default: return true;
    }
}

bool race_setup_menu_adjust(const RaceSetupMenu *menu, SessionConfig *cfg, int direction)
{
    if (menu == NULL || cfg == NULL || direction == 0) return false;
    const int row = menu->cursor;
    if (!race_setup_menu_row_is_active(menu, cfg, row)) return false;
    const int step = (direction > 0) ? 1 : -1;

    switch (row) {
        case RACE_SETUP_ROW_TRACK: {
            int idx = track_index(menu, cfg->trackId);
            if (idx < 0) idx = 0; /* config names a track the catalog no longer has */
            idx += step;
            if (idx < 0) idx = menu->trackCount - 1;
            if (idx >= menu->trackCount) idx = 0;
            copy_text(cfg->trackId, sizeof(cfg->trackId), menu->tracks[idx].id);
            /* Pit rules are a property of the pair, not of the switch: a track without pit
             * geometry cannot carry them, and leaving the flag set would fail the start gate
             * from a row the player is no longer looking at. */
            if (!menu->tracks[idx].hasPits) cfg->pitLaneEnabled = false;
            return true;
        }
        case RACE_SETUP_ROW_MODE: {
            cfg->mode = (cfg->mode == RACE_MODE_RACE) ? RACE_MODE_TIME_TRIAL : RACE_MODE_RACE;
            /* A time trial is solo by definition — the validator says so, so the control that
             * makes it one has to empty the field rather than leave a refusal behind. */
            if (cfg->mode == RACE_MODE_TIME_TRIAL) cfg->aiCount = 0;
            return true;
        }
        case RACE_SETUP_ROW_LAPS:
            cfg->targetLaps = clamp_int(cfg->targetLaps + step, 1, RACE_SETUP_MAX_LAPS);
            return true;
        case RACE_SETUP_ROW_AI_COUNT:
            cfg->aiCount = clamp_int(cfg->aiCount + step, 0, ai_count_max(menu));
            return true;
        case RACE_SETUP_ROW_AI_DIFFICULTY: {
            int tier = cfg->aiDifficulty + step;
            if (tier < 0) tier = SESSION_DIFFICULTY_COUNT - 1;
            if (tier >= SESSION_DIFFICULTY_COUNT) tier = 0;
            cfg->aiDifficulty = tier;
            return true;
        }
        case RACE_SETUP_ROW_COUNTDOWN: {
            float countdown = cfg->countdownS + (float)step * RACE_SETUP_COUNTDOWN_STEP_S;
            if (countdown < 0.0f) countdown = 0.0f;
            if (countdown > RACE_SETUP_MAX_COUNTDOWN_S) countdown = RACE_SETUP_MAX_COUNTDOWN_S;
            cfg->countdownS = countdown;
            return true;
        }
        case RACE_SETUP_ROW_DAMAGE: {
            int mode = cfg->damageMode + step;
            if (mode < DAMAGE_OFF) mode = DAMAGE_MECHANICAL;
            if (mode > DAMAGE_MECHANICAL) mode = DAMAGE_OFF;
            cfg->damageMode = mode;
            return true;
        }
        case RACE_SETUP_ROW_RECOVERY:
            cfg->stuckRecoveryEnabled = !cfg->stuckRecoveryEnabled;
            return true;
        case RACE_SETUP_ROW_TRACK_LIMITS:
            cfg->trackLimitsEnabled = !cfg->trackLimitsEnabled;
            return true;
        case RACE_SETUP_ROW_PIT_LANE: cfg->pitLaneEnabled = !cfg->pitLaneEnabled; return true;
        case RACE_SETUP_ROW_WEATHER: {
            int idx = weather_index(cfg) + step;
            if (idx < 0) idx = RACE_SETUP_WEATHER_COUNT - 1;
            if (idx >= RACE_SETUP_WEATHER_COUNT) idx = 0;
            cfg->environment.precipitation = kWeatherPrecipitation[idx];
            return true;
        }
        case RACE_SETUP_ROW_ABS:
            cfg->absLevel = clamp_int(cfg->absLevel + step, 0, SESSION_ASSIST_MAX);
            return true;
        case RACE_SETUP_ROW_TCS:
            cfg->tcsLevel = clamp_int(cfg->tcsLevel + step, 0, SESSION_ASSIST_MAX);
            return true;
        default: return false;
    }
}

void race_setup_menu_row_label(int row, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    static const char *const kLabels[RACE_SETUP_ROW_COUNT] = {
        "track",   "mode",   "laps",     "ai opponents", "ai difficulty",
        "start",   "damage", "recovery", "track limits", "pit lane",
        "weather", "abs",    "tcs",
    };
    if (row < 0 || row >= RACE_SETUP_ROW_COUNT) {
        out[0] = '\0';
        return;
    }
    copy_text(out, cap, kLabels[row]);
}

void race_setup_menu_row_value(const RaceSetupMenu *menu, const SessionConfig *cfg, int row,
                               char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    out[0] = '\0';
    if (menu == NULL || cfg == NULL || row < 0 || row >= RACE_SETUP_ROW_COUNT) return;

    static const char *const kDifficulty[SESSION_DIFFICULTY_COUNT] = { "easy", "medium",
                                                                       "hard" };
    static const char *const kDamage[] = { "off", "cosmetic", "mechanical" };

    switch (row) {
        case RACE_SETUP_ROW_TRACK: copy_text(out, cap, cfg->trackId); break;
        case RACE_SETUP_ROW_MODE:
            copy_text(out, cap, (cfg->mode == RACE_MODE_RACE) ? "race" : "time trial");
            break;
        case RACE_SETUP_ROW_LAPS: (void)snprintf(out, cap, "%d", cfg->targetLaps); break;
        case RACE_SETUP_ROW_AI_COUNT:
            if (cfg->mode == RACE_MODE_TIME_TRIAL) {
                copy_text(out, cap, "solo");
            } else {
                (void)snprintf(out, cap, "%d", cfg->aiCount);
            }
            break;
        case RACE_SETUP_ROW_AI_DIFFICULTY:
            if (cfg->aiDifficulty >= 0 && cfg->aiDifficulty < SESSION_DIFFICULTY_COUNT) {
                copy_text(out, cap, kDifficulty[cfg->aiDifficulty]);
            }
            break;
        case RACE_SETUP_ROW_COUNTDOWN:
            if (cfg->countdownS <= 0.0f) {
                copy_text(out, cap, "immediate");
            } else {
                (void)snprintf(out, cap, "%.0f s countdown", (double)cfg->countdownS);
            }
            break;
        case RACE_SETUP_ROW_DAMAGE:
            if (cfg->damageMode >= DAMAGE_OFF && cfg->damageMode <= DAMAGE_MECHANICAL) {
                copy_text(out, cap, kDamage[cfg->damageMode]);
            }
            break;
        case RACE_SETUP_ROW_RECOVERY:
            copy_text(out, cap, cfg->stuckRecoveryEnabled ? "on" : "off");
            break;
        case RACE_SETUP_ROW_TRACK_LIMITS:
            copy_text(out, cap, cfg->trackLimitsEnabled ? "enforced" : "off");
            break;
        case RACE_SETUP_ROW_PIT_LANE:
            if (!selected_track_has_pits(menu, cfg)) {
                copy_text(out, cap, "track has no pits");
            } else {
                copy_text(out, cap, cfg->pitLaneEnabled ? "on" : "off");
            }
            break;
        case RACE_SETUP_ROW_WEATHER:
            copy_text(out, cap, kWeatherName[weather_index(cfg)]);
            break;
        case RACE_SETUP_ROW_ABS: (void)snprintf(out, cap, "%d", cfg->absLevel); break;
        case RACE_SETUP_ROW_TCS: (void)snprintf(out, cap, "%d", cfg->tcsLevel); break;
        default: break;
    }
}

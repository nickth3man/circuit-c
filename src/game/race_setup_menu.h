/*
 * race_setup_menu.h — the headless race-setup model: what the pre-race screen can offer and
 * what moving one of its controls does to a SessionConfig. Raylib-free, like setup_editor.h,
 * so the screen's behaviour is testable without a window.
 *
 * WHY A MODEL RATHER THAN A DRAW FUNCTION. `session_config_validate()` is the gate a start goes
 * through, and until now the only callers that could satisfy it were tests and command-line
 * tooling — the menu restarted the default session instead. A screen that edits the config
 * directly would be free to build combinations the gate then refuses (AI opponents in a time
 * trial, pit rules on a track with no pit geometry), which turns a menu into a guessing game.
 * So the rules live here, in adjust(): every reachable edit lands on a config that still
 * validates, and the refusal path stays for content that changes under the open menu.
 *
 * THE CAR IS NOT A ROW HERE. Car choice already has a screen — the roster carousel on the main
 * menu, with its own recall file and setup editor. This model owns the rest of the bundle, and
 * the start path copies the carousel's selected id into the config. Two controls for one field
 * would be two places for it to disagree.
 *
 * TRACKS ARE SNAPSHOTTED AT INIT. The catalog is read once into fixed arrays: no pointers into
 * the catalog survive the call, so the model is plain value data and safe to keep in the
 * persistent Game block across a hot reload.
 */
#ifndef CIRCUIT_RACE_SETUP_MENU_H
#define CIRCUIT_RACE_SETUP_MENU_H

#include <stdbool.h>
#include <stddef.h>

#include "game/session_config.h"

#define RACE_SETUP_MAX_TRACKS 32
#define RACE_SETUP_LABEL_CHARS 24
#define RACE_SETUP_VALUE_CHARS 48

/* Display order of the screen, top to bottom. */
typedef enum {
    RACE_SETUP_ROW_TRACK = 0,
    RACE_SETUP_ROW_MODE,
    RACE_SETUP_ROW_LAPS,
    RACE_SETUP_ROW_AI_COUNT,
    RACE_SETUP_ROW_AI_DIFFICULTY,
    RACE_SETUP_ROW_COUNTDOWN,
    RACE_SETUP_ROW_DAMAGE,
    RACE_SETUP_ROW_RECOVERY,
    RACE_SETUP_ROW_TRACK_LIMITS,
    RACE_SETUP_ROW_PIT_LANE,
    RACE_SETUP_ROW_WEATHER,
    RACE_SETUP_ROW_ABS,
    RACE_SETUP_ROW_TCS,
    RACE_SETUP_ROW_COUNT
} RaceSetupRow;

/* One shipped track, as much of it as the screen needs. `hasPits` is the authored answer to
 * whether pit rules may be switched on at all. */
typedef struct {
    char id[TRACK_ID_CHARS];
    bool hasPits;
} RaceSetupTrackEntry;

typedef struct {
    RaceSetupTrackEntry tracks[RACE_SETUP_MAX_TRACKS];
    int trackCount;
    /* AI-eligible cars in the roster. The AI field cannot be raised past what the roster can
     * actually fill, because game_configure_session() refuses a field it cannot staff and the
     * player would have no way to see why. */
    int aiEligibleCars;
    int cursor; /* RACE_SETUP_ROW_* */
} RaceSetupMenu;

/* Snapshot the track catalog and the roster's AI-eligible count, and park the cursor on the
 * first row. Safe to call again to pick up content edited while the game was running. */
void race_setup_menu_init(RaceSetupMenu *menu);

/* Move the cursor by `direction` rows with wraparound. Returns false for direction == 0. */
bool race_setup_menu_move_cursor(RaceSetupMenu *menu, int direction);

/*
 * Apply one step of the row under the cursor to `cfg`, in `direction` (-1 / +1).
 *
 * Every edit keeps the config inside session_config_validate()'s rules: switching to a time
 * trial empties the AI field, choosing a track without pit geometry switches pit rules off, and
 * the AI field clamps to the cars the roster can actually staff. Returns false when the control
 * did not move — a bad argument, or a row that has nothing to offer (the AI field in a time
 * trial, pit rules on a track with no pits).
 */
bool race_setup_menu_adjust(const RaceSetupMenu *menu, SessionConfig *cfg, int direction);

/* Row label, e.g. "track". Writes an empty string for an out-of-range row. */
void race_setup_menu_row_label(int row, char *out, size_t cap);

/* The row's current value as the screen shows it, e.g. "race", "8", "mechanical". Writes an
 * empty string for an out-of-range row or a NULL config. */
void race_setup_menu_row_value(const RaceSetupMenu *menu, const SessionConfig *cfg, int row,
                               char *out, size_t cap);

/* False when the row is present but cannot be moved in the current config — the AI rows in a
 * time trial, pit rules on a track without pit geometry. The screen greys these out rather
 * than letting a keypress do nothing without saying why. */
bool race_setup_menu_row_is_active(const RaceSetupMenu *menu, const SessionConfig *cfg,
                                   int row);

#endif /* CIRCUIT_RACE_SETUP_MENU_H */

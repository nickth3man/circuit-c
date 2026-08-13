/*
 * settings_menu.h — the headless model behind the settings, controls and accessibility screen
 * (MAP.md priority 4).
 *
 * Same shape as race_setup_menu.h and for the same reason: the rules about what a control can
 * do live here, raylib-free, so the screen's behaviour is testable without a window and the
 * renderer only draws what this says.
 *
 * THREE TABS, ONE SCREEN. Settings, controls and accessibility are three groups of rows rather
 * than three screens, because a player looking for "the thing that turns the flashing off" does
 * not know which of the three we filed it under. The tab is part of the model so the screen
 * cannot invent a fourth.
 *
 * REBINDING IS A CAPTURE, NOT A CYCLE. On the controls tab, confirming a row arms capture and
 * the next key the player presses becomes the binding. The key code is passed in rather than
 * polled, so a test can drive the whole flow — arm, press, conflict, cancel — with no keyboard.
 */
#ifndef CIRCUIT_SETTINGS_MENU_H
#define CIRCUIT_SETTINGS_MENU_H

#include <stdbool.h>
#include <stddef.h>

#include "game/input_bindings.h"
#include "game/player_profile.h"

typedef enum {
    SETTINGS_TAB_GENERAL = 0,
    SETTINGS_TAB_CONTROLS,
    SETTINGS_TAB_ACCESSIBILITY,
    SETTINGS_TAB_COUNT
} SettingsTab;

/* Rows of SETTINGS_TAB_GENERAL. */
typedef enum {
    SETTINGS_ROW_MASTER_VOLUME = 0,
    SETTINGS_ROW_SFX_VOLUME,
    SETTINGS_ROW_MUSIC_VOLUME,
    SETTINGS_ROW_VSYNC,
    SETTINGS_ROW_UI_SCALE,
    SETTINGS_ROW_GENERAL_COUNT
} SettingsGeneralRow;

/* Rows of SETTINGS_TAB_ACCESSIBILITY. */
typedef enum {
    SETTINGS_ROW_REDUCED_SHAKE = 0,
    SETTINGS_ROW_REDUCED_FLASHES,
    SETTINGS_ROW_AUDIO_COUNTDOWN,
    SETTINGS_ROW_ACCESSIBILITY_COUNT
} SettingsAccessibilityRow;

#define SETTINGS_LABEL_CHARS 32
#define SETTINGS_VALUE_CHARS 32

typedef struct {
    int tab;    /* SETTINGS_TAB_* */
    int cursor; /* row within the current tab */
    /* Controls tab only: the action whose next keypress will be captured, or -1 when nothing is
     * armed. */
    int capturingAction;
    /* The action a refused rebind collided with, or -1. Cleared by any further navigation, so
     * the message describes the attempt the player just made and not an older one. */
    int conflictAction;
} SettingsMenu;

/* Open on the first tab, first row, nothing armed. */
void settings_menu_init(SettingsMenu *menu);

/* How many rows the current tab has. */
int settings_menu_row_count(const SettingsMenu *menu);

/* Move the cursor by one row with wraparound. Returns false for direction == 0 or while a
 * capture is armed — a player choosing a key must not be able to walk off the row. */
bool settings_menu_move_cursor(SettingsMenu *menu, int direction);

/* Move to the next/previous tab with wraparound, parking the cursor on its first row. Refused
 * while a capture is armed, for the same reason. */
bool settings_menu_move_tab(SettingsMenu *menu, int direction);

/*
 * Adjust the row under the cursor by one step. Volumes and the UI scale move by a fixed
 * increment and clamp; toggles flip. On the controls tab this cycles the binding through the
 * bindable keys, skipping any that would conflict — the keyboard-free way to rebind, kept
 * alongside capture so a player without a working confirm key is not locked out.
 *
 * Returns false when nothing moved.
 */
bool settings_menu_adjust(SettingsMenu *menu, PlayerProfile *profile, InputBindings *bindings,
                          int direction);

/* Controls tab: arm capture on the row under the cursor. Returns false on any other tab. */
bool settings_menu_begin_capture(SettingsMenu *menu);

/* Cancel an armed capture. Returns false when nothing was armed. */
bool settings_menu_cancel_capture(SettingsMenu *menu);

/*
 * Feed a pressed key to an armed capture.
 *
 * On success the binding moves and the capture disarms. On a conflict nothing changes, the
 * capture disarms, and `menu->conflictAction` names the action that already owns the key so the
 * screen can say which one. A key this build cannot bind is refused the same way, with
 * `conflictAction` left at -1. Returns false when no capture was armed or the rebind was
 * refused.
 */
bool settings_menu_apply_capture(SettingsMenu *menu, InputBindings *bindings, int key);

/* Row label for the current tab, e.g. "master volume" or "Throttle". */
void settings_menu_row_label(const SettingsMenu *menu, int row, char *out, size_t cap);

/* Row value as the screen shows it, e.g. "80%", "on", "W", or "press a key" while armed. */
void settings_menu_row_value(const SettingsMenu *menu, const PlayerProfile *profile,
                             const InputBindings *bindings, int row, char *out, size_t cap);

/* The tab's name, for the header. */
const char *settings_menu_tab_name(int tab);

#endif /* CIRCUIT_SETTINGS_MENU_H */

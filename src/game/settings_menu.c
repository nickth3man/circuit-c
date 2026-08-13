/*
 * settings_menu.c — the headless settings/controls/accessibility model.
 */
#include "game/settings_menu.h"

#include <stdio.h>
#include <string.h>

/* Steps a control moves by. Volumes in twentieths so the whole range is reachable in a
 * reasonable number of presses; the UI scale in tenths across the range the profile allows. */
#define SETTINGS_VOLUME_STEP 0.05f
#define SETTINGS_UI_SCALE_STEP 0.1f
#define SETTINGS_UI_SCALE_MIN 0.5f
#define SETTINGS_UI_SCALE_MAX 2.0f

static float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static float clamp_range(float v, float lo, float hi)
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

void settings_menu_init(SettingsMenu *menu)
{
    if (menu == NULL) return;
    menu->tab = SETTINGS_TAB_GENERAL;
    menu->cursor = 0;
    menu->capturingAction = -1;
    menu->conflictAction = -1;
}

int settings_menu_row_count(const SettingsMenu *menu)
{
    if (menu == NULL) return 0;
    switch (menu->tab) {
        case SETTINGS_TAB_GENERAL: return SETTINGS_ROW_GENERAL_COUNT;
        case SETTINGS_TAB_CONTROLS: return INPUT_ACTION_COUNT;
        case SETTINGS_TAB_ACCESSIBILITY: return SETTINGS_ROW_ACCESSIBILITY_COUNT;
        default: return 0;
    }
}

bool settings_menu_move_cursor(SettingsMenu *menu, int direction)
{
    if (menu == NULL || direction == 0) return false;
    if (menu->capturingAction >= 0) return false;
    const int count = settings_menu_row_count(menu);
    if (count <= 0) return false;
    int cursor = menu->cursor + ((direction > 0) ? 1 : -1);
    if (cursor < 0) cursor = count - 1;
    if (cursor >= count) cursor = 0;
    menu->cursor = cursor;
    menu->conflictAction = -1;
    return true;
}

bool settings_menu_move_tab(SettingsMenu *menu, int direction)
{
    if (menu == NULL || direction == 0) return false;
    if (menu->capturingAction >= 0) return false;
    int tab = menu->tab + ((direction > 0) ? 1 : -1);
    if (tab < 0) tab = SETTINGS_TAB_COUNT - 1;
    if (tab >= SETTINGS_TAB_COUNT) tab = 0;
    menu->tab = tab;
    menu->cursor = 0;
    menu->conflictAction = -1;
    return true;
}

/* Cycle `action`'s binding to the next bindable key that does not conflict. Returns false when
 * the whole table was walked without finding one, which cannot happen with today's table but is
 * the honest answer if the action set ever outgrows the key set. */
static bool cycle_binding(InputBindings *bindings, int action, int step)
{
    const int count = input_bindable_key_count();
    if (count <= 0) return false;
    int start = 0;
    for (int i = 0; i < count; i++) {
        if (input_bindable_key_at(i) == bindings->key[action]) {
            start = i;
            break;
        }
    }
    for (int n = 1; n <= count; n++) {
        int idx = start + step * n;
        while (idx < 0) idx += count;
        idx %= count;
        if (input_bindings_rebind(bindings, action, input_bindable_key_at(idx), NULL))
            return true;
    }
    return false;
}

bool settings_menu_adjust(SettingsMenu *menu, PlayerProfile *profile, InputBindings *bindings,
                          int direction)
{
    if (menu == NULL || profile == NULL || bindings == NULL || direction == 0) return false;
    if (menu->capturingAction >= 0) return false;
    const int step = (direction > 0) ? 1 : -1;
    menu->conflictAction = -1;

    if (menu->tab == SETTINGS_TAB_CONTROLS) {
        if (menu->cursor < 0 || menu->cursor >= INPUT_ACTION_COUNT) return false;
        return cycle_binding(bindings, menu->cursor, step);
    }

    if (menu->tab == SETTINGS_TAB_ACCESSIBILITY) {
        switch (menu->cursor) {
            case SETTINGS_ROW_REDUCED_SHAKE:
                profile->reducedCameraShake = !profile->reducedCameraShake;
                return true;
            case SETTINGS_ROW_REDUCED_FLASHES:
                profile->reducedFlashes = !profile->reducedFlashes;
                return true;
            case SETTINGS_ROW_AUDIO_COUNTDOWN:
                profile->audioCountdownCue = !profile->audioCountdownCue;
                return true;
            default: return false;
        }
    }

    switch (menu->cursor) {
        case SETTINGS_ROW_MASTER_VOLUME:
            profile->masterVolume =
                clamp01(profile->masterVolume + (float)step * SETTINGS_VOLUME_STEP);
            return true;
        case SETTINGS_ROW_SFX_VOLUME:
            profile->sfxVolume =
                clamp01(profile->sfxVolume + (float)step * SETTINGS_VOLUME_STEP);
            return true;
        case SETTINGS_ROW_MUSIC_VOLUME:
            profile->musicVolume =
                clamp01(profile->musicVolume + (float)step * SETTINGS_VOLUME_STEP);
            return true;
        case SETTINGS_ROW_VSYNC: profile->vsyncEnabled = !profile->vsyncEnabled; return true;
        case SETTINGS_ROW_UI_SCALE:
            profile->uiScale =
                clamp_range(profile->uiScale + (float)step * SETTINGS_UI_SCALE_STEP,
                            SETTINGS_UI_SCALE_MIN, SETTINGS_UI_SCALE_MAX);
            return true;
        default: return false;
    }
}

bool settings_menu_begin_capture(SettingsMenu *menu)
{
    if (menu == NULL || menu->tab != SETTINGS_TAB_CONTROLS) return false;
    if (menu->cursor < 0 || menu->cursor >= INPUT_ACTION_COUNT) return false;
    menu->capturingAction = menu->cursor;
    menu->conflictAction = -1;
    return true;
}

bool settings_menu_cancel_capture(SettingsMenu *menu)
{
    if (menu == NULL || menu->capturingAction < 0) return false;
    menu->capturingAction = -1;
    return true;
}

bool settings_menu_apply_capture(SettingsMenu *menu, InputBindings *bindings, int key)
{
    if (menu == NULL || bindings == NULL) return false;
    const int action = menu->capturingAction;
    if (action < 0) return false;
    menu->capturingAction = -1;
    int conflict = -1;
    if (input_bindings_rebind(bindings, action, key, &conflict)) {
        menu->conflictAction = -1;
        return true;
    }
    /* A conflict names the other action; an unbindable key leaves it at -1 and the screen says
     * so instead of blaming a control that did nothing wrong. */
    menu->conflictAction = conflict;
    return false;
}

const char *settings_menu_tab_name(int tab)
{
    switch (tab) {
        case SETTINGS_TAB_GENERAL: return "general";
        case SETTINGS_TAB_CONTROLS: return "controls";
        case SETTINGS_TAB_ACCESSIBILITY: return "accessibility";
        default: return "";
    }
}

void settings_menu_row_label(const SettingsMenu *menu, int row, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    out[0] = '\0';
    if (menu == NULL || row < 0 || row >= settings_menu_row_count(menu)) return;

    static const char *const kGeneral[SETTINGS_ROW_GENERAL_COUNT] = {
        "master volume", "effects volume", "music volume", "vsync", "ui scale",
    };
    static const char *const kAccess[SETTINGS_ROW_ACCESSIBILITY_COUNT] = {
        "reduced camera shake",
        "reduced flashing",
        "audio countdown cue",
    };

    switch (menu->tab) {
        case SETTINGS_TAB_GENERAL: copy_text(out, cap, kGeneral[row]); break;
        case SETTINGS_TAB_CONTROLS: copy_text(out, cap, input_action_label(row)); break;
        case SETTINGS_TAB_ACCESSIBILITY: copy_text(out, cap, kAccess[row]); break;
        default: break;
    }
}

void settings_menu_row_value(const SettingsMenu *menu, const PlayerProfile *profile,
                             const InputBindings *bindings, int row, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    out[0] = '\0';
    if (menu == NULL || profile == NULL || row < 0 || row >= settings_menu_row_count(menu))
        return;

    if (menu->tab == SETTINGS_TAB_CONTROLS) {
        if (menu->capturingAction == row) {
            copy_text(out, cap, "press a key");
            return;
        }
        copy_text(out, cap, (bindings != NULL) ? input_bindings_label(bindings, row) : "?");
        return;
    }

    if (menu->tab == SETTINGS_TAB_ACCESSIBILITY) {
        bool on = false;
        switch (row) {
            case SETTINGS_ROW_REDUCED_SHAKE: on = profile->reducedCameraShake; break;
            case SETTINGS_ROW_REDUCED_FLASHES: on = profile->reducedFlashes; break;
            case SETTINGS_ROW_AUDIO_COUNTDOWN: on = profile->audioCountdownCue; break;
            default: return;
        }
        copy_text(out, cap, on ? "on" : "off");
        return;
    }

    switch (row) {
        case SETTINGS_ROW_MASTER_VOLUME:
            (void)snprintf(out, cap, "%.0f%%", (double)(profile->masterVolume * 100.0f));
            break;
        case SETTINGS_ROW_SFX_VOLUME:
            (void)snprintf(out, cap, "%.0f%%", (double)(profile->sfxVolume * 100.0f));
            break;
        case SETTINGS_ROW_MUSIC_VOLUME:
            (void)snprintf(out, cap, "%.0f%%", (double)(profile->musicVolume * 100.0f));
            break;
        case SETTINGS_ROW_VSYNC:
            copy_text(out, cap, profile->vsyncEnabled ? "on" : "off");
            break;
        case SETTINGS_ROW_UI_SCALE:
            (void)snprintf(out, cap, "%.1fx", (double)profile->uiScale);
            break;
        default: break;
    }
}

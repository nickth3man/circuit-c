/*
 * input_bindings.c — the action/key table, raylib-free.
 */
#include "game/input_bindings.h"

#include <stdio.h>
#include <string.h>

/*
 * raylib's key codes, written out.
 *
 * These are GLFW's values and raylib exposes them unchanged; input.c asserts each one against
 * the real constant at compile time, so this table cannot drift without breaking the build.
 * Only the keys a player may reasonably bind are listed — the set a controls screen offers is
 * exactly this table, so an unbindable key cannot be reached by cycling.
 */
typedef struct {
    int key;
    const char *id;    /* persisted form, e.g. "KEY_W" */
    const char *label; /* prompt form, e.g. "W" */
} BindableKey;

static const BindableKey kKeys[] = {
    { 65, "KEY_A", "A" },
    { 66, "KEY_B", "B" },
    { 67, "KEY_C", "C" },
    { 68, "KEY_D", "D" },
    { 69, "KEY_E", "E" },
    { 70, "KEY_F", "F" },
    { 71, "KEY_G", "G" },
    { 72, "KEY_H", "H" },
    { 73, "KEY_I", "I" },
    { 74, "KEY_J", "J" },
    { 75, "KEY_K", "K" },
    { 76, "KEY_L", "L" },
    { 77, "KEY_M", "M" },
    { 78, "KEY_N", "N" },
    { 79, "KEY_O", "O" },
    { 80, "KEY_P", "P" },
    { 81, "KEY_Q", "Q" },
    { 82, "KEY_R", "R" },
    { 83, "KEY_S", "S" },
    { 84, "KEY_T", "T" },
    { 85, "KEY_U", "U" },
    { 86, "KEY_V", "V" },
    { 87, "KEY_W", "W" },
    { 88, "KEY_X", "X" },
    { 89, "KEY_Y", "Y" },
    { 90, "KEY_Z", "Z" },
    { 48, "KEY_ZERO", "0" },
    { 49, "KEY_ONE", "1" },
    { 50, "KEY_TWO", "2" },
    { 51, "KEY_THREE", "3" },
    { 52, "KEY_FOUR", "4" },
    { 53, "KEY_FIVE", "5" },
    { 54, "KEY_SIX", "6" },
    { 55, "KEY_SEVEN", "7" },
    { 56, "KEY_EIGHT", "8" },
    { 57, "KEY_NINE", "9" },
    { 32, "KEY_SPACE", "SPACE" },
    { 257, "KEY_ENTER", "ENTER" },
    { 258, "KEY_TAB", "TAB" },
    { 259, "KEY_BACKSPACE", "BKSP" },
    { 262, "KEY_RIGHT", "RIGHT" },
    { 263, "KEY_LEFT", "LEFT" },
    { 264, "KEY_DOWN", "DOWN" },
    { 265, "KEY_UP", "UP" },
    { 266, "KEY_PAGE_UP", "PGUP" },
    { 267, "KEY_PAGE_DOWN", "PGDN" },
    { 268, "KEY_HOME", "HOME" },
    { 269, "KEY_END", "END" },
    { 290, "KEY_F1", "F1" },
    { 291, "KEY_F2", "F2" },
    { 292, "KEY_F3", "F3" },
    { 293, "KEY_F4", "F4" },
    { 294, "KEY_F5", "F5" },
    { 295, "KEY_F6", "F6" },
    { 296, "KEY_F7", "F7" },
    { 297, "KEY_F8", "F8" },
    { 340, "KEY_LEFT_SHIFT", "LSHIFT" },
    { 341, "KEY_LEFT_CONTROL", "LCTRL" },
    { 342, "KEY_LEFT_ALT", "LALT" },
    { 344, "KEY_RIGHT_SHIFT", "RSHIFT" },
    { 345, "KEY_RIGHT_CONTROL", "RCTRL" },
    { 346, "KEY_RIGHT_ALT", "RALT" },
    { 44, "KEY_COMMA", "," },
    { 46, "KEY_PERIOD", "." },
    { 47, "KEY_SLASH", "/" },
    { 59, "KEY_SEMICOLON", ";" },
    { 45, "KEY_MINUS", "-" },
    { 61, "KEY_EQUAL", "=" },
    { 91, "KEY_LEFT_BRACKET", "[" },
    { 93, "KEY_RIGHT_BRACKET", "]" },
};

#define BINDABLE_KEY_COUNT ((int)(sizeof(kKeys) / sizeof(kKeys[0])))

/* Ids and labels, in InputAction order. The ids are what a saved profile contains, so they are
 * part of the file format and must not be renamed. */
static const char *const kActionId[INPUT_ACTION_COUNT] = {
    "steer_left", "steer_right", "throttle",    "brake",    "handbrake",
    "shift_up",   "shift_down",  "toggle_auto", "pause",    "reset",
    "debug",      "menu_left",   "menu_right",  "menu_up",  "menu_down",
    "setup",      "setup_reset", "race_setup",  "settings",
};

static const char *const kActionLabel[INPUT_ACTION_COUNT] = {
    "Steer left",   "Steer right", "Throttle",     "Brake",         "Handbrake",
    "Shift up",     "Shift down",  "Auto gearbox", "Pause / start", "Reset / back",
    "Diagnostics",  "Menu left",   "Menu right",   "Menu up",       "Menu down",
    "Setup screen", "Reset setup", "Race setup",   "Settings",
};

/* The shipped layout, in InputAction order. This is the keyboard the game has always had. */
static const int kDefaultKey[INPUT_ACTION_COUNT] = {
    65,  /* steer left      A */
    68,  /* steer right     D */
    87,  /* throttle        W */
    83,  /* brake           S */
    32,  /* handbrake       SPACE */
    69,  /* shift up        E */
    81,  /* shift down      Q */
    84,  /* toggle auto     T */
    80,  /* pause / start   P */
    82,  /* reset / back    R */
    290, /* diagnostics     F1 */
    263, /* menu left       LEFT */
    262, /* menu right      RIGHT */
    265, /* menu up         UP */
    264, /* menu down       DOWN */
    83,  /* setup screen    S — shares the brake key; menus and driving never read together */
    68,  /* reset setup     D — likewise shares steer right */
    258, /* race setup      TAB */
    79,  /* settings        O */
};

void input_bindings_set_default(InputBindings *bindings)
{
    if (bindings == NULL) return;
    for (int i = 0; i < INPUT_ACTION_COUNT; i++) bindings->key[i] = kDefaultKey[i];
}

const char *input_action_id(int action)
{
    if (action < 0 || action >= INPUT_ACTION_COUNT) return "";
    return kActionId[action];
}

const char *input_action_label(int action)
{
    if (action < 0 || action >= INPUT_ACTION_COUNT) return "";
    return kActionLabel[action];
}

static const BindableKey *find_key(int key)
{
    for (int i = 0; i < BINDABLE_KEY_COUNT; i++) {
        if (kKeys[i].key == key) return &kKeys[i];
    }
    return NULL;
}

const char *input_key_id(int key)
{
    const BindableKey *k = find_key(key);
    return (k != NULL) ? k->id : "";
}

const char *input_key_label(int key)
{
    const BindableKey *k = find_key(key);
    return (k != NULL) ? k->label : "?";
}

int input_key_from_id(const char *name)
{
    if (name == NULL) return -1;
    for (int i = 0; i < BINDABLE_KEY_COUNT; i++) {
        if (strcmp(kKeys[i].id, name) == 0) return kKeys[i].key;
    }
    return -1;
}

int input_bindable_key_count(void)
{
    return BINDABLE_KEY_COUNT;
}

int input_bindable_key_at(int index)
{
    if (index < 0 || index >= BINDABLE_KEY_COUNT) return -1;
    return kKeys[index].key;
}

/*
 * Which other actions may legitimately share a key.
 *
 * Two actions collide only if they can be read on the same screen. Driving and the menus never
 * are: the setup screen's keys are sampled while the car is parked in the menu, and the driving
 * keys are sampled while there is no menu. Enforcing a single global uniqueness rule would make
 * the shipped default layout — which has always had S mean both "brake" and "setup screen" —
 * illegal, and would force the player to find keys for controls they never use together.
 */
static bool actions_share_a_screen(int a, int b)
{
    /* Group 0: read while driving. Group 1: read on a menu. An action in both groups (pause and
     * reset are, deliberately) collides with everything. */
    const bool aDriving = (a <= INPUT_ACTION_DEBUG);
    const bool bDriving = (b <= INPUT_ACTION_DEBUG);
    const bool aBoth = (a == INPUT_ACTION_PAUSE || a == INPUT_ACTION_RESET);
    const bool bBoth = (b == INPUT_ACTION_PAUSE || b == INPUT_ACTION_RESET);
    if (aBoth || bBoth) return true;
    return aDriving == bDriving;
}

bool input_bindings_rebind(InputBindings *bindings, int action, int key, int *conflict)
{
    if (conflict != NULL) *conflict = -1;
    if (bindings == NULL || action < 0 || action >= INPUT_ACTION_COUNT) return false;
    if (find_key(key) == NULL) return false;
    if (bindings->key[action] == key) return true; /* already there: not a conflict */

    for (int i = 0; i < INPUT_ACTION_COUNT; i++) {
        if (i == action || bindings->key[i] != key) continue;
        if (!actions_share_a_screen(i, action)) continue;
        if (conflict != NULL) *conflict = i;
        return false;
    }
    bindings->key[action] = key;
    return true;
}

const char *input_bindings_label(const InputBindings *bindings, int action)
{
    if (bindings == NULL || action < 0 || action >= INPUT_ACTION_COUNT) return "?";
    return input_key_label(bindings->key[action]);
}

void input_bindings_from_profile(InputBindings *bindings, const PlayerProfile *profile)
{
    if (bindings == NULL) return;
    input_bindings_set_default(bindings);
    if (profile == NULL) return;
    for (int i = 0; i < INPUT_ACTION_COUNT; i++) {
        const char *keyName = player_profile_bound_key(profile, kActionId[i]);
        if (keyName == NULL) continue;
        const int key = input_key_from_id(keyName);
        /* A key this build cannot bind leaves the default in place. Dropping the action to
         * "unbound" instead would let a profile written by a newer build disarm a control here,
         * with no way for the player to tell why the car stopped responding. */
        if (key >= 0) bindings->key[i] = key;
    }
}

bool input_bindings_to_profile(const InputBindings *bindings, PlayerProfile *profile)
{
    if (bindings == NULL || profile == NULL) return false;
    /* Rewritten wholesale: the profile's own rebind refuses a key already held by another
     * action, which is exactly wrong when replacing the whole table (every key is still held by
     * its previous owner). The bindings have already been validated by input_bindings_rebind. */
    profile->bindingCount = 0;
    if (INPUT_ACTION_COUNT > PLAYER_PROFILE_MAX_BINDINGS) return false;
    for (int i = 0; i < INPUT_ACTION_COUNT; i++) {
        const char *keyName = input_key_id(bindings->key[i]);
        if (keyName[0] == '\0') continue;
        snprintf(profile->bindings[profile->bindingCount].action,
                 sizeof(profile->bindings[0].action), "%s", kActionId[i]);
        snprintf(profile->bindings[profile->bindingCount].key, sizeof(profile->bindings[0].key),
                 "%s", keyName);
        profile->bindingCount++;
    }
    return true;
}

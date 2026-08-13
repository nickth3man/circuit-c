/*
 * input_bindings.h — what each key does, as data (MAP.md priorities 4 and 6).
 *
 * WHY THIS EXISTS. The rebinding library in player_profile.c has been able to store an
 * action -> key mapping for some time, and nothing read it: input.c asked raylib for KEY_W by
 * name, and the on-screen prompts were literal strings saying "W throttle". So a player could
 * rebind throttle, have it saved, reloaded and migrated, and still drive with W. This module is
 * the missing half — the one place that answers "which key is this action on right now" — and
 * both the sampler and the prompts read it.
 *
 * RAYLIB-FREE ON PURPOSE. The key codes here are raylib's own numeric values, written out
 * rather than included, so the whole model — defaults, rebinding, conflict detection, profile
 * round-trip, prompt labels — is testable in the headless build that never links raylib.
 * input.c compiles a _Static_assert per key proving the two agree, so a raylib that renumbered
 * a key would fail the build rather than silently rebind the game.
 *
 * KEY NAMES ARE THE PERSISTED FORM. The profile stores "KEY_W", not 87: a number would make a
 * saved profile depend on a numbering the player cannot see and we do not own.
 */
#ifndef CIRCUIT_INPUT_BINDINGS_H
#define CIRCUIT_INPUT_BINDINGS_H

#include <stdbool.h>
#include <stddef.h>

#include "game/player_profile.h"

/*
 * Everything a player may rebind.
 *
 * Steering is two actions rather than an axis because a keyboard has no axis; a gamepad stick
 * drives `steer` directly and is not bound here. The menu actions are included because MAP.md
 * priority 4 asks for menu navigation and confirmation to be rebindable too — a player who
 * cannot reach the controls screen cannot fix the controls.
 */
typedef enum {
    INPUT_ACTION_STEER_LEFT = 0,
    INPUT_ACTION_STEER_RIGHT,
    INPUT_ACTION_THROTTLE,
    INPUT_ACTION_BRAKE,
    INPUT_ACTION_HANDBRAKE,
    INPUT_ACTION_SHIFT_UP,
    INPUT_ACTION_SHIFT_DOWN,
    INPUT_ACTION_TOGGLE_AUTO,
    INPUT_ACTION_PAUSE, /* also: confirm / start, on the menu */
    INPUT_ACTION_RESET, /* also: back / cancel, on the menu */
    INPUT_ACTION_DEBUG, /* diagnostics overlay */
    INPUT_ACTION_MENU_LEFT,
    INPUT_ACTION_MENU_RIGHT,
    INPUT_ACTION_MENU_UP,
    INPUT_ACTION_MENU_DOWN,
    INPUT_ACTION_SETUP_TOGGLE,
    INPUT_ACTION_SETUP_RESET,
    INPUT_ACTION_RACE_SETUP_TOGGLE,
    INPUT_ACTION_SETTINGS_TOGGLE,
    INPUT_ACTION_COUNT
} InputAction;

/* One key code per action. Plain value data: safe in the persistent Game block. Named so
 * input.h can forward-declare it without adopting this header. */
typedef struct InputBindings {
    int key[INPUT_ACTION_COUNT];
} InputBindings;

/* The shipped layout: the keys the game has always used. */
void input_bindings_set_default(InputBindings *bindings);

/* The action's stable id, as stored in the profile ("throttle"). "" when out of range. */
const char *input_action_id(int action);

/* The action's player-facing label ("Throttle"). "" when out of range. */
const char *input_action_label(int action);

/* A key code's persisted name ("KEY_W"), or "" when the key is not one this game can bind. */
const char *input_key_id(int key);

/* A key code's short prompt label ("W", "SPACE"), or "?" for an unbindable key. */
const char *input_key_label(int key);

/* The key code for a persisted name, or -1 when the name is not one this game can bind. */
int input_key_from_id(const char *name);

/* How many keys can be bound, and the code of the i-th — enough for a controls screen to offer
 * a list without knowing the table. */
int input_bindable_key_count(void);
int input_bindable_key_at(int index);

/*
 * Bind `action` to `key`.
 *
 * A key already used by a DIFFERENT action is a conflict: nothing is changed, `conflict`
 * (when non-NULL) receives that action, and the call returns false. Rebinding an action to the
 * key it already holds succeeds and changes nothing. This is the same contract
 * player_profile_rebind() has, so a controls screen behaves identically whether it is editing
 * the live bindings or the saved ones.
 */
bool input_bindings_rebind(InputBindings *bindings, int action, int key, int *conflict);

/* The prompt label for whatever `action` is currently on ("W"). Never NULL. */
const char *input_bindings_label(const InputBindings *bindings, int action);

/*
 * Adopt every binding the profile names, leaving the shipped default for actions it does not.
 * An action bound to a key this build cannot bind is ignored rather than dropping the action to
 * "unbound" — a profile written by a newer build must not disarm a control in an older one.
 */
void input_bindings_from_profile(InputBindings *bindings, const PlayerProfile *profile);

/* Write every binding into the profile. Returns false when the profile could not hold them
 * all, which can only happen if the action table outgrows PLAYER_PROFILE_MAX_BINDINGS. */
bool input_bindings_to_profile(const InputBindings *bindings, PlayerProfile *profile);

#endif /* CIRCUIT_INPUT_BINDINGS_H */

/*
 * race_entrant.c — bounded, ordered entrant storage.
 *
 * Every function here is plain value manipulation on a caller-owned RaceRoster. Nothing
 * allocates, nothing keeps a pointer, and nothing reads the clock or a global, which is what
 * lets the roster live inside the persistent Game block and be reasoned about deterministically.
 */
#include "game/race_entrant.h"

#include <stddef.h>
#include <string.h>

void race_roster_init(RaceRoster *roster)
{
    if (roster == NULL) return;
    memset(roster, 0, sizeof(*roster));
    roster->nextId = 1u;       /* 0 is reserved for "no entrant" */
    roster->reuseFloorId = 1u; /* nothing has been retired yet */
    roster->localEntrantId = RACE_ENTRANT_ID_NONE;
}

const RaceEntrant *race_roster_find_const(const RaceRoster *roster, EntrantId id)
{
    if (roster == NULL || id == RACE_ENTRANT_ID_NONE) return NULL;
    for (int i = 0; i < roster->count; i++) {
        if (roster->entrants[i].id == id) return &roster->entrants[i];
    }
    return NULL;
}

RaceEntrant *race_roster_find(RaceRoster *roster, EntrantId id)
{
    if (roster == NULL || id == RACE_ENTRANT_ID_NONE) return NULL;
    for (int i = 0; i < roster->count; i++) {
        if (roster->entrants[i].id == id) return &roster->entrants[i];
    }
    return NULL;
}

const RaceEntrant *race_roster_local_const(const RaceRoster *roster)
{
    if (roster == NULL) return NULL;
    return race_roster_find_const(roster, roster->localEntrantId);
}

RaceEntrant *race_roster_local(RaceRoster *roster)
{
    if (roster == NULL) return NULL;
    return race_roster_find(roster, roster->localEntrantId);
}

bool race_roster_spawn(RaceRoster *roster, const RaceEntrantSpawn *spawn, EntrantId *outId)
{
    if (roster == NULL || spawn == NULL) return false;
    if (roster->count >= RACE_MAX_ENTRANTS) return false;

    const EntrantId id = (spawn->id != RACE_ENTRANT_ID_NONE) ? spawn->id : roster->nextId;
    if (id == RACE_ENTRANT_ID_NONE || id > RACE_ENTRANT_ID_MAX) return false;
    /* Occupied is a duplicate; below the floor is an identity that has already been used and
     * retired. Both hand one session id to two competitors, which is the one thing the id is
     * supposed to rule out. */
    if (id < roster->reuseFloorId) return false;
    if (race_roster_find_const(roster, id) != NULL) return false;
    /* One local designation per session. See the header: the alternative is last-write-wins,
     * which moves presentation onto another car with nothing said. */
    if (spawn->localPlayer && roster->localEntrantId != RACE_ENTRANT_ID_NONE) return false;

    /* Build the entrant completely before it is visible in the roster, so a rejected
     * definition/setup pair leaves the collection exactly as it was. */
    RaceEntrant entrant;
    memset(&entrant, 0, sizeof(entrant));
    entrant.progress.lastCrossedIndex = -1;
    entrant.progress.ticksSinceCross = 1000;

    if (spawn->definition != NULL)
        entrant.definition = *spawn->definition;
    else
        vehicle_definition_set_default(&entrant.definition);

    if (spawn->setup != NULL) {
        if (!vehicle_setup_is_valid(&entrant.definition, spawn->setup)) return false;
        entrant.setup = *spawn->setup;
    } else {
        vehicle_setup_set_default(&entrant.definition, &entrant.setup);
    }

    if (!vehicle_instance_init(&entrant.instance, &entrant.definition, &entrant.setup))
        return false;

    controller_init(&entrant.controller, spawn->controllerKind);
    controller_output_zero(&entrant.controllerOutput);
    entrant.id = id;
    entrant.gridSlot = (spawn->gridSlot >= 0) ? spawn->gridSlot : -1;

    /* Sorted insertion. The roster is small and spawning is rare, so the shift costs nothing
     * measurable and buys an iteration order that does not depend on insertion history. */
    int slot = roster->count;
    for (int i = 0; i < roster->count; i++) {
        if (roster->entrants[i].id > id) {
            slot = i;
            break;
        }
    }
    for (int i = roster->count; i > slot; i--) roster->entrants[i] = roster->entrants[i - 1];
    roster->entrants[slot] = entrant;
    roster->count++;

    /* Same reservation as above keeps this from wrapping onto RACE_ENTRANT_ID_NONE: once the
     * cursor reaches RACE_ENTRANT_ID_MAX + 1, automatic spawns fail the range check instead of
     * silently asking for the reserved zero. */
    if (id >= roster->nextId) roster->nextId = id + 1u;
    if (spawn->localPlayer) roster->localEntrantId = id;
    if (outId != NULL) *outId = id;
    return true;
}

bool race_roster_despawn(RaceRoster *roster, EntrantId id)
{
    if (roster == NULL || id == RACE_ENTRANT_ID_NONE) return false;
    for (int i = 0; i < roster->count; i++) {
        if (roster->entrants[i].id != id) continue;
        for (int j = i; j + 1 < roster->count; j++)
            roster->entrants[j] = roster->entrants[j + 1];
        roster->count--;
        memset(&roster->entrants[roster->count], 0, sizeof(roster->entrants[0]));
        /* Retire the identity. `id + 1u` cannot wrap: RACE_ENTRANT_ID_MAX reserves the top
         * value precisely so "one past the last id" is always representable. */
        if (id >= roster->reuseFloorId) roster->reuseFloorId = id + 1u;
        if (roster->localEntrantId == id) roster->localEntrantId = RACE_ENTRANT_ID_NONE;
        return true;
    }
    return false;
}

void race_entrant_reset(RaceEntrant *entrant)
{
    if (entrant == NULL) return;
    vehicle_instance_reset(&entrant->instance);
    controller_reset(&entrant->controller);
    memset(&entrant->result, 0, sizeof(entrant->result));
    /* `controllerOutput` is deliberately untouched. It is this tick's decision, already made
     * by the time a reset command is applied, and the session still has to read the gear
     * requests out of it. Zeroing it here would swallow the shift the driver asked for on the
     * same tick they pressed reset. */
}

void race_roster_reset(RaceRoster *roster)
{
    if (roster == NULL) return;
    for (int i = 0; i < roster->count; i++) race_entrant_reset(&roster->entrants[i]);
}

int race_roster_pair_count(const RaceRoster *roster)
{
    if (roster == NULL || roster->count < 2) return 0;
    return roster->count * (roster->count - 1) / 2;
}

bool race_roster_pair_at(const RaceRoster *roster, int index, int *aIndex, int *bIndex)
{
    if (roster == NULL || index < 0 || index >= race_roster_pair_count(roster)) return false;

    /* Row-major over the strict upper triangle: (0,1) (0,2) ... (0,n-1) (1,2) ... */
    int remaining = index;
    for (int a = 0; a + 1 < roster->count; a++) {
        const int rowLength = roster->count - 1 - a;
        if (remaining < rowLength) {
            if (aIndex != NULL) *aIndex = a;
            if (bIndex != NULL) *bIndex = a + 1 + remaining;
            return true;
        }
        remaining -= rowLength;
    }
    return false;
}

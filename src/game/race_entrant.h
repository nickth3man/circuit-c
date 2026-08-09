/*
 * race_entrant.h — one competitor, and the bounded collection that owns every competitor.
 *
 * WHY THIS EXISTS. Everything a car needs to be simulated used to live directly on `Game`:
 * one VehicleInstance, one Controller, one RacerProgress. That is a singleton, and a singleton
 * cannot hold a grid. `RaceEntrant` is the aggregate that owns exactly one competitor's mutable
 * state, and `RaceRoster` is the fixed-capacity, explicitly ordered collection of them.
 *
 * ORDER IS PART OF THE CONTRACT. Entrants are stored packed in `entrants[0 .. count)` in
 * ascending `EntrantId`, and every iteration, pair generation, and checksum walks them in that
 * order. Ids are assigned in ascending order, are never reused inside a session, and a spawn
 * that names its own id is inserted at its sorted position — so the order two entrants are
 * added in cannot change the order they are simulated in. That is what makes a checksum
 * comparable between a session built by loading content one way and the same session built by
 * loading it another way.
 *
 * NO ALLOCATION, NO POINTERS. The roster is a plain value array inside the persistent `Game`
 * block, so it survives a hot reload for the same reason everything else there does: it stores
 * no function pointer and nothing that points into the reloadable module. `RACE_MAX_ENTRANTS`
 * is a bound rather than a growth policy on purpose.
 *
 * WHAT AN ENTRANT DOES NOT OWN. Immutable content is copied in rather than aliased: an entrant
 * holds its own `VehicleDefinition` and frozen `VehicleSetup` so nothing it does can reach
 * another entrant's car. When the content catalog lands (issue #29) the definition becomes a
 * shared, hashed catalog entry and this copy becomes a reference. Track geometry is already
 * shared and immutable; only the entrant's `RacerProgress` moves around it.
 *
 * See docs/SIMULATION_OWNERSHIP.md for the full ownership table.
 */
#ifndef CIRCUIT_RACE_ENTRANT_H
#define CIRCUIT_RACE_ENTRANT_H

#include <stdbool.h>
#include <stdint.h>

#include "game/controller.h"
#include "game/controller_output.h"
#include "physics/vehicle.h"
#include "world/track.h"

/* Grid bound. Storage is a plain array in Game, so this is a memory decision, not a design
 * limit: one entrant costs roughly 2.6 KB. */
#define RACE_MAX_ENTRANTS 8

/*
 * Session-local competitor identity. Nonzero for an occupied slot; `RACE_ENTRANT_ID_NONE`
 * means "no entrant". Assigned in ascending order when a slot is filled and never reused, so
 * an id identifies one competitor for the whole session even after other entrants leave.
 */
typedef uint32_t EntrantId;
#define RACE_ENTRANT_ID_NONE 0u

/*
 * Where this entrant finished. Storage only: the rules that decide a finishing order belong to
 * the race session (issue #54), and nothing in this module writes anything but the reset value.
 * It lives here because a result is per-competitor state, and putting it anywhere else would
 * make two entrants share one outcome.
 */
typedef struct {
    bool finished;
    int finishPosition; /* 1-based once classified; 0 while the entrant is still racing */
} EntrantResult;

/*
 * One competitor.
 *
 * FIELD ORDER IS LOAD-BEARING. The six simulation members come first, in this order, because
 * `Game` overlays its temporary one-entrant compatibility spellings onto `entrants[0]`. game.h
 * asserts the layout agreement at compile time, so reordering these breaks the build rather
 * than the simulation.
 */
typedef struct {
    VehicleInstance instance;          /* mutable vehicle: pose, wheels, transmission, damage */
    Controller controller;             /* kind, frozen config, private decision memory */
    ControllerOutput controllerOutput; /* what the controller asked for this tick, pre-gating */
    RacerProgress progress;            /* this entrant's lap cursor around the shared track */
    VehicleDefinition definition;      /* immutable content, copied in (see the header note) */
    VehicleSetup setup;                /* frozen for the session at spawn */

    EntrantId id;         /* ascending, stable, never reused */
    int gridSlot;         /* starting slot; -1 when the session has not assigned one */
    EntrantResult result; /* per-entrant outcome storage */
} RaceEntrant;

/*
 * Every competitor in the session, packed and ordered.
 *
 * `entrants` must stay first: Game's compatibility view overlays `entrants[0]`.
 */
typedef struct {
    RaceEntrant entrants[RACE_MAX_ENTRANTS];
    int count;                /* occupied slots, always the prefix [0, count) */
    EntrantId nextId;         /* next auto-assigned id; monotonic, never reused */
    EntrantId localEntrantId; /* the human-designated entrant, or RACE_ENTRANT_ID_NONE */
} RaceRoster;

/*
 * What to put in a slot. A zeroed spawn is valid and means "the default car, driven by a
 * human, id assigned for me, no grid slot yet".
 */
typedef struct {
    EntrantId id;                        /* 0 assigns the next id; nonzero inserts sorted */
    const VehicleDefinition *definition; /* NULL uses the built-in default definition */
    const VehicleSetup *setup;           /* NULL uses that definition's default setup */
    ControllerKind controllerKind;
    bool localPlayer; /* this entrant is the one the local presentation follows */
    int gridSlot;     /* negative means unassigned */
} RaceEntrantSpawn;

/* Empty the roster: no entrants, no local designation, ids restarting from 1. */
void race_roster_init(RaceRoster *roster);

/*
 * Add one entrant.
 *
 * The new entrant is inserted at its sorted position by id, so `entrants[0 .. count)` stays
 * ascending whatever order the caller spawns in. `spawn->id` of 0 takes the next id; a nonzero
 * id is used verbatim and pushes `nextId` past it. Returns false and changes nothing when the
 * roster is full, the definition/setup pair is invalid, or the id is already taken. Writes the
 * assigned id to `outId` when that is non-NULL.
 */
bool race_roster_spawn(RaceRoster *roster, const RaceEntrantSpawn *spawn, EntrantId *outId);

/*
 * Remove one entrant and close the gap, preserving ascending order for everyone else. The id is
 * not reused. Clears the local designation when the local entrant is the one that left.
 * Returns false when no entrant has that id.
 */
bool race_roster_despawn(RaceRoster *roster, EntrantId id);

/*
 * Put one entrant back on the grid: vehicle state zeroed to its compiled spec, and the private
 * controller memory cleared so it does not drive to a plan computed for where it used to be.
 * Its identity, definition, setup, controller kind and grid slot survive — a reset changes the
 * situation, not who is racing. Lap progress is track state and is reset separately through
 * track_reset_progress_at().
 */
void race_entrant_reset(RaceEntrant *entrant);

/* race_entrant_reset() for every entrant, in ascending id order. */
void race_roster_reset(RaceRoster *roster);

/* The entrant with this id, or NULL. */
RaceEntrant *race_roster_find(RaceRoster *roster, EntrantId id);
const RaceEntrant *race_roster_find_const(const RaceRoster *roster, EntrantId id);

/* The human-designated entrant, or NULL when the session has none (a headless AI-only field).
 * Presentation — camera, audio, particles, HUD — follows this entrant and nothing else. */
RaceEntrant *race_roster_local(RaceRoster *roster);
const RaceEntrant *race_roster_local_const(const RaceRoster *roster);

/*
 * Deterministic unordered pair enumeration, for the stages that must consider every couple of
 * cars exactly once — collision broadphase first among them (issues #26/#27).
 *
 * `index` runs over [0, race_roster_pair_count()), and each step yields the entrant indices of
 * one pair with `*aIndex < *bIndex`. Because slots are ordered by ascending id, that is the
 * `(minId, maxId)` ordering docs/SIMULATION_OWNERSHIP.md requires.
 */
int race_roster_pair_count(const RaceRoster *roster);
bool race_roster_pair_at(const RaceRoster *roster, int index, int *aIndex, int *bIndex);

#endif /* CIRCUIT_RACE_ENTRANT_H */

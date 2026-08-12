/*
 * championship.h — issue #58: events, qualifying grids, points, standings, progression.
 *
 * A championship is a calendar of events (track + distance) over which drivers accumulate
 * points. Timing/classification stay in the session authorities; this module only maps
 * classified results to points and keeps deterministic standings. All rules are data-driven
 * (points table, dropped results, tie-breaks) and versioned for persistence.
 */
#ifndef CIRCUIT_CHAMPIONSHIP_H
#define CIRCUIT_CHAMPIONSHIP_H

#include <stdbool.h>
#include <stdint.h>

#include "content/vehicle_manifest.h" /* VEHICLE_CONTENT_ID_CAPACITY */
#include "world/track.h"              /* TRACK_ID_CHARS */

#define CHAMP_MAX_DRIVERS 8
#define CHAMP_MAX_EVENTS 16
#define CHAMP_POINTS_SLOTS 10 /* points awarded for positions 1..10 */

/* Season rules. All data-driven, none hard-coded into scoring code. */
typedef struct {
    int points[CHAMP_POINTS_SLOTS]; /* points per finishing position (1-based) */
    int dropCount;                  /* worst results dropped from the total */
    int eventCount;                 /* events in the calendar */
    char eventTrack[CHAMP_MAX_EVENTS][TRACK_ID_CHARS];
    int eventLaps[CHAMP_MAX_EVENTS];
    uint32_t version; /* schema/rule version for persistence validation */
} ChampionshipConfig;

/* One driver's running standing. */
typedef struct {
    char driverId[VEHICLE_CONTENT_ID_CAPACITY]; /* stable identity across events */
    int points;                                 /* total after drops */
    int wins;
    int bestFinishes[CHAMP_POINTS_SLOTS]; /* count of finishes at each position */
    int eventsCompleted;
    int eventPoints[CHAMP_MAX_EVENTS]; /* raw points per event; -1 = did not participate */
} ChampionshipStanding;

typedef struct {
    ChampionshipStanding standings[CHAMP_MAX_DRIVERS];
    int count;
    uint32_t version; /* matches ChampionshipConfig.version; mismatch = incompatible save */
} ChampionshipStandings;

/* Default points table (25-18-15-12-10-8-6-4-2-1), one drop, empty calendar. */
void championship_config_default(ChampionshipConfig *config);
/* A 3-event calendar on the given tracks. */
void championship_config_three_events(ChampionshipConfig *config);

/* Points awarded for a finishing position (1-based); 0 beyond the table. */
int championship_points_for_position(const ChampionshipConfig *config, int position);

/* Register a driver (idempotent). Returns false when the grid is full. */
bool championship_add_driver(ChampionshipStandings *standings, const char *driverId);

/* Apply one classified event result: position 0 means DNF/DNS (no points), `disqualified`
 * forces zero points for the event without consuming a drop. Recomputes totals after the
 * drop rule. */
bool championship_apply_event(const ChampionshipConfig *config,
                              ChampionshipStandings *standings, const char *driverId,
                              int finishedPosition, bool disqualified);

/* Recompute points/wins after drops; call after a batch of apply_event calls or a load. */
void championship_recompute(const ChampionshipConfig *config, ChampionshipStandings *standings);

/* Rank standings in place by (points desc, wins desc, best-finishes lexicographic desc,
 * driverId asc). Returns the ranked count. */
int championship_rank(const ChampionshipConfig *config, ChampionshipStandings *standings);

/* Deterministic serialization for profile persistence. Returns bytes written. */
int championship_serialize(const ChampionshipConfig *config,
                           const ChampionshipStandings *standings, char *out, size_t cap);
/* Parse a serialized championship; validates the version field and all ranges. */
bool championship_deserialize(ChampionshipConfig *config, ChampionshipStandings *standings,
                              const char *text);

#endif /* CIRCUIT_CHAMPIONSHIP_H */

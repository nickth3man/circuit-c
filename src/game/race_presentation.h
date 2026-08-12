/*
 * race_presentation.h — issue #56: immutable presentation snapshots for the HUD and results.
 *
 * A presentation snapshot is a read-only copy of the live race state at one tick, derived from
 * the session/profile authorities. The HUD reads it and computes nothing: timing, classification,
 * penalties, and records all come from the authoritative session stage, never from the render
 * path. Snapshots are plain values, safe to hold across a frame.
 */
#ifndef CIRCUIT_RACE_PRESENTATION_H
#define CIRCUIT_RACE_PRESENTATION_H

#include "game/race_session.h"
#include "world/track.h"

/* One entrant's live HUD row. */
typedef struct {
    EntrantId entrantId;
    int livePosition; /* 1-based running position from race_session_live_order */
    int lapsCompleted;
    float gapToLeaderS; /* session time behind the leader (0 for P1) */
    float lastLapTimeS;
    float bestLapTimeS;
    bool finished;
    bool isLocal; /* this is the entrant the camera/HUD follows */
} PresentationEntrantRow;

/* The full HUD snapshot at one tick. */
typedef struct {
    RacePhase phase;
    int countdownTicksRemaining;
    float sessionTimeS;
    int targetLaps;

    PresentationEntrantRow rows[RACE_MAX_ENTRANTS];
    int entrantCount;
    int localPosition; /* live position of the local entrant (0 = none) */

    /* Field-wide aggregates. */
    float fastestLapTimeS;
    EntrantId fastestLapEntrantId;

    /* Local entrant vehicle state (presentation only; the solver is the authority). */
    float fuelPercent;
    float tireWearPercent;
    float damagePercent;
    int absLevel;
    int tcsLevel;
    bool wrongWay;
    int pendingPenalties;
} RacePresentationSnapshot;

/* Build a snapshot from the authoritative session state for the local entrant. The track is
 * needed for nothing the snapshot reads today, but the signature reserves it for gap/distance
 * calculations that the HUD may need without re-localizing. */
void race_presentation_snapshot(const RaceSession *session, const TrackDefinition *track,
                                RacePresentationSnapshot *out);

/* Retry: reset the session to a fresh grid with the same rules and roster (issue #56). The
 * roster's entrants, definitions, and setups are preserved; only their mutable simulation state
 * is reset. Returns false when the session has no roster or no track. */
bool race_session_retry(RaceSession *session, const TrackDefinition *track);

#endif /* CIRCUIT_RACE_PRESENTATION_H */

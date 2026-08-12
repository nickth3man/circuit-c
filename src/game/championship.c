/*
 * championship.c — issue #58 implementation.
 */
#include "game/championship.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void championship_config_default(ChampionshipConfig *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    const int f1[] = { 25, 18, 15, 12, 10, 8, 6, 4, 2, 1 };
    for (int i = 0; i < CHAMP_POINTS_SLOTS; i++) config->points[i] = f1[i];
    config->dropCount = 1;
    config->version = 1u;
}

void championship_config_three_events(ChampionshipConfig *config)
{
    championship_config_default(config);
    config->eventCount = 3;
    snprintf(config->eventTrack[0], TRACK_ID_CHARS, "%s", "chicane");
    snprintf(config->eventTrack[1], TRACK_ID_CHARS, "%s", "sprint");
    snprintf(config->eventTrack[2], TRACK_ID_CHARS, "%s", "technical");
    config->eventLaps[0] = 3;
    config->eventLaps[1] = 2;
    config->eventLaps[2] = 4;
}

int championship_points_for_position(const ChampionshipConfig *config, int position)
{
    if (config == NULL || position <= 0 || position > CHAMP_POINTS_SLOTS) return 0;
    return config->points[position - 1];
}

bool championship_add_driver(ChampionshipStandings *standings, const char *driverId)
{
    if (standings == NULL || driverId == NULL || driverId[0] == '\0') return false;
    for (int i = 0; i < standings->count; i++) {
        if (strcmp(standings->standings[i].driverId, driverId) == 0) return true; /* exists */
    }
    if (standings->count >= CHAMP_MAX_DRIVERS) return false;
    ChampionshipStanding *s = &standings->standings[standings->count];
    memset(s, 0, sizeof(*s));
    snprintf(s->driverId, sizeof(s->driverId), "%s", driverId);
    for (int e = 0; e < CHAMP_MAX_EVENTS; e++) s->eventPoints[e] = -1;
    standings->count++;
    return true;
}

static void recompute_one(const ChampionshipConfig *config, ChampionshipStanding *s)
{
    if (config == NULL || s == NULL) return;
    /* Raw totals first. eventPoints stores the finishing POSITION (1-based), 0 = DNF/DNS,
     * -2 = DSQ; points are derived through the table so a rule change is a data change. */
    int rawTotal = 0;
    int wins = 0;
    int completed = 0;
    memset(s->bestFinishes, 0, sizeof(s->bestFinishes));
    for (int e = 0; e < CHAMP_MAX_EVENTS; e++) {
        const int position = s->eventPoints[e];
        if (position < 0) continue; /* -1 not run, -2 DSQ */
        const int pts = championship_points_for_position(config, position);
        rawTotal += pts;
        completed++;
        if (position >= 1 && position <= CHAMP_POINTS_SLOTS) {
            s->bestFinishes[position - 1]++;
            if (position == 1) wins++;
        }
    }
    /* Drop the worst `dropCount` scoring results (DSQ results never drop). A drop only
     * applies once the driver has completed more events than it can drop — dropping the only
     * result of a one-race season would make a win worth nothing. The drop is computed from a
     * local sorted copy, never by mutating the stored results, so incremental apply_event
     * calls and persistence round-trips see the same totals. */
    int scoring[CHAMP_MAX_EVENTS];
    int sc = 0;
    for (int e = 0; e < CHAMP_MAX_EVENTS; e++) {
        const int position = s->eventPoints[e];
        if (position < 0) continue;
        scoring[sc++] = championship_points_for_position(config, position);
    }
    for (int i = 1; i < sc; i++) {
        const int key = scoring[i];
        int j = i - 1;
        while (j >= 0 && scoring[j] > key) {
            scoring[j + 1] = scoring[j];
            j--;
        }
        scoring[j + 1] = key;
    }
    int dropped = 0;
    if (completed > config->dropCount) {
        for (int i = 0; i < config->dropCount && i < sc; i++) dropped += scoring[i];
    }
    s->points = rawTotal - dropped;
    s->wins = wins;
    s->eventsCompleted = completed;
}

void championship_recompute(const ChampionshipConfig *config, ChampionshipStandings *standings)
{
    if (config == NULL || standings == NULL) return;
    for (int i = 0; i < standings->count; i++) {
        recompute_one(config, &standings->standings[i]);
    }
}

bool championship_apply_event(const ChampionshipConfig *config,
                              ChampionshipStandings *standings, const char *driverId,
                              int finishedPosition, bool disqualified)
{
    if (config == NULL || standings == NULL || driverId == NULL) return false;
    if (!championship_add_driver(standings, driverId)) return false;
    ChampionshipStanding *s = NULL;
    for (int i = 0; i < standings->count; i++) {
        if (strcmp(standings->standings[i].driverId, driverId) == 0)
            s = &standings->standings[i];
    }
    if (s == NULL) return false;
    /* Find the first unused event slot. */
    int slot = -1;
    for (int e = 0; e < CHAMP_MAX_EVENTS; e++) {
        if (s->eventPoints[e] < 0) {
            slot = e;
            break;
        }
    }
    if (slot < 0) return false;
    /* DNS/DNF/DSQ: position 0 scores nothing and is droppable; DSQ (stored -2) scores nothing
     * and is never dropped. */
    s->eventPoints[slot] = disqualified ? -2 : finishedPosition;
    /* Recompute this driver's totals. */
    recompute_one(config, s);
    return true;
}

static int compare_standings(const ChampionshipConfig *config, const ChampionshipStanding *a,
                             const ChampionshipStanding *b)
{
    (void)config; /* reserved for future tie-break rules */
    if (a->points != b->points) return (a->points > b->points) ? -1 : 1;
    if (a->wins != b->wins) return (a->wins > b->wins) ? -1 : 1;
    for (int p = 0; p < CHAMP_POINTS_SLOTS; p++) {
        if (a->bestFinishes[p] != b->bestFinishes[p])
            return (a->bestFinishes[p] > b->bestFinishes[p]) ? -1 : 1;
    }
    return strcmp(a->driverId, b->driverId);
}

int championship_rank(const ChampionshipConfig *config, ChampionshipStandings *standings)
{
    if (config == NULL || standings == NULL) return 0;
    /* Insertion sort. */
    for (int i = 1; i < standings->count; i++) {
        ChampionshipStanding key = standings->standings[i];
        int j = i - 1;
        while (j >= 0 && compare_standings(config, &standings->standings[j], &key) > 0) {
            standings->standings[j + 1] = standings->standings[j];
            j--;
        }
        standings->standings[j + 1] = key;
    }
    return standings->count;
}

int championship_serialize(const ChampionshipConfig *config,
                           const ChampionshipStandings *standings, char *out, size_t cap)
{
    if (config == NULL || standings == NULL || out == NULL || cap == 0) return 0;
    int used = snprintf(out, cap,
                        "{\"schema\":\"circuit/championship\",\"version\":%u,"
                        "\"dropCount\":%d,\"eventCount\":%d,\"points\":[",
                        config->version, config->dropCount, config->eventCount);
    for (int i = 0; i < CHAMP_POINTS_SLOTS && used < (int)cap; i++) {
        used += snprintf(out + used, (size_t)(cap - used), "%d%s", config->points[i],
                         (i + 1 < CHAMP_POINTS_SLOTS) ? "," : "");
    }
    used += snprintf(out + used, (size_t)(cap - used), "],\"drivers\":[");
    for (int d = 0; d < standings->count && used < (int)cap; d++) {
        const ChampionshipStanding *s = &standings->standings[d];
        used += snprintf(out + used, (size_t)(cap - used),
                         "{\"id\":\"%s\",\"points\":%d,"
                         "\"wins\":%d,\"events\":[",
                         s->driverId, s->points, s->wins);
        for (int e = 0; e < CHAMP_MAX_EVENTS && used < (int)cap; e++) {
            used += snprintf(out + used, (size_t)(cap - used), "%d%s", s->eventPoints[e],
                             (e + 1 < CHAMP_MAX_EVENTS) ? "," : "");
        }
        used += snprintf(out + used, (size_t)(cap - used), "]}%s",
                         (d + 1 < standings->count) ? "," : "");
    }
    used += snprintf(out + used, (size_t)(cap - used), "]}\n");
    return (used < (int)cap) ? used : 0;
}

bool championship_deserialize(ChampionshipConfig *config, ChampionshipStandings *standings,
                              const char *text)
{
    if (config == NULL || standings == NULL || text == NULL) return false;
    ChampionshipConfig cfg;
    ChampionshipStandings st;
    memset(&cfg, 0, sizeof(cfg));
    memset(&st, 0, sizeof(st));
    championship_config_default(&cfg);

    /* Version check is the compatibility gate. */
    const char *vs = strstr(text, "\"version\":");
    if (vs == NULL) return false;
    vs += strlen("\"version\":");
    char *vEnd = NULL;
    const unsigned long version = strtoul(vs, &vEnd, 10);
    if (vEnd == vs || version != 1u) return false;
    cfg.version = 1u;

    /* Parse points table. */
    const char *p = strstr(text, "\"points\":[");
    if (p == NULL) return false;
    p += strlen("\"points\":[");
    for (int i = 0; i < CHAMP_POINTS_SLOTS; i++) {
        char *end = NULL;
        const long v = strtol(p, &end, 10);
        if (end == p || v < 0 || v > 1000) return false;
        cfg.points[i] = (int)v;
        p = end;
        if (*p == ',') p++;
    }
    const char *dc = strstr(text, "\"dropCount\":");
    if (dc == NULL) return false;
    dc += strlen("\"dropCount\":");
    char *dcEnd = NULL;
    const long dropCount = strtol(dc, &dcEnd, 10);
    if (dcEnd == dc || dropCount < 0 || dropCount > CHAMP_MAX_EVENTS) return false;
    cfg.dropCount = (int)dropCount;

    /* Parse drivers. */
    const char *d = strstr(text, "\"drivers\":[");
    if (d != NULL) {
        d += strlen("\"drivers\":[");
        while (*d != ']' && *d != '\0' && st.count < CHAMP_MAX_DRIVERS) {
            const char *idStart = strstr(d, "\"id\":\"");
            if (idStart == NULL) break;
            idStart += strlen("\"id\":\"");
            const char *idEnd = strchr(idStart, '"');
            if (idEnd == NULL) break;
            char driverId[VEHICLE_CONTENT_ID_CAPACITY];
            const size_t len = (size_t)(idEnd - idStart);
            if (len >= sizeof(driverId)) return false;
            memcpy(driverId, idStart, len);
            driverId[len] = '\0';
            (void)championship_add_driver(&st, driverId);
            ChampionshipStanding *s = &st.standings[st.count - 1];
            for (int e = 0; e < CHAMP_MAX_EVENTS; e++) s->eventPoints[e] = -1;
            const char *ev = strstr(idEnd, "\"events\":[");
            if (ev == NULL) return false;
            ev += strlen("\"events\":[");
            for (int e = 0; e < CHAMP_MAX_EVENTS; e++) {
                char *end = NULL;
                const long v = strtol(ev, &end, 10);
                if (end == ev) break;
                s->eventPoints[e] = (int)v;
                ev = end;
                if (*ev == ',') ev++;
            }
            d = ev;
            while (*d != '\0' && *d != '}') d++;
            if (*d == '}') d++;
            if (*d == ',') d++;
        }
    }
    championship_recompute(&cfg, &st);
    *config = cfg;
    *standings = st;
    return true;
}

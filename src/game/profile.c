/*
 * profile.c — the built-in profiling backend (-DDRIFTY_PROFILE).
 *
 * Deliberately small: a fixed table of zones keyed by the name pointer, total and worst-case
 * nanoseconds, and a summary printed at shutdown. It answers "which zone got slower?"
 * without a dependency, a server, or a capture file. When that stops being enough, vendor
 * Tracy and rebuild with -DDRIFTY_TRACY; the call sites do not change.
 *
 * Compiled into every configuration so the build file lists stay identical; without
 * DRIFTY_PROFILE it contains nothing.
 */
#include "game/profile.h"

#if defined(DRIFTY_PROFILE) && !defined(DRIFTY_TRACY)

#include <string.h>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif

#define PROFILE_MAX_ZONES 64

typedef struct {
    const char *name;
    uint64_t totalNs;
    uint64_t worstNs;
    uint64_t calls;
} ProfileZone;

static ProfileZone g_zones[PROFILE_MAX_ZONES];
static int g_zoneCount = 0;
static uint64_t g_frames = 0;

uint64_t profile_now_ns(void)
{
#if defined(_WIN32)
    static LARGE_INTEGER frequency;
    if (frequency.QuadPart == 0) QueryPerformanceFrequency(&frequency);
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint64_t)((now.QuadPart * 1000000000LL) / frequency.QuadPart);
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;
#endif
}

void profile_zone_end(const char *name, uint64_t startedNs)
{
    const uint64_t elapsed = profile_now_ns() - startedNs;

    for (int i = 0; i < g_zoneCount; i++) {
        if (g_zones[i].name == name) {
            g_zones[i].totalNs += elapsed;
            g_zones[i].calls++;
            if (elapsed > g_zones[i].worstNs) g_zones[i].worstNs = elapsed;
            return;
        }
    }
    if (g_zoneCount >= PROFILE_MAX_ZONES) return;

    g_zones[g_zoneCount].name = name;
    g_zones[g_zoneCount].totalNs = elapsed;
    g_zones[g_zoneCount].worstNs = elapsed;
    g_zones[g_zoneCount].calls = 1;
    g_zoneCount++;
}

void profile_frame_mark(void)
{
    g_frames++;
}

void profile_report(FILE *out)
{
    if (out == NULL) out = stderr;

    fprintf(out, "\nPROFILE: %llu frames, %d zones\n", (unsigned long long)g_frames,
            g_zoneCount);
    fprintf(out, "%-24s %10s %12s %12s %12s\n", "zone", "calls", "total ms", "mean us",
            "worst us");
    for (int i = 0; i < g_zoneCount; i++) {
        const ProfileZone *zone = &g_zones[i];
        const double totalMs = (double)zone->totalNs / 1.0e6;
        const double meanUs =
            (zone->calls > 0) ? (double)zone->totalNs / (double)zone->calls / 1.0e3 : 0.0;
        fprintf(out, "%-24s %10llu %12.3f %12.3f %12.3f\n", zone->name,
                (unsigned long long)zone->calls, totalMs, meanUs,
                (double)zone->worstNs / 1.0e3);
    }
}

#else

/* ISO C forbids an empty translation unit. */
typedef int drifty_profile_translation_unit_not_empty;

#endif

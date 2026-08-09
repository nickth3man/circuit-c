/*
 * profile.h — zone instrumentation with two backends and no mandatory dependency.
 *
 *   default            the macros compile to nothing at all
 *   -DCIRCUIT_PROFILE   built-in accumulating timers; a summary is printed at shutdown
 *   -DCIRCUIT_TRACY     zones are forwarded to Tracy (third_party/tracy must be present)
 *
 * Usage is a matched pair, which is what lets the same call sites serve both backends:
 *
 *     CIRCUIT_ZONE_BEGIN(update, "FixedUpdate");
 *     ...
 *     CIRCUIT_ZONE_END(update);
 *
 * `make profile` picks the backend for you: Tracy when it is vendored, the built-in timers
 * otherwise. Neither is ever part of a release build.
 *
 * Zone names must be string literals with static lifetime — both backends key on the
 * pointer, not on the characters.
 */
#ifndef CIRCUIT_PROFILE_H
#define CIRCUIT_PROFILE_H

#include <stdint.h>
#include <stdio.h>

#if defined(CIRCUIT_TRACY)

#include "tracy/TracyC.h"

#define CIRCUIT_ZONE_BEGIN(handle, name) TracyCZoneN(handle, name, 1)
#define CIRCUIT_ZONE_END(handle) TracyCZoneEnd(handle)
#define CIRCUIT_FRAME_MARK() TracyCFrameMark
#define CIRCUIT_PROFILE_REPORT(out) ((void)(out))

#elif defined(CIRCUIT_PROFILE)

/* Monotonic nanoseconds since an arbitrary origin. */
uint64_t profile_now_ns(void);
void profile_zone_end(const char *name, uint64_t startedNs);
void profile_frame_mark(void);
void profile_report(FILE *out);

#define CIRCUIT_ZONE_BEGIN(handle, name) \
    const char *handle##_name = (name);  \
    const uint64_t handle##_started = profile_now_ns()

#define CIRCUIT_ZONE_END(handle) profile_zone_end(handle##_name, handle##_started)
#define CIRCUIT_FRAME_MARK() profile_frame_mark()
#define CIRCUIT_PROFILE_REPORT(out) profile_report(out)

#else

#define CIRCUIT_ZONE_BEGIN(handle, name) ((void)0)
#define CIRCUIT_ZONE_END(handle) ((void)0)
#define CIRCUIT_FRAME_MARK() ((void)0)
#define CIRCUIT_PROFILE_REPORT(out) ((void)(out))

#endif

#endif /* CIRCUIT_PROFILE_H */

/*
 * build_info.h — provenance of the running binary.
 *
 * The build scripts pass -DCIRCUIT_BUILD_COMMIT=..., -DCIRCUIT_BUILD_DIRTY=... and
 * -DCIRCUIT_BUILD_MODE=... on the command line. Everything falls back to "unknown" so a
 * hand-rolled compile still builds. Nothing here affects the simulation: it exists so that a
 * failure bundle can be reproduced without reconstructing the environment by hand.
 *
 * Deliberately no __DATE__ / __TIME__: they would make otherwise identical builds differ.
 */
#ifndef CIRCUIT_BUILD_INFO_H
#define CIRCUIT_BUILD_INFO_H

#include <stdio.h>

#ifndef CIRCUIT_BUILD_COMMIT
#define CIRCUIT_BUILD_COMMIT "unknown"
#endif

#ifndef CIRCUIT_BUILD_DIRTY
#define CIRCUIT_BUILD_DIRTY "unknown"
#endif

#ifndef CIRCUIT_BUILD_BRANCH
#define CIRCUIT_BUILD_BRANCH "unknown"
#endif

#ifndef CIRCUIT_BUILD_MODE
#define CIRCUIT_BUILD_MODE "unknown"
#endif

#ifndef CIRCUIT_BUILD_FLAGS
#define CIRCUIT_BUILD_FLAGS "unknown"
#endif

#if defined(__clang__)
#define CIRCUIT_BUILD_COMPILER "clang " __clang_version__
#elif defined(__GNUC__)
#define CIRCUIT_BUILD_COMPILER "gcc " __VERSION__
#elif defined(_MSC_VER)
#define CIRCUIT_BUILD_COMPILER "msvc"
#else
#define CIRCUIT_BUILD_COMPILER "unknown"
#endif

#if defined(_WIN32)
#define CIRCUIT_BUILD_PLATFORM "windows-x64"
#elif defined(__linux__)
#define CIRCUIT_BUILD_PLATFORM "linux-x64"
#elif defined(__APPLE__)
#define CIRCUIT_BUILD_PLATFORM "macos"
#else
#define CIRCUIT_BUILD_PLATFORM "unknown"
#endif

/* One `key: value` line per fact. This is exactly what git_info.txt contains. */
static inline void build_info_write(FILE *out)
{
    if (out == NULL) return;
    fprintf(out, "commit:   %s\n", CIRCUIT_BUILD_COMMIT);
    fprintf(out, "branch:   %s\n", CIRCUIT_BUILD_BRANCH);
    fprintf(out, "dirty:    %s\n", CIRCUIT_BUILD_DIRTY);
    fprintf(out, "mode:     %s\n", CIRCUIT_BUILD_MODE);
    fprintf(out, "compiler: %s\n", CIRCUIT_BUILD_COMPILER);
    fprintf(out, "flags:    %s\n", CIRCUIT_BUILD_FLAGS);
    fprintf(out, "platform: %s\n", CIRCUIT_BUILD_PLATFORM);
}

#endif /* CIRCUIT_BUILD_INFO_H */

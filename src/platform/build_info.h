/*
 * build_info.h — provenance of the running binary.
 *
 * The build scripts pass -DDRIFTY_BUILD_COMMIT=..., -DDRIFTY_BUILD_DIRTY=... and
 * -DDRIFTY_BUILD_MODE=... on the command line. Everything falls back to "unknown" so a
 * hand-rolled compile still builds. Nothing here affects the simulation: it exists so that a
 * failure bundle can be reproduced without reconstructing the environment by hand.
 *
 * Deliberately no __DATE__ / __TIME__: they would make otherwise identical builds differ.
 */
#ifndef DRIFTY_BUILD_INFO_H
#define DRIFTY_BUILD_INFO_H

#include <stdio.h>

#ifndef DRIFTY_BUILD_COMMIT
#define DRIFTY_BUILD_COMMIT "unknown"
#endif

#ifndef DRIFTY_BUILD_DIRTY
#define DRIFTY_BUILD_DIRTY "unknown"
#endif

#ifndef DRIFTY_BUILD_BRANCH
#define DRIFTY_BUILD_BRANCH "unknown"
#endif

#ifndef DRIFTY_BUILD_MODE
#define DRIFTY_BUILD_MODE "unknown"
#endif

#ifndef DRIFTY_BUILD_FLAGS
#define DRIFTY_BUILD_FLAGS "unknown"
#endif

#if defined(__clang__)
#define DRIFTY_BUILD_COMPILER "clang " __clang_version__
#elif defined(__GNUC__)
#define DRIFTY_BUILD_COMPILER "gcc " __VERSION__
#elif defined(_MSC_VER)
#define DRIFTY_BUILD_COMPILER "msvc"
#else
#define DRIFTY_BUILD_COMPILER "unknown"
#endif

#if defined(_WIN32)
#define DRIFTY_BUILD_PLATFORM "windows-x64"
#elif defined(__linux__)
#define DRIFTY_BUILD_PLATFORM "linux-x64"
#elif defined(__APPLE__)
#define DRIFTY_BUILD_PLATFORM "macos"
#else
#define DRIFTY_BUILD_PLATFORM "unknown"
#endif

/* One `key: value` line per fact. This is exactly what git_info.txt contains. */
static inline void build_info_write(FILE *out)
{
    if (out == NULL) return;
    fprintf(out, "commit:   %s\n", DRIFTY_BUILD_COMMIT);
    fprintf(out, "branch:   %s\n", DRIFTY_BUILD_BRANCH);
    fprintf(out, "dirty:    %s\n", DRIFTY_BUILD_DIRTY);
    fprintf(out, "mode:     %s\n", DRIFTY_BUILD_MODE);
    fprintf(out, "compiler: %s\n", DRIFTY_BUILD_COMPILER);
    fprintf(out, "flags:    %s\n", DRIFTY_BUILD_FLAGS);
    fprintf(out, "platform: %s\n", DRIFTY_BUILD_PLATFORM);
}

#endif /* DRIFTY_BUILD_INFO_H */

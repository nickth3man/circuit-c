/*
 * content_paths.h — issue #46: content discovery independent of the current working directory.
 *
 * The game reads its selectable content (data/tracks, data/vehicles, class rules) relative to
 * a PRODUCT ROOT. The root is discovered in order:
 *
 *   1. CIRCUIT_CONTENT_DIR (environment override, e.g. a custom install layout)
 *   2. the executable's own directory  (packaged bundle: <bundle>/circuit.exe + <bundle>/data/)
 *   3. the working directory            (dev workflow: run from the repository root)
 *
 * The first candidate whose `data` directory exists wins. Every content loader routes its
 * paths through content_path_resolve(), so a packaged build launched from any directory finds
 * the same selectable content. Headless mode never opens a window, so resolution is pure path
 * arithmetic plus an existence check.
 */
#ifndef CIRCUIT_CONTENT_PATHS_H
#define CIRCUIT_CONTENT_PATHS_H

#include <stddef.h>

/* Resolve a content-relative path (e.g. "data/tracks") against the product root and write the
 * result into `out`. Never fails: the worst case returns `rel` unchanged (working-directory
 * semantics, which preserves every existing dev/test invocation). */
void content_path_resolve(const char *rel, char *out, size_t cap);

/* The discovered product root ("" when nothing was found and the CWD fallback is in effect). */
void content_path_root(char *out, size_t cap);

#endif /* CIRCUIT_CONTENT_PATHS_H */

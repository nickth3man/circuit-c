/*
 * content_paths.c — issue #46 implementation.
 */
#include "core/content_paths.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#if defined(_WIN32)
#include <windows.h>
#endif

#define CONTENT_ENV_OVERRIDE "CIRCUIT_CONTENT_DIR"
#define CONTENT_DATA_PROBE "data"

/* Path separators: '/' is valid on Windows and POSIX; use it everywhere. */

static void dirname_of(const char *path, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    size_t len = strlen(path);
    size_t cut = 0;
    for (size_t i = 0; i < len; i++) {
        if (path[i] == '/' || path[i] == '\\') cut = i;
    }
    if (cut == 0) {
        out[0] = '.';
        out[1] = '\0';
        return;
    }
    if (cut + 1 >= cap) cut = cap - 2;
    memcpy(out, path, cut);
    out[cut] = '\0';
}

static bool dir_exists(const char *path)
{
    if (path == NULL || path[0] == '\0') return false;
    struct stat st;
    return stat(path, &st) == 0 && (st.st_mode & S_IFDIR) != 0;
}

static bool probe_root(const char *root, char *out, size_t cap)
{
    if (root == NULL || root[0] == '\0') return false;
    char probe[1024];
    snprintf(probe, sizeof(probe), "%s/%s", root, CONTENT_DATA_PROBE);
    if (!dir_exists(probe)) return false;
    snprintf(out, cap, "%s", root);
    return true;
}

static void executable_dir(char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    out[0] = '\0';
#if defined(_WIN32)
    char exe[1024];
    const DWORD n = GetModuleFileNameA(NULL, exe, (DWORD)sizeof(exe));
    if (n > 0 && n < (DWORD)sizeof(exe)) dirname_of(exe, out, cap);
#elif defined(__linux__)
    char link[4096];
    const ssize_t n = readlink("/proc/self/exe", link, sizeof(link) - 1);
    if (n > 0) {
        link[n] = '\0';
        dirname_of(link, out, cap);
    }
#else
    (void)out;
#endif
}

static void discover_root(char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    out[0] = '\0';

    /* 1. Environment override. */
    const char *env = getenv(CONTENT_ENV_OVERRIDE);
    if (probe_root(env, out, cap)) return;

    /* 2. Executable-relative (packaged bundle layout). */
    char exeDir[1024];
    executable_dir(exeDir, sizeof(exeDir));
    if (probe_root(exeDir, out, cap)) return;
    /* The bundle may live one level below a wrapper (e.g. build/release/ in-tree). */
    {
        char up[1024];
        snprintf(up, sizeof(up), "%s/..", exeDir);
        if (probe_root(up, out, cap)) return;
    }

    /* 3. Working directory (dev workflow). */
    (void)probe_root(".", out, cap);
}

void content_path_resolve(const char *rel, char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    if (rel == NULL || rel[0] == '\0') {
        out[0] = '\0';
        return;
    }
    char root[1024];
    discover_root(root, sizeof(root));
    if (root[0] != '\0') {
        snprintf(out, cap, "%s/%s", root, rel);
    } else {
        snprintf(out, cap, "%s", rel); /* CWD semantics: preserves dev/test behaviour */
    }
}

void content_path_root(char *out, size_t cap)
{
    if (out == NULL || cap == 0) return;
    discover_root(out, cap);
}

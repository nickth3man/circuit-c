/*
 * hotreload_posix.c — dlopen / dlsym / dlclose loader.
 *
 * Same safe-swap sequence as hotreload_windows.c: a candidate module is loaded and fully
 * resolved before the working module is disturbed, and any failure leaves the running
 * module and its function table untouched.
 *
 * NOT VERIFIED ON A REAL POSIX HOST. This project's development environment is Windows;
 * this file is written to be compile-ready and to mirror the Windows loader exactly, but it
 * has not been exercised. Treat the first Linux or macOS run as a bring-up task.
 *
 * POSIX does not lock a mapped shared object the way Windows does, so copying to a unique
 * name is not strictly required. It is done anyway: dlopen caches by resolved path, so
 * reopening the same filename can hand back the already-mapped object instead of the newly
 * built one.
 */
#if !defined(_WIN32) && defined(DRIFTY_HOT_RELOAD)

#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "raylib.h"

#include "hotreload.h"

#define MODULE_PATH_MAX 512

/* The platform-side table. One definition per entry point, generated from the one list. */
#define ENTRY(name, ...) name##_t *name = NULL;
GAME_ENTRY_POINTS
#undef ENTRY

typedef struct {
    void *handle;
    char  loadedPath[MODULE_PATH_MAX];

    #define ENTRY(name, ...) name##_t *name;
    GAME_ENTRY_POINTS
    #undef ENTRY
} GameModule;

static GameModule g_active;
static long       g_lastModTime = 0;
static long       g_lastFailedModTime = 0;   /* so a broken build logs once, not every frame */
static int        g_loadCounter = 0;

static bool copy_file(const char *from, const char *to)
{
    FILE *src = fopen(from, "rb");
    if (src == NULL) return false;

    FILE *dst = fopen(to, "wb");
    if (dst == NULL) {
        fclose(src);
        return false;
    }

    char buffer[64 * 1024];
    bool ok = true;
    for (;;) {
        const size_t got = fread(buffer, 1, sizeof(buffer), src);
        if (got == 0) {
            ok = (ferror(src) == 0);
            break;
        }
        if (fwrite(buffer, 1, got, dst) != got) {
            ok = false;
            break;
        }
    }

    if (fclose(dst) != 0) ok = false;
    fclose(src);

    if (!ok) unlink(to);
    return ok;
}

static void unload_module(GameModule *module)
{
    if (module == NULL) return;

    if (module->handle != NULL) {
        dlclose(module->handle);
        module->handle = NULL;
    }
    if (module->loadedPath[0] != '\0') {
        unlink(module->loadedPath);
        module->loadedPath[0] = '\0';
    }
}

/* logFailures is cleared while retrying a module that has already been reported, so a
 * broken build does not fill the log once per frame. */
static bool load_candidate(GameModule *out, bool logFailures)
{
    GameModule candidate;
    memset(&candidate, 0, sizeof(candidate));

    g_loadCounter++;
    snprintf(candidate.loadedPath, sizeof(candidate.loadedPath),
             "build/game_load_%d.so", g_loadCounter);

    if (!copy_file(GAME_MODULE_NAME, candidate.loadedPath)) {
        if (logFailures) {
            TRACELOG(LOG_WARNING, "HOTRELOAD: could not copy %s to %s",
                     GAME_MODULE_NAME, candidate.loadedPath);
        }
        return false;
    }

    candidate.handle = dlopen(candidate.loadedPath, RTLD_NOW | RTLD_LOCAL);
    if (candidate.handle == NULL) {
        if (logFailures) {
            TRACELOG(LOG_ERROR, "HOTRELOAD: could not load %s (%s)",
                     candidate.loadedPath, dlerror());
        }
        unlink(candidate.loadedPath);
        return false;
    }

    #define ENTRY(name, ...)                                                       \
        candidate.name = (name##_t *)dlsym(candidate.handle, #name);               \
        if (candidate.name == NULL) {                                              \
            if (logFailures) {                                                     \
                TRACELOG(LOG_ERROR, "HOTRELOAD: missing symbol %s in %s",          \
                         #name, candidate.loadedPath);                             \
            }                                                                      \
            unload_module(&candidate);                                             \
            return false;                                                          \
        }
    GAME_ENTRY_POINTS
    #undef ENTRY

    *out = candidate;
    return true;
}

static void publish(const GameModule *module)
{
    #define ENTRY(name, ...) name = module->name;
    GAME_ENTRY_POINTS
    #undef ENTRY
}

bool Game_LoadModule(void)
{
    GameModule candidate;
    if (!load_candidate(&candidate, true)) return false;

    unload_module(&g_active);
    g_active = candidate;
    publish(&g_active);

    g_lastModTime = GetFileModTime(GAME_MODULE_NAME);
    TRACELOG(LOG_INFO, "HOTRELOAD: loaded %s", GAME_MODULE_NAME);
    return true;
}

bool Game_MaybeHotReload(Game *game)
{
    const long modTime = GetFileModTime(GAME_MODULE_NAME);
    if (modTime == 0 || modTime == g_lastModTime) return false;

    const bool firstAttempt = (modTime != g_lastFailedModTime);

    GameModule candidate;
    if (!load_candidate(&candidate, firstAttempt)) {
        /* g_lastModTime is left alone so the next frame retries. */
        if (firstAttempt) {
            g_lastFailedModTime = modTime;
            TRACELOG(LOG_WARNING,
                     "HOTRELOAD: candidate rejected; staying on the previous module");
        }
        return false;
    }

    g_lastModTime = modTime;

    if (g_active.handle != NULL) game_pre_reload(game);

    GameModule previous = g_active;
    g_active = candidate;
    publish(&g_active);

    game_post_reload(game);

    unload_module(&previous);

    TRACELOG(LOG_INFO, "HOTRELOAD: reloaded %s", GAME_MODULE_NAME);
    return true;
}

void Game_UnloadModule(void)
{
    unload_module(&g_active);

    #define ENTRY(name, ...) name = NULL;
    GAME_ENTRY_POINTS
    #undef ENTRY
}

#else

/* Not the active platform for this build configuration. */
typedef int drifty_hotreload_posix_unused;

#endif /* !_WIN32 && DRIFTY_HOT_RELOAD */

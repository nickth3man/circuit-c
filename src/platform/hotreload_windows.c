/*
 * hotreload_windows.c — LoadLibrary / GetProcAddress / FreeLibrary loader.
 *
 * This is the primary platform target. The loader never unloads a working module until a
 * replacement has been proven good:
 *
 *   1. Poll the watched module's modification time.
 *   2. Copy it to a uniquely named load target. Windows locks a loaded DLL, so the copy is
 *      what lets the next build overwrite build/game.dll while the game is running.
 *   3. LoadLibrary the copy.
 *   4. Resolve every required entry point into a TEMPORARY table.
 *   5. On any failure, free only the candidate, delete its copy, keep the previous module
 *      and its table, log, and keep running.
 *   6. Only once the swap is certain to succeed, call the OLD module's game_pre_reload().
 *   7. Publish the new table.
 *   8. Call the NEW module's game_post_reload().
 *   9. Free the old module and delete its copy.
 *
 * The linker creates its output file before filling it, so a poll can otherwise catch a
 * zero-length DLL. build.sh links to build/dev/game_tmp.tmp and renames, which makes the swap
 * atomic; the CopyFile step here also fails cleanly if the file is still locked by the
 * linker, and the modification time is deliberately NOT recorded in that case so the next
 * frame retries.
 */
#if defined(_WIN32) && defined(CIRCUIT_HOT_RELOAD)

#include "raylib.h"

#define WIN32_LEAN_AND_MEAN
#define NOGDI  /* wingdi.h declares Rectangle(), which collides with raylib's type */
#define NOUSER /* winuser.h declares CloseWindow/ShowCursor/DrawText/LoadImage */
#define NOMINMAX
#include <windows.h>
#undef near
#undef far

#include <stdio.h>
#include <string.h>

#include "platform/hotreload.h"

/* The platform-side table. One definition per entry point, generated from the one list. */
#define ENTRY(name, ...) name##_t *name = NULL;
GAME_ENTRY_POINTS
#undef ENTRY

typedef struct {
    HMODULE handle;
    char loadedPath[MAX_PATH];

#define ENTRY(name, ...) name##_t *name;
    GAME_ENTRY_POINTS
#undef ENTRY
} GameModule;

static GameModule g_active;
static long g_lastModTime = 0;
static long g_lastFailedModTime = 0; /* so a broken build logs once, not every frame */
static int g_loadCounter = 0;

#define MODULE_COPY_PREFIX "build/dev/game_load_"
#define MODULE_COPY_GLOB "build\\dev\\game_load_*.dll"

/* Delete leftovers from a previous run that ended without unloading cleanly. */
static void sweep_stale_copies(void)
{
    WIN32_FIND_DATAA find;
    const HANDLE search = FindFirstFileA(MODULE_COPY_GLOB, &find);
    if (search == INVALID_HANDLE_VALUE) return;

    do {
        /* FindFirstFileA reports the bare file name, so the directory is put back here. */
        char path[MAX_PATH + 16]; /* room for the "build/dev/" prefix and the terminator */
        snprintf(path, sizeof(path), "build/dev/%s", find.cFileName);
        DeleteFileA(path); /* still-locked copies simply survive; harmless */
    } while (FindNextFileA(search, &find));

    FindClose(search);
}

static void unload_module(GameModule *module)
{
    if (module == NULL) return;

    if (module->handle != NULL) {
        FreeLibrary(module->handle);
        module->handle = NULL;
    }
    if (module->loadedPath[0] != '\0') {
        DeleteFileA(module->loadedPath);
        module->loadedPath[0] = '\0';
    }
}

/* Load a candidate and resolve every entry point into it. Returns false without touching
 * anything the caller already has. logFailures is cleared while retrying a module that has
 * already been reported, so a broken build does not fill the log once per frame. */
static bool load_candidate(GameModule *out, bool logFailures)
{
    GameModule candidate;
    memset(&candidate, 0, sizeof(candidate));

    g_loadCounter++;
    snprintf(candidate.loadedPath, sizeof(candidate.loadedPath), MODULE_COPY_PREFIX "%d.dll",
             g_loadCounter);

    if (!CopyFileA(GAME_MODULE_NAME, candidate.loadedPath, FALSE)) {
        if (logFailures) {
            TRACELOG(LOG_WARNING, "HOTRELOAD: could not copy %s to %s (error %lu)",
                     GAME_MODULE_NAME, candidate.loadedPath, GetLastError());
        }
        return false;
    }

    candidate.handle = LoadLibraryA(candidate.loadedPath);
    if (candidate.handle == NULL) {
        if (logFailures) {
            TRACELOG(LOG_ERROR, "HOTRELOAD: could not load %s (error %lu)",
                     candidate.loadedPath, GetLastError());
        }
        DeleteFileA(candidate.loadedPath);
        return false;
    }

#define ENTRY(name, ...)                                                          \
    candidate.name = (name##_t *)(void *)GetProcAddress(candidate.handle, #name); \
    if (candidate.name == NULL) {                                                 \
        if (logFailures) {                                                        \
            TRACELOG(LOG_ERROR, "HOTRELOAD: missing symbol %s in %s", #name,      \
                     candidate.loadedPath);                                       \
        }                                                                         \
        unload_module(&candidate);                                                \
        return false;                                                             \
    }
    GAME_ENTRY_POINTS
#undef ENTRY

    *out = candidate;
    return true;
}

/* Copy a validated table into the globals the rest of the platform calls through. */
static void publish(const GameModule *module)
{
#define ENTRY(name, ...) name = module->name;
    GAME_ENTRY_POINTS
#undef ENTRY
}

bool Game_LoadModule(void)
{
    sweep_stale_copies();

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

    /* Validate the replacement BEFORE disturbing anything that currently works. */
    const bool firstAttempt = (modTime != g_lastFailedModTime);

    GameModule candidate;
    if (!load_candidate(&candidate, firstAttempt)) {
        /* g_lastModTime is intentionally left alone so the next frame retries. A compile
         * error, or a DLL the linker has not finished writing, keeps the game alive on the
         * module it already has. */
        if (firstAttempt) {
            g_lastFailedModTime = modTime;
            TRACELOG(LOG_WARNING,
                     "HOTRELOAD: candidate rejected; staying on the previous module");
        }
        return false;
    }

    g_lastModTime = modTime;

    /* The swap is now certain to succeed, so the old module may release what it owns. */
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
typedef int circuit_hotreload_windows_unused;

#endif /* _WIN32 && CIRCUIT_HOT_RELOAD */

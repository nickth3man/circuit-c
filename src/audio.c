/*
 * audio.c — sample-based audio system (Phase 6).
 *
 * All raylib audio calls are guarded by #if !defined(DRIFTY_HEADLESS) so the headless test
 * binary compiles this TU into no-op stubs and never touches the audio device.
 *
 * Hot-reload safety:
 *   - audio_init guards InitAudioDevice with a static bool — double-init is impossible.
 *   - audio_pre_reload only unloads sound assets; the audio device stays open.
 *   - audio_post_reload reloads the sound assets on the new module.
 *   - File-static storage avoids any Game layout change (no restart required).
 *
 * Graceful degradation:
 *   - Each sound asset is loaded only if the file exists AND LoadSound/LoadMusicStream
 *     returns a valid handle. Per-sound "loaded" flags gate all playback.
 *   - If the audio device fails to initialise, every function early-returns.
 *   - Missing WAV files = silent operation; no errors printed beyond the TRACELOG at init.
 */
#include "audio.h"

#include <math.h>
#include <stdbool.h>

#if !defined(DRIFTY_HEADLESS)

#include "raylib.h"
#include "config.h"   /* FIXED_DT_S */

/* ── file-static state (no Game layout change) ──────────────────────────── */

static bool   s_deviceReady   = false;
static bool   s_engineLoaded  = false;
static bool   s_screechLoaded = false;
static bool   s_thudLoaded    = false;
static Music  s_engineMusic   = {0};
static Music  s_screechMusic  = {0};
static Sound  s_thudSound     = {0};
static double s_lastThudTime  = -1.0;
static float  s_screechVolume = 0.0f;

/* ── helpers ────────────────────────────────────────────────────────────── */

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static Sound try_load_sound(const char *path, bool *ok)
{
    *ok = false;
    if (!FileExists(path)) return (Sound){0};
    Sound s = LoadSound(path);
    if (s.frameCount == 0) return (Sound){0};
    *ok = true;
    return s;
}

static Music try_load_music(const char *path, bool *ok)
{
    *ok = false;
    if (!FileExists(path)) return (Music){0};
    Music m = LoadMusicStream(path);
    if (m.frameCount == 0) return (Music){0};
    *ok = true;
    return m;
}

/* ── public API ─────────────────────────────────────────────────────────── */

void audio_init(void)
{
    if (s_deviceReady) return;  /* already initialised; survives hot reload */

    InitAudioDevice();
    if (!IsAudioDeviceReady()) return;
    s_deviceReady = true;

    s_engineMusic  = try_load_music("resources/audio/engine.wav",  &s_engineLoaded);
    s_screechMusic = try_load_music("resources/audio/screech.wav", &s_screechLoaded);
    s_thudSound    = try_load_sound("resources/audio/thud.wav",    &s_thudLoaded);

    if (s_engineLoaded) {
        s_engineMusic.looping = true;
        PlayMusicStream(s_engineMusic);
    }
    if (s_screechLoaded) {
        s_screechMusic.looping = true;
        PlayMusicStream(s_screechMusic);
    }

    TraceLog(LOG_INFO, "AUDIO: device ready. engine=%d screech=%d thud=%d",
             (int)s_engineLoaded, (int)s_screechLoaded, (int)s_thudLoaded);
}

void audio_shutdown(void)
{
    if (!s_deviceReady) return;

    if (s_engineLoaded)  { StopMusicStream(s_engineMusic);   UnloadMusicStream(s_engineMusic);   }
    if (s_screechLoaded) { StopMusicStream(s_screechMusic);  UnloadMusicStream(s_screechMusic);  }
    if (s_thudLoaded)    { UnloadSound(s_thudSound);          }

    s_engineLoaded  = false;
    s_screechLoaded = false;
    s_thudLoaded    = false;

    CloseAudioDevice();
    s_deviceReady = false;
}

void audio_pre_reload(void)
{
    /* Unload sound assets only — the audio device is process-global and stays open. */
    if (!s_deviceReady) return;

    if (s_engineLoaded)  { StopMusicStream(s_engineMusic);   UnloadMusicStream(s_engineMusic);   }
    if (s_screechLoaded) { StopMusicStream(s_screechMusic);  UnloadMusicStream(s_screechMusic);  }
    if (s_thudLoaded)    { UnloadSound(s_thudSound);          }

    s_engineLoaded  = false;
    s_screechLoaded = false;
    s_thudLoaded    = false;
    s_screechVolume = 0.0f;
    /* s_deviceReady remains true — device was never closed. */
}

void audio_post_reload(void)
{
    if (!s_deviceReady) return;  /* device was never ready */

    s_engineMusic  = try_load_music("resources/audio/engine.wav",  &s_engineLoaded);
    s_screechMusic = try_load_music("resources/audio/screech.wav", &s_screechLoaded);
    s_thudSound    = try_load_sound("resources/audio/thud.wav",    &s_thudLoaded);

    if (s_engineLoaded) {
        s_engineMusic.looping = true;
        PlayMusicStream(s_engineMusic);
    }
    if (s_screechLoaded) {
        s_screechMusic.looping = true;
        PlayMusicStream(s_screechMusic);
    }
}

void audio_update(float engineRpm, float idleRpm, float redlineRpm,
                  bool physicallySliding, float speedMps, float dt)
{
    (void)dt;  /* reserved; screech fade uses its own rate */
    if (!s_deviceReady) return;

    /* ── engine: streaming Music, pitch maps to RPM ── */
    if (s_engineLoaded) {
        UpdateMusicStream(s_engineMusic);

        float range = redlineRpm - idleRpm;
        float t = 0.0f;
        if (range > 1.0f) {
            t = clampf((engineRpm - idleRpm) / range, 0.0f, 1.0f);
        }
        float pitch = 0.5f + 1.5f * t;        /* idle→0.5, redline→2.0 */
        pitch = clampf(pitch, 0.25f, 4.0f);    /* hard guard */
        SetMusicPitch(s_engineMusic, pitch);
    }

    /* ── screech: looping Music, volume fades with slide intensity ── */
    if (s_screechLoaded) {
        UpdateMusicStream(s_screechMusic);

        float target = 0.0f;
        if (physicallySliding && speedMps > 0.5f) {
            target = clampf(speedMps / 30.0f, 0.0f, 1.0f);
        }

        /* Exponential moving average: crosses the full range in ~0.25 s. */
        float rate = 16.0f;  /* 1/s */
        float factor = 1.0f - expf(-rate * FIXED_DT_S);
        s_screechVolume = lerpf(s_screechVolume, target, factor);
        SetMusicVolume(s_screechMusic, s_screechVolume);
    }
}

void audio_play_collision_thud(void)
{
    if (!s_deviceReady || !s_thudLoaded) return;

    double now = GetTime();
    /* Rate-limit: at most one thud per ~0.1 s to avoid machine-gunning. */
    if (s_lastThudTime >= 0.0 && (now - s_lastThudTime) < 0.1) return;
    s_lastThudTime = now;
    PlaySound(s_thudSound);
}

#else  /* ── DRIFTY_HEADLESS: complete no-op stubs ──────────────────────── */

void audio_init(void)                {}
void audio_shutdown(void)            {}
void audio_pre_reload(void)          {}
void audio_post_reload(void)         {}
void audio_update(float engineRpm, float idleRpm, float redlineRpm,
                  bool physicallySliding, float speedMps, float dt)
{
    (void)engineRpm; (void)idleRpm; (void)redlineRpm;
    (void)physicallySliding; (void)speedMps; (void)dt;
}
void audio_play_collision_thud(void) {}

#endif /* !DRIFTY_HEADLESS */

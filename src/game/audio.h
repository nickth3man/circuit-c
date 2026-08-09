/*
 * audio.h — hot-reload-safe, sample-based audio system for Phase 6.
 *
 * File-static state in audio.c avoids modifying the Game layout, keeping the dev loop
 * restart-free for audio work. Every function is safe to call when the audio device is
 * unavailable or assets are missing — degradation is silent, never a crash.
 *
 * Headless builds (CIRCUIT_HEADLESS) compile the entire implementation into no-op stubs so
 * the test binary never touches the audio device.
 *
 * Asset convention (the developer drops these in to activate audio):
 *   resources/audio/engine.wav  — short looping engine tone (pitch maps to RPM)
 *   resources/audio/screech.wav — short looping tire screech (volume maps to slide)
 *   resources/audio/thud.wav    — short one-shot impact sound
 *
 * Without these files the game runs silently; no errors are printed.
 */
#ifndef CIRCUIT_AUDIO_H
#define CIRCUIT_AUDIO_H

#include <stdbool.h>

void audio_init(void);
void audio_shutdown(void);
void audio_pre_reload(void);
void audio_post_reload(void);
void audio_update(float engineRpm, float idleRpm, float redlineRpm, bool physicallySliding,
                  float speedMps, float dt);
void audio_play_collision_thud(void);

#endif /* CIRCUIT_AUDIO_H */

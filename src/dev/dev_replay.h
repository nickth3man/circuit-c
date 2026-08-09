/*
 * dev_replay.h — on-disk replay timelines and the data behind the replay inspector.
 *
 * The in-memory ReplayBuffer (src/game/replay.h) is already the deterministic record of an input
 * timeline. This file makes it durable and inspectable:
 *
 *   - dev_replay_save / dev_replay_load move a timeline through a versioned `.bin` file,
 *   - dev_replay_parse is the same reader over a memory buffer, so fuzz/fuzz_replay.c can
 *     hammer it without touching the filesystem,
 *   - dev_replay_collect_events turns the timeline into the markers the inspector's
 *     scrubber draws (throttle, brake, handbrake, shifts, resets, steering reversals).
 *
 * Field-by-field I/O, never a struct blob: no padding assumptions, and a file written by one
 * compiler reads back in another. Values are host-endian floats, which is honest for a
 * Windows-only project and is recorded in the header so a future reader can tell.
 *
 * raylib-free: linked into the headless test executable as well as the game module.
 */
#ifndef CIRCUIT_DEV_REPLAY_H
#define CIRCUIT_DEV_REPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "game/replay.h"

#define DEV_REPLAY_MAGIC 0x4C505244u /* 'DRPL' little-endian */
#define DEV_REPLAY_VERSION 1u
#define DEV_REPLAY_LABEL_CHARS 32

/* Everything about a timeline except the frames themselves. */
typedef struct {
    uint32_t version;
    uint32_t fixedHz;   /* ticks per second the timeline was recorded at */
    uint64_t firstTick; /* absolute fixed tick of frame 0 */
    uint32_t frameCount;
    uint32_t seed;          /* scenario seed, 0 when not seeded */
    uint32_t finalChecksum; /* game_state_checksum at the end of the recording */
    char label[DEV_REPLAY_LABEL_CHARS];
} DevReplayInfo;

typedef enum {
    DEV_REPLAY_EVENT_THROTTLE = 0, /* throttle crossed the half-open threshold */
    DEV_REPLAY_EVENT_BRAKE,
    DEV_REPLAY_EVENT_HANDBRAKE,
    DEV_REPLAY_EVENT_SHIFT_UP,
    DEV_REPLAY_EVENT_SHIFT_DOWN,
    DEV_REPLAY_EVENT_RESET,
    DEV_REPLAY_EVENT_STEER_REVERSAL,
    DEV_REPLAY_EVENT_COUNT
} DevReplayEventKind;

typedef struct {
    uint64_t tick; /* absolute fixed tick */
    int index;     /* frame index within the retained window */
    DevReplayEventKind kind;
    float value; /* the new value that triggered the marker */
} DevReplayEvent;

/* Write the retained window of rb to path. label may be NULL. */
bool dev_replay_save(const ReplayBuffer *rb, const char *path, const char *label, uint32_t seed,
                     uint32_t finalChecksum);

/* Read a timeline. On success rb holds the frames in REPLAY_MODE_IDLE, ready for
 * replay_begin_playback(). info may be NULL. rb is untouched on failure. */
bool dev_replay_load(ReplayBuffer *rb, const char *path, DevReplayInfo *info);

/* The same reader over memory. Returns false for anything malformed — short buffers,
 * bad magic, unsupported version, implausible counts. Never reads past `length`. */
bool dev_replay_parse(const void *data, size_t length, ReplayBuffer *rb, DevReplayInfo *info);

/* Markers derived from the input timeline, oldest first. Returns the number written, which
 * is at most capacity. */
int dev_replay_collect_events(const ReplayBuffer *rb, DevReplayEvent *out, int capacity);

/* Retained frame at window index i (0 = oldest), or NULL when out of range. */
const ReplayFrame *dev_replay_frame_at(const ReplayBuffer *rb, int index);

#endif /* CIRCUIT_DEV_REPLAY_H */

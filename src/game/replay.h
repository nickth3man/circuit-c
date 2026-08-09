/*
 * replay.h — deterministic fixed-tick input timeline.
 *
 * The timeline is indexed by fixed tick rather than by wall-clock timestamp: every entry is
 * exactly one 1/FIXED_HZ step, so the wall-clock time of entry i is
 * (firstTick + i) * FIXED_DT_S. That removes any need to reconcile a recorded timestamp
 * with the accumulator, which is what makes replay bit-reproducible.
 *
 * Storage is a fixed-capacity ring embedded in Game. No allocation happens at any point,
 * so nothing is allocated during the game loop.
 *
 * OVERFLOW BEHAVIOUR IS A DOCUMENTED RING. Once REPLAY_CAPACITY_TICKS entries are held,
 * recording another tick discards the oldest one, advances firstTick, and increments
 * overwrittenTicks. This is intentional: the buffer's purpose is to replay the most recent
 * N seconds, so the recent window is the part worth keeping. Playback always walks the
 * retained window oldest-first.
 *
 * RECOVERING STATE AFTER A RESTART.
 *
 * Struct layout changes, platform-layer edits, and crashes all require a restart. The
 * timestamped input recording required by Phase 0 doubles as the recovery mechanism: replay
 * the recorded input timeline at maximum speed on startup to arrive back at the moment of
 * interest in a fraction of a second. This is worth wiring to a key (replay last N seconds)
 * early, since it makes restarts cheap enough that hot reload becomes a convenience rather
 * than a necessity.
 *
 * This header and its implementation are raylib-free and are linked by the headless
 * test executable.
 */
#ifndef CIRCUIT_REPLAY_H
#define CIRCUIT_REPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "core/config.h"
#include "game/input.h"

/* Packed one-shot commands, one bit each. */
#define REPLAY_BIT_PAUSE 0x01u
#define REPLAY_BIT_RESET 0x02u
#define REPLAY_BIT_DEBUG 0x04u
#define REPLAY_BIT_SHIFT_UP 0x08u
#define REPLAY_BIT_SHIFT_DOWN 0x10u

typedef struct {
    float steer;         /* -1 .. +1, left positive */
    float throttle;      /* 0 .. 1 */
    float brake;         /* 0 .. 1 */
    float handbrake;     /* 0 .. 1 */
    uint8_t oneshotBits; /* REPLAY_BIT_* */
} ReplayFrame;

typedef enum { REPLAY_MODE_IDLE = 0, REPLAY_MODE_RECORDING, REPLAY_MODE_PLAYBACK } ReplayMode;

typedef struct {
    ReplayFrame frames[REPLAY_CAPACITY_TICKS];
    int head;                  /* ring index of the oldest retained frame */
    int count;                 /* retained frames, 0 .. REPLAY_CAPACITY_TICKS */
    int playbackCursor;        /* 0 .. count, offset from head */
    uint64_t firstTick;        /* absolute fixed tick of the oldest retained frame */
    uint64_t overwrittenTicks; /* frames discarded by the ring, cumulative */
    ReplayMode mode;
} ReplayBuffer;

/* Discard everything and return to REPLAY_MODE_IDLE. */
void replay_reset(ReplayBuffer *rb);

/* Discard everything and begin recording. startTick is the absolute fixed tick that the
 * first recorded frame corresponds to. */
void replay_begin_recording(ReplayBuffer *rb, uint64_t startTick);

/* Append one fixed tick of input. No-op unless the buffer is recording. */
void replay_record(ReplayBuffer *rb, const Input *in);

/* Rewind to the oldest retained frame and begin playback. Returns false (and stays IDLE)
 * when there is nothing recorded. Recording stops; the retained frames are preserved, so
 * the same timeline can be replayed repeatedly. */
bool replay_begin_playback(ReplayBuffer *rb);

/* Pop the next recorded tick into *out. Returns false when the timeline is exhausted or
 * the buffer is not in playback, leaving *out untouched. */
bool replay_next(ReplayBuffer *rb, Input *out);

/* Leave playback or recording without discarding the retained frames. */
void replay_stop(ReplayBuffer *rb);

/* Wall-clock time in seconds of retained frame index i (0 = oldest). */
double replay_frame_time_s(const ReplayBuffer *rb, int index);

/* Conversions between an Input and its packed timeline representation. */
ReplayFrame replay_pack(const Input *in);
void replay_unpack(const ReplayFrame *frame, Input *out);

#endif /* CIRCUIT_REPLAY_H */

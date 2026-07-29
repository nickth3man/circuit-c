#include "game/replay.h"

#include <string.h>

ReplayFrame replay_pack(const Input *in)
{
    ReplayFrame frame;
    memset(&frame, 0, sizeof(frame));
    if (in == NULL) return frame;

    frame.steer     = in->steer;
    frame.throttle  = in->throttle;
    frame.brake     = in->brake;
    frame.handbrake = in->handbrake;

    uint8_t bits = 0;
    if (in->pausePressed)     bits = (uint8_t)(bits | REPLAY_BIT_PAUSE);
    if (in->resetPressed)     bits = (uint8_t)(bits | REPLAY_BIT_RESET);
    if (in->debugPressed)     bits = (uint8_t)(bits | REPLAY_BIT_DEBUG);
    if (in->shiftUpPressed)   bits = (uint8_t)(bits | REPLAY_BIT_SHIFT_UP);
    if (in->shiftDownPressed) bits = (uint8_t)(bits | REPLAY_BIT_SHIFT_DOWN);
    frame.oneshotBits = bits;

    return frame;
}

void replay_unpack(const ReplayFrame *frame, Input *out)
{
    if (out == NULL) return;
    input_zero(out);
    if (frame == NULL) return;

    out->steer     = frame->steer;
    out->throttle  = frame->throttle;
    out->brake     = frame->brake;
    out->handbrake = frame->handbrake;

    out->pausePressed     = (frame->oneshotBits & REPLAY_BIT_PAUSE)      != 0;
    out->resetPressed     = (frame->oneshotBits & REPLAY_BIT_RESET)      != 0;
    out->debugPressed     = (frame->oneshotBits & REPLAY_BIT_DEBUG)      != 0;
    out->shiftUpPressed   = (frame->oneshotBits & REPLAY_BIT_SHIFT_UP)   != 0;
    out->shiftDownPressed = (frame->oneshotBits & REPLAY_BIT_SHIFT_DOWN) != 0;
}

void replay_reset(ReplayBuffer *rb)
{
    if (rb == NULL) return;
    rb->head             = 0;
    rb->count            = 0;
    rb->playbackCursor   = 0;
    rb->firstTick        = 0;
    rb->overwrittenTicks = 0;
    rb->mode             = REPLAY_MODE_IDLE;
    /* frames[] is left as-is: entries outside [head, head+count) are never read. */
}

void replay_begin_recording(ReplayBuffer *rb, uint64_t startTick)
{
    if (rb == NULL) return;
    replay_reset(rb);
    rb->firstTick = startTick;
    rb->mode      = REPLAY_MODE_RECORDING;
}

void replay_record(ReplayBuffer *rb, const Input *in)
{
    if (rb == NULL || rb->mode != REPLAY_MODE_RECORDING) return;

    int slot;
    if (rb->count < REPLAY_CAPACITY_TICKS) {
        slot = (rb->head + rb->count) % REPLAY_CAPACITY_TICKS;
        rb->count++;
    } else {
        /* Ring is full: drop the oldest tick. */
        slot      = rb->head;
        rb->head  = (rb->head + 1) % REPLAY_CAPACITY_TICKS;
        rb->firstTick++;
        rb->overwrittenTicks++;
    }

    rb->frames[slot] = replay_pack(in);
}

bool replay_begin_playback(ReplayBuffer *rb)
{
    if (rb == NULL || rb->count <= 0) return false;
    rb->playbackCursor = 0;
    rb->mode           = REPLAY_MODE_PLAYBACK;
    return true;
}

bool replay_next(ReplayBuffer *rb, Input *out)
{
    if (rb == NULL || out == NULL) return false;
    if (rb->mode != REPLAY_MODE_PLAYBACK) return false;
    if (rb->playbackCursor >= rb->count) return false;

    const int slot = (rb->head + rb->playbackCursor) % REPLAY_CAPACITY_TICKS;
    replay_unpack(&rb->frames[slot], out);
    rb->playbackCursor++;
    return true;
}

void replay_stop(ReplayBuffer *rb)
{
    if (rb == NULL) return;
    rb->mode = REPLAY_MODE_IDLE;
}

int replay_remaining(const ReplayBuffer *rb)
{
    if (rb == NULL || rb->mode != REPLAY_MODE_PLAYBACK) return 0;
    return rb->count - rb->playbackCursor;
}

double replay_frame_time_s(const ReplayBuffer *rb, int index)
{
    if (rb == NULL || index < 0 || index >= rb->count) return 0.0;
    return (double)(rb->firstTick + (uint64_t)index) * (double)FIXED_DT_S;
}

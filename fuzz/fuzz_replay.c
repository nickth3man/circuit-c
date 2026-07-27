/*
 * fuzz_replay.c — the replay timeline reader.
 *
 * Replay files travel: they come out of failure bundles, get attached to issues, and are
 * loaded by a build that did not write them. dev_replay_parse() must therefore treat every
 * byte as hostile. The properties:
 *
 *   - no input reads past the buffer or crashes;
 *   - an accepted timeline has a frame count within capacity and finite, in-range inputs;
 *   - a rejected timeline leaves nothing half-loaded that a caller could then play back.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev_replay.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

static void check_frame_range(float value, float low, float high)
{
    if (!isfinite(value) || value < low || value > high) abort();
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    static ReplayBuffer buffer;
    DevReplayInfo info;

    memset(&info, 0, sizeof(info));
    replay_reset(&buffer);

    if (!dev_replay_parse(data, size, &buffer, &info)) {
        return 0;
    }

    if (buffer.count < 0 || buffer.count > REPLAY_CAPACITY_TICKS) abort();
    if ((uint32_t)buffer.count != info.frameCount) abort();
    if (buffer.mode != REPLAY_MODE_IDLE) abort();

    for (int i = 0; i < buffer.count; i++) {
        const ReplayFrame *frame = dev_replay_frame_at(&buffer, i);
        if (frame == NULL) abort();
        check_frame_range(frame->steer, -1.0f, 1.0f);
        check_frame_range(frame->throttle, 0.0f, 1.0f);
        check_frame_range(frame->brake, 0.0f, 1.0f);
        check_frame_range(frame->handbrake, 0.0f, 1.0f);
    }

    /* An accepted timeline must be playable without further validation. */
    static DevReplayEvent events[64];
    (void)dev_replay_collect_events(&buffer, events, 64);

    if (replay_begin_playback(&buffer)) {
        Input in;
        while (replay_next(&buffer, &in)) {
            check_frame_range(in.steer, -1.0f, 1.0f);
        }
    }
    return 0;
}

#if !defined(DRIFTY_LIBFUZZER)
int main(int argc, char **argv)
{
    static unsigned char bytes[1 << 20];

    for (int i = 1; i < argc; i++) {
        FILE *file = fopen(argv[i], "rb");
        if (file == NULL) {
            fprintf(stderr, "could not open %s\n", argv[i]);
            return 1;
        }
        const size_t size = fread(bytes, 1, sizeof(bytes), file);
        fclose(file);
        LLVMFuzzerTestOneInput(bytes, size);
        printf("ok %s (%zu bytes)\n", argv[i], size);
    }

    if (argc == 1) {
        memset(bytes, 0, 128);
        LLVMFuzzerTestOneInput(bytes, 0);
        LLVMFuzzerTestOneInput(bytes, 16);
        LLVMFuzzerTestOneInput(bytes, 128);
        memset(bytes, 0xFF, 128);
        LLVMFuzzerTestOneInput(bytes, 128);
        printf("ok 4 built-in samples\n");
    }
    return 0;
}
#endif

#include "dev/dev_replay.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "core/config.h"

/*
 * File layout (all offsets in bytes, host endianness, IEEE-754 binary32 floats):
 *
 *   0   u32   magic 'DRPL'
 *   4   u32   version
 *   8   u32   fixedHz
 *   12  u32   frameCount
 *   16  u64   firstTick
 *   24  u32   seed
 *   28  u32   finalChecksum
 *   32  char  label[32]
 *   64        frameCount records of 20 bytes: f32 steer, throttle, brake, handbrake,
 *             u8 oneshotBits, 3 bytes of zero padding.
 */
#define DEV_REPLAY_HEADER_BYTES 64
#define DEV_REPLAY_FRAME_BYTES 20

const ReplayFrame *dev_replay_frame_at(const ReplayBuffer *rb, int index)
{
    if (rb == NULL || index < 0 || index >= rb->count) return NULL;
    const int slot = (rb->head + index) % REPLAY_CAPACITY_TICKS;
    return &rb->frames[slot];
}

/* ---------------------------------------------------------------------------- writing -- */

static bool write_u32(FILE *file, uint32_t value)
{
    return fwrite(&value, sizeof(value), 1, file) == 1;
}

static bool write_u64(FILE *file, uint64_t value)
{
    return fwrite(&value, sizeof(value), 1, file) == 1;
}

static bool write_f32(FILE *file, float value)
{
    return fwrite(&value, sizeof(value), 1, file) == 1;
}

bool dev_replay_save(const ReplayBuffer *rb, const char *path, const char *label, uint32_t seed,
                     uint32_t finalChecksum)
{
    if (rb == NULL || path == NULL) return false;

    FILE *file = fopen(path, "wb");
    if (file == NULL) return false;

    char labelBytes[DEV_REPLAY_LABEL_CHARS];
    memset(labelBytes, 0, sizeof(labelBytes));
    if (label != NULL) {
        strncpy(labelBytes, label, sizeof(labelBytes) - 1u);
    }

    bool ok = true;
    ok = ok && write_u32(file, DEV_REPLAY_MAGIC);
    ok = ok && write_u32(file, DEV_REPLAY_VERSION);
    ok = ok && write_u32(file, (uint32_t)FIXED_HZ);
    ok = ok && write_u32(file, (uint32_t)rb->count);
    ok = ok && write_u64(file, rb->firstTick);
    ok = ok && write_u32(file, seed);
    ok = ok && write_u32(file, finalChecksum);
    ok = ok && (fwrite(labelBytes, sizeof(labelBytes), 1, file) == 1);

    static const unsigned char padding[3] = { 0, 0, 0 };
    for (int i = 0; ok && i < rb->count; i++) {
        const ReplayFrame *frame = dev_replay_frame_at(rb, i);
        ok = ok && write_f32(file, frame->steer);
        ok = ok && write_f32(file, frame->throttle);
        ok = ok && write_f32(file, frame->brake);
        ok = ok && write_f32(file, frame->handbrake);
        ok = ok && (fwrite(&frame->oneshotBits, 1, 1, file) == 1);
        ok = ok && (fwrite(padding, sizeof(padding), 1, file) == 1);
    }

    ok = ok && !ferror(file);
    return (fclose(file) == 0) && ok;
}

/* ---------------------------------------------------------------------------- reading -- */

typedef struct {
    const unsigned char *data;
    size_t length;
    size_t cursor;
    bool failed;
} Reader;

static void read_bytes(Reader *reader, void *out, size_t count)
{
    if (reader->failed || reader->cursor + count > reader->length) {
        reader->failed = true;
        return;
    }
    memcpy(out, reader->data + reader->cursor, count);
    reader->cursor += count;
}

static uint32_t read_u32(Reader *reader)
{
    uint32_t value = 0;
    read_bytes(reader, &value, sizeof(value));
    return value;
}

static uint64_t read_u64(Reader *reader)
{
    uint64_t value = 0;
    read_bytes(reader, &value, sizeof(value));
    return value;
}

static float read_f32(Reader *reader)
{
    float value = 0.0f;
    read_bytes(reader, &value, sizeof(value));
    return value;
}

static float sanitize(float value, float low, float high)
{
    if (!isfinite(value)) return 0.0f;
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

bool dev_replay_parse(const void *data, size_t length, ReplayBuffer *rb, DevReplayInfo *info)
{
    if (data == NULL || rb == NULL) return false;
    if (length < DEV_REPLAY_HEADER_BYTES) return false;

    Reader reader = { (const unsigned char *)data, length, 0, false };

    const uint32_t magic = read_u32(&reader);
    if (reader.failed || magic != DEV_REPLAY_MAGIC) return false;

    DevReplayInfo header;
    memset(&header, 0, sizeof(header));
    header.version = read_u32(&reader);
    header.fixedHz = read_u32(&reader);
    header.frameCount = read_u32(&reader);
    header.firstTick = read_u64(&reader);
    header.seed = read_u32(&reader);
    header.finalChecksum = read_u32(&reader);
    read_bytes(&reader, header.label, DEV_REPLAY_LABEL_CHARS);
    if (reader.failed) return false;

    header.label[DEV_REPLAY_LABEL_CHARS - 1] = '\0';
    if (header.version != DEV_REPLAY_VERSION) return false;
    if (header.fixedHz == 0u || header.fixedHz > 100000u) return false;
    if (header.frameCount > (uint32_t)REPLAY_CAPACITY_TICKS) return false;

    /* Every declared frame must actually be present; a truncated file is a bad file. */
    const size_t needed = (size_t)DEV_REPLAY_HEADER_BYTES +
                          (size_t)header.frameCount * (size_t)DEV_REPLAY_FRAME_BYTES;
    if (length < needed) return false;

    /* Build into a local buffer so a failure halfway through changes nothing. */
    replay_reset(rb);
    for (uint32_t i = 0; i < header.frameCount; i++) {
        ReplayFrame frame;
        memset(&frame, 0, sizeof(frame));
        frame.steer = sanitize(read_f32(&reader), -1.0f, 1.0f);
        frame.throttle = sanitize(read_f32(&reader), 0.0f, 1.0f);
        frame.brake = sanitize(read_f32(&reader), 0.0f, 1.0f);
        frame.handbrake = sanitize(read_f32(&reader), 0.0f, 1.0f);
        read_bytes(&reader, &frame.oneshotBits, 1);
        reader.cursor += 3u; /* padding; bounds already guaranteed by `needed` */
        if (reader.failed) return false;
        rb->frames[i] = frame;
    }

    rb->head = 0;
    rb->count = (int)header.frameCount;
    rb->playbackCursor = 0;
    rb->firstTick = header.firstTick;
    rb->overwrittenTicks = 0;
    rb->mode = REPLAY_MODE_IDLE;

    if (info != NULL) *info = header;
    return true;
}

bool dev_replay_load(ReplayBuffer *rb, const char *path, DevReplayInfo *info)
{
    if (rb == NULL || path == NULL) return false;

    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;

    static const size_t maxBytes =
        (size_t)DEV_REPLAY_HEADER_BYTES +
        (size_t)REPLAY_CAPACITY_TICKS * (size_t)DEV_REPLAY_FRAME_BYTES;

    /* The format has a hard upper bound, so the whole file fits in one fixed buffer and
     * nothing is allocated. It is module-static rather than stack-resident only to keep the
     * frame small; it is never referenced from Game, so hot reload is unaffected. */
    static unsigned char
        buffer[DEV_REPLAY_HEADER_BYTES + REPLAY_CAPACITY_TICKS * DEV_REPLAY_FRAME_BYTES];
    const size_t read = fread(buffer, 1, maxBytes, file);
    const bool tooLarge = (fgetc(file) != EOF);
    fclose(file);

    if (tooLarge) return false;
    return dev_replay_parse(buffer, read, rb, info);
}

/* ----------------------------------------------------------------------------- events -- */

static bool crossed(float previous, float current, float threshold)
{
    return (previous < threshold) != (current < threshold);
}

int dev_replay_collect_events(const ReplayBuffer *rb, DevReplayEvent *out, int capacity)
{
    if (rb == NULL || out == NULL || capacity <= 0) return 0;

    int written = 0;
    ReplayFrame previous;
    memset(&previous, 0, sizeof(previous));

    for (int i = 0; i < rb->count && written < capacity; i++) {
        const ReplayFrame *frame = dev_replay_frame_at(rb, i);
        const uint64_t tick = rb->firstTick + (uint64_t)i;

        struct {
            bool fired;
            DevReplayEventKind kind;
            float value;
        } candidates[] = {
            { crossed(previous.throttle, frame->throttle, 0.5f), DEV_REPLAY_EVENT_THROTTLE,
              frame->throttle },
            { crossed(previous.brake, frame->brake, 0.5f), DEV_REPLAY_EVENT_BRAKE,
              frame->brake },
            { crossed(previous.handbrake, frame->handbrake, 0.5f), DEV_REPLAY_EVENT_HANDBRAKE,
              frame->handbrake },
            { (frame->oneshotBits & REPLAY_BIT_SHIFT_UP) != 0u, DEV_REPLAY_EVENT_SHIFT_UP,
              0.0f },
            { (frame->oneshotBits & REPLAY_BIT_SHIFT_DOWN) != 0u, DEV_REPLAY_EVENT_SHIFT_DOWN,
              0.0f },
            { (frame->oneshotBits & REPLAY_BIT_RESET) != 0u, DEV_REPLAY_EVENT_RESET, 0.0f },
            /* A reversal is a sign change with real intent on both sides, not the noise
             * around centre that a controller produces while holding straight. */
            { (previous.steer > 0.2f && frame->steer < -0.2f) ||
                  (previous.steer < -0.2f && frame->steer > 0.2f),
              DEV_REPLAY_EVENT_STEER_REVERSAL, frame->steer },
        };

        for (size_t c = 0; c < sizeof(candidates) / sizeof(candidates[0]); c++) {
            if (!candidates[c].fired || written >= capacity) continue;
            out[written].tick = tick;
            out[written].index = i;
            out[written].kind = candidates[c].kind;
            out[written].value = candidates[c].value;
            written++;
        }

        previous = *frame;
    }

    return written;
}

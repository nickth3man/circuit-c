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
 *   64        field-by-field ReplayVehicleSnapshot: u32 valid, followed when valid by
 *             definition identity, setup, and authoritative mutable vehicle state.
 *   ...       frameCount records of 20 bytes: f32 steer, throttle, brake, handbrake,
 *             u8 oneshotBits, 3 bytes of zero padding.
 */
#define DEV_REPLAY_HEADER_BYTES 64
#define DEV_REPLAY_FRAME_BYTES 20
#define DEV_REPLAY_SNAPSHOT_MAX_BYTES 4096

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

static bool write_i32(FILE *file, int32_t value)
{
    return fwrite(&value, sizeof(value), 1, file) == 1;
}

static bool write_bytes(FILE *file, const void *data, size_t count)
{
    return fwrite(data, 1, count, file) == count;
}

static bool write_vehicle_snapshot(FILE *file, const ReplayVehicleSnapshot *snapshot)
{
    bool ok = write_u32(file, snapshot->valid ? 1u : 0u);
    if (!ok || !snapshot->valid) return ok;

#define WRITE_F32(value) ok = ok && write_f32(file, (value))
#define WRITE_U32(value) ok = ok && write_u32(file, (uint32_t)(value))
#define WRITE_I32(value) ok = ok && write_i32(file, (int32_t)(value))

    ok = write_bytes(file, snapshot->definitionId, sizeof(snapshot->definitionId));
    WRITE_U32(snapshot->definitionVersion);
    WRITE_U32(snapshot->definitionHash);

    const VehicleSetup *setup = &snapshot->setup;
    WRITE_F32(setup->tirePressureFrontKpa);
    WRITE_F32(setup->tirePressureRearKpa);
    WRITE_F32(setup->suspCamberFrontRad);
    WRITE_F32(setup->suspCamberRearRad);
    WRITE_F32(setup->suspToeFrontRad);
    WRITE_F32(setup->suspToeRearRad);
    WRITE_F32(setup->suspCasterFrontRad);
    WRITE_F32(setup->suspCasterRearRad);
    for (int i = 0; i < MAX_GEARS; i++) WRITE_F32(setup->gearRatios[i]);
    WRITE_I32(setup->gearCount);
    WRITE_F32(setup->reverseGearRatio);
    WRITE_F32(setup->finalDriveRatio);
    WRITE_F32(setup->brakeBiasFront);
    WRITE_F32(setup->differentialMode);
    WRITE_F32(setup->differentialBiasRatio);
    WRITE_F32(setup->differentialPreloadNm);

    const VehicleState *vehicle = &snapshot->vehicle;
    WRITE_F32(vehicle->positionM.x);
    WRITE_F32(vehicle->positionM.y);
    WRITE_F32(vehicle->headingRad);
    WRITE_F32(vehicle->velocityLongitudinalMps);
    WRITE_F32(vehicle->velocityLateralMps);
    WRITE_F32(vehicle->yawRateRadS);
    WRITE_F32(vehicle->frontRoadWheelAngleRad);
    WRITE_F32(vehicle->engineRpm);
    WRITE_I32(vehicle->selectedGear);
    WRITE_F32(vehicle->filteredLongAccelMps2);
    WRITE_F32(vehicle->prevLongAccelMps2);
    WRITE_F32(vehicle->filteredLatAccelMps2);
    WRITE_F32(vehicle->prevLatAccelMps2);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &vehicle->wheels[i];
        WRITE_F32(wheel->localPositionM.x);
        WRITE_F32(wheel->localPositionM.y);
        WRITE_F32(wheel->steerAngleRad);
        WRITE_F32(wheel->angularVelocityRadS);
        WRITE_F32(wheel->normalLoadN);
        WRITE_F32(wheel->slipAngleRad);
        WRITE_F32(wheel->slipRatio);
        WRITE_F32(wheel->forceLongitudinalN);
        WRITE_F32(wheel->forceLateralN);
        WRITE_F32(wheel->forceLateralRelaxedN);
        WRITE_F32(wheel->frictionUsage);
        WRITE_U32(wheel->locked ? 1u : 0u);
        WRITE_I32(wheel->surfaceId);
    }

    const VehicleRenderState *render = &snapshot->renderState;
    WRITE_F32(render->prevPositionM.x);
    WRITE_F32(render->prevPositionM.y);
    WRITE_F32(render->prevHeadingRad);
    for (int i = 0; i < WHEEL_COUNT; i++) WRITE_F32(render->prevWheelAngleRad[i]);
    WRITE_F32(render->currPositionM.x);
    WRITE_F32(render->currPositionM.y);
    WRITE_F32(render->currHeadingRad);
    for (int i = 0; i < WHEEL_COUNT; i++) WRITE_F32(render->currWheelAngleRad[i]);

    WRITE_U32(snapshot->autoTrans.enabled ? 1u : 0u);
    WRITE_U32(snapshot->autoTrans.forwardOnly ? 1u : 0u);
    WRITE_I32(snapshot->autoTrans.driveState);
    WRITE_F32(snapshot->autoTrans.neutralTimer);
    WRITE_F32(snapshot->vehicleControls.steer);
    WRITE_F32(snapshot->vehicleControls.throttle);
    WRITE_F32(snapshot->vehicleControls.brake);
    WRITE_F32(snapshot->vehicleControls.handbrake);
    WRITE_F32(snapshot->fuelKg);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        WRITE_F32(snapshot->tireState[i].pressureKpa);
        WRITE_F32(snapshot->tireState[i].temperatureC);
        WRITE_F32(snapshot->tireState[i].wear);
    }
    WRITE_F32(snapshot->damage);
    WRITE_F32(snapshot->crashLockoutTimerS);

#undef WRITE_F32
#undef WRITE_U32
#undef WRITE_I32
    return ok;
}

bool dev_replay_save(const ReplayBuffer *rb, const char *path, const char *label, uint32_t seed,
                     uint32_t finalChecksum)
{
    if (rb == NULL || path == NULL) return false;
    if (rb->initialVehicle.valid && rb->overwrittenTicks > 0u) return false;

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
    ok = ok && write_vehicle_snapshot(file, &rb->initialVehicle);

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

static float read_finite_f32(Reader *reader)
{
    float value = read_f32(reader);
    if (!isfinite(value)) reader->failed = true;
    return value;
}

static int32_t read_i32(Reader *reader)
{
    int32_t value = 0;
    read_bytes(reader, &value, sizeof(value));
    return value;
}

static bool read_bool(Reader *reader)
{
    const uint32_t value = read_u32(reader);
    if (value > 1u) reader->failed = true;
    return value != 0u;
}

static void read_vehicle_snapshot(Reader *reader, ReplayVehicleSnapshot *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    const bool valid = read_bool(reader);
    if (reader->failed || !valid) return;

#define READ_F32(target) (target) = read_finite_f32(reader)
#define READ_U32(target) (target) = read_u32(reader)
#define READ_I32(target) (target) = read_i32(reader)

    read_bytes(reader, snapshot->definitionId, sizeof(snapshot->definitionId));
    snapshot->definitionId[sizeof(snapshot->definitionId) - 1u] = '\0';
    READ_U32(snapshot->definitionVersion);
    READ_U32(snapshot->definitionHash);

    VehicleSetup *setup = &snapshot->setup;
    READ_F32(setup->tirePressureFrontKpa);
    READ_F32(setup->tirePressureRearKpa);
    READ_F32(setup->suspCamberFrontRad);
    READ_F32(setup->suspCamberRearRad);
    READ_F32(setup->suspToeFrontRad);
    READ_F32(setup->suspToeRearRad);
    READ_F32(setup->suspCasterFrontRad);
    READ_F32(setup->suspCasterRearRad);
    for (int i = 0; i < MAX_GEARS; i++) READ_F32(setup->gearRatios[i]);
    READ_I32(setup->gearCount);
    READ_F32(setup->reverseGearRatio);
    READ_F32(setup->finalDriveRatio);
    READ_F32(setup->brakeBiasFront);
    READ_F32(setup->differentialMode);
    READ_F32(setup->differentialBiasRatio);
    READ_F32(setup->differentialPreloadNm);

    VehicleState *vehicle = &snapshot->vehicle;
    READ_F32(vehicle->positionM.x);
    READ_F32(vehicle->positionM.y);
    READ_F32(vehicle->headingRad);
    READ_F32(vehicle->velocityLongitudinalMps);
    READ_F32(vehicle->velocityLateralMps);
    READ_F32(vehicle->yawRateRadS);
    READ_F32(vehicle->frontRoadWheelAngleRad);
    READ_F32(vehicle->engineRpm);
    READ_I32(vehicle->selectedGear);
    READ_F32(vehicle->filteredLongAccelMps2);
    READ_F32(vehicle->prevLongAccelMps2);
    READ_F32(vehicle->filteredLatAccelMps2);
    READ_F32(vehicle->prevLatAccelMps2);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        WheelState *wheel = &vehicle->wheels[i];
        READ_F32(wheel->localPositionM.x);
        READ_F32(wheel->localPositionM.y);
        READ_F32(wheel->steerAngleRad);
        READ_F32(wheel->angularVelocityRadS);
        READ_F32(wheel->normalLoadN);
        READ_F32(wheel->slipAngleRad);
        READ_F32(wheel->slipRatio);
        READ_F32(wheel->forceLongitudinalN);
        READ_F32(wheel->forceLateralN);
        READ_F32(wheel->forceLateralRelaxedN);
        READ_F32(wheel->frictionUsage);
        wheel->locked = read_bool(reader);
        READ_I32(wheel->surfaceId);
    }

    VehicleRenderState *render = &snapshot->renderState;
    READ_F32(render->prevPositionM.x);
    READ_F32(render->prevPositionM.y);
    READ_F32(render->prevHeadingRad);
    for (int i = 0; i < WHEEL_COUNT; i++) READ_F32(render->prevWheelAngleRad[i]);
    READ_F32(render->currPositionM.x);
    READ_F32(render->currPositionM.y);
    READ_F32(render->currHeadingRad);
    for (int i = 0; i < WHEEL_COUNT; i++) READ_F32(render->currWheelAngleRad[i]);

    snapshot->autoTrans.enabled = read_bool(reader);
    snapshot->autoTrans.forwardOnly = read_bool(reader);
    READ_I32(snapshot->autoTrans.driveState);
    READ_F32(snapshot->autoTrans.neutralTimer);
    READ_F32(snapshot->vehicleControls.steer);
    READ_F32(snapshot->vehicleControls.throttle);
    READ_F32(snapshot->vehicleControls.brake);
    READ_F32(snapshot->vehicleControls.handbrake);
    READ_F32(snapshot->fuelKg);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        READ_F32(snapshot->tireState[i].pressureKpa);
        READ_F32(snapshot->tireState[i].temperatureC);
        READ_F32(snapshot->tireState[i].wear);
    }
    READ_F32(snapshot->damage);
    READ_F32(snapshot->crashLockoutTimerS);

#undef READ_F32
#undef READ_U32
#undef READ_I32
    if (!reader->failed) snapshot->valid = true;
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

    ReplayVehicleSnapshot snapshot;
    read_vehicle_snapshot(&reader, &snapshot);
    if (reader.failed) return false;

    /* Every declared frame must actually be present; a truncated file is a bad file. */
    if ((size_t)header.frameCount >
        (reader.length - reader.cursor) / (size_t)DEV_REPLAY_FRAME_BYTES)
        return false;

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
    rb->initialVehicle = snapshot;

    if (info != NULL) *info = header;
    return true;
}

bool dev_replay_load(ReplayBuffer *rb, const char *path, DevReplayInfo *info)
{
    if (rb == NULL || path == NULL) return false;

    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;

    static const size_t maxBytes =
        (size_t)DEV_REPLAY_HEADER_BYTES + (size_t)DEV_REPLAY_SNAPSHOT_MAX_BYTES +
        (size_t)REPLAY_CAPACITY_TICKS * (size_t)DEV_REPLAY_FRAME_BYTES;

    /* The format has a hard upper bound, so the whole file fits in one fixed buffer and
     * nothing is allocated. It is module-static rather than stack-resident only to keep the
     * frame small; it is never referenced from Game, so hot reload is unaffected. */
    static unsigned char buffer[DEV_REPLAY_HEADER_BYTES + DEV_REPLAY_SNAPSHOT_MAX_BYTES +
                                REPLAY_CAPACITY_TICKS * DEV_REPLAY_FRAME_BYTES];
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

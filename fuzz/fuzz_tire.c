/*
 * fuzz_tire.c — the pure tire and friction functions.
 *
 * These take floats straight from the simulation, so the interesting inputs are not
 * "malformed" but extreme: huge slip, zero load, negative speed, denormals. Physics
 * functions are fuzzed for PROPERTIES rather than for exact values, because the exact value
 * is the thing under development:
 *
 *   - finite inputs produce finite outputs;
 *   - zero slip produces zero force;
 *   - the combined force never exceeds the friction circle by more than FRICTION_TOLERANCE;
 *   - normal loads are never turned into negative grip.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/config.h"
#include "physics/tire.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

/* Map four bytes onto a bounded float, avoiding NaN inputs: the contract is about what the
 * functions do with *finite* values. */
static float bounded_float(const uint8_t *bytes, float low, float high)
{
    uint32_t raw = 0;
    memcpy(&raw, bytes, sizeof(raw));
    const float unit = (float)(raw % 100001u) / 100000.0f;
    return low + unit * (high - low);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (size < 24) return 0;

    const float slipAngle = bounded_float(data + 0, -6.5f, 6.5f);
    const float slipRatio = bounded_float(data + 4, -8.0f, 8.0f);
    const float stiffness = bounded_float(data + 8, 0.1f, 40.0f);
    const float shape = bounded_float(data + 12, 0.5f, 3.0f);
    const float friction = bounded_float(data + 16, 0.0f, 3.0f);
    const float load = bounded_float(data + 20, 0.0f, 20000.0f);

    const float lateral = tire_normalized_curve(stiffness, shape, slipAngle);
    const float longitudinal = tire_normalized_curve(stiffness, shape, slipRatio);
    if (!isfinite(lateral) || !isfinite(longitudinal)) abort();
    if (fabsf(lateral) > 1.0f || fabsf(longitudinal) > 1.0f) abort();

    /* sin(C * atan(B * 0)) is exactly zero for every B and C. */
    if (tire_normalized_curve(stiffness, shape, 0.0f) != 0.0f) abort();

    const float lateralForceN =
        tire_lateral_force_n(slipAngle, load, stiffness, shape, friction);
    const float longitudinalForceN =
        tire_longitudinal_force_n(slipRatio, load, stiffness, shape, friction);
    if (!isfinite(lateralForceN) || !isfinite(longitudinalForceN)) abort();

    const float slip = tire_slip_ratio(
        bounded_float(data + 0, -400.0f, 400.0f), bounded_float(data + 4, 0.01f, 1.0f),
        bounded_float(data + 8, -150.0f, 150.0f), SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP);
    if (!isfinite(slip) || fabsf(slip) > SLIP_RATIO_CLAMP + 1e-4f) abort();

    /* The friction ellipse: the limited pair must sit inside it, and the reported usage must
     * never claim more than the whole budget. */
    const float longitudinalLimitN = friction * load;
    const float lateralLimitN = friction * load * 1.05f;
    float limitedLongitudinalN = 0.0f;
    float limitedLateralN = 0.0f;
    float usage = -1.0f;
    tire_apply_combined_limit(longitudinalForceN, lateralForceN, longitudinalLimitN,
                              lateralLimitN, &limitedLongitudinalN, &limitedLateralN, &usage);

    if (!isfinite(limitedLongitudinalN) || !isfinite(limitedLateralN) || !isfinite(usage)) {
        abort();
    }
    if (usage < 0.0f || usage > 1.0f + FRICTION_TOLERANCE) abort();

    if (longitudinalLimitN > 0.0f && lateralLimitN > 0.0f) {
        const float nx = limitedLongitudinalN / longitudinalLimitN;
        const float ny = limitedLateralN / lateralLimitN;
        if (sqrtf(nx * nx + ny * ny) > 1.0f + FRICTION_TOLERANCE) abort();
    } else if (limitedLongitudinalN != 0.0f || limitedLateralN != 0.0f) {
        /* No grip budget means no force, not a small one. */
        abort();
    }
    return 0;
}

#if !defined(CIRCUIT_LIBFUZZER)
int main(int argc, char **argv)
{
    static unsigned char bytes[64];

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
        /* A deterministic sweep, so this target is useful even without libFuzzer. */
        uint32_t state = 0x1234567u;
        for (int i = 0; i < 20000; i++) {
            for (size_t b = 0; b < sizeof(bytes); b += 4) {
                state = state * 1664525u + 1013904223u;
                memcpy(bytes + b, &state, sizeof(state));
            }
            LLVMFuzzerTestOneInput(bytes, sizeof(bytes));
        }
        printf("ok 20000 pseudo-random inputs\n");
    }
    return 0;
}
#endif

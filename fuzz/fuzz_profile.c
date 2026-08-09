/*
 * fuzz_profile.c — the tuning-profile parser.
 *
 * A profile is text a human edits by hand and a coding agent generates, so it will be
 * malformed sooner or later. The properties asserted here are the ones that matter:
 *
 *   - no input crashes, reads out of bounds, or leaves the parser in a loop;
 *   - a profile that would produce an invalid spec changes nothing at all;
 *   - whatever comes out is a spec the simulation would accept.
 *
 * Build and run with `make fuzz` (clang + libFuzzer). The same file also builds as a plain
 * executable that replays saved corpus files, so a crash found in CI can be reproduced on
 * Windows where libFuzzer is not available.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/dev_params.h"
#include "physics/vehicle.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    VehicleSpec defaults;
    vehicle_spec_set_default(&defaults);

    VehicleSpec spec = defaults;
    int applied = 0;
    int unknown = 0;
    int rejected = 0;

    const bool ok =
        dev_params_apply_text(&spec, (const char *)data, size, &applied, &unknown, &rejected);

    if (ok) {
        /* Accepting an input means promising the result is usable. */
        if (!vehicle_spec_is_valid(&spec)) abort();
    } else {
        /* Refusing an input means promising nothing was changed. */
        if (memcmp(&spec, &defaults, sizeof(VehicleSpec)) != 0) abort();
    }

    if (applied < 0 || unknown < 0 || rejected < 0) abort();
    if (applied > dev_params_count()) abort();
    return 0;
}

#if !defined(CIRCUIT_LIBFUZZER)
/* Standalone driver: `fuzz_profile corpus/file ...` replays inputs without libFuzzer. */
int main(int argc, char **argv)
{
    static unsigned char buffer[1 << 20];

    for (int i = 1; i < argc; i++) {
        FILE *file = fopen(argv[i], "rb");
        if (file == NULL) {
            fprintf(stderr, "could not open %s\n", argv[i]);
            return 1;
        }
        const size_t size = fread(buffer, 1, sizeof(buffer), file);
        fclose(file);
        LLVMFuzzerTestOneInput(buffer, size);
        printf("ok %s (%zu bytes)\n", argv[i], size);
    }

    if (argc == 1) {
        const char *samples[] = {
            "",
            "body.mass = 1400\n",
            "body.mass=\n",
            "= 5\n",
            "body.mass = nan\n",
            "engine.idle_rpm = 9999\nengine.redline_rpm = 100\n",
            "\n\n#comment\n;comment\n   \t\n",
        };
        for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
            LLVMFuzzerTestOneInput((const uint8_t *)samples[i], strlen(samples[i]));
        }
        printf("ok %d built-in samples\n", (int)(sizeof(samples) / sizeof(samples[0])));
    }
    return 0;
}
#endif

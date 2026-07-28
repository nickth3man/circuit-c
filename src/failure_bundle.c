/* localtime_r needs a POSIX feature macro before <time.h> on strict C11 Linux builds. */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

#include "failure_bundle.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "build_info.h"
#include "config.h"
#include "dev_params.h"
#include "dev_replay.h"
#include "telemetry.h"

/* Directory names must survive being typed, tab-completed, and pasted into a shell. */
static void sanitize_component(const char *in, char *out, size_t capacity)
{
    size_t written = 0;
    if (capacity == 0) return;
    for (size_t i = 0; in != NULL && in[i] != '\0' && written + 1 < capacity; i++) {
        const char c = in[i];
        const bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
        out[written++] = safe ? c : '-';
    }
    if (written == 0 && capacity > 1) out[written++] = 'x';
    out[written] = '\0';
}

static void timestamp_now(char *out, size_t capacity)
{
    const time_t now = time(NULL);
    struct tm parts;
    memset(&parts, 0, sizeof(parts));
    bool ok = false;

#if defined(_WIN32)
    ok = (localtime_s(&parts, &now) == 0);
#else
    ok = (localtime_r(&now, &parts) != NULL);
#endif

    if (!ok || strftime(out, capacity, "%Y%m%d-%H%M%S", &parts) == 0) {
        snprintf(out, capacity, "%llu", (unsigned long long)now);
    }
}

/* Byte-for-byte copy. Returns false when either side could not be opened or the copy was
 * short; the caller records that in summary.json rather than failing the whole bundle. */
static bool copy_file(const char *fromPath, const char *toPath)
{
    if (fromPath == NULL || toPath == NULL) return false;

    FILE *from = fopen(fromPath, "rb");
    if (from == NULL) return false;

    FILE *to = fopen(toPath, "wb");
    if (to == NULL) {
        fclose(from);
        return false;
    }

    char buffer[8192];
    bool ok = true;
    size_t read;
    while ((read = fread(buffer, 1, sizeof(buffer), from)) > 0) {
        if (fwrite(buffer, 1, read, to) != read) { ok = false; break; }
    }
    if (ferror(from)) ok = false;

    fclose(from);
    return (fclose(to) == 0) && ok;
}

static bool write_text_file(const char *path, const char *text)
{
    FILE *file = fopen(path, "wb");
    if (file == NULL) return false;
    if (text != NULL) fputs(text, file);
    return fclose(file) == 0;
}

/* Minimal JSON string escaping: quotes, backslashes, and control characters. */
static void write_json_string(FILE *out, const char *text)
{
    fputc('"', out);
    for (size_t i = 0; text != NULL && text[i] != '\0'; i++) {
        const unsigned char c = (unsigned char)text[i];
        switch (c) {
            case '"':  fputs("\\\"", out); break;
            case '\\': fputs("\\\\", out); break;
            case '\n': fputs("\\n", out);  break;
            case '\r': fputs("\\r", out);  break;
            case '\t': fputs("\\t", out);  break;
            default:
                if (c < 0x20u) fprintf(out, "\\u%04x", c);
                else fputc((int)c, out);
                break;
        }
    }
    fputc('"', out);
}

bool failure_bundle_write(const char *rootDir, const FailureBundle *bundle,
                          char *dirOut, size_t dirCapacity)
{
    if (bundle == NULL) return false;

    const char *root = (rootDir != NULL && rootDir[0] != '\0') ? rootDir : "artifacts";
    if (!telemetry_ensure_dir(root)) return false;

    char scenario[64];
    char stamp[32];
    sanitize_component(bundle->scenario, scenario, sizeof(scenario));
    timestamp_now(stamp, sizeof(stamp));

    char dir[512];
    snprintf(dir, sizeof(dir), "%s/failure-%s-%s", root, scenario, stamp);
    if (!telemetry_ensure_dir(dir)) return false;

    char path[640];
    bool replayWritten = false;
    bool telemetryCopied = false;
    bool screenshotCopied = false;

    /* failure.txt — the failing check, verbatim. */
    snprintf(path, sizeof(path), "%s/failure.txt", dir);
    if (!write_text_file(path, bundle->failureText != NULL ? bundle->failureText
                                                          : "(no failure text recorded)\n")) {
        fprintf(stderr, "BUNDLE: could not write %s\n", path);
    }

    /* git_info.txt — how this binary was built. */
    snprintf(path, sizeof(path), "%s/git_info.txt", dir);
    {
        FILE *file = fopen(path, "wb");
        if (file != NULL) {
            build_info_write(file);
            fclose(file);
        } else {
            fprintf(stderr, "BUNDLE: could not write %s\n", path);
        }
    }

    /* config_snapshot.txt — fixed infrastructure constants plus every tunable's value. */
    snprintf(path, sizeof(path), "%s/config_snapshot.txt", dir);
    {
        FILE *file = fopen(path, "wb");
        if (file != NULL) {
            fprintf(file, "fixed_hz=%d\n", FIXED_HZ);
            fprintf(file, "fixed_dt_s=%.9f\n", (double)FIXED_DT_S);
            fprintf(file, "max_physics_steps=%d\n", MAX_PHYSICS_STEPS);
            fprintf(file, "max_frame_time_s=%.6f\n", (double)MAX_FRAME_TIME_S);
            fprintf(file, "replay_capacity_ticks=%d\n", REPLAY_CAPACITY_TICKS);
            fprintf(file, "pixels_per_meter=%.6f\n", (double)PIXELS_PER_METER);
            fprintf(file, "seed=%u\n", bundle->seed);
            fprintf(file, "active_profile=%s\n",
                    bundle->activeProfile != NULL ? bundle->activeProfile : "unknown");
            if (bundle->spec != NULL) {
                fprintf(file, "\n");
                dev_params_write_metadata(file, bundle->spec);
                fprintf(file, "\n# overrides: ");
                if (dev_params_write_overrides(file, bundle->spec) == 0) {
                    fprintf(file, "(none — every tunable is at its default)");
                }
                fprintf(file, "\n");
            }
            fclose(file);
        } else {
            fprintf(stderr, "BUNDLE: could not write %s\n", path);
        }
    }

    /* replay.bin — the minimal failing input timeline. */
    if (bundle->replay != NULL) {
        snprintf(path, sizeof(path), "%s/replay.bin", dir);
        replayWritten = dev_replay_save(bundle->replay, path, bundle->scenario,
                                        bundle->seed, bundle->checksum);
        if (!replayWritten) fprintf(stderr, "BUNDLE: could not write %s\n", path);
    }

    /* telemetry.csv / screenshot.png — copied verbatim when the run produced them. */
    if (bundle->telemetryPath != NULL) {
        snprintf(path, sizeof(path), "%s/telemetry.csv", dir);
        telemetryCopied = copy_file(bundle->telemetryPath, path);
    }
    if (bundle->screenshotPath != NULL) {
        snprintf(path, sizeof(path), "%s/screenshot.png", dir);
        screenshotCopied = copy_file(bundle->screenshotPath, path);
    }

    /* summary.json — the machine-readable index of everything above. */
    snprintf(path, sizeof(path), "%s/summary.json", dir);
    {
        FILE *file = fopen(path, "wb");
        if (file != NULL) {
            fprintf(file, "{\n");
            fprintf(file, "  \"scenario\": ");
            write_json_string(file, bundle->scenario);
            fprintf(file, ",\n  \"timestamp\": ");
            write_json_string(file, stamp);
            fprintf(file, ",\n  \"failing_tick\": %llu",
                    (unsigned long long)bundle->failingTick);
            fprintf(file, ",\n  \"state_checksum\": \"%08x\"", bundle->checksum);
            fprintf(file, ",\n  \"seed\": %u", bundle->seed);
            fprintf(file, ",\n  \"checks_run\": %d", bundle->checksRun);
            fprintf(file, ",\n  \"checks_failed\": %d", bundle->checksFailed);
            fprintf(file, ",\n  \"failure\": ");
            write_json_string(file, bundle->failureText);
            fprintf(file, ",\n  \"active_profile\": ");
            write_json_string(file, bundle->activeProfile != NULL
                                  ? bundle->activeProfile : "unknown");
            fprintf(file, ",\n  \"tunables_modified\": %d",
                    bundle->spec != NULL ? dev_params_modified_count(bundle->spec) : 0);
            fprintf(file, ",\n  \"build\": {\n");
            fprintf(file, "    \"commit\": ");
            write_json_string(file, DRIFTY_BUILD_COMMIT);
            fprintf(file, ",\n    \"branch\": ");
            write_json_string(file, DRIFTY_BUILD_BRANCH);
            fprintf(file, ",\n    \"dirty\": ");
            write_json_string(file, DRIFTY_BUILD_DIRTY);
            fprintf(file, ",\n    \"mode\": ");
            write_json_string(file, DRIFTY_BUILD_MODE);
            fprintf(file, ",\n    \"compiler\": ");
            write_json_string(file, DRIFTY_BUILD_COMPILER);
            fprintf(file, ",\n    \"flags\": ");
            write_json_string(file, DRIFTY_BUILD_FLAGS);
            fprintf(file, ",\n    \"platform\": ");
            write_json_string(file, DRIFTY_BUILD_PLATFORM);
            fprintf(file, "\n  },\n");
            fprintf(file, "  \"files\": {\n");
            fprintf(file, "    \"failure.txt\": true,\n");
            fprintf(file, "    \"git_info.txt\": true,\n");
            fprintf(file, "    \"config_snapshot.txt\": true,\n");
            fprintf(file, "    \"replay.bin\": %s,\n", replayWritten ? "true" : "false");
            fprintf(file, "    \"telemetry.csv\": %s,\n", telemetryCopied ? "true" : "false");
            fprintf(file, "    \"screenshot.png\": %s\n", screenshotCopied ? "true" : "false");
            fprintf(file, "  }\n}\n");
            fclose(file);
        } else {
            fprintf(stderr, "BUNDLE: could not write %s\n", path);
        }
    }

    if (dirOut != NULL && dirCapacity > 0) {
        snprintf(dirOut, dirCapacity, "%s", dir);
    }
    return true;
}

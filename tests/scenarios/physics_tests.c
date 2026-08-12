/*
 * physics_tests.c — telemetry, the canonical structures, tires, the drivetrain, the
 * acceleration filter, load transfer, resistance, and the closed-form vehicle scenarios.
 *
 * It also implements set_vehicle_rolling_speed (declared in scenario_shared.h), which the
 * handling and gameplay groups use to start a run from cruise.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <time.h>

#include "support/test_harness.h"
#include "support/simulation_fixture.h"
#include "test_scenarios.h"
#include "scenario_shared.h"

#include "dev/car_corpus.h"
#include "game/car_roster.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_presets.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "physics/drivetrain.h"
#include "dev/failure_bundle.h"
#include "physics/surface.h"
#include "game/game.h"
#include "game/input.h"
#include "core/math_utils.h"
#include "game/particle.h"
#include "physics/physics.h"
#include "render/render.h"
#include "game/replay.h"
#include "game/validation_metrics.h"
#include "game/run_report.h"
#include "platform/timestep.h"
#include "physics/tire.h"
#include "core/units.h"
#include "world/collision.h"

/* ------------------------------------------------------------------------------------- */
/* Scenario: telemetry                                                                     */
/* ------------------------------------------------------------------------------------- */

#define TELEMETRY_PATH TELEMETRY_DIR "/phase2_determinism.csv"
#define TELEMETRY_PATH_REPEAT TELEMETRY_DIR "/phase2_determinism_repeat.csv"
#define PHASE2_LAUNCH_TELEMETRY TELEMETRY_DIR "/phase2_launch_stop.csv"

static bool files_equal(const char *pathA, const char *pathB)
{
    FILE *a = fopen(pathA, "rb");
    FILE *b = fopen(pathB, "rb");
    if (a == NULL || b == NULL) {
        if (a != NULL) fclose(a);
        if (b != NULL) fclose(b);
        return false;
    }
    bool equal = true;
    for (;;) {
        const int ca = fgetc(a);
        const int cb = fgetc(b);
        if (ca != cb) {
            equal = false;
            break;
        }
        if (ca == EOF) break;
    }
    fclose(a);
    fclose(b);
    return equal;
}

static void scenario_telemetry(void)
{
    check(telemetry_ensure_dir(TELEMETRY_DIR), "the telemetry directory exists or was created");

    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the input script\n");
        exit(126);
    }
    script_build(frames, SCRIPT_FRAMES);

    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_PATH);
    check(opened, "telemetry_open('%s') succeeded", TELEMETRY_PATH);

    Game *game = alloc_game();
    const uint32_t checksum =
        run_recording(game, frames, SCRIPT_FRAMES, PIXELS_PER_METER, opened ? &writer : NULL);

    check(writer.rowCount == (long)SCRIPT_FRAMES,
          "one row was written per render frame (%ld of %d)", writer.rowCount, SCRIPT_FRAMES);
    check(telemetry_close(&writer), "telemetry_close reported success");

    TelemetryWriter repeatWriter;
    const bool repeatOpened = telemetry_open(&repeatWriter, TELEMETRY_PATH_REPEAT);
    Game *repeatGame = alloc_game();
    const uint32_t repeatChecksum =
        run_recording(repeatGame, frames, SCRIPT_FRAMES, PIXELS_PER_METER,
                      repeatOpened ? &repeatWriter : NULL);
    check(repeatOpened && telemetry_close(&repeatWriter),
          "the repeated telemetry run writes and closes successfully");
    check(repeatChecksum == checksum && files_equal(TELEMETRY_PATH, TELEMETRY_PATH_REPEAT),
          "identical runs produce byte-identical telemetry and checksum");

    /* Read it back: stable header, expected row count, final checksum present. */
    {
        FILE *file = fopen(TELEMETRY_PATH, "rb");
        check(file != NULL, "the telemetry file can be reopened");
        if (file != NULL) {
            char line[2048];
            long dataRows = 0;
            char lastLine[2048];
            lastLine[0] = '\0';

            if (fgets(line, sizeof(line), file) != NULL) {
                line[strcspn(line, "\r\n")] = '\0';
                check(strcmp(line, telemetry_header()) == 0,
                      "the header row is exactly the documented schema");
            } else {
                check(false, "the telemetry file has a header row");
            }

            while (fgets(line, sizeof(line), file) != NULL) {
                if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
                dataRows++;
                memcpy(lastLine, line,
                       sizeof(lastLine) < sizeof(line) ? sizeof(lastLine) : sizeof(line));
                lastLine[sizeof(lastLine) - 1] = '\0';
            }
            fclose(file);

            check(dataRows == (long)SCRIPT_FRAMES, "the file holds %d data rows (got %ld)",
                  SCRIPT_FRAMES, dataRows);

            char expectedTail[32];
            snprintf(expectedTail, sizeof(expectedTail), ",%u", checksum);

            /* Phase 5: the header and every data row carry the same column count, and the
             * lap-validation columns are present by name. `line` was overwritten by the read
             * loop, so the header comes from telemetry_header() and the data row from lastLine. */
            {
                const char *header = telemetry_header();
                long headerCommas = 0;
                for (const char *p = header; *p != '\0'; p++)
                    if (*p == ',') headerCommas++;
                long tailCommas = 0;
                for (const char *p = lastLine; *p != '\0'; p++)
                    if (*p == ',') tailCommas++;
                check(headerCommas == tailCommas,
                      "header and data row agree on column count (%ld commas)", headerCommas);
                check(strstr(header, "checkpoint_index") != NULL &&
                          strstr(header, "lap_state") != NULL &&
                          strstr(header, "on_track") != NULL &&
                          strstr(header, "distance_to_centerline_m") != NULL,
                      "the Phase 5 lap-validation columns are in the header");
                check(strstr(header, "slip_angle_front_left_rad") != NULL &&
                          strstr(header, "slip_angle_front_right_rad") != NULL &&
                          strstr(header, "slip_angle_rear_left_rad") != NULL &&
                          strstr(header, "slip_angle_rear_right_rad") != NULL &&
                          strstr(header, "slip_ratio_front_left") != NULL &&
                          strstr(header, "slip_ratio_front_right") != NULL &&
                          strstr(header, "slip_ratio_rear_left") != NULL &&
                          strstr(header, "slip_ratio_rear_right") != NULL,
                      "the per-wheel slip columns are in the header");
            }
        }
    }

    /* The per-wheel slip columns must carry the wheel states the tire model was evaluated at,
     * wired to the correct axle. The axle columns are defined as the mean of their two wheels,
     * so that identity is what pins the front/rear halves of the projection in place. */
    {
        const TelemetryRow row = test_telemetry_row_from_game(game, 1);
        check(isfinite(row.slipAngleFrontLeftRad) && isfinite(row.slipAngleFrontRightRad) &&
                  isfinite(row.slipAngleRearLeftRad) && isfinite(row.slipAngleRearRightRad) &&
                  isfinite(row.slipRatioFrontLeft) && isfinite(row.slipRatioFrontRight) &&
                  isfinite(row.slipRatioRearLeft) && isfinite(row.slipRatioRearRight),
              "every per-wheel slip column is finite");
        check(fabsf(0.5f * (row.slipRatioFrontLeft + row.slipRatioFrontRight) -
                    row.frontSlipRatio) < 1e-6f,
              "front_slip_ratio is the mean of the two front wheel columns");
        check(fabsf(0.5f * (row.slipRatioRearLeft + row.slipRatioRearRight) -
                    row.rearSlipRatio) < 1e-6f,
              "rear_slip_ratio is the mean of the two rear wheel columns");
    }

    /* Failure handling: an unwritable path must be reported, not ignored. The writer logs
     * to stderr when it fails, so one TELEMETRY error line below is expected output. */
    {
        printf("    (the next TELEMETRY error line is the expected failure-path output)\n");

        TelemetryWriter bad;
        const bool badOpen =
            telemetry_open(&bad, TELEMETRY_DIR "/no_such_directory/telemetry.csv");
        check(!badOpen, "telemetry_open reports failure for an unwritable path");
        check(!telemetry_close(&bad), "telemetry_close propagates the earlier failure");

        TelemetryRow row;
        memset(&row, 0, sizeof(row));
        check(!telemetry_write_row(&bad, &row), "writing to a failed writer is refused");
    }

    free(game);
    free(repeatGame);
    free(frames);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: run-report — validation metrics + run.json writer                            */
/* ------------------------------------------------------------------------------------- */

/* A hand-built row stream whose metrics are computed by hand below. dt is 0.1 s between
 * rows so the interval detectors have unambiguous durations to reason about. */
static void build_report_fixture(TelemetryRow *rows)
{
    memset(rows, 0, sizeof(TelemetryRow) * 6);
    const float fuLimit = 0.99f;
    const float fuLow = 0.50f;

    for (int i = 0; i < 6; i++) {
        rows[i].tick = (uint64_t)i;
        rows[i].timeS = (double)i * 0.1;
    }

    rows[0].speedMps = 10.0f;
    rows[0].yawRateRadS = 0.5f;
    rows[0].solvedLongAccelMps2 = 2.0f;
    rows[0].lateralAccelMps2 = 1.0f;
    rows[0].frontFrictionUsage = fuLow;
    rows[0].rearFrictionUsage = fuLow;
    rows[0].onTrack = 1;
    rows[1].speedMps = 12.0f;
    rows[1].yawRateRadS = 0.5f;
    rows[1].solvedLongAccelMps2 = 2.0f;
    rows[1].lateralAccelMps2 = 1.0f;
    rows[1].frontFrictionUsage = 0.6f;
    rows[1].rearFrictionUsage = 0.6f;
    rows[1].onTrack = 1;
    rows[1].crashLockoutS = 0.4f;
    rows[1].checkpointEvent = 1;
    rows[2].speedMps = 20.0f;
    rows[2].yawRateRadS = 1.0f;
    rows[2].solvedLongAccelMps2 = 3.0f;
    rows[2].lateralAccelMps2 = 2.0f;
    rows[2].frontFrictionUsage = fuLimit;
    rows[2].rearFrictionUsage = fuLimit;
    rows[2].onTrack = 1;
    rows[2].crashLockoutS = 0.3f;
    rows[2].bodySideslipRad = 1.6f;
    rows[3].speedMps = 20.0f;
    rows[3].yawRateRadS = 1.0f;
    rows[3].solvedLongAccelMps2 = 3.0f;
    rows[3].lateralAccelMps2 = 2.0f;
    rows[3].frontFrictionUsage = fuLimit;
    rows[3].rearFrictionUsage = fuLimit;
    rows[3].onTrack = 0;
    rows[3].crashLockoutS = 0.2f;
    rows[3].bodySideslipRad = 1.6f;
    rows[4].speedMps = 20.0f;
    rows[4].yawRateRadS = 1.0f;
    rows[4].solvedLongAccelMps2 = 3.0f;
    rows[4].lateralAccelMps2 = 2.0f;
    rows[4].frontFrictionUsage = fuLimit;
    rows[4].rearFrictionUsage = fuLimit;
    rows[4].onTrack = 0;
    rows[4].crashLockoutS = 0.1f;
    rows[4].bodySideslipRad = 1.6f;
    rows[5].speedMps = 20.0f;
    rows[5].yawRateRadS = 0.5f;
    rows[5].solvedLongAccelMps2 = 3.0f;
    rows[5].lateralAccelMps2 = 2.0f;
    rows[5].frontFrictionUsage = fuLow;
    rows[5].rearFrictionUsage = fuLow;
    rows[5].onTrack = 1;
    rows[5].checkpointEvent = 3;
}

static void scenario_run_report(void)
{
    TelemetryRow rows[6];
    build_report_fixture(rows);

    ValidationMetrics m;
    validation_metrics_compute(rows, 6, &m);

    /* Hand-computed: speeds [10,12,20,20,20,20], dt 0.1. */
    check_near(m.maxSpeedMps, 20.0, 1e-6, "max speed is 20 m/s");
    check_near(m.meanSpeedMps, 17.0, 1e-6, "mean speed is 102/6 = 17 m/s");
    check_near(m.minMovingSpeedMps, 10.0, 1e-6, "min moving speed is 10 m/s");
    check_near(m.medianSpeedMps, 20.0, 1e-6, "median speed is 20 m/s");
    check_near(m.maxAbsSideslipRad, 1.6, 1e-6, "max |sideslip| is 1.6 rad");
    check_near(m.maxYawRateRadS, 1.0, 1e-6, "max |yaw rate| is 1.0 rad/s");
    check_near(m.maxFrictionUsage, 0.99, 1e-6, "max friction usage is 0.99");
    check_near(m.timeAtFrictionLimitS, 0.3, 1e-6, "0.3 s at the friction limit (rows 2-4)");
    check_near(m.maxLongAccelMps2, 3.0, 1e-6, "max long accel is 3.0 m/s^2");
    check_near(m.minLongAccelMps2, 2.0, 1e-6, "min long accel is 2.0 m/s^2");
    check_near(m.maxAbsLatAccelMps2, 2.0, 1e-6, "max |lat accel| is 2.0 m/s^2");
    check_near(m.timeBelow5mpsS, 0.0, 1e-6, "no time below 5 m/s");

    /* One collision: the lockout timer's 0 -> 0.4 rising edge at row 1. */
    check(m.collisions == 1, "exactly one collision (rising edge at row 1; got %d)",
          m.collisions);
    /* One spin: rows 2-4 hold the attitude for 0.3 s >= 0.25 s. */
    check(m.spinEvents == 1, "exactly one spin event (0.3 s >= 0.25 s; got %d)", m.spinEvents);
    /* One off-track: rows 3-4 off-racing-surface for 0.2 s >= 0.1 s. */
    check(m.offTrackEvents == 1, "exactly one off-track event (0.2 s >= 0.1 s; got %d)",
          m.offTrackEvents);
    check_near(m.offTrackTimeS, 0.2, 1e-6, "0.2 s spent off the racing surface");
    /* Checkpoints: row 1 in-order, row 5 lap-complete. */
    check(m.checkpointsPassed == 2, "two checkpoint crossings counted (got %d)",
          m.checkpointsPassed);
    check(m.outOfOrderEvents == 0, "no out-of-order crossings (got %d)", m.outOfOrderEvents);
    /* Out-lap completes at row 5 (the only lap-complete event). */
    check_near(m.outLapTimeS, 0.5, 1e-6, "out-lap time is 0.5 s");
    /* And nothing after it, so every timed-lap field stays at its zero. */
    check(m.timedLapsCompleted == 0, "no timed laps completed (got %d)", m.timedLapsCompleted);
    check_near(m.timedLapTimeS, 0.0, 1e-6, "no first timed lap time");
    check_near(m.bestTimedLapTimeS, 0.0, 1e-6, "no best timed lap time");
    check_near(m.meanTimedLapTimeS, 0.0, 1e-6, "no mean timed lap time");

    /* Three timed laps, measured from the previous lap's finish rather than from the start of
     * the run. Lap completions at t = 0.5, 1.2, 1.8 and 2.6 s make the out-lap 0.5 s and the
     * timed laps 0.7, 0.6 and 0.8 s — deliberately unequal, because summing to a mean is the
     * only thing that separates a working per-lap delta from one that reports total elapsed. */
    {
        TelemetryRow laps[27];
        memset(laps, 0, sizeof(laps));
        for (int i = 0; i < 27; i++) {
            laps[i].timeS = (double)(i + 1) * 0.1;
            laps[i].speedMps = 20.0f;
            laps[i].onTrack = 1;
        }
        laps[4].checkpointEvent = 3;  /* t = 0.5 — out-lap done */
        laps[11].checkpointEvent = 3; /* t = 1.2 — timed lap 1: 0.7 s */
        laps[17].checkpointEvent = 3; /* t = 1.8 — timed lap 2: 0.6 s */
        laps[25].checkpointEvent = 3; /* t = 2.6 — timed lap 3: 0.8 s */

        ValidationMetrics lm;
        validation_metrics_compute(laps, 27, &lm);

        check(lm.timedLapsCompleted == 3, "three timed laps completed (got %d)",
              lm.timedLapsCompleted);
        check_near(lm.outLapTimeS, 0.5, 1e-5, "out-lap is 0.5 s");
        check_near(lm.timedLapTimesS[0], 0.7, 1e-5, "timed lap 1 is 0.7 s");
        check_near(lm.timedLapTimesS[1], 0.6, 1e-5, "timed lap 2 is 0.6 s");
        check_near(lm.timedLapTimesS[2], 0.8, 1e-5, "timed lap 3 is 0.8 s");
        check_near(lm.timedLapTimeS, 0.7, 1e-5, "timed_lap_time_s still means the first one");
        check_near(lm.bestTimedLapTimeS, 0.6, 1e-5, "best timed lap is 0.6 s");
        check_near(lm.meanTimedLapTimeS, 2.1 / 3.0, 1e-5, "mean timed lap is 0.7 s");
    }

    /* Empty input is safe. */
    ValidationMetrics zero;
    validation_metrics_compute(NULL, 0, &zero);
    check(zero.rowCount == 0, "a NULL/empty input reduces to zero metrics");

    /* run.json: a failing run carries the injected reason and every required key. */
    RunReportInput in;
    memset(&in, 0, sizeof(in));
    in.runId = "20260807-000000-chicane_v1-rwd_grip";
    in.carId = "rwd_grip";
    in.carDisplayName = "RWD Grip";
    in.carDrivetrain = "RWD";
    in.carMassKg = 1220.0;
    in.carSpecHash = "deadbeef";
    in.trackId = "chicane";
    in.trackVersion = "chicane_v1";
    in.trackGeometryHash = "cafebabe";
    in.trackCheckpointCount = 8;
    in.trackLengthM = 642.0;
    in.fixedHz = 120;
    in.telemetryHz = 60;
    in.videoFps = 60;
    in.buildCommit = "abc1234";
    in.buildDirty = false;
    in.finalStateChecksum = "11223344";
    in.tickBudget = 14400;
    in.ticksRun = 8460;
    in.metrics = &m;
    in.hasVideo = true;
    in.hasReplay = true;
    /* No-run sentinels: this fixture predates the classification block; the checkpoint fields
     * must serialize as "nothing crossed" rather than a zeroed gate 0 (PR #80 review). */
    in.lastCheckpointIndex = -1;
    in.expectedCheckpointIndex = -1;

    /* Injected failure: one checkpoint was missed. */
    in.status = RUN_FAIL_CHECKPOINT_MISSED;
    in.checkpointsPassed = 7;
    in.checkpointsMissed = 1;
    in.outOfOrderEvents = 0;

    const char *failPath = TELEMETRY_DIR "/run_report_fail.json";
    check(run_report_write(failPath, &in), "run_report_write succeeded for a failing run");

    char buf[8192];
    FILE *f = fopen(failPath, "rb");
    check(f != NULL, "the failing run.json can be reopened");
    if (f != NULL) {
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        buf[n] = '\0';
        fclose(f);
        check(strstr(buf, "\"schema_version\"") != NULL, "run.json carries schema_version");
        check(strstr(buf, "\"run_id\"") != NULL, "run.json carries run_id");
        check(strstr(buf, "\"car\"") != NULL && strstr(buf, "\"rwd_grip\"") != NULL,
              "run.json carries the car block");
        check(strstr(buf, "\"track\"") != NULL && strstr(buf, "\"chicane_v1\"") != NULL,
              "run.json carries the track block");
        check(strstr(buf, "\"sim\"") != NULL && strstr(buf, "\"abc1234\"") != NULL,
              "run.json carries the sim block with the build commit");
        check(strstr(buf, "\"metrics\"") != NULL, "run.json carries the metrics block");
        check(strstr(buf, "\"artifacts\"") != NULL, "run.json carries the artifacts block");
        check(strstr(buf, "\"status\": \"FAIL\"") != NULL, "a failed run reports status FAIL");
        check(strstr(buf, "\"checkpoint_missed\"") != NULL,
              "the injected failure reason is present");
        check(strstr(buf, "\"checkpoints_missed\": 1") != NULL,
              "the missed-checkpoint count is present");
        check(strstr(buf, "\"last_checkpoint_index\": -1") != NULL,
              "a no-run report states no gate was crossed");
        check(strstr(buf, "\"expected_checkpoint_index\": -1") != NULL,
              "a no-run report states no gate was owed");
        check(strstr(buf, "\"classification\"") != NULL,
              "run.json carries the classification block (schema 1.1.0)");
        check(strstr(buf, "\"reason\": \"unclassified\"") != NULL,
              "an unclassified run never serializes as a pass");
        check(strstr(buf, "\"first_fault_tick\": 0") != NULL,
              "an unclassified run carries no fault tick");
        check(strstr(buf, "\"contributing\": []") != NULL,
              "an unclassified run carries no contributing events");
    }

    /* A passing run reports PASS and a null failure_reason. */
    in.status = RUN_PASS;
    in.checkpointsPassed = 8;
    in.checkpointsMissed = 0;
    const char *passPath = TELEMETRY_DIR "/run_report_pass.json";
    check(run_report_write(passPath, &in), "run_report_write succeeded for a passing run");
    f = fopen(passPath, "rb");
    if (f != NULL) {
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        buf[n] = '\0';
        fclose(f);
        check(strstr(buf, "\"status\": \"PASS\"") != NULL, "a passing run reports status PASS");
        check(strstr(buf, "\"failure_reason\": null") != NULL,
              "a passing run has a null failure_reason");
    }
}

/* ------------------------------------------------------------------------------------- */
/* Phase 1 focused unit checks                                                            */
/* ------------------------------------------------------------------------------------- */

static void phase1_fixture(VehicleSpec *spec, VehicleState *state, VehicleDerived *derived,
                           VehicleRenderState *renderState)
{
    vehicle_spec_set_default(spec);
    vehicle_state_reset(spec, state, derived, renderState);
}

static void scenario_vehicle_units(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);

    check(vehicle_spec_is_valid(&spec), "the default vehicle specification is valid");
    check_near(spec.wheelbaseM, spec.cgToFrontM + spec.cgToRearM, 1e-7,
               "wheelbase equals the two CG lever arms");
    check(spec.gearCount <= MAX_GEARS, "canonical gear storage capacity is respected");

    VehicleSpec invalid = spec;
    invalid.massKg = 0.0f;
    check(!vehicle_spec_is_valid(&invalid), "zero mass is rejected");
    invalid = spec;
    invalid.wheelbaseM += 0.1f;
    check(!vehicle_spec_is_valid(&invalid), "an inconsistent wheelbase is rejected");

    check_near(state.wheels[WHEEL_FRONT_LEFT].localPositionM.x, spec.cgToFrontM, 0.0,
               "front-left contact X uses +cgToFront");
    check_near(state.wheels[WHEEL_REAR_RIGHT].localPositionM.x, -spec.cgToRearM, 0.0,
               "rear-right contact X uses -cgToRear");
    check(state.wheels[WHEEL_FRONT_LEFT].localPositionM.y > 0.0f &&
              state.wheels[WHEEL_REAR_LEFT].localPositionM.y > 0.0f,
          "left wheel contacts use positive body Y");
    check(state.wheels[WHEEL_FRONT_RIGHT].localPositionM.y < 0.0f &&
              state.wheels[WHEEL_REAR_RIGHT].localPositionM.y < 0.0f,
          "right wheel contacts use negative body Y");

    float frontLoadN;
    float rearLoadN;
    physics_static_axle_loads(&spec, &frontLoadN, &rearLoadN);
    check_near(frontLoadN + rearLoadN, spec.massKg * GRAVITY_MPS2, 0.01,
               "static axle loads sum to mass times gravity");
    check_near(state.wheels[WHEEL_FRONT_LEFT].normalLoadN, frontLoadN * 0.5f, 0.01,
               "front axle load is split evenly");
    check_near(state.wheels[WHEEL_REAR_RIGHT].normalLoadN, rearLoadN * 0.5f, 0.01,
               "rear axle load is split evenly");
    VehicleSpec rearwardCg = spec;
    rearwardCg.cgToFrontM = 1.40f;
    rearwardCg.cgToRearM = 1.15f;
    rearwardCg.wheelbaseM = 2.55f;
    float movedFrontLoadN;
    float movedRearLoadN;
    physics_static_axle_loads(&rearwardCg, &movedFrontLoadN, &movedRearLoadN);
    check(movedFrontLoadN < frontLoadN && movedRearLoadN > rearLoadN,
          "moving the CG rearward moves static load rearward");

    spec.ackermannPercent = 0.0f;
    physics_update_steering(&spec, &state, 1.0f, 0.05f);
    check_near(state.frontRoadWheelAngleRad, spec.maxSteerRateRadS * 0.05f, 1e-7,
               "left steering maps positive and obeys the steering rate");
    /* With Ackermann off, the rack angle reaches both front wheels identically; the only thing
     * separating them is static toe, mirrored across the axle (issue #14). */
    check_near(state.wheels[WHEEL_FRONT_LEFT].steerAngleRad +
                   state.wheels[WHEEL_FRONT_RIGHT].steerAngleRad,
               2.0 * (double)state.frontRoadWheelAngleRad, 1e-7,
               "parallel steering: the front pair still averages the rack angle");
    check_near(state.wheels[WHEEL_FRONT_RIGHT].steerAngleRad -
                   state.wheels[WHEEL_FRONT_LEFT].steerAngleRad,
               2.0 * (double)spec.suspToeFrontRad, 1e-7,
               "front toe-in opens the pair by twice the authored toe, right minus left");
    /* The rear axle has no rack, so its heading is its static toe and nothing else. */
    check_near(state.wheels[WHEEL_REAR_LEFT].steerAngleRad, -(double)spec.suspToeRearRad, 1e-7,
               "the rear-left wheel carries only its static toe");
    check_near(state.wheels[WHEEL_REAR_RIGHT].steerAngleRad, (double)spec.suspToeRearRad, 1e-7,
               "the rear-right wheel carries only its static toe, mirrored");
    const float beforeReturn = state.frontRoadWheelAngleRad;
    physics_update_steering(&spec, &state, 0.0f, 0.01f);
    check_near(beforeReturn - state.frontRoadWheelAngleRad, spec.steerReturnRateRadS * 0.01f,
               1e-6, "return-to-center uses its configured rate");
    for (int i = 0; i < 100; i++) physics_update_steering(&spec, &state, 1.0f, FIXED_DT_S);
    check_near(state.frontRoadWheelAngleRad, spec.maxRoadWheelAngleRad, 1e-6,
               "steering clamps at the maximum road-wheel angle");

    state.velocityLongitudinalMps = 10.0f;
    state.velocityLateralMps = 2.0f;
    state.yawRateRadS = 1.0f;
    const Vector2 flVelocity = physics_contact_point_velocity_body(
        &state, state.wheels[WHEEL_FRONT_LEFT].localPositionM);
    const Vector2 frVelocity = physics_contact_point_velocity_body(
        &state, state.wheels[WHEEL_FRONT_RIGHT].localPositionM);
    check(flVelocity.x < frVelocity.x,
          "yaw makes left/right longitudinal contact velocities distinct");
    check_near(flVelocity.y, 2.0 + spec.cgToFrontM, 1e-6,
               "front lateral contact velocity includes +lf*r");
    const Vector2 rearCenterVelocity =
        physics_contact_point_velocity_body(&state, (Vector2){ -spec.cgToRearM, 0.0f });
    check_near(rearCenterVelocity.y, 2.0 - spec.cgToRearM, 1e-6,
               "rear lateral contact velocity includes -lr*r");
    state.yawRateRadS = 0.0f;
    const Vector2 noYawVelocity = physics_contact_point_velocity_body(
        &state, state.wheels[WHEEL_FRONT_LEFT].localPositionM);
    check_near(noYawVelocity.x, state.velocityLongitudinalMps, 0.0,
               "zero yaw reduces contact longitudinal velocity to body velocity");
    check_near(noYawVelocity.y, state.velocityLateralMps, 0.0,
               "zero yaw reduces contact lateral velocity to body velocity");

    state.frontRoadWheelAngleRad = 0.0f;
    state.velocityLongitudinalMps = 10.0f;
    state.velocityLateralMps = 0.0f;
    float frontSlip;
    float rearSlip;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check_near(frontSlip, 0.0, 0.0, "straight travel has zero front slip");
    check_near(rearSlip, 0.0, 0.0, "straight travel has zero rear slip");
    state.velocityLateralMps = 1.0f;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check(frontSlip > 0.0f && rearSlip > 0.0f,
          "positive lateral velocity produces positive slip angles");
    state.velocityLateralMps = 0.0f;
    state.yawRateRadS = 0.5f;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check(frontSlip > 0.0f && rearSlip < 0.0f,
          "positive yaw separates front and rear slip signs");
    const float originalFrontSlip = frontSlip;
    VehicleSpec longerFront = spec;
    longerFront.cgToFrontM += 0.3f;
    longerFront.wheelbaseM = longerFront.cgToFrontM + longerFront.cgToRearM;
    physics_axle_slip_angles(&longerFront, &state, &frontSlip, &rearSlip);
    check(frontSlip > originalFrontSlip,
          "changing cgToFront changes the front axle slip response");
    state.velocityLongitudinalMps = -2.0f;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check(isfinite(frontSlip) && isfinite(rearSlip),
          "the documented reverse convention remains finite");

    check_near(tire_lateral_force_n(0.0f, 500.0f, 10.0f, 1.45f, 1.2f), 0.0, 0.0,
               "nonlinear tire force is zero at zero slip");
    check(tire_lateral_force_n(0.1f, 500.0f, 10.0f, 1.45f, 1.2f) < 0.0f,
          "positive lateral slip produces an opposing force");
    check(tire_lateral_force_n(-0.1f, 500.0f, 10.0f, 1.45f, 1.2f) > 0.0f,
          "negative lateral slip produces an opposing force");

    const Vector2 unrotated =
        physics_rotate_wheel_force_to_body((Vector2){ 12.0f, -34.0f }, 0.0f);
    check(unrotated.x == 12.0f && unrotated.y == -34.0f,
          "zero steering leaves the wheel-frame force unchanged");
    const Vector2 rotated =
        physics_rotate_wheel_force_to_body((Vector2){ 0.0f, 100.0f }, CIRCUIT_PI * 0.5f);
    check_near(rotated.x, -100.0, 1e-4,
               "positive steering rotates lateral force toward -body X");
    check_near(rotated.y, 0.0, 1e-4, "a ninety-degree force rotation has zero body Y");
    check_near(sqrtf(rotated.x * rotated.x + rotated.y * rotated.y), 100.0, 1e-4,
               "force rotation preserves magnitude");
    const Vector2 rotatedNegative =
        physics_rotate_wheel_force_to_body((Vector2){ 0.0f, 100.0f }, -0.25f);
    check(rotatedNegative.x > 0.0f && rotatedNegative.y > 0.0f,
          "negative steering rotates positive lateral force toward +body X");
    check(fabsf(tire_lateral_force_n(0.1f, 1000.0f, 20.0f, 1.45f, 1.0f)) >
              fabsf(tire_lateral_force_n(0.1f, 1000.0f, 5.0f, 1.45f, 1.0f)),
          "front/rear stiffness parameters can act independently");
    check(fabsf(tire_lateral_force_n(2.0f, 500.0f, 10.0f, 1.45f, 0.8f)) <
              fabsf(tire_lateral_force_n(2.0f, 500.0f, 10.0f, 1.45f, 1.2f)),
          "front/rear friction parameters set independent saturation limits");

    check_near(physics_low_speed_blend(LOW_SPEED_BEGIN_MPS - 0.001f), 0.0, 0.0,
               "low-speed blend is kinematic below its lower endpoint");
    check(physics_low_speed_blend(2.25f) > 0.0f && physics_low_speed_blend(2.25f) < 1.0f,
          "low-speed blend is continuous inside the transition");
    check_near(physics_low_speed_blend(LOW_SPEED_END_MPS + 0.001f), 1.0, 0.0,
               "low-speed blend is dynamic above its upper endpoint");

    VehicleRenderState wrapState;
    memset(&wrapState, 0, sizeof(wrapState));
    wrapState.prevHeadingRad = CIRCUIT_PI - 0.1f;
    wrapState.currHeadingRad = -CIRCUIT_PI + 0.1f;
    wrapState.prevWheelAngleRad[0] = CIRCUIT_PI - 0.2f;
    wrapState.currWheelAngleRad[0] = -CIRCUIT_PI + 0.2f;
    const VehicleDrawState draw = render_interpolate_vehicle(&wrapState, 0.5f);
    check(fabsf(fabsf(draw.headingRad) - CIRCUIT_PI) < 1e-4f,
          "render heading interpolation takes the shortest path across angle wrap");
    check(fabsf(fabsf(draw.wheelAngleRad[0]) - CIRCUIT_PI) < 1e-4f,
          "wheel interpolation also takes the shortest wrapped path");
}

static void scenario_tire(void)
{
    const float loadN = 3000.0f;
    const float mu = 1.2f;
    const float b = 10.0f;
    const float c = 1.45f;

    check_near(tire_normalized_curve(b, c, 0.0f), 0.0, 0.0,
               "normalized curve is zero at zero slip");
    check_near(tire_lateral_force_n(0.0f, loadN, b, c, mu), 0.0, 0.0,
               "lateral force is zero at zero slip");
    const float positiveFy = tire_lateral_force_n(0.1f, loadN, b, c, mu);
    const float negativeFy = tire_lateral_force_n(-0.1f, loadN, b, c, mu);
    check(positiveFy < 0.0f && negativeFy > 0.0f,
          "lateral force opposes positive and negative slip");
    check_near(positiveFy, -negativeFy, 0.001, "lateral curve is sign symmetric");
    {
        const float epsilon = 1e-5f;
        const float measuredSlope = tire_lateral_force_n(epsilon, loadN, b, c, mu) / epsilon;
        check_near(measuredSlope, -mu * loadN * b * c, 2.0,
                   "small-slip lateral slope matches -mu*Fz*B*C");
    }
    {
        float peak = 0.0f;
        float peakSlip = 0.0f;
        for (int i = 0; i <= 20000; i++) {
            const float slip = (float)i * 0.0001f;
            const float force = fabsf(tire_lateral_force_n(slip, loadN, b, c, mu));
            if (force > peak) {
                peak = force;
                peakSlip = slip;
            }
        }
        check_near(peak, mu * loadN, 1.0, "lateral peak is approximately mu*Fz");
        const float postPeak = fabsf(tire_lateral_force_n(4.0f, loadN, b, c, mu));
        check(postPeak < peak && postPeak > 0.5f * peak,
              "lateral force falls after the peak without collapsing");
        check(peakSlip > 0.0f && peakSlip < 1.0f,
              "lateral peak occurs at a finite positive slip");
    }
    check(isfinite(tire_lateral_force_n(1000.0f, loadN, b, c, mu)) &&
              isfinite(tire_lateral_force_n(-1000.0f, loadN, b, c, mu)),
          "large positive and negative lateral slips remain finite");
    check(fabsf(tire_lateral_force_n(0.1f, loadN, 5.0f, c, mu)) <
              fabsf(tire_lateral_force_n(0.1f, loadN, 15.0f, c, mu)),
          "independent lateral B parameters change force buildup");
    check(fabsf(tire_lateral_force_n(0.2f, loadN, b, c, 1.0f)) <
              fabsf(tire_lateral_force_n(0.2f, loadN, b, c, 1.3f)),
          "independent lateral mu parameters change force magnitude");

    check_near(tire_longitudinal_force_n(0.0f, loadN, TIRE_B_LONG, TIRE_C_LONG, 1.0f), 0.0, 0.0,
               "longitudinal force is zero at zero slip");
    check(tire_longitudinal_force_n(0.1f, loadN, TIRE_B_LONG, TIRE_C_LONG, 1.0f) > 0.0f &&
              tire_longitudinal_force_n(-0.1f, loadN, TIRE_B_LONG, TIRE_C_LONG, 1.0f) < 0.0f,
          "longitudinal force follows slip-ratio sign");
    {
        float peak = 0.0f;
        for (int i = 0; i <= 20000; i++) {
            const float force = fabsf(tire_longitudinal_force_n(
                (float)i * 0.0001f, loadN, TIRE_B_LONG, TIRE_C_LONG, 1.0f));
            if (force > peak) peak = force;
        }
        check_near(peak, loadN, 1.0, "longitudinal peak is approximately mu*Fz");
    }

    check_near(tire_slip_ratio(10.0f / WHEEL_RADIUS_M, WHEEL_RADIUS_M, 10.0f,
                               SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               0.0, 1e-6, "free rolling has zero slip ratio");
    check(tire_slip_ratio(15.0f / WHEEL_RADIUS_M, WHEEL_RADIUS_M, 10.0f, SLIP_SPEED_EPSILON_MPS,
                          SLIP_RATIO_CLAMP) > 0.0f,
          "wheelspin has positive slip ratio");
    check_near(
        tire_slip_ratio(0.0f, WHEEL_RADIUS_M, 10.0f, SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
        -1.0, 1e-6, "locked forward braking has slip ratio -1");
    check_near(tire_slip_ratio(-10.0f / WHEEL_RADIUS_M, WHEEL_RADIUS_M, -10.0f,
                               SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               0.0, 1e-6, "free rolling in reverse has zero slip ratio");
    check_near(
        tire_slip_ratio(0.1f, WHEEL_RADIUS_M, 0.0f, SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
        0.1 * WHEEL_RADIUS_M / SLIP_SPEED_EPSILON_MPS, 1e-6,
        "denominator floor produces a small finite near-zero slip");
    check_near(tire_slip_ratio(1000.0f, WHEEL_RADIUS_M, 0.0f, SLIP_SPEED_EPSILON_MPS,
                               SLIP_RATIO_CLAMP),
               SLIP_RATIO_CLAMP, 0.0, "positive slip ratio clamps");
    check_near(tire_slip_ratio(-1000.0f, WHEEL_RADIUS_M, 0.0f, SLIP_SPEED_EPSILON_MPS,
                               SLIP_RATIO_CLAMP),
               -SLIP_RATIO_CLAMP, 0.0, "negative slip ratio clamps");

    {
        float fx;
        float fy;
        float usage;
        tire_apply_combined_limit(300.0f, 400.0f, 1000.0f, 1000.0f, &fx, &fy, &usage);
        check_near(fx, 300.0, 1e-6, "inside ellipse retains longitudinal force");
        check_near(fy, 400.0, 1e-6, "inside ellipse retains lateral force");
        check_near(usage, 0.5, 1e-6, "inside ellipse reports normalized usage");

        tire_apply_combined_limit(1000.0f, 1000.0f, 1000.0f, 1000.0f, &fx, &fy, &usage);
        check_near(fx, 1000.0 / sqrt(2.0), 0.01,
                   "diagonal saturation scales longitudinal force");
        check_near(fy, 1000.0 / sqrt(2.0), 0.01, "diagonal saturation scales lateral force");
        check_near(usage, 1.0, 0.0, "saturated usage is capped at one");
        check_near(fx / fy, 1.0, 1e-6, "combined limit preserves direction");

        tire_apply_combined_limit(1000.0f, 2000.0f, 1000.0f, 2000.0f, &fx, &fy, &usage);
        check_near(sqrt((fx / 1000.0) * (fx / 1000.0) + (fy / 2000.0) * (fy / 2000.0)), 1.0,
                   1e-6, "asymmetric limits saturate on their ellipse");

        tire_apply_combined_limit(100.0f, 100.0f, 0.0f, 0.0f, &fx, &fy, &usage);
        check(fx == 0.0f && fy == 0.0f && usage == 0.0f,
              "zero load produces finite zero combined forces");
    }
}

static void scenario_drivetrain(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);

    check_near(drivetrain_engine_torque_at_rpm(&spec, spec.engineIdleRpm),
               spec.engineTorqueCurveNm[0], 0.0, "engine curve at idle");
    check_near(drivetrain_engine_torque_at_rpm(&spec, spec.engineRedlineRpm),
               spec.engineTorqueCurveNm[ENGINE_CURVE_POINTS - 1], 0.0,
               "engine curve at redline");
    const float sampleStepRpm =
        (spec.engineRedlineRpm - spec.engineIdleRpm) / (float)(ENGINE_CURVE_POINTS - 1);
    check_near(
        drivetrain_engine_torque_at_rpm(&spec, spec.engineIdleRpm + 2.0f * sampleStepRpm),
        spec.engineTorqueCurveNm[2], 1e-5, "engine curve returns exact interior samples");
    check_near(
        drivetrain_engine_torque_at_rpm(&spec, spec.engineIdleRpm + 2.5f * sampleStepRpm),
        0.5f * (spec.engineTorqueCurveNm[2] + spec.engineTorqueCurveNm[3]), 1e-4,
        "engine curve interpolates sample midpoints");
    check_near(drivetrain_engine_torque_at_rpm(&spec, 0.0f), spec.engineTorqueCurveNm[0], 0.0,
               "engine curve clamps below idle");
    check_near(drivetrain_engine_torque_at_rpm(&spec, 99999.0f),
               spec.engineTorqueCurveNm[ENGINE_CURVE_POINTS - 1], 0.0,
               "engine curve clamps above redline");

    const float firstRatio = spec.gearRatios[0] * spec.finalDriveRatio;
    check_near(drivetrain_total_gear_ratio(&spec, 1), firstRatio, 1e-6,
               "first gear includes final drive");
    check_near(drivetrain_total_gear_ratio(&spec, -1),
               -spec.reverseGearRatio * spec.finalDriveRatio, 1e-6,
               "reverse gear ratio is negative");
    check_near(drivetrain_total_gear_ratio(&spec, 0), 0.0, 0.0, "neutral total ratio is zero");
    check_near(drivetrain_engine_rpm(&spec, 1, 20.0f),
               clampf(20.0f * firstRatio * 60.0f / CIRCUIT_TWO_PI, spec.engineIdleRpm,
                      spec.engineRedlineRpm),
               1e-4, "engine RPM derives from driven wheel speed and gearing");

    /* All four wheels are handed to the drivetrain; the layout decides which it drives. */
    const float restOmega[WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };
    const float noReaction[WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

    DrivetrainTorques neutral =
        drivetrain_calculate_torques(&spec, 0, restOmega, noReaction, 1.0f, 0.0f, 0.0f);
    check_near(neutral.drivelineTorqueNm, 0.0, 0.0, "neutral transmits no drive torque");
    DrivetrainTorques forward =
        drivetrain_calculate_torques(&spec, 1, restOmega, noReaction, 1.0f, 0.0f, 0.0f);
    DrivetrainTorques reverse =
        drivetrain_calculate_torques(&spec, -1, restOmega, noReaction, 1.0f, 0.0f, 0.0f);
    check(forward.drivelineTorqueNm > 0.0f && reverse.drivelineTorqueNm < 0.0f,
          "forward and reverse produce opposite driveline torque");
    check(forward.driveTorqueNm[WHEEL_FRONT_LEFT] == 0.0f &&
              forward.driveTorqueNm[WHEEL_FRONT_RIGHT] == 0.0f,
          "front wheels receive no drive torque");
    check_near(forward.driveTorqueNm[WHEEL_REAR_LEFT], forward.driveTorqueNm[WHEEL_REAR_RIGHT],
               0.0, "rear wheels receive equal drive torque");
    VehicleSpec halfEfficiency = spec;
    halfEfficiency.drivetrainEfficiency *= 0.5f;
    DrivetrainTorques half = drivetrain_calculate_torques(&halfEfficiency, 1, restOmega,
                                                          noReaction, 1.0f, 0.0f, 0.0f);
    check_near(half.drivelineTorqueNm, forward.drivelineTorqueNm * 0.5f, 1e-4,
               "drivetrain efficiency scales output torque");

    const float rollingOmega[WHEEL_COUNT] = { 10.0f, 10.0f, 10.0f, 10.0f };
    DrivetrainTorques braking =
        drivetrain_calculate_torques(&spec, 1, rollingOmega, noReaction, 0.0f, 1.0f, 1.0f);
    const float frontBrake = braking.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                             braking.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT];
    const float rearBrake = braking.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                            braking.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT];
    check_near(frontBrake + rearBrake, spec.maxBrakeTorqueNm, 1e-4,
               "full service-brake split sums to configured torque");
    check_near(frontBrake / spec.maxBrakeTorqueNm, spec.brakeBiasFront, 1e-6,
               "front service-brake torque follows bias");
    check(braking.handbrakeTorqueNm[WHEEL_FRONT_LEFT] == 0.0f &&
              braking.handbrakeTorqueNm[WHEEL_FRONT_RIGHT] == 0.0f,
          "handbrake is rear-only");
    check_near(braking.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
                   braking.handbrakeTorqueNm[WHEEL_REAR_RIGHT],
               spec.handbrakeTorqueNm, 1e-5, "full handbrake saturates at configured torque");

    bool locked = false;
    const float spun =
        drivetrain_integrate_wheel(0.0f, 0.0f, 120.0f, 0.0f, 0.0f, 0.0f, spec.wheelRadiusM,
                                   spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(spun > 0.0f && !locked, "drive torque spins a wheel forward");
    const float reactionBalanced = drivetrain_integrate_wheel(
        10.0f, 10.0f * spec.wheelRadiusM, 100.0f, 0.0f, 0.0f, 100.0f / spec.wheelRadiusM,
        spec.wheelRadiusM, spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check_near(reactionBalanced, 10.0, 1e-6,
               "tire reaction torque balances equal drive torque");
    const float braked =
        drivetrain_integrate_wheel(10.0f, 3.1f, 0.0f, 100.0f, 0.0f, 0.0f, spec.wheelRadiusM,
                                   spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(braked < 10.0f && braked >= 0.0f, "service-brake torque opposes positive rotation");
    const float stopped =
        drivetrain_integrate_wheel(0.1f, 3.1f, 0.0f, 1000.0f, 0.0f, 0.0f, spec.wheelRadiusM,
                                   spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(stopped == 0.0f && locked, "brake torque detects lock and cannot reverse the wheel");
    const float released =
        drivetrain_integrate_wheel(stopped, 3.1f, 0.0f, 0.0f, 0.0f, -1000.0f, spec.wheelRadiusM,
                                   spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(released > 0.0f && !locked, "tire reaction spins a locked wheel after brake release");
}

/* Put the vehicle at a steady speed with every wheel already rolling at it, so a scenario
 * can start from cruise instead of spending seconds accelerating into position. */
void set_vehicle_rolling_speed(Game *game, float velocityLongitudinalMps)
{
    game->vehicle.velocityLongitudinalMps = velocityLongitudinalMps;
    const float omega = velocityLongitudinalMps / game->spec.wheelRadiusM;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        game->vehicle.wheels[i].angularVelocityRadS = omega;
    }
}

/* The same, for the bare spec/state fixtures that do not own a Game. */
static void set_rolling_wheels(const VehicleSpec *spec, VehicleState *state,
                               float velocityLongitudinalMps)
{
    const float omega = velocityLongitudinalMps / spec->wheelRadiusM;
    for (int i = 0; i < WHEEL_COUNT; i++) state->wheels[i].angularVelocityRadS = omega;
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: accel-filter — the load-transfer acceleration filter, in isolation             */
/* ------------------------------------------------------------------------------------- */

static void scenario_accel_filter(void)
{
    const float rateHz = LOAD_FILTER_RATE_HZ;
    const float dt = FIXED_DT_S;
    const float alpha = 1.0f - expf(-rateHz * dt);

    /* Zero in, zero out, forever. */
    {
        float filtered = 0.0f;
        for (int i = 0; i < 600; i++)
            filtered = physics_filter_long_accel(filtered, 0.0f, rateHz, dt);
        check_near((double)filtered, 0.0, 0.0, "a zero input never moves the filter off zero");
    }

    /* One step matches the documented closed form exactly. */
    check_near((double)physics_filter_long_accel(0.0f, 4.0f, rateHz, dt), 4.0 * (double)alpha,
               1e-6, "one filter step equals (previous - filtered) * (1 - exp(-rate*dt))");

    /* Positive step: monotonic rise, no overshoot, converging on the input. */
    {
        float filtered = 0.0f;
        float previous = -1.0f;
        bool monotonic = true;
        bool bounded = true;
        for (int i = 0; i < 600; i++) {
            filtered = physics_filter_long_accel(filtered, 5.0f, rateHz, dt);
            if (filtered < previous) monotonic = false;
            if (filtered > 5.0f + 1e-6f) bounded = false;
            previous = filtered;
        }
        check(monotonic, "a positive step converges monotonically");
        check(bounded, "a first-order filter never overshoots its input");
        check_near((double)filtered, 5.0, 1e-4, "it converges on the step value");
    }

    /* Negative step (braking) is the mirror image. */
    {
        float filtered = 0.0f;
        float previous = 1.0f;
        bool monotonic = true;
        for (int i = 0; i < 600; i++) {
            filtered = physics_filter_long_accel(filtered, -8.0f, rateHz, dt);
            if (filtered > previous) monotonic = false;
            previous = filtered;
        }
        check(monotonic, "a negative step converges monotonically downward");
        check_near((double)filtered, -8.0, 1e-4, "it converges on the braking value");
    }

    /* Rate ordering: a faster filter is always further along at the same time. */
    {
        float slow = 0.0f;
        float fast = 0.0f;
        bool ordered = true;
        for (int i = 0; i < 120; i++) {
            slow = physics_filter_long_accel(slow, 5.0f, 5.0f, dt);
            fast = physics_filter_long_accel(fast, 5.0f, 40.0f, dt);
            if (fast < slow - 1e-6f) ordered = false;
        }
        check(ordered, "a higher filter rate tracks the input at least as closely");
    }

    /* Degenerate arguments leave the state alone rather than poisoning it. */
    check_near((double)physics_filter_long_accel(2.0f, 5.0f, rateHz, 0.0f), 2.0, 0.0,
               "a zero timestep is a no-op");
    check_near((double)physics_filter_long_accel(2.0f, 5.0f, 0.0f, dt), 2.0, 0.0,
               "a zero rate is a no-op");
    check(isfinite(physics_filter_long_accel(2.0f, (float)NAN, rateHz, dt)),
          "a non-finite input cannot produce a non-finite filter state");

    /* Determinism: the same sequence twice is bit-identical. */
    {
        float a = 0.0f;
        float b = 0.0f;
        for (int i = 0; i < 240; i++) {
            const float input = (i < 120) ? 3.0f : -6.0f;
            a = physics_filter_long_accel(a, input, rateHz, dt);
            b = physics_filter_long_accel(b, input, rateHz, dt);
        }
        check(memcmp(&a, &b, sizeof(float)) == 0, "the filter is bit-deterministic");
    }

    /* The whole-vehicle wiring: the filter must consume the PREVIOUS step's solved value,
     * so a first tick from rest sees zero acceleration however hard the throttle is held. */
    {
        Game *game = alloc_game();
        game_init(game);
        game->input.throttle = 1.0f;
        game_fixed_update(game, FIXED_DT_S);
        check_near((double)game->derived.previousLongAccelMps2, 0.0, 0.0,
                   "the first tick filters the previous step's zero, not its own acceleration");
        check_near((double)game->derived.filteredLongAccelMps2, 0.0, 0.0,
                   "so the filtered value is still zero after one tick");
        check_near((double)game->derived.staticFrontLoadN,
                   (double)game->derived.normalLoadFrontN, 1e-3,
                   "and the first tick's load is the pure static split");

        const float solvedFirst = game->derived.solvedLongAccelMps2;
        game_fixed_update(game, FIXED_DT_S);
        check_near((double)game->derived.previousLongAccelMps2, (double)solvedFirst, 0.0,
                   "the next tick filters exactly the previous tick's solved acceleration");
        check_near((double)game->derived.filteredLongAccelMps2, (double)(solvedFirst * alpha),
                   1e-6, "one filter step of that value, matching the closed form");

        /* A reset clears both halves of the filter state. */
        for (int i = 0; i < 240; i++) game_fixed_update(game, FIXED_DT_S);
        check(fabsf(game->vehicle.filteredLongAccelMps2) > 0.1f,
              "the filter has accumulated history to clear (%.4f)",
              (double)game->vehicle.filteredLongAccelMps2);
        game_reset_sim(game);
        check_near((double)game->vehicle.filteredLongAccelMps2, 0.0, 0.0,
                   "reset zeroes the filtered acceleration");
        check_near((double)game->vehicle.prevLongAccelMps2, 0.0, 0.0,
                   "reset zeroes the previous acceleration");
        free(game);
    }

    /* No yaw or lateral contamination: the stored value is ax_body, not dvx_dt, so pure
     * rotation with a lateral velocity must not register as longitudinal acceleration. */
    {
        VehicleSpec spec;
        VehicleState state;
        VehicleDerived derived;
        VehicleRenderState renderState;
        phase1_fixture(&spec, &state, &derived, &renderState);
        ControllerOutput input;
        controller_output_zero(&input);

        state.velocityLongitudinalMps = 10.0f;
        state.velocityLateralMps = 0.0f;
        state.yawRateRadS = 0.0f;
        set_rolling_wheels(&spec, &state, 10.0f);
        physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL, &input,
                             FIXED_DT_S);
        const float straightAx = derived.solvedLongAccelMps2;
        check_near((double)straightAx, (double)(derived.totalBodyForceN.x / spec.massKg), 0.0,
                   "solved acceleration is exactly totalBodyForceX / mass");

        VehicleState rotating;
        VehicleDerived rotatingDerived;
        VehicleRenderState rotatingRender;
        vehicle_state_reset(&spec, &rotating, &rotatingDerived, &rotatingRender);
        rotating.velocityLongitudinalMps = 10.0f;
        rotating.velocityLateralMps = 2.0f;
        rotating.yawRateRadS = 0.5f;
        set_rolling_wheels(&spec, &rotating, 10.0f);
        physics_fixed_update(&spec, &rotating, &rotatingDerived, &rotatingRender, NULL, NULL,
                             NULL, &input, FIXED_DT_S);

        /* dvx_dt carries r*vy = 0.5 * 2.0 = 1.0 m/s^2. The stored value must not. */
        const float transportTermMps2 = rotating.yawRateRadS * 2.0f;
        check(fabsf(rotatingDerived.solvedLongAccelMps2 - straightAx) <
                  0.5f * transportTermMps2,
              "the stored acceleration excludes the r*vy transport term "
              "(straight %.4f, rotating %.4f, transport %.4f m/s^2)",
              (double)straightAx, (double)rotatingDerived.solvedLongAccelMps2,
              (double)transportTermMps2);
        check_near((double)rotatingDerived.solvedLongAccelMps2,
                   (double)rotating.prevLongAccelMps2, 0.0,
                   "the diagnostic and the stored state are the same number");
        check_near((double)rotatingDerived.solvedLongAccelMps2,
                   (double)(rotatingDerived.totalBodyForceN.x / spec.massKg), 0.0,
                   "yaw and lateral transport cannot contaminate the force acceleration");
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: load-transfer — static and dynamic axle loads                                  */
/* ------------------------------------------------------------------------------------- */

static void scenario_load_transfer(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    const float weightN = spec.massKg * GRAVITY_MPS2;

    /* Every case below is solved at zero road speed, so the aerodynamic vertical term is
     * exactly zero and these remain assertions about the static split and the longitudinal
     * transfer alone. The aero contribution has its own scenario (`aero-loads`). */

    /* Static distribution follows the CG position: a CG nearer the front axle puts more
     * weight on it, which is l_r / L, not l_f / L. */
    {
        const AxleLoads loads = physics_axle_loads(&spec, 0.0f, 0.0f);
        check_near((double)loads.staticFrontN,
                   (double)(weightN * spec.cgToRearM / spec.wheelbaseM), 0.01,
                   "static front load is m*g*l_r/L");
        check_near((double)loads.staticRearN,
                   (double)(weightN * spec.cgToFrontM / spec.wheelbaseM), 0.01,
                   "static rear load is m*g*l_f/L");
        check_near((double)(loads.staticFrontN + loads.staticRearN), (double)weightN, 0.01,
                   "static loads sum to mass * gravity");
        check_near((double)loads.transferN, 0.0, 1e-4, "zero acceleration transfers no load");
        check_near((double)loads.frontN, (double)loads.staticFrontN, 1e-3,
                   "and the dynamic front load is the static one");
        check(loads.staticFrontN > loads.staticRearN,
              "the default CG sits ahead of centre, loading the front axle more "
              "(%.1f vs %.1f N)",
              (double)loads.staticFrontN, (double)loads.staticRearN);
    }

    /* Moving the CG forward and back must move the static split, both ways. */
    {
        VehicleSpec forward = spec;
        forward.cgToFrontM = 0.90f;
        forward.cgToRearM = 1.65f;
        forward.wheelbaseM = forward.cgToFrontM + forward.cgToRearM;
        const AxleLoads front = physics_axle_loads(&forward, 0.0f, 0.0f);

        VehicleSpec rearward = spec;
        rearward.cgToFrontM = 1.65f;
        rearward.cgToRearM = 0.90f;
        rearward.wheelbaseM = rearward.cgToFrontM + rearward.cgToRearM;
        const AxleLoads rear = physics_axle_loads(&rearward, 0.0f, 0.0f);

        check(front.staticFrontN > rear.staticFrontN + 100.0f,
              "moving the CG forward increases the static front load (%.1f -> %.1f N)",
              (double)rear.staticFrontN, (double)front.staticFrontN);
        check(rear.staticRearN > front.staticRearN + 100.0f,
              "moving the CG rearward increases the static rear load (%.1f -> %.1f N)",
              (double)front.staticRearN, (double)rear.staticRearN);
        check_near((double)(front.staticFrontN + front.staticRearN), (double)weightN, 0.01,
                   "a forward CG still weighs mass * gravity");
        check_near((double)(rear.staticFrontN + rear.staticRearN), (double)weightN, 0.01,
                   "a rearward CG still weighs mass * gravity");
    }

    /* Acceleration transfers rearward, braking forward, and the transfer is exactly
     * m * ax * h / L in both directions. */
    {
        const float axMps2 = 5.0f;
        const AxleLoads accelerating = physics_axle_loads(&spec, axMps2, 0.0f);
        const float expectedN = spec.massKg * axMps2 * spec.cgHeightM / spec.wheelbaseM;
        check_near((double)accelerating.transferN, (double)expectedN, 0.01,
                   "load transfer is m * ax * h / L");
        check(accelerating.unclampedFrontN < accelerating.staticFrontN,
              "accelerating unloads the front axle (%.1f -> %.1f N)",
              (double)accelerating.staticFrontN, (double)accelerating.unclampedFrontN);
        check(accelerating.unclampedRearN > accelerating.staticRearN,
              "accelerating loads the rear axle (%.1f -> %.1f N)",
              (double)accelerating.staticRearN, (double)accelerating.unclampedRearN);
        check_near((double)(accelerating.unclampedFrontN + accelerating.unclampedRearN),
                   (double)weightN, 0.01,
                   "transfer moves load without creating or destroying any");

        const AxleLoads braking = physics_axle_loads(&spec, -axMps2, 0.0f);
        check_near((double)braking.transferN, -(double)expectedN, 0.01,
                   "braking transfer is the exact mirror of accelerating transfer");
        check(braking.unclampedFrontN > braking.staticFrontN,
              "braking loads the front axle (%.1f -> %.1f N)", (double)braking.staticFrontN,
              (double)braking.unclampedFrontN);
        check(braking.unclampedRearN < braking.staticRearN,
              "braking unloads the rear axle (%.1f -> %.1f N)", (double)braking.staticRearN,
              (double)braking.unclampedRearN);
        check_near((double)(braking.unclampedFrontN + braking.unclampedRearN), (double)weightN,
                   0.01, "and still sums to mass * gravity");
    }

    /* CG height scales the transfer and nothing else. */
    {
        VehicleSpec low = spec;
        low.cgHeightM = 0.25f;
        VehicleSpec high = spec;
        high.cgHeightM = 0.75f;
        const AxleLoads lowLoads = physics_axle_loads(&low, 5.0f, 0.0f);
        const AxleLoads highLoads = physics_axle_loads(&high, 5.0f, 0.0f);
        check_near((double)highLoads.transferN, 3.0 * (double)lowLoads.transferN, 0.05,
                   "transfer is linear in CG height");
        check_near((double)lowLoads.staticFrontN, (double)highLoads.staticFrontN, 0.01,
                   "CG height does not change the static split");
    }

    /* The minimum-load clamp catches an unloaded axle without renormalising the other. */
    {
        const AxleLoads extreme = physics_axle_loads(&spec, 40.0f, 0.0f);
        check(extreme.unclampedFrontN < 0.0f,
              "an extreme acceleration drives the unclamped front load negative (%.1f N)",
              (double)extreme.unclampedFrontN);
        check_near((double)extreme.frontN, (double)MIN_NORMAL_LOAD_N, 1e-3,
                   "the clamped front load stops at MIN_NORMAL_LOAD_N");
        check_near((double)extreme.rearN, (double)extreme.unclampedRearN, 0.01,
                   "the rear axle is not renormalised to absorb the clamped difference");
        check_near((double)(extreme.unclampedFrontN + extreme.unclampedRearN), (double)weightN,
                   0.01, "the unclamped pair still sums to mass * gravity after clamping");

        const AxleLoads reverse = physics_axle_loads(&spec, -40.0f, 0.0f);
        check_near((double)reverse.rearN, (double)MIN_NORMAL_LOAD_N, 1e-3,
                   "extreme braking clamps the rear load instead");

        check(isfinite(extreme.frontN) && isfinite(extreme.rearN) && isfinite(reverse.frontN) &&
                  isfinite(reverse.rearN),
              "extreme but finite acceleration produces finite loads");
        check(extreme.frontN > 0.0f && extreme.rearN > 0.0f && reverse.frontN > 0.0f &&
                  reverse.rearN > 0.0f,
              "no clamped load is ever negative, so no wheel can generate negative grip");
    }

    /* Load reaches the tires: capacity follows the dynamic load, split evenly per axle. */
    {
        Game *game = alloc_game();
        game_init(game);
        game->input.throttle = 1.0f;
        for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);

        check(game->derived.loadTransferN > 0.0f,
              "full throttle produces a positive (rearward) transfer (%.1f N)",
              (double)game->derived.loadTransferN);
        check(game->derived.normalLoadFrontN < game->derived.staticFrontLoadN,
              "the front axle is carrying less than its static load");
        check(game->derived.normalLoadRearN > game->derived.staticRearLoadN,
              "the rear axle is carrying more than its static load");
        check_near((double)(game->vehicle.wheels[WHEEL_FRONT_LEFT].normalLoadN +
                            game->vehicle.wheels[WHEEL_FRONT_RIGHT].normalLoadN),
                   (double)game->derived.normalLoadFrontN, 1e-2,
                   "the two front wheel loads sum to the front axle load");
        check_near((double)(game->vehicle.wheels[WHEEL_REAR_LEFT].normalLoadN +
                            game->vehicle.wheels[WHEEL_REAR_RIGHT].normalLoadN),
                   (double)game->derived.normalLoadRearN, 1e-2,
                   "the two rear wheel loads sum to the rear axle load");
        check_near((double)game->vehicle.wheels[WHEEL_FRONT_LEFT].normalLoadN,
                   (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].normalLoadN, 0.0,
                   "left and right stay equal: lateral load transfer is Phase 4");

        /* Friction capacity is mu * Fz, so a heavier axle can make more force. */
        const float rearCapacityN = game->spec.tireMuLatRear * game->derived.normalLoadRearN;
        const float staticRearCapacityN =
            game->spec.tireMuLatRear * game->derived.staticRearLoadN;
        check(rearCapacityN > staticRearCapacityN,
              "the loaded rear axle has more lateral capacity than at rest (%.0f > %.0f N)",
              (double)rearCapacityN, (double)staticRearCapacityN);
        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: aero-loads — speed-squared aerodynamic vertical load and balance (issue #17)   */
/* ------------------------------------------------------------------------------------- */

/*
 * The authored lift coefficients and reference areas were inert: a wing sized the sprite and
 * nothing else. They now produce a vertical load on their own axle. What is asserted here:
 *
 *   1. ZERO IS ZERO. No coefficient, or no speed, and the axle loads are bit-identical to the
 *      pure static-plus-transfer solution. This is the criterion that makes the whole feature
 *      safe to add to an existing baseline: a car with no aero is unchanged.
 *   2. SPEED SQUARED. Doubling the speed quadruples the load, exactly.
 *   3. SIGN. A positive (lift) coefficient unloads its axle; a negative one (a wing) loads it.
 *      Each coefficient reaches ONLY its own axle, which is what makes the balance authorable.
 *   4. CAPACITY AND FLOOR. Downforce raises the tire force available at speed; enough lift
 *      drives the unclamped load negative, and the clamp still refuses to hand a wheel a
 *      negative contact load.
 */
static void scenario_aero_loads(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);

    /* 1. A car with no aero devices is the old model exactly, at any speed. */
    {
        VehicleSpec neutral = spec;
        neutral.aeroLiftCoefFront = 0.0f;
        neutral.aeroLiftCoefRear = 0.0f;
        const AxleLoads still = physics_axle_loads(&neutral, 1.5f, 0.0f);
        const AxleLoads fast = physics_axle_loads(&neutral, 1.5f, 80.0f);
        check(still.frontN == fast.frontN && still.rearN == fast.rearN,
              "zero lift coefficients make speed irrelevant to axle load (%.6f / %.6f N)",
              (double)fast.frontN, (double)fast.rearN);
        check(fast.aeroFrontN == 0.0f && fast.aeroRearN == 0.0f,
              "and report no aerodynamic load at all");

        /* Speed alone, with the stock coefficients, is what turns the term on. */
        const AxleLoads stockStill = physics_axle_loads(&spec, 1.5f, 0.0f);
        check(stockStill.aeroFrontN == 0.0f && stockStill.aeroRearN == 0.0f,
              "a stationary car generates no aerodynamic vertical load either");
    }

    /* 2. Speed squared, checked against the closed form and against itself. */
    {
        VehicleSpec wing = spec;
        wing.aeroLiftCoefFront = -0.8f;
        wing.aeroLiftCoefRear = -1.6f;
        float frontN = 0.0f, rearN = 0.0f;
        physics_aero_vertical_loads(&wing, 40.0f, &frontN, &rearN);
        const float expectedFrontN = -0.5f * AIR_DENSITY_KGM3 * wing.aeroLiftCoefFront *
                                     wing.aeroRefAreaFrontM2 * 40.0f * 40.0f;
        const float expectedRearN = -0.5f * AIR_DENSITY_KGM3 * wing.aeroLiftCoefRear *
                                    wing.aeroRefAreaRearM2 * 40.0f * 40.0f;
        check_near((double)frontN, (double)expectedFrontN, 0.01,
                   "front downforce is -0.5*rho*Cl*A*v^2");
        check_near((double)rearN, (double)expectedRearN, 0.01,
                   "rear downforce uses its own coefficient and its own area");

        float halfFrontN = 0.0f, halfRearN = 0.0f;
        physics_aero_vertical_loads(&wing, 20.0f, &halfFrontN, &halfRearN);
        check_near((double)frontN, 4.0 * (double)halfFrontN, 0.05,
                   "halving the speed quarters the front load");
        check_near((double)rearN, 4.0 * (double)halfRearN, 0.05,
                   "halving the speed quarters the rear load");
    }

    /* 3. Sign, and axle isolation. */
    {
        VehicleSpec frontWing = spec;
        frontWing.aeroLiftCoefFront = -1.0f;
        frontWing.aeroLiftCoefRear = 0.0f;
        const AxleLoads loads = physics_axle_loads(&frontWing, 0.0f, 50.0f);
        check(loads.aeroFrontN > 0.0f,
              "a negative front coefficient is downforce: it LOADS the front axle (%.1f N)",
              (double)loads.aeroFrontN);
        check(loads.aeroRearN == 0.0f,
              "and it reaches only the front axle, so aero balance is authorable");
        check(loads.frontN > loads.staticFrontN + 100.0f,
              "the front axle carries more than its static load at speed (%.1f > %.1f N)",
              (double)loads.frontN, (double)loads.staticFrontN);
        check_near((double)loads.rearN, (double)loads.staticRearN, 0.01,
                   "while the rear axle carries exactly its static load");

        VehicleSpec frontLift = spec;
        frontLift.aeroLiftCoefFront = 1.0f;
        frontLift.aeroLiftCoefRear = 0.0f;
        const AxleLoads lifted = physics_axle_loads(&frontLift, 0.0f, 50.0f);
        check(lifted.aeroFrontN < 0.0f,
              "a positive front coefficient is lift: it UNLOADS the front axle (%.1f N)",
              (double)lifted.aeroFrontN);
        check_near((double)lifted.aeroFrontN, -(double)loads.aeroFrontN, 0.01,
                   "equal and opposite coefficients give equal and opposite loads");
    }

    /* 4. The vertical sum closes, downforce buys grip, and lift cannot go through the floor. */
    {
        VehicleSpec wing = spec;
        wing.aeroLiftCoefFront = -1.2f;
        wing.aeroLiftCoefRear = -2.0f;
        const AxleLoads loads = physics_axle_loads(&wing, 3.0f, 60.0f);
        const float weightN = wing.massKg * GRAVITY_MPS2;
        check_near((double)(loads.unclampedFrontN + loads.unclampedRearN),
                   (double)(weightN + loads.aeroFrontN + loads.aeroRearN), 0.05,
                   "the unclamped pair sums to weight plus the two aerodynamic loads");
        check(loads.aeroFrontN + loads.aeroRearN > 1000.0f,
              "a real wing package is worth more than a kilonewton at 60 m/s (%.0f N)",
              (double)(loads.aeroFrontN + loads.aeroRearN));

        /* Enough lift to fly: the model must report the negative unclamped load honestly and
         * still refuse to give a wheel negative grip. */
        VehicleSpec flying = spec;
        flying.aeroLiftCoefFront = 1.0f;
        flying.aeroRefAreaFrontM2 = 2.0f;
        const AxleLoads airborne = physics_axle_loads(&flying, 0.0f, 100.0f);
        check(airborne.unclampedFrontN < 0.0f,
              "extreme lift drives the unclamped front load negative (%.1f N)",
              (double)airborne.unclampedFrontN);
        check_near((double)airborne.frontN, (double)MIN_NORMAL_LOAD_N, 1e-3,
                   "and the clamp still stops the front load at MIN_NORMAL_LOAD_N");
    }

    /* 4b. An unrepresentable wing is refused, and a merely enormous one is reported rather than
     * silently rounded to "no aerodynamics at all". Computing the product in float would
     * overflow to infinity and the old fallback turned that into zero, which is the one answer
     * a car with the most downforce in the roster must never get. */
    {
        VehicleSpec absurd = spec;
        absurd.aeroLiftCoefFront = -FLT_MAX;
        absurd.aeroRefAreaFrontM2 = 2.0f;
        check(!vehicle_spec_is_valid(&absurd),
              "a lift coefficient whose load cannot be represented is refused at load time");

        /* Large enough that a float-only product overflows partway through (rho*Cl*A alone is
         * about 1e35 and the v^2 factor is another 1e4), small enough to remain representable
         * in the double the model now uses. */
        VehicleSpec enormous = spec;
        enormous.aeroLiftCoefFront = -1.0e34f;
        enormous.aeroRefAreaFrontM2 = 2.0f;
        check(vehicle_spec_is_valid(&enormous),
              "an enormous but representable wing is still valid content");
        float enormousFrontN = 0.0f;
        physics_aero_vertical_loads(&enormous, 100.0f, &enormousFrontN, NULL);
        check(isfinite(enormousFrontN) && enormousFrontN > 1.0e30f,
              "and it reports its downforce instead of collapsing to zero (%.3e N)",
              (double)enormousFrontN);
    }

    /* 5. It reaches the running solver, not only the helper: at speed the reported axle loads
     * differ from the static split by exactly the reported aerodynamic term. */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.aeroLiftCoefFront = -1.0f;
        game->spec.aeroLiftCoefRear = -1.8f;
        set_vehicle_rolling_speed(game, 55.0f);
        game->input.throttle = 0.0f;
        for (int i = 0; i < 10; i++) game_fixed_update(game, FIXED_DT_S);

        check(game->derived.aeroVerticalFrontN > 100.0f &&
                  game->derived.aeroVerticalRearN > 100.0f,
              "the solver reports downforce on both axles at 55 m/s (%.0f / %.0f N)",
              (double)game->derived.aeroVerticalFrontN,
              (double)game->derived.aeroVerticalRearN);
        check_near(
            (double)(game->derived.unclampedFrontLoadN + game->derived.unclampedRearLoadN),
            (double)(game->spec.massKg * GRAVITY_MPS2 + game->derived.aeroVerticalFrontN +
                     game->derived.aeroVerticalRearN),
            1.0, "the solver's own load solution closes on weight plus downforce");
        check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
              "a heavily winged car at speed is still a valid state");
        free(game);
    }

    /* 6. Balance moves high-speed handling the way a race engineer expects: the same total
     * downforce placed forward makes the car turn in harder, placed rearward makes it push.
     * Compared as the axle slip-angle difference, which is the sign of understeer itself, so
     * the check reads the balance rather than a proxy for it. */
    {
        const float coefficients[2][2] = { { -1.6f, -0.4f }, { -0.4f, -1.6f } };
        float balanceRad[2] = { 0.0f, 0.0f };
        for (int b = 0; b < 2; b++) {
            Game *game = alloc_game();
            game_init(game);
            game->spec.aeroLiftCoefFront = coefficients[b][0];
            game->spec.aeroLiftCoefRear = coefficients[b][1];
            set_vehicle_rolling_speed(game, 50.0f);
            game->input.steer = 0.12f;
            game->input.throttle = 0.20f;
            for (int i = 0; i < 240; i++) game_fixed_update(game, FIXED_DT_S);
            /* Positive = the front is sliding more than the rear = understeer. */
            balanceRad[b] =
                fabsf(game->derived.frontSlipAngleRad) - fabsf(game->derived.rearSlipAngleRad);
            free(game);
        }
        check(balanceRad[0] < balanceRad[1],
              "moving the same downforce rearward pushes the balance toward understeer "
              "(front-biased %.4f rad vs rear-biased %.4f rad)",
              (double)balanceRad[0], (double)balanceRad[1]);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: resistance — aerodynamic drag and rolling resistance                           */
/* ------------------------------------------------------------------------------------- */

static void scenario_resistance(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);

    const float referenceN =
        0.5f * AIR_DENSITY_KGM3 * spec.dragCoefficient * spec.frontalAreaM2;

    /* ------------------------------------------------------------------ aerodynamic drag -- */

    {
        float magnitudeN = -1.0f;
        const Vector2 atRest = physics_aero_drag_body_n(&spec, 0.0f, 0.0f, &magnitudeN);
        check(atRest.x == 0.0f && atRest.y == 0.0f, "drag is exactly zero at rest");
        check_near((double)magnitudeN, 0.0, 0.0, "and reports zero magnitude");
    }

    {
        float magnitudeN = 0.0f;
        const Vector2 forward = physics_aero_drag_body_n(&spec, 20.0f, 0.0f, &magnitudeN);
        check_near((double)magnitudeN, (double)(referenceN * 400.0f), 0.01,
                   "drag magnitude is 0.5 * rho * Cd * A * v^2");
        check(forward.x < 0.0f, "forward motion produces rearward drag (%.2f N)",
              (double)forward.x);
        check_near((double)forward.y, 0.0, 1e-6, "and no lateral component");

        const Vector2 reverse = physics_aero_drag_body_n(&spec, -20.0f, 0.0f, NULL);
        check(reverse.x > 0.0f, "reverse motion produces forward drag (%.2f N)",
              (double)reverse.x);
        check_near((double)reverse.x, -(double)forward.x, 1e-3,
                   "drag is symmetric in the direction of travel");
    }

    {
        /* A car travelling sideways still pushes air. This is the check that catches drag
         * being applied along body X regardless of where the car is actually going. */
        float magnitudeN = 0.0f;
        const Vector2 sideways = physics_aero_drag_body_n(&spec, 0.0f, 15.0f, &magnitudeN);
        check_near((double)magnitudeN, (double)(referenceN * 225.0f), 0.01,
                   "pure lateral motion produces the same quadratic magnitude");
        check(sideways.y < 0.0f && fabsf(sideways.x) < 1e-6f,
              "and it acts purely along -body Y (%.2f, %.2f)", (double)sideways.x,
              (double)sideways.y);
    }

    {
        /* Diagonal motion: the force is antiparallel to the velocity vector. */
        const float vx = 12.0f;
        const float vy = -9.0f; /* speed 15 */
        float magnitudeN = 0.0f;
        const Vector2 diagonal = physics_aero_drag_body_n(&spec, vx, vy, &magnitudeN);
        const float speedMps = sqrtf(vx * vx + vy * vy);
        check_near((double)magnitudeN, (double)(referenceN * speedMps * speedMps), 0.01,
                   "diagonal drag uses the full speed, not just the forward component");
        check_near((double)diagonal.x, (double)(-magnitudeN * vx / speedMps), 1e-3,
                   "the X component follows the velocity direction");
        check_near((double)diagonal.y, (double)(-magnitudeN * vy / speedMps), 1e-3,
                   "the Y component follows the velocity direction");
        const float dot = diagonal.x * vx + diagonal.y * vy;
        check(dot < 0.0f, "drag opposes the velocity vector (dot %.3f)", (double)dot);
        check_near((double)sqrtf(diagonal.x * diagonal.x + diagonal.y * diagonal.y),
                   (double)magnitudeN, 1e-2, "and its length is the reported magnitude");
    }

    {
        /* Quadratic scaling: doubling speed quadruples drag. */
        float lowN = 0.0f;
        float highN = 0.0f;
        physics_aero_drag_body_n(&spec, 10.0f, 0.0f, &lowN);
        physics_aero_drag_body_n(&spec, 20.0f, 0.0f, &highN);
        check(fabsf(highN - 4.0f * lowN) < 0.01f,
              "drag is quadratic in speed (%.2f N at 10 m/s -> %.2f N at 20 m/s)", (double)lowN,
              (double)highN);
    }

    {
        VehicleSpec upright = spec;
        VehicleSpec raked = spec;
        upright.windscreenRakeRad = VEH_WINDSCREEN_RAKE_MIN_RAD;
        raked.windscreenRakeRad = VEH_WINDSCREEN_RAKE_MAX_RAD;
        const float uprightCd = vehicle_effective_drag_coefficient(&upright);
        const float neutralCd = vehicle_effective_drag_coefficient(&spec);
        const float rakedCd = vehicle_effective_drag_coefficient(&raked);
        check(uprightCd > neutralCd && neutralCd > rakedCd,
              "increased windscreen rake reduces effective Cd monotonically");
        check_near((double)neutralCd, (double)spec.dragCoefficient, 1e-7,
                   "default windscreen rake preserves the declared Cd exactly");

        VehicleSpec partial = spec;
        partial.windscreenRakeRad = 0.0f;
        check_near((double)vehicle_effective_drag_coefficient(&partial),
                   (double)partial.dragCoefficient, 1e-7,
                   "out-of-domain rake on a partial spec preserves base Cd");
    }

    {
        /* Near zero it stays finite and small rather than dividing by a vanishing speed. */
        bool finite = true;
        float previousN = 0.0f;
        bool monotonic = true;
        for (int i = 0; i <= 200; i++) {
            const float v = (float)i * 1e-5f;
            float magnitudeN = 0.0f;
            const Vector2 drag = physics_aero_drag_body_n(&spec, v, 0.0f, &magnitudeN);
            if (!isfinite(drag.x) || !isfinite(drag.y) || !isfinite(magnitudeN)) finite = false;
            if (magnitudeN < previousN - 1e-9f) monotonic = false;
            previousN = magnitudeN;
        }
        check(finite, "drag stays finite as speed approaches zero");
        check(monotonic, "and never spikes on the way down");
    }

    /* --------------------------------------------------------------- rolling resistance -- */

    const float loadN = 3000.0f;
    const float coefficient = ROLLING_RESISTANCE_COEF;

    {
        float magnitudeN = -1.0f;
        const Vector2 atRest = physics_rolling_resistance_body_n(
            coefficient, loadN, (Vector2){ 0.0f, 0.0f }, &magnitudeN);
        check(atRest.x == 0.0f && atRest.y == 0.0f,
              "rolling resistance is exactly zero at rest, inventing no direction");
        check_near((double)magnitudeN, 0.0, 0.0, "and reports zero magnitude");
    }

    {
        float magnitudeN = 0.0f;
        const Vector2 forward = physics_rolling_resistance_body_n(
            coefficient, loadN, (Vector2){ 5.0f, 0.0f }, &magnitudeN);
        check_near((double)magnitudeN, (double)(coefficient * loadN), 1e-4,
                   "rolling resistance magnitude is the coefficient times the normal load");
        check(forward.x < 0.0f, "it opposes forward motion (%.3f N)", (double)forward.x);

        const Vector2 reverse = physics_rolling_resistance_body_n(
            coefficient, loadN, (Vector2){ -5.0f, 0.0f }, NULL);
        check(reverse.x > 0.0f, "it opposes reverse motion too (%.3f N)", (double)reverse.x);
        check_near((double)reverse.x, -(double)forward.x, 1e-5,
                   "and is symmetric in direction");
    }

    {
        /* Load scaling is what makes rolling resistance move with load transfer. */
        float lightN = 0.0f;
        float heavyN = 0.0f;
        physics_rolling_resistance_body_n(coefficient, 2000.0f, (Vector2){ 5.0f, 0.0f },
                                          &lightN);
        physics_rolling_resistance_body_n(coefficient, 6000.0f, (Vector2){ 5.0f, 0.0f },
                                          &heavyN);
        check(fabsf(heavyN - 3.0f * lightN) < 1e-3f,
              "rolling resistance is linear in normal load (%.3f N at 2 kN -> %.3f N at 6 kN)",
              (double)lightN, (double)heavyN);
        check(fabsf(lightN - heavyN) > 1e-3f,
              "so front and rear contributions differ once load has transferred");
    }

    {
        /* Unlike drag it does not vanish with speed — it is a load-driven force, not a
         * velocity-driven one — but it must fade to zero at rest rather than chatter. */
        float slowN = 0.0f;
        float fastN = 0.0f;
        physics_rolling_resistance_body_n(coefficient, loadN, (Vector2){ 4.0f, 0.0f }, &slowN);
        physics_rolling_resistance_body_n(coefficient, loadN, (Vector2){ 30.0f, 0.0f }, &fastN);
        check_near((double)fastN, (double)slowN, 1e-5,
                   "rolling resistance does not grow with speed the way drag does");

        bool finite = true;
        bool bounded = true;
        for (int i = 0; i <= 400; i++) {
            const float v = (float)i * 1e-4f;
            float magnitudeN = 0.0f;
            const Vector2 rolling = physics_rolling_resistance_body_n(
                coefficient, loadN, (Vector2){ v, 0.0f }, &magnitudeN);
            if (!isfinite(rolling.x) || !isfinite(magnitudeN)) finite = false;
            if (magnitudeN > coefficient * loadN + 1e-6f) bounded = false;
        }
        check(finite, "rolling resistance stays finite as the contact speed approaches zero");
        check(bounded, "and never exceeds coefficient * load");
    }

    {
        /* Diagonal contact velocity: opposite the contact velocity vector. */
        const Vector2 contact = { 6.0f, -8.0f };
        float magnitudeN = 0.0f;
        const Vector2 rolling =
            physics_rolling_resistance_body_n(coefficient, loadN, contact, &magnitudeN);
        const float dot = rolling.x * contact.x + rolling.y * contact.y;
        check(dot < 0.0f, "rolling resistance opposes the contact velocity (dot %.3f)",
              (double)dot);
        check_near((double)sqrtf(rolling.x * rolling.x + rolling.y * rolling.y),
                   (double)magnitudeN, 1e-4, "its length is the reported magnitude");
    }

    /* ------------------------------------------------------- no temporary Phase 2 path -- */

    {
        /* The removed Phase 2 term was a body force linear in speed. If any equivalent
         * survived, total resistance at a given speed would exceed the two physical forms
         * by an amount that grows with speed. Compare the model against the closed forms. */
        Game *game = alloc_game();
        game_init(game);
        game->vehicle.selectedGear = 0;
        set_vehicle_rolling_speed(game, 25.0f);
        game_fixed_update(game, FIXED_DT_S);

        float expectedDragN = 0.0f;
        physics_aero_drag_body_n(&game->spec, game->vehicle.velocityLongitudinalMps,
                                 game->vehicle.velocityLateralMps, &expectedDragN);
        const float expectedRollingN =
            ROLLING_RESISTANCE_COEF * game->spec.massKg * GRAVITY_MPS2;

        check_near((double)game->derived.aeroDragMagnitudeN, (double)expectedDragN, 0.5,
                   "the model's drag is exactly the closed form, with nothing added");
        check_near((double)game->derived.rollingResistanceMagnitudeN, (double)expectedRollingN,
                   5.0,
                   "the model's rolling resistance is the coefficient times the total load");

        const float linearPhase2N = 120.0f * 25.0f; /* the removed term at this speed */
        check(game->derived.aeroDragMagnitudeN + game->derived.rollingResistanceMagnitudeN <
                  linearPhase2N,
              "total resistance is far below the removed linear term (%.1f N vs %.1f N)",
              (double)(game->derived.aeroDragMagnitudeN +
                       game->derived.rollingResistanceMagnitudeN),
              (double)linearPhase2N);
        free(game);
    }

    /* -------------------------------------------------------- zero-crossing, in the model -- */

    {
        /* Resistance alone must bring the car to rest and stop there, never push it back. */
        Game *game = alloc_game();
        game_init(game);
        game->vehicle.selectedGear = 0;
        set_vehicle_rolling_speed(game, 0.02f);
        bool reversed = false;
        bool finite = true;
        for (int i = 0; i < 600; i++) {
            game_fixed_update(game, FIXED_DT_S);
            if (game->vehicle.velocityLongitudinalMps < -1e-6f) reversed = true;
            if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived)) {
                finite = false;
            }
        }
        check(!reversed, "resistance never drives the vehicle backwards through zero");
        check(finite, "and the state stays valid the whole way down");
        check(fabsf(game->vehicle.velocityLongitudinalMps) < 0.02f,
              "the vehicle ends at or below where it started (%.6f m/s)",
              (double)game->vehicle.velocityLongitudinalMps);
        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: solver-stages — the staged solver's ordering, contracts, and failure report    */
/* ------------------------------------------------------------------------------------- */

/* A vehicle rolling forward at a modest speed with a little steering: enough for every stage
 * to have real work to do, and slow enough that nothing saturates. */
static void solver_fixture(Game *game, float speedMps, float steer)
{
    game_init(game);
    game->state = STATE_PLAYING;
    game->autoTrans.enabled = false;
    game->vehicle.selectedGear = 2;
    game->vehicle.velocityLongitudinalMps = speedMps;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        game->vehicle.wheels[i].angularVelocityRadS =
            speedMps / vehicle_wheel_radius_m(&game->spec, i);
    }
    game->input.steer = steer;
}

/*
 * What this asserts is the decomposition itself, not the physics.
 *
 * The stages run in a fixed order that does not depend on the vehicle's state; a prefix of
 * them can be run and inspected, which is the whole point of the scratch object; each stage
 * writes what its contract says and leaves the later stages' outputs alone; the step is
 * bit-reproducible; and when a step does go non-finite, the rollback restores the previous
 * state and the report names the stage that broke it.
 */
static void scenario_solver_stages(void)
{
    /* ---- 1. A prefix runs, and stops where it was told to ---- */
    {
        Game *game = alloc_game();
        solver_fixture(game, 20.0f, 0.2f);

        ControllerOutput controls;
        controller_output_zero(&controls);
        controls.throttle = 0.4f;

        PhysicsStep step;
        check(physics_step_init(&step, &game->spec, &game->vehicle, &game->derived,
                                &game->renderState, NULL, NULL, NULL, &controls, FIXED_DT_S),
              "a step initialises from a valid vehicle");
        check(step.completedStage == PHYSICS_STAGE_NONE,
              "and starts having completed nothing (got %d)", (int)step.completedStage);

        const Vector2 positionBefore = game->vehicle.positionM;
        physics_step_run(&step, PHYSICS_STAGE_NORMAL_LOADS);
        check(step.completedStage == PHYSICS_STAGE_NORMAL_LOADS,
              "running through a stage stops there (got %s)",
              physics_stage_name(step.completedStage));
        check(game->vehicle.positionM.x == positionBefore.x &&
                  game->vehicle.positionM.y == positionBefore.y,
              "and the integration stages have not run, so the car has not moved");

        /* The normal-load stage's contract: four positive loads that sum to the vehicle's
         * weight plus whatever the air is pressing down with, because nothing is airborne and
         * the longitudinal transfer only moves load about. */
        float sumFzN = 0.0f;
        bool allPositive = true;
        for (int i = 0; i < WHEEL_COUNT; i++) {
            sumFzN += game->vehicle.wheels[i].normalLoadN;
            if (game->vehicle.wheels[i].normalLoadN <= 0.0f) allPositive = false;
        }
        check(allPositive, "every wheel carries a positive normal load");
        check_near((double)sumFzN,
                   (double)(game->spec.massKg * GRAVITY_MPS2 +
                            game->derived.aeroVerticalFrontN + game->derived.aeroVerticalRearN),
                   1.0, "the four normal loads sum to the weight plus the aerodynamic load");

        /* Continuing from where it stopped completes the step exactly once more. */
        physics_step_run(&step, PHYSICS_STAGE_COUNT - 1);
        check(step.completedStage == PHYSICS_STAGE_DIAGNOSTICS,
              "continuing finishes the remaining stages (got %s)",
              physics_stage_name(step.completedStage));
        check(game->vehicle.positionM.x != positionBefore.x, "and now the car has moved");
        free(game);
    }

    /* ---- 2. Stage contracts, one at a time ---- */
    {
        Game *game = alloc_game();
        solver_fixture(game, 25.0f, 0.3f);
        ControllerOutput controls;
        controller_output_zero(&controls);
        controls.throttle = 0.5f;
        controls.steer = 0.3f;

        PhysicsStep step;
        (void)physics_step_init(&step, &game->spec, &game->vehicle, &game->derived,
                                &game->renderState, NULL, NULL, NULL, &controls, FIXED_DT_S);

        const Vector2 renderCurrBefore = game->renderState.currPositionM;
        physics_step_run(&step, PHYSICS_STAGE_BEGIN);
        check(game->renderState.prevPositionM.x == renderCurrBefore.x &&
                  game->renderState.prevPositionM.y == renderCurrBefore.y,
              "begin shifts the render history so this step can write a new current pose");

        const float steerBefore = game->vehicle.frontRoadWheelAngleRad;
        physics_step_run(&step, PHYSICS_STAGE_STEERING);
        check(game->vehicle.frontRoadWheelAngleRad > steerBefore,
              "steering actuates toward the demand (%.6f -> %.6f)", (double)steerBefore,
              (double)game->vehicle.frontRoadWheelAngleRad);

        physics_step_run(&step, PHYSICS_STAGE_POWERTRAIN);
        check(step.torques.totalGearRatio != 0.0f,
              "powertrain resolves a gear ratio (got %.4f)",
              (double)step.torques.totalGearRatio);
        check(game->vehicle.engineRpm >= game->spec.engineIdleRpm,
              "and an engine speed at or above idle (got %.1f)",
              (double)game->vehicle.engineRpm);

        physics_step_run(&step, PHYSICS_STAGE_WHEEL_KINEMATICS);
        bool slipFinite = true;
        for (int i = 0; i < WHEEL_COUNT; i++) {
            if (!isfinite(game->vehicle.wheels[i].slipAngleRad) ||
                !isfinite(game->vehicle.wheels[i].slipRatio))
                slipFinite = false;
        }
        check(slipFinite, "wheel kinematics produces a finite slip angle and ratio per wheel");
        check(game->derived.wheelContactVelocityBodyMps[WHEEL_FRONT_LEFT].x > 0.0f,
              "and contact-point velocities that follow the body");

        physics_step_run(&step, PHYSICS_STAGE_TIRE_FORCES);
        bool insideFrictionCircle = true;
        for (int i = 0; i < WHEEL_COUNT; i++) {
            const WheelState *wheel = &game->vehicle.wheels[i];
            if (wheel->frictionUsage < 0.0f || wheel->frictionUsage > 1.0f + 1e-3f)
                insideFrictionCircle = false;
        }
        check(insideFrictionCircle, "tire forces leave every wheel inside its friction circle");

        physics_step_run(&step, PHYSICS_STAGE_RESISTANCE);
        check(step.resistanceBodyN.x < 0.0f, "resistance opposes forward motion (got %.4f N)",
              (double)step.resistanceBodyN.x);
        check(step.longitudinalResistanceN == step.resistanceBodyN.x,
              "and the longitudinal component the integrator uses is that same value");

        physics_step_run(&step, PHYSICS_STAGE_ACCUMULATE);
        check_near((double)game->derived.totalBodyForceN.y,
                   (double)(game->derived.frontBodyForceN.y + game->derived.rearBodyForceN.y +
                            step.resistanceBodyN.y),
                   1e-3, "accumulate sums the axles and the resistance and nothing else");

        const float vxBefore = game->vehicle.velocityLongitudinalMps;
        physics_step_run(&step, PHYSICS_STAGE_INTEGRATE_BODY);
        check_near(
            (double)game->vehicle.velocityLongitudinalMps,
            (double)(vxBefore + step.solved.velocityLongitudinalDerivativeMps2 * FIXED_DT_S),
            1e-4, "the body integrates by exactly the solved derivative times dt");

        physics_step_run(&step, PHYSICS_STAGE_DIAGNOSTICS);
        check_near((double)game->derived.speedMps,
                   (double)hypotf(game->vehicle.velocityLongitudinalMps,
                                  game->vehicle.velocityLateralMps),
                   1e-4, "diagnostics reports the speed of the integrated velocity");
        check(game->derived.solverFailedStage == (int)PHYSICS_STAGE_NONE ||
                  step.completedStage == PHYSICS_STAGE_DIAGNOSTICS,
              "and a healthy step reports no failing stage");
        free(game);
    }

    /* ---- 3. The step is reproducible: same input state, same result, bit for bit ---- */
    {
        Game *a = alloc_game();
        Game *b = alloc_game();
        solver_fixture(a, 30.0f, -0.4f);
        solver_fixture(b, 30.0f, -0.4f);

        ControllerOutput controls;
        controller_output_zero(&controls);
        controls.throttle = 0.7f;
        controls.brake = 0.1f;

        for (int i = 0; i < 240; i++) {
            physics_fixed_update(&a->spec, &a->vehicle, &a->derived, &a->renderState, NULL,
                                 NULL, NULL, &controls, FIXED_DT_S);
            physics_fixed_update(&b->spec, &b->vehicle, &b->derived, &b->renderState, NULL,
                                 NULL, NULL, &controls, FIXED_DT_S);
        }
        check(memcmp(&a->vehicle, &b->vehicle, sizeof(VehicleState)) == 0,
              "240 steps from one starting state reproduce byte-identical vehicle state");
        free(b);
        free(a);
    }

    /* ---- 4. A non-finite step rolls back and names the stage that broke ---- */
    {
        Game *game = alloc_game();
        solver_fixture(game, 10.0f, 0.0f);

        ControllerOutput controls;
        controller_output_zero(&controls); /* no brake: the powertrain stage stays clean */

        /* Finite going in, catastrophic once the load transfer multiplies it by mass and CG
         * height. The load stage is therefore the first one whose output stops being finite. */
        game->vehicle.prevLongAccelMps2 = 1.0e38f;
        game->vehicle.filteredLongAccelMps2 = 1.0e38f;
        check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
              "precondition: the poisoned state is still finite before the step");

        /* Why the load stage and not another: the poisoned acceleration is finite, so every
         * stage before this one passes it along untouched. The load stage is the first to
         * multiply it — by mass and CG height — and that product overflows to infinity, which
         * lands in the rear wheels' normal load. */

        /* Prove the poison really does produce a state the validator rejects, without going
         * through physics_fixed_update() — which asserts, and would abort a build that has
         * assertions enabled. */
        {
            Game *poisoned = alloc_game();
            solver_fixture(poisoned, 10.0f, 0.0f);
            poisoned->vehicle.prevLongAccelMps2 = 1.0e38f;
            poisoned->vehicle.filteredLongAccelMps2 = 1.0e38f;
            PhysicsStep bad;
            (void)physics_step_init(&bad, &poisoned->spec, &poisoned->vehicle,
                                    &poisoned->derived, &poisoned->renderState, NULL, NULL,
                                    NULL, &controls, FIXED_DT_S);
            physics_step_run(&bad, PHYSICS_STAGE_COUNT - 1);
            check(!physics_state_is_valid(&poisoned->spec, &poisoned->vehicle,
                                          &poisoned->derived),
                  "the poisoned step really does end in a state the validator rejects");
            free(poisoned);
        }

#if defined(NDEBUG)
        /* The rollback path itself runs through physics_fixed_update(), whose assert() is a
         * deliberate stop-the-world in an asserting build. Exercise it only where assertions
         * are compiled out — the canonical test binary — so the sanitizer build, which keeps
         * them, is not aborted by a failure this scenario caused on purpose. */
        const VehicleState before = game->vehicle;
        physics_fixed_update(&game->spec, &game->vehicle, &game->derived, &game->renderState,
                             NULL, NULL, NULL, &controls, FIXED_DT_S);

        check(memcmp(&game->vehicle, &before, sizeof(VehicleState)) == 0,
              "a step that goes non-finite rolls the vehicle back to where it started");
        check(game->derived.solverFailedStage == (int)PHYSICS_STAGE_NORMAL_LOADS,
              "and the report names the stage that broke it: normal-loads (got %s)",
              physics_stage_name((PhysicsStage)game->derived.solverFailedStage));

        /* A healthy step afterwards clears the report rather than leaving it latched. */
        game->vehicle.prevLongAccelMps2 = 0.0f;
        game->vehicle.filteredLongAccelMps2 = 0.0f;
        physics_fixed_update(&game->spec, &game->vehicle, &game->derived, &game->renderState,
                             NULL, NULL, NULL, &controls, FIXED_DT_S);
        check(game->derived.solverFailedStage == (int)PHYSICS_STAGE_NONE,
              "a healthy step clears the failure report (got %s)",
              physics_stage_name((PhysicsStage)game->derived.solverFailedStage));
#endif
        free(game);
    }

    /* ---- 5. Every stage has a name, so a report can never be a bare number ---- */
    {
        bool named = true;
        for (PhysicsStage s = PHYSICS_STAGE_NONE; s < PHYSICS_STAGE_COUNT;
             s = (PhysicsStage)(s + 1)) {
            const char *name = physics_stage_name(s);
            if (name == NULL || name[0] == '\0') named = false;
        }
        check(named, "every stage reports a non-empty name");
    }
}

static void scenario_rest(void)
{
    Game *game = alloc_game();
    game_init(game);
    input_zero(&game->input);
    for (int i = 0; i < 1200; i++) game_fixed_update(game, FIXED_DT_S);
    check_near(game->vehicle.positionM.x, 0.0, 1e-7, "rest position X remains fixed");
    check_near(game->vehicle.positionM.y, 0.0, 1e-7, "rest position Y remains fixed");
    check_near(game->derived.speedMps, 0.0, 1e-7, "rest speed remains zero");
    check_near(game->vehicle.yawRateRadS, 0.0, 1e-7, "rest yaw rate remains zero");
    /* Static toe scrubs even at rest (issue #14): the two mirrored contact patches cancel
     * laterally — asserted immediately below — while their longitudinal projections add, so the
     * solver reports a small drag. Alignment can only ever retard the car; a positive value here
     * would mean toe was propelling it, and the pose checks above already prove it moves
     * nothing at all. */
    check(game->derived.totalBodyForceN.x < 0.0f && game->derived.totalBodyForceN.x > -20.0f,
          "rest longitudinal force is a bounded toe-scrub drag, never a thrust (%.4f N)",
          (double)game->derived.totalBodyForceN.x);
    check_near(game->derived.totalBodyForceN.y, 0.0, 1e-7, "rest lateral force remains zero");
    check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
          "rest state remains finite and inside safety bounds");

    /* And alignment is the ONLY source of it: zero the toe and the residual vanishes exactly. */
    {
        Game *unaligned = alloc_game();
        game_init(unaligned);
        input_zero(&unaligned->input);
        unaligned->spec.suspToeFrontRad = 0.0f;
        unaligned->spec.suspToeRearRad = 0.0f;
        for (int i = 0; i < 1200; i++) game_fixed_update(unaligned, FIXED_DT_S);
        check_near(unaligned->derived.totalBodyForceN.x, 0.0, 1e-7,
                   "with zero toe a resting car reports no longitudinal force at all");
        free(unaligned);
    }

    game->input.steer = 1.0f;
    for (int i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);
    check_near(game->derived.speedMps, 0.0, 1e-7,
               "steering while stationary creates no vehicle motion");
    check_near(game->vehicle.yawRateRadS, 0.0, 1e-7,
               "steering while stationary creates no yaw");
    check_near(game->derived.lateralAccelerationMps2, 0.0, 1e-7,
               "steering while stationary creates no lateral acceleration");
    game->input.steer = 0.0f;
    game->input.handbrake = 1.0f;
    for (int i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);
    check_near(game->derived.speedMps, 0.0, 1e-7,
               "handbrake at rest cannot launch the vehicle");
    check_near(game->vehicle.yawRateRadS, 0.0, 1e-7,
               "handbrake at rest cannot rotate the vehicle");
    free(game);
}

static void scenario_launch_stop(void)
{
    Game *game = alloc_game();
    game_init(game);
    /* Brake means brake here: the driver aid would turn a held pedal at rest into reverse. */
    game->autoTrans.enabled = false;
    TelemetryWriter writer;
    const bool telemetryOpened = telemetry_open(&writer, PHASE2_LAUNCH_TELEMETRY);
    game->input.throttle = 1.0f;
    float maxRearSlip = 0.0f;
    float maxRearForceN = 0.0f;
    float maxEngineRpm = game->vehicle.engineRpm;
    float maxFrontSlip = 0.0f;
    for (int i = 0; i < 600; i++) {
        game_fixed_update(game, FIXED_DT_S);
        maxRearSlip = fmaxf(maxRearSlip, game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio);
        maxRearForceN =
            fmaxf(maxRearForceN, game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                                     game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN);
        maxEngineRpm = fmaxf(maxEngineRpm, game->vehicle.engineRpm);
        maxFrontSlip =
            fmaxf(maxFrontSlip, fabsf(game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio));
        if (telemetryOpened && (i + 1) % 120 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    const float launchSpeed = game->vehicle.velocityLongitudinalMps;
    check(launchSpeed > 10.0f, "straight launch builds forward speed (%.3f m/s)",
          (double)launchSpeed);
    check(fabsf(game->vehicle.velocityLateralMps) < 1e-5f,
          "straight launch keeps lateral velocity near zero");
    check(fabsf(game->vehicle.yawRateRadS) < 1e-5f, "straight launch keeps yaw rate near zero");
    check(game->vehicle.positionM.x > 20.0f && fabsf(game->vehicle.positionM.y) < 1e-5f,
          "straight launch advances along world +X only");
    check(maxRearSlip > 0.1f && maxRearForceN > 1000.0f,
          "rear wheelspin creates positive longitudinal tire force "
          "(slip %.3f, Fx %.1f N)",
          (double)maxRearSlip, (double)maxRearForceN);
    check(maxEngineRpm > game->spec.engineIdleRpm,
          "engine RPM rises from driven-wheel speed (max %.0f rpm)", (double)maxEngineRpm);
    check(maxFrontSlip < 0.1f, "front wheels remain close to free rolling (max |slip| %.3f)",
          (double)maxFrontSlip);
    check_near(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS,
               game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS, 0.0,
               "locked rear axle stays synchronized");
    printf("    acceleration: %.3f m/s, %.3f m, rear slip %.3f, rear Fx %.1f N, %.0f rpm\n",
           (double)launchSpeed, (double)game->vehicle.positionM.x, (double)maxRearSlip,
           (double)maxRearForceN, (double)maxEngineRpm);

    game->input.throttle = 0.0f;
    for (int i = 0; i < 180; i++) game_fixed_update(game, FIXED_DT_S);
    game->input.brake = 1.0f;
    float previousAbsSpeed = fabsf(game->vehicle.velocityLongitudinalMps);
    bool monotonic = true;
    float maxSpeedIncrease = 0.0f;
    int maxSpeedIncreaseTick = 0;
    float maxRiseBefore = 0.0f;
    float maxRiseAfter = 0.0f;
    float maxRiseRearSlip = 0.0f;
    float maxRiseRearFx = 0.0f;
    float minimumSlip = 0.0f;
    for (int i = 0; i < 600; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const float absSpeed = fabsf(game->vehicle.velocityLongitudinalMps);
        const float speedIncrease = absSpeed - previousAbsSpeed;
        minimumSlip = fminf(minimumSlip, game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio);
        if (speedIncrease > maxSpeedIncrease) {
            maxSpeedIncrease = speedIncrease;
            maxSpeedIncreaseTick = i;
            maxRiseBefore = previousAbsSpeed;
            maxRiseAfter = absSpeed;
            maxRiseRearSlip = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
            maxRiseRearFx = game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                            game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
        }
        if (speedIncrease > 1e-5f) monotonic = false;
        previousAbsSpeed = absSpeed;
        if (telemetryOpened && (i + 1) % 120 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    check(monotonic,
          "service braking reduces speed without an impulse reversal "
          "(max rise %.7f at %d: %.7f -> %.7f, rear slip %.3f Fx %.1f)",
          (double)maxSpeedIncrease, maxSpeedIncreaseTick, (double)maxRiseBefore,
          (double)maxRiseAfter, (double)maxRiseRearSlip, (double)maxRiseRearFx);
    check(game->vehicle.velocityLongitudinalMps >= -1e-6f,
          "ordinary braking does not accelerate the vehicle backward");
    check(fabsf(game->vehicle.velocityLongitudinalMps) < 1e-5f,
          "braking settles at zero speed");
    check(minimumSlip < -0.1f, "service braking creates negative wheel slip (min %.3f)",
          (double)minimumSlip);
    const float actualFrontBrake = game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                                   game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT];
    const float actualTotalBrake = actualFrontBrake +
                                   game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                                   game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT];
    check(actualFrontBrake >= actualTotalBrake * game->spec.brakeBiasFront - 1e-4f &&
              actualFrontBrake <= actualTotalBrake * 0.85f + 1e-4f,
          "service-brake front torque reserves forward load capacity (front %.1f of %.1f Nm)",
          (double)actualFrontBrake, (double)actualTotalBrake);
    printf("    braking: %.3f -> %.6f m/s, min slip %.3f, no reversal\n", (double)launchSpeed,
           (double)game->derived.speedMps, (double)minimumSlip);
    check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
          "launch and stopping remain finite");
    check(telemetryOpened && telemetry_close(&writer),
          "launch/stop telemetry writes successfully");
    free(game);
}

static void scenario_coast_down(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_DIR "/phase2_coast_down.csv");

    game->input.throttle = 1.0f;
    for (int i = 0; i < 480; i++) game_fixed_update(game, FIXED_DT_S);
    const float initialSpeed = game->derived.speedMps;
    const float driveBeforeLift = game->derived.drivelineTorqueNm;
    game->input.throttle = 0.0f;

    bool finite = true;
    float peakSpeedAfterSettling = 0.0f;
    float previousAbsOmega = fabsf(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS);
    int omegaSignChanges = 0;
    float previousOmega = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    for (int i = 0; i < 720; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (i >= 60 && game->derived.speedMps > peakSpeedAfterSettling) {
            peakSpeedAfterSettling = game->derived.speedMps;
        }
        const float omega = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
        if (omega * previousOmega < 0.0f) omegaSignChanges++;
        previousOmega = omega;
        previousAbsOmega = fabsf(omega);
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived))
            finite = false;
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    (void)previousAbsOmega;
    check(initialSpeed > 5.0f, "coast-down begins from useful speed (%.3f m/s)",
          (double)initialSpeed);
    check(game->derived.drivelineTorqueNm < driveBeforeLift,
          "throttle lift reduces driveline torque");
    check(game->derived.speedMps < initialSpeed,
          "engine braking, drag, and rolling resistance reduce coast speed");
    check(peakSpeedAfterSettling <= initialSpeed + 0.05f,
          "coast-down has no post-lift force spike (peak %.3f vs %.3f)",
          (double)peakSpeedAfterSettling, (double)initialSpeed);
    check(omegaSignChanges == 0,
          "rear wheel speed does not oscillate through zero during coast-down");
    check(finite, "coast-down remains finite");
    check(opened && telemetry_close(&writer), "coast-down telemetry writes successfully");
    free(game);

    /* Phase 3: the two resistance forms are separate, and each behaves like itself. Sampled
     * at decreasing speed from a free-rolling, neutral-gear car so nothing else is acting. */
    {
        Game *probe = alloc_game();
        game_init(probe);
        probe->vehicle.selectedGear = 0;

        struct {
            float speedMps;
            float dragN;
            float rollingN;
            float loadN;
        } samples[4];
        static const float speeds[4] = { 30.0f, 20.0f, 15.0f, 10.0f };
        for (int s = 0; s < 4; s++) {
            game_reset_sim(probe);
            probe->vehicle.selectedGear = 0;
            set_vehicle_rolling_speed(probe, speeds[s]);
            game_fixed_update(probe, FIXED_DT_S);
            samples[s].speedMps = speeds[s];
            samples[s].dragN = probe->derived.aeroDragMagnitudeN;
            samples[s].rollingN = probe->derived.rollingResistanceMagnitudeN;
            samples[s].loadN = probe->derived.normalLoadFrontN + probe->derived.normalLoadRearN;
        }

        /* Drag falls with the square of speed: halving speed quarters it. */
        check(fabsf(samples[0].dragN - 4.0f * samples[2].dragN) < 1.0f,
              "aerodynamic drag falls with speed squared (30 m/s %.1f N, 15 m/s %.1f N)",
              (double)samples[0].dragN, (double)samples[2].dragN);
        check(samples[0].dragN > samples[1].dragN && samples[1].dragN > samples[2].dragN &&
                  samples[2].dragN > samples[3].dragN,
              "drag decreases monotonically as the car slows");

        /* Rolling resistance does not: it tracks load, which is near static while coasting. */
        check(fabsf(samples[0].rollingN - samples[3].rollingN) < 5.0f,
              "rolling resistance is load-driven, not speed-driven (%.1f N at 30 m/s vs "
              "%.1f N at 10 m/s)",
              (double)samples[0].rollingN, (double)samples[3].rollingN);
        check_near((double)samples[0].rollingN,
                   (double)(ROLLING_RESISTANCE_COEF * samples[0].loadN), 5.0,
                   "and equals the coefficient times the current dynamic load");

        /* Resistance never changes sign while the direction of travel does not. */
        game_reset_sim(probe);
        probe->vehicle.selectedGear = 0;
        set_vehicle_rolling_speed(probe, 28.0f);
        bool directionHeld = true;
        bool spikeFree = true;
        float previousTotalN = 1e9f;
        for (int i = 0; i < 1800; i++) {
            game_fixed_update(probe, FIXED_DT_S);
            if (probe->vehicle.velocityLongitudinalMps <= 0.0f) break;
            if (probe->derived.aeroDragBodyN.x > 0.0f ||
                probe->derived.rollingResistanceBodyN.x > 0.0f)
                directionHeld = false;
            const float totalN = fabsf(probe->derived.aeroDragBodyN.x) +
                                 fabsf(probe->derived.rollingResistanceBodyN.x);
            if (totalN > previousTotalN + 1.0f) spikeFree = false;
            previousTotalN = totalN;
        }
        check(directionHeld,
              "both resistance forces keep opposing forward travel for the whole coast");
        check(spikeFree, "and neither spikes as the car slows");
        free(probe);
    }
}

static void scenario_braking_cornering(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_DIR "/phase2_braking_cornering.csv");
    game->vehicle.selectedGear = 0;
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.steer = 0.28f;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }

    game->input.brake = 1.0f;
    bool sharedBudgetObserved = false;
    float maxUsage = 0.0f;
    bool withinLimit = true;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        for (int w = 0; w < WHEEL_COUNT; w++) {
            const WheelState *wheel = &game->vehicle.wheels[w];
            const SurfaceSpec *sv_test = Surface_Get(wheel->surfaceId);
            const float muScale_test = game->derived.tireLoadSensitivityMuScale[w];
            const float lateralMu =
                (w <= WHEEL_FRONT_RIGHT) ? game->spec.tireMuLatFront : game->spec.tireMuLatRear;
            const float nx = wheel->forceLongitudinalN /
                             (game->spec.tireMuLongScale * sv_test->muLongitudinal *
                              muScale_test * wheel->normalLoadN);
            const float ny = wheel->forceLateralN /
                             (lateralMu * (sv_test->muLateral / SURFACE_REFERENCE_MU_LAT) *
                              muScale_test * wheel->normalLoadN);
            const float usage = sqrtf(nx * nx + ny * ny);
            if (usage > 1.0f + FRICTION_TOLERANCE) withinLimit = false;
            if (wheel->frictionUsage > maxUsage) maxUsage = wheel->frictionUsage;
            if (fabsf(game->derived.pureLateralForceN[w]) >
                    fabsf(wheel->forceLateralN) + 1.0f &&
                fabsf(wheel->forceLongitudinalN) > 100.0f) {
                sharedBudgetObserved = true;
            }
        }
    }
    check(maxUsage > 0.9f, "corner braking raises friction usage (max %.3f)", (double)maxUsage);
    check(sharedBudgetObserved,
          "corner braking reduces lateral force through the shared friction budget");
    check(withinLimit, "every corner-braking wheel remains within its friction ellipse");
    check(game->derived.speedMps < 12.0f, "corner braking reduces vehicle speed");
    check(opened && telemetry_close(&writer), "corner-braking telemetry writes successfully");
    free(game);
}

static void scenario_power_oversteer(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_DIR "/phase2_power_oversteer.csv");
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.steer = 0.20f;
    game->input.throttle = 0.15f;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }

    game->input.throttle = 1.0f;
    float maxRearSlip = -1000.0f;
    float maxRearUsage = 0.0f;
    float maxFrontUsage = 0.0f;
    float minimumRearLateralRatio = 1.0f;
    for (int i = 0; i < 180; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const WheelState *front = &game->vehicle.wheels[WHEEL_FRONT_LEFT];
        const WheelState *rear = &game->vehicle.wheels[WHEEL_REAR_LEFT];
        maxRearSlip = fmaxf(maxRearSlip, rear->slipRatio);
        maxRearUsage = fmaxf(maxRearUsage, rear->frictionUsage);
        maxFrontUsage = fmaxf(maxFrontUsage, front->frictionUsage);
        const float pureRearFy = fabsf(game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            minimumRearLateralRatio =
                fminf(minimumRearLateralRatio, fabsf(rear->forceLateralN) / pureRearFy);
        }
    }
    const float sideslipUnderPower = fabsf(game->derived.bodySideslipRad);
    game->input.throttle = 0.0f;
    float bestRecoveredRatio = 0.0f;
    for (int i = 0; i < 240; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const float pureRearFy = fabsf(game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            bestRecoveredRatio =
                fmaxf(bestRecoveredRatio,
                      fabsf(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLateralN) / pureRearFy);
        }
    }
    check(maxRearSlip > 0.1f, "power oversteer raises driven-wheel slip ratio (max %.3f)",
          (double)maxRearSlip);
    check(maxRearUsage >= maxFrontUsage - 0.05f,
          "rear friction usage reaches saturation at least as strongly as front "
          "(rear %.3f front %.3f)",
          (double)maxRearUsage, (double)maxFrontUsage);
    check(minimumRearLateralRatio < 0.9f,
          "rear combined slip reduces lateral authority (ratio %.3f)",
          (double)minimumRearLateralRatio);
    check(sideslipUnderPower > 0.01f,
          "power-oversteer maneuver develops body sideslip (%.3f rad)",
          (double)sideslipUnderPower);
    check(bestRecoveredRatio > minimumRearLateralRatio + 0.05f,
          "throttle lift restores rear lateral authority (%.3f -> %.3f)",
          (double)minimumRearLateralRatio, (double)bestRecoveredRatio);
    check(opened && telemetry_close(&writer), "power-oversteer telemetry writes successfully");
    free(game);
}

static void scenario_handbrake_entry(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_DIR "/phase2_handbrake_entry.csv");
    game->vehicle.selectedGear = 0;
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.steer = 0.22f;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    const float yawBefore = fabsf(game->vehicle.yawRateRadS);

    game->input.handbrake = 1.0f;
    bool rearLocked = false;
    float minimumRearSlip = 0.0f;
    float maxRearUsage = 0.0f;
    float minimumRearLateralRatio = 1.0f;
    float maxYaw = yawBefore;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const WheelState *rear = &game->vehicle.wheels[WHEEL_REAR_LEFT];
        rearLocked = rearLocked || rear->locked;
        minimumRearSlip = fminf(minimumRearSlip, rear->slipRatio);
        maxRearUsage = fmaxf(maxRearUsage, rear->frictionUsage);
        maxYaw = fmaxf(maxYaw, fabsf(game->vehicle.yawRateRadS));
        const float pureRearFy = fabsf(game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            minimumRearLateralRatio =
                fminf(minimumRearLateralRatio, fabsf(rear->forceLateralN) / pureRearFy);
        }
    }
    game->input.handbrake = 0.0f;
    float recoveredRatio = 0.0f;
    for (int i = 0; i < 240; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const float pureRearFy = fabsf(game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            recoveredRatio =
                fmaxf(recoveredRatio,
                      fabsf(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLateralN) / pureRearFy);
        }
    }
    check(game->derived.handbrakeTorqueNm[WHEEL_FRONT_LEFT] == 0.0f &&
              game->derived.handbrakeTorqueNm[WHEEL_FRONT_RIGHT] == 0.0f,
          "handbrake torque remains rear-only");
    check(rearLocked, "handbrake entry locks the rear axle");
    check(minimumRearSlip < -0.5f, "handbrake creates negative rear slip (min %.3f)",
          (double)minimumRearSlip);
    check(maxRearUsage > 0.9f, "handbrake consumes rear friction budget (max %.3f)",
          (double)maxRearUsage);
    check(minimumRearLateralRatio < 0.9f,
          "handbrake reduces rear lateral force through combined grip");
    check(maxYaw > yawBefore, "handbrake entry increases yaw response without direct yaw code");
    check(recoveredRatio > minimumRearLateralRatio + 0.05f,
          "handbrake release restores rear lateral authority");
    check(opened && telemetry_close(&writer), "handbrake-entry telemetry writes successfully");
    free(game);
}

static void scenario_low_speed(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->input.throttle = 0.50f;
    game->input.steer = 0.35f;
    float previousYaw = 0.0f;
    float maxYawJump = 0.0f;
    bool finite = true;
    bool crossedBlend = false;
    for (int i = 0; i < 480; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const float jump = fabsf(game->vehicle.yawRateRadS - previousYaw);
        if (jump > maxYawJump) maxYawJump = jump;
        previousYaw = game->vehicle.yawRateRadS;
        if (game->derived.lowSpeedBlend > 0.0f && game->derived.lowSpeedBlend < 1.0f) {
            crossedBlend = true;
        }
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived))
            finite = false;
    }
    check(crossedBlend, "slow launch traverses the kinematic/dynamic blend");
    check(maxYawJump < 0.1f,
          "yaw response stays continuous through blend thresholds (jump %.4f)",
          (double)maxYawJump);
    check(game->vehicle.yawRateRadS > 0.0f,
          "left steering grows positive yaw as speed increases");
    check(finite, "every sampled low-speed state remains finite");
    free(game);
}

static void scenario_reverse(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->autoTrans.enabled = false; /* the gear is selected explicitly here */
    game->vehicle.selectedGear = -1;
    game->input.throttle = 0.8f;
    game->input.steer = 0.25f;
    bool finite = true;
    for (int i = 0; i < 180; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived))
            finite = false;
    }
    check(game->vehicle.velocityLongitudinalMps < -0.5f,
          "explicit reverse direction launches backward (vx %.3f m/s)",
          (double)game->vehicle.velocityLongitudinalMps);
    check(game->vehicle.positionM.x < 0.0f, "reverse launch moves toward world -X");
    check(game->vehicle.yawRateRadS < 0.0f,
          "left steering while reversing produces the documented opposite yaw direction");
    check(isfinite(game->derived.frontSlipAngleRad) && isfinite(game->derived.rearSlipAngleRad),
          "reverse slip angles have no singularity");
    check(finite, "slow reverse remains finite and stable");

    game->input.throttle = 0.0f;
    game->input.steer = 0.0f;
    game->input.brake = 1.0f;
    for (int i = 0; i < 360; i++) game_fixed_update(game, FIXED_DT_S);
    check(game->vehicle.velocityLongitudinalMps <= 1e-6f,
          "braking from reverse does not launch forward");
    check(fabsf(game->vehicle.velocityLongitudinalMps) < 1e-5f,
          "reverse braking settles at zero");
    free(game);
}

static void scenario_steering_sign(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);
    state.velocityLongitudinalMps = 8.0f;
    ControllerOutput input;
    controller_output_zero(&input);
    input.steer = 0.5f;
    for (int i = 0; i < 30; i++) {
        physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL, &input,
                             FIXED_DT_S);
    }
    check(state.frontRoadWheelAngleRad > 0.0f, "left input produces positive road-wheel angle");
    check(derived.frontLateralForceN > 0.0f,
          "left steering initially produces a leftward front tire force");
    check(derived.totalYawTorqueNm > 0.0f, "left steering produces positive yaw torque");
    check(state.yawRateRadS > 0.0f, "left steering from forward travel produces positive yaw");
    check(state.headingRad > 0.0f, "heading changes only after positive yaw rate integrates");
    check_near(state.wheels[WHEEL_FRONT_LEFT].forceLateralN +
                   state.wheels[WHEEL_FRONT_RIGHT].forceLateralN,
               derived.frontLateralForceN, 0.0,
               "the split front wheel diagnostics sum exactly to the axle force");
}

static void scenario_lever_arm(void)
{
    VehicleSpec a;
    VehicleSpec b;
    VehicleState sa;
    VehicleState sb;
    VehicleDerived da;
    VehicleDerived db;
    VehicleRenderState ra;
    VehicleRenderState rb;
    phase1_fixture(&a, &sa, &da, &ra);
    b = a;
    b.cgToFrontM += 0.35f;
    b.cgToRearM -= 0.20f;
    b.wheelbaseM = b.cgToFrontM + b.cgToRearM;
    vehicle_state_reset(&b, &sb, &db, &rb);
    sa.velocityLongitudinalMps = sb.velocityLongitudinalMps = 9.0f;
    sa.yawRateRadS = sb.yawRateRadS = 0.4f;
    float af;
    float ar;
    float bf;
    float br;
    physics_axle_slip_angles(&a, &sa, &af, &ar);
    physics_axle_slip_angles(&b, &sb, &bf, &br);
    check(fabsf(af - bf) > 1e-4f, "front lever-arm change alters front slip");
    check(fabsf(ar - br) > 1e-4f, "rear lever-arm change alters rear slip");
    ControllerOutput input;
    controller_output_zero(&input);
    input.steer = 0.3f;
    physics_fixed_update(&a, &sa, &da, &ra, NULL, NULL, NULL, &input, FIXED_DT_S);
    physics_fixed_update(&b, &sb, &db, &rb, NULL, NULL, NULL, &input, FIXED_DT_S);
    check(fabsf(da.totalYawTorqueNm - db.totalYawTorqueNm) > 1.0f,
          "lever-arm changes measurably alter yaw torque");
    check(fabsf(sa.yawRateRadS - sb.yawRateRadS) > 1e-6f,
          "lever-arm changes alter integrated yaw response");
}

static void scenario_integration(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);
    ControllerOutput input;
    controller_output_zero(&input);
    input.throttle = 1.0f;
    physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL, &input,
                         FIXED_DT_S);
    check_near(state.velocityLongitudinalMps, 0.0, 1e-7,
               "the first launch tick spins the driven wheels before tire force develops");
    physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL, &input,
                         FIXED_DT_S);
    check(state.velocityLongitudinalMps > 0.0f,
          "the next tick accelerates from drivetrain-generated wheel slip");
    check_near(state.positionM.x, state.velocityLongitudinalMps * FIXED_DT_S, 1e-6,
               "position uses the updated semi-implicit velocity");

    state.headingRad = CIRCUIT_PI - 0.001f;
    state.yawRateRadS = 1.0f;
    state.velocityLongitudinalMps = 0.0f;
    input.throttle = 0.0f;
    physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL, &input,
                         FIXED_DT_S);
    check(state.headingRad >= -CIRCUIT_PI && state.headingRad < CIRCUIT_PI,
          "integrated heading remains wrapped");
}

static void scenario_fixed_rate(void)
{
    Game *direct = alloc_game();
    Game *accumulated = alloc_game();
    game_init(direct);
    game_init(accumulated);
    direct->input.throttle = accumulated->input.throttle = 0.7f;
    direct->input.steer = accumulated->input.steer = 0.15f;
    for (int i = 0; i < 600; i++) game_fixed_update(direct, FIXED_DT_S);
    for (int i = 0; i < 300; i++) {
        const TimestepResult step =
            timestep_advance(&accumulated->accumulatorS, &accumulated->physicsBacklogDrops,
                             FIXED_DT_S * 2.0f, fixed_update_adapter, accumulated);
        accumulated->lastSubstepCount = step.substeps;
    }
    check(accumulated->physicsBacklogDrops == 0, "fixed-rate consistency run drops no backlog");
    check(direct->sim.tick == accumulated->sim.tick,
          "direct and accumulator stepping execute the same tick count");
    check(direct->stateChecksum == accumulated->stateChecksum,
          "direct and accumulator stepping produce identical checksums (%08x)",
          direct->stateChecksum);
    check(memcmp(&direct->vehicle, &accumulated->vehicle, sizeof(VehicleState)) == 0,
          "direct and accumulator vehicle states are bit-identical");
    free(accumulated);
    free(direct);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: params — the tunable registry                                                 */
/* ------------------------------------------------------------------------------------- */

static void scenario_params(void)
{
    VehicleSpec defaults;
    vehicle_spec_set_default(&defaults);

    const int count = dev_params_count();
    check(count > 0, "the parameter registry is not empty (%d entries)", count);

    /* The registry's declared defaults ARE the config.h values. If this fails, a constant
     * was changed in one place and not the other, and every profile, slider range, and
     * documentation table below it is now lying. */
    int mismatches = 0;
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        const float actual = dev_param_get(&defaults, param);
        if (fabsf(actual - param->defaultValue) >
            fmaxf(fabsf(param->defaultValue), 1.0f) * 1e-6f) {
            check(false,
                  "registry default for '%s' is %g but vehicle_spec_set_default gives %g",
                  param->name, (double)param->defaultValue, (double)actual);
            mismatches++;
        }
    }
    check(mismatches == 0, "every registry default matches vehicle_spec_set_default()");

    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        check(param->minimum <= param->defaultValue && param->defaultValue <= param->maximum,
              "'%s' default %g lies inside its range [%g, %g]", param->name,
              (double)param->defaultValue, (double)param->minimum, (double)param->maximum);
        check(dev_param_find(param->name) == param, "'%s' is findable by name", param->name);
    }

    /* Setting clamps, rejects non-finite input, and refuses derived readouts. */
    VehicleSpec spec = defaults;
    const DevParameter *mass = dev_param_find("body.mass");
    check(mass != NULL && mass->derived, "body.mass is a derived readout");
    check(!dev_param_set(&spec, mass, 1e9f), "a derived parameter refuses writes");
    check_near(dev_param_get(&spec, mass), mass->defaultValue, 1e-3,
               "a refused derived write leaves the value untouched");

    const DevParameter *wheelbase = dev_param_find("body.wheelbase");
    check(wheelbase != NULL && !wheelbase->derived, "body.wheelbase is primary");
    check(dev_param_set(&spec, wheelbase, 1e9f), "an out-of-range primary set clamps");
    check_near(dev_param_get(&spec, wheelbase), wheelbase->maximum, 1e-3,
               "an out-of-range set clamps");
    check(!dev_param_set(&spec, wheelbase, NAN), "a NaN set is refused");
    check_near(dev_param_get(&spec, wheelbase), wheelbase->maximum, 1e-3,
               "a refused set leaves the value untouched");
    check_near(spec.wheelbaseM, spec.cgToFrontM + spec.cgToRearM, 1e-4,
               "CG distances follow the primary wheelbase");
    check(vehicle_spec_is_valid(&spec), "the spec stays valid after tuning");

    check(dev_params_modified_count(&spec) == 1, "one primary parameter differs from default");
    const DevParameter *track = dev_param_find("body.track_front");
    check(dev_param_set(&spec, track, track->maximum), "a second primary set succeeds");
    check(dev_params_modified_count(&spec) == 2, "two primary parameters differ from defaults");
    dev_params_reset_all(&spec);
    check(dev_params_modified_count(&spec) == 0, "reset_all restores every default");

    /* Stock mass-particle model recovers the historical defaults. */
    check_near(defaults.massKg, VEH_MASS_KG, 1e-3, "stock mass from particles");
    check_near(defaults.cgToFrontM, VEH_CG_TO_FRONT_M, 1e-3,
               "stock cg_to_front from particles");
    check_near(defaults.cgToRearM, VEH_CG_TO_REAR_M, 1e-3, "stock cg_to_rear from particles");
    check_near(defaults.cgHeightM, VEH_CG_HEIGHT_M, 1e-3, "stock cg_height from particles");
    check_near(defaults.yawInertiaKgM2, VEH_YAW_INERTIA_KGM2, 1e-2,
               "stock yaw inertia from particles");
    check_near(defaults.wheelRadiusFrontM, WHEEL_RADIUS_M, 1e-4,
               "stock front radius from tires");
    check_near(defaults.wheelRadiusRearM, WHEEL_RADIUS_M, 1e-4, "stock rear radius from tires");

    /* Profile round-trip (primaries only — derived rows are omitted from saves). */
    check(telemetry_ensure_dir("data/vehicles"),
          "the vehicle profile directory exists or was created");
    int primaryCount = 0;
    for (int i = 0; i < count; i++) {
        if (!dev_param_at(i)->derived) primaryCount++;
    }
    dev_param_set(&spec, dev_param_find("tire.lat_rear.mu"), 1.05f);
    dev_param_set(&spec, dev_param_find("brake.bias_front"), 0.55f);
    const char *profilePath = "data/vehicles/_test_roundtrip.txt";
    check(dev_params_save(&spec, profilePath), "a profile writes to disk");

    VehicleSpec loaded;
    vehicle_spec_set_default(&loaded);
    int applied = 0, unknown = 0, rejected = 0;
    check(dev_params_load(&loaded, profilePath, &applied, &unknown, &rejected),
          "the profile loads back");
    check(applied == primaryCount, "every primary parameter round-tripped (%d applied)",
          applied);
    check(unknown == 0 && rejected == 0, "no unknown or rejected keys (%d/%d)", unknown,
          rejected);
    check_near(loaded.tireMuLatRear, 1.05f, 1e-6, "a tuned tire value survives the round trip");
    check_near(loaded.brakeBiasFront, 0.55f, 1e-6, "a tuned brake value survives");
    remove(profilePath);

    /* The parser is a fuzz target, so its refusal behaviour is asserted here too. */
    VehicleSpec probe;
    vehicle_spec_set_default(&probe);
    const char *garbage = "body.mass\nnot even close\n= = =\nbody.mass = \nbody.mass = abc\n";
    check(
        dev_params_apply_text(&probe, garbage, strlen(garbage), &applied, &unknown, &rejected),
        "a garbage profile is survivable");
    check(applied == 0, "no garbage line was applied");
    check(rejected > 0, "garbage lines are counted as rejected (%d)", rejected);
    check(memcmp(&probe, &defaults, sizeof(VehicleSpec)) == 0, "garbage changed nothing");

    const char *unknownKeys = "no.such.parameter = 1.0\nbody.mass = 1300\n";
    check(dev_params_apply_text(&probe, unknownKeys, strlen(unknownKeys), &applied, &unknown,
                                &rejected),
          "an unknown key is skipped rather than failing the load");
    check(applied == 1 && unknown == 1, "one applied (mass alias), one unknown (%d/%d)",
          applied, unknown);
    check_near(probe.massKg, 1300.0f, 1e-2, "body.mass alias scales particles");

    /* A profile that would produce an invalid spec must change nothing at all. */
    VehicleSpec guarded;
    vehicle_spec_set_default(&guarded);
    const VehicleSpec beforeGuard = guarded;
    /* idle above redline after clamping is impossible via the registry ranges; force an
     * invalid candidate by applying a primary that makes CG distances non-positive. */
    const char *invalid =
        "body.wheelbase = 1.80\nmass.engine_x = 4.00\nmass.chassis_x = 4.00\n";
    const bool accepted =
        dev_params_apply_text(&guarded, invalid, strlen(invalid), NULL, NULL, NULL);
    if (accepted) {
        check(vehicle_spec_is_valid(&guarded), "accepted profile left a valid spec");
    } else {
        check(memcmp(&guarded, &beforeGuard, sizeof(VehicleSpec)) == 0,
              "a refused profile changes nothing");
    }
    check(vehicle_spec_is_valid(&guarded), "the spec is valid whatever the profile said");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: param-audit — what every VehicleSpec field actually does (issue 12)            */
/* ------------------------------------------------------------------------------------- */

/*
 * The registry claims a class for every tunable. This scenario refuses to take that on
 * trust. It proves four separate things:
 *
 *   1. COVERAGE — every float byte of VehicleSpec is described by exactly one registry
 *      entry, so a new field cannot be added without being classified.
 *   2. DERIVED — an entry marked derived is exactly one that vehicle_spec_refresh_derived()
 *      recomputes, and no other entry is touched by a refresh.
 *   3. OWNER — a setup-owned entry is exactly one that VehicleSetup overwrites when
 *      vehicle_instance_derive() compiles a definition; a definition-owned entry survives.
 *   4. EFFECT — perturbing a `physics` entry changes a simulated trajectory, and perturbing
 *      an `appearance` or `inactive` entry leaves it bit-identical. This is the honest core
 *      of issue 12: camber, toe, caster, wheel and anti-roll rates, travel, roll centres,
 *      tire pressure and the aero lift coefficients are all authored and all inert, and this
 *      check is what stops a future edit from quietly claiming otherwise.
 *
 * It deliberately activates nothing. A failure here means the registry's claim and the code
 * disagree, and the fix is to correct the claim or the code, not to widen the tolerance.
 */

#define PARAM_AUDIT_DRIVE_TICKS 540
#define PARAM_AUDIT_COLLIDE_TICKS 150

static uint32_t param_audit_hash_bytes(uint32_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    for (size_t i = 0; i < size; i++) {
        hash ^= bytes[i];
        hash *= 0x01000193u;
    }
    return hash;
}

/* A scripted run that reaches every driveline, steering, brake and tire path: launch from
 * rest through all five gears at full throttle with steering applied, then trail-brake, then
 * handbrake, then reverse. Gears are selected directly because physics_fixed_update() does
 * not shift — that is the transmission's job, not the solver's. */
static uint32_t param_audit_drive_signature(const VehicleSpec *spec, bool *allFiniteOut)
{
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    VehicleTireState tireState[WHEEL_COUNT];
    float fuelKg = spec->massFuelKg;
    vehicle_state_reset(spec, &state, &derived, &renderState);
    for (int w = 0; w < WHEEL_COUNT; w++) {
        const bool front = w <= WHEEL_FRONT_RIGHT;
        tireState[w].pressureKpa =
            front ? spec->tirePressureFrontKpa : spec->tirePressureRearKpa;
        tireState[w].temperatureC = TIRE_AMBIENT_TEMP_C;
        tireState[w].wear = 0.0f;
    }

    ControllerOutput input;
    controller_output_zero(&input);

    /* The fuel model mutates the entrant spec's mass/CG each tick (issue #24), so the audit
     * works on a mutable copy and re-derives the mass from the live fuel before each step. */
    VehicleSpec working = *spec;

    for (int tick = 0; tick < PARAM_AUDIT_DRIVE_TICKS; tick++) {
        if (tick < 450) {
            state.selectedGear = 1 + tick / 90;
            if (state.selectedGear > spec->gearCount) state.selectedGear = spec->gearCount;
            /* With the dynamic engine (#23), issue one phased shift mid-run so the clutch
             * cut, gear swap and re-engage are exercised: a locked-clutch dynamic engine is
             * deliberately identical to the kinematic one, so without the shift the audit
             * would correctly see "nothing changed". */
            if (spec->engineInertiaKgM2 > 0.0f && tick == 300 &&
                state.selectedGear < spec->gearCount) {
                (void)drivetrain_request_shift(&state, state.selectedGear + 1);
            }
            input.throttle = 1.0f;
            input.brake = 0.0f;
            input.handbrake = 0.0f;
            input.steer = 0.15f;
        } else if (tick < 500) {
            input.throttle = 0.0f;
            input.brake = 1.0f;
            input.steer = 0.6f;
        } else if (tick < 520) {
            input.brake = 0.0f;
            input.handbrake = 1.0f;
            input.steer = 0.6f;
        } else {
            state.selectedGear = -1;
            input.handbrake = 0.0f;
            input.throttle = 1.0f;
            input.steer = 0.0f;
        }
        if (working.fuelEnabled > 0.0f) {
            vehicle_spec_set_fuel_mass(&working, fuelKg);
        }
        physics_fixed_update(&working, &state, &derived, &renderState, tireState, NULL, &fuelKg,
                             &input, FIXED_DT_S);
    }

    if (allFiniteOut != NULL) {
        *allFiniteOut = isfinite(state.positionM.x) && isfinite(state.positionM.y) &&
                        isfinite(state.velocityLongitudinalMps) && isfinite(state.yawRateRadS);
    }
    return param_audit_hash_bytes(0x811c9dc5u, &state, sizeof(state));
}

/* A second run that scrubs a barrier, so the collision material parameters are exercised by
 * the same harness rather than being exempted from it. */
static uint32_t param_audit_collide_signature(const VehicleSpec *spec,
                                              const TrackDefinition *track, int *contactsOut)
{
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    vehicle_state_reset(spec, &state, &derived, &renderState);

    /* The parking lot's bottom perimeter runs along y = -150 with a 4 m half-width, so the
     * outer barrier stands at y = -154. Start just inside it, drifting into it. */
    state.positionM = (Vector2){ 0.0f, -152.0f };
    state.velocityLongitudinalMps = 14.0f;
    state.velocityLateralMps = -3.0f;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        state.wheels[i].angularVelocityRadS = 14.0f / vehicle_wheel_radius_m(spec, i);
    }
    renderState.prevPositionM = renderState.currPositionM = state.positionM;
    renderState.prevHeadingRad = renderState.currHeadingRad = state.headingRad;

    ControllerOutput input;
    controller_output_zero(&input);
    input.throttle = 0.4f;
    input.steer = -0.2f;
    state.selectedGear = 3;

    float crashLockoutTimerS = 0.0f;
    int contacts = 0;
    CollisionWorld world;
    if (!collision_world_build_from_track(&world, track)) {
        /* A build failure is a harness failure, not a valid signature: 0u is a legitimate
         * hash, so returning it silently would collapse base and perturbed signatures to
         * the same 0 and mask any collision-parameter regression as "no behavior change". */
        check(false, "param audit: the collide track could not build a collision world");
        return 0u;
    }
    for (int tick = 0; tick < PARAM_AUDIT_COLLIDE_TICKS; tick++) {
        physics_fixed_update(spec, &state, &derived, &renderState, NULL, NULL, NULL, &input,
                             FIXED_DT_S);
        contacts += collision_resolve_track(&world, 1u, spec, &state, &renderState,
                                            &crashLockoutTimerS);
    }

    if (contactsOut != NULL) *contactsOut = contacts;
    uint32_t hash = param_audit_hash_bytes(0x811c9dc5u, &state, sizeof(state));
    return param_audit_hash_bytes(hash, &crashLockoutTimerS, sizeof(crashLockoutTimerS));
}

static uint32_t param_audit_signature(const VehicleSpec *spec, const TrackDefinition *track)
{
    const uint32_t drive = param_audit_drive_signature(spec, NULL);
    const uint32_t collide = param_audit_collide_signature(spec, track, NULL);
    return param_audit_hash_bytes(drive, &collide, sizeof(collide));
}

/* Candidate perturbations, largest last: a quarter-range step reaches every continuous
 * parameter, and the wider steps exist for the enum-valued ones (drive.layout, drive.diff_mode)
 * where a quarter step truncates back to the same enum. */
static int param_audit_candidates(const VehicleSpec *base, const DevParameter *param,
                                  VehicleSpec *out, int capacity)
{
    const float span = param->maximum - param->minimum;
    const float deltas[4] = { 0.25f * span, -0.25f * span, 0.75f * span, -0.75f * span };
    const float baseValue = dev_param_get(base, param);
    int count = 0;

    for (int i = 0; i < 4 && count < capacity; i++) {
        VehicleSpec candidate = *base;
        float target = baseValue + deltas[i];
        if (target < param->minimum) target = param->minimum;
        if (target > param->maximum) target = param->maximum;
        if (!dev_param_set(&candidate, param, target)) continue;
        if (fabsf(dev_param_get(&candidate, param) - baseValue) <= 1e-7f) continue;
        if (!vehicle_spec_is_valid(&candidate)) continue;
        out[count++] = candidate;
    }
    return count;
}

static void param_audit_check_coverage(void)
{
    const int count = dev_params_count();
    unsigned char covered[sizeof(VehicleSpec)];
    memset(covered, 0, sizeof(covered));

    int overlaps = 0;
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        check(param->offset + sizeof(float) <= sizeof(VehicleSpec),
              "'%s' lies inside VehicleSpec", param->name);
        for (size_t b = 0; b < sizeof(float); b++) {
            if (covered[param->offset + b] != 0) overlaps++;
            covered[param->offset + b] = 1;
        }
    }
    /* The int and bool live in the typed companion audit rather than being exempted. */
    for (int i = 0; i < dev_spec_field_audit_count(); i++) {
        const DevSpecFieldAudit *field = dev_spec_field_audit_at(i);
        check(field->offset + field->size <= sizeof(VehicleSpec),
              "typed field '%s' lies inside VehicleSpec", field->name);
        for (size_t b = 0; b < field->size; b++) {
            if (covered[field->offset + b] != 0) overlaps++;
            covered[field->offset + b] = 1;
        }
    }
    check(overlaps == 0, "no two audit entries describe the same VehicleSpec bytes");
    check(dev_spec_field_audit_count() == 2,
          "both non-float VehicleSpec fields have typed audit entries");

    /* Unused gear-ratio slots are array capacity rather than authored fields. */
    for (size_t b = (size_t)GEAR_COUNT * sizeof(float); b < (size_t)MAX_GEARS * sizeof(float);
         b++) {
        covered[offsetof(VehicleSpec, gearRatios) + b] = 1;
    }

    int uncovered = 0;
    size_t firstUncovered = 0;
    for (size_t b = 0; b < sizeof(VehicleSpec); b++) {
        if (covered[b] != 0) continue;
        /* Trailing alignment padding after the final bool is the only legitimate gap. */
        if (b > offsetof(VehicleSpec, lateralLoadTransferEnabled)) continue;
        if (uncovered == 0) firstUncovered = b;
        uncovered++;
    }
    check(uncovered == 0,
          "every VehicleSpec field is classified exactly once (%d unclassified byte(s), "
          "first at offset %u of %u)",
          uncovered, (unsigned)firstUncovered, (unsigned)sizeof(VehicleSpec));
}

static void param_audit_check_derived(const VehicleSpec *defaults)
{
    const int count = dev_params_count();
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);

        check((param->classification == DEV_CLASS_DERIVED) == param->derived,
              "'%s' agrees with itself about being derived", param->name);
        check((param->owner == DEV_OWNER_DERIVED) == param->derived,
              "'%s' derived readouts are owned beside the definition", param->name);

        /* Overwrite the one field, refresh, and see whether the refresh owns it. */
        VehicleSpec probe = *defaults;
        float *field = (float *)(void *)((unsigned char *)&probe + param->offset);
        const float marker = param->defaultValue + 1.0f;
        *field = marker;
        vehicle_spec_refresh_derived(&probe);

        if (param->derived) {
            check(fabsf(*field - param->defaultValue) <=
                      fmaxf(fabsf(param->defaultValue), 1.0f) * 1e-5f,
                  "'%s' is recomputed by vehicle_spec_refresh_derived()", param->name);
        } else {
            check(*field == marker, "'%s' is authored, not recomputed by a refresh",
                  param->name);
        }
    }
}

static void param_audit_check_owner(const VehicleSpec *defaults)
{
    VehicleDefinition stock;
    check(vehicle_definition_init(&stock, "audit/stock", "audit/stock", 1u, defaults),
          "the audit stock definition validates");

    VehicleSetup setup;
    vehicle_setup_set_default(&stock, &setup);

    const int count = dev_params_count();
    int skipped = 0;
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        if (param->derived) continue;

        VehicleSpec perturbed[4];
        if (param_audit_candidates(defaults, param, perturbed, 1) != 1) {
            skipped++;
            continue;
        }

        /* Move the DEFINITION only, then compile it against the stock setup. A setup-owned
         * field is overwritten on the way through; a definition-owned field survives. */
        VehicleDefinition moved;
        if (!vehicle_definition_init(&moved, "audit/moved", "audit/moved", 1u, &perturbed[0])) {
            skipped++;
            continue;
        }
        VehicleInstance instance;
        memset(&instance, 0, sizeof(instance));
        if (!vehicle_instance_derive(&instance, &moved, &setup)) {
            skipped++;
            continue;
        }

        const float authored = dev_param_get(&perturbed[0], param);
        const float compiled = dev_param_get(&instance.spec, param);
        const bool survived =
            fabsf(compiled - authored) <= 1e-6f * fmaxf(fabsf(authored), 1.0f);

        if (param->owner == DEV_OWNER_SETUP) {
            check(!survived,
                  "'%s' is setup-owned: VehicleSetup overwrites the definition value",
                  param->name);
        } else {
            check(survived, "'%s' is definition-owned: it survives vehicle_instance_derive()",
                  param->name);
        }
    }
    check(skipped == 0, "every primary parameter had a valid owner probe (%d skipped)",
          skipped);
}

static void param_audit_check_effect(const VehicleSpec *defaults, const TrackDefinition *track)
{
    /* Three baselines, because some parameters are live only under a configuration the stock
     * car does not use, and "live only when enabled" is still live:
     *
     *   - drive.front_torque_split does nothing until the layout is all-wheel drive;
     *   - steer.speed_ref does nothing while steer.speed_min_factor is 1.0, which is the
     *     stock value and the documented way to disable speed-sensitive steering;
     *   - aero.ref_area_front/rear are the A in 0.5*rho*Cl*A*v^2, so they do nothing while the
     *     stock car's lift coefficients are zero — which they are, deliberately, because no
     *     reviewed aerodynamic data exists for it (see src/core/config.h).
     *
     * Baseline 1 turns the first two on, baseline 2 fits the car with a wing. Neither is a
     * physics change: each configures the same solver. */
    VehicleSpec bases[3];
    bases[0] = *defaults;
    bases[1] = *defaults;
    bases[2] = *defaults;
    check(dev_param_set(&bases[1], dev_param_find("drive.layout"), (float)DRIVE_LAYOUT_AWD),
          "the audit second baseline configures an all-wheel-drive layout");
    check(dev_param_set(&bases[1], dev_param_find("steer.speed_min_factor"), 0.5f),
          "the audit second baseline enables speed-sensitive steering");
    check(dev_param_set(&bases[2], dev_param_find("aero.lift_front"), -1.0f) &&
              dev_param_set(&bases[2], dev_param_find("aero.lift_rear"), -1.5f),
          "the audit third baseline fits front and rear aerodynamic devices");

    for (int b = 0; b < 3; b++) {
        bool allFinite = false;
        int contacts = 0;
        (void)param_audit_drive_signature(&bases[b], &allFinite);
        (void)param_audit_collide_signature(&bases[b], track, &contacts);
        check(allFinite, "audit baseline %d stays finite through the scripted run", b);
        check(contacts > 0, "audit baseline %d actually strikes a barrier (%d contacts)", b,
              contacts);
    }

    uint32_t baseline[3];
    for (int b = 0; b < 3; b++) baseline[b] = param_audit_signature(&bases[b], track);

    const int count = dev_params_count();
    int unproven = 0;
    int leaked = 0;
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        if (param->derived) continue;

        bool changed = false;
        int probes = 0;
        for (int b = 0; b < 3; b++) {
            VehicleSpec candidates[4];
            const int candidateCount = param_audit_candidates(&bases[b], param, candidates, 4);
            probes += candidateCount;
            for (int c = 0; c < candidateCount; c++) {
                if (param_audit_signature(&candidates[c], track) != baseline[b]) {
                    changed = true;
                    if (param->classification == DEV_CLASS_PHYSICS_INPUT) break;
                    /* For an appearance or inactive claim, report the first leak by name. */
                    check(false, "'%s' is classified %s but changed the simulation",
                          param->name, dev_param_class_name(param->classification));
                    leaked++;
                    break;
                }
            }
            if (changed && param->classification == DEV_CLASS_PHYSICS_INPUT) break;
        }

        if (probes == 0) {
            check(false, "'%s' has no in-range perturbation to test", param->name);
            continue;
        }
        if (param->classification == DEV_CLASS_PHYSICS_INPUT && !changed) {
            /* The dynamic-engine clutch/duration fields (#23) are coupled to engine_inertia,
             * and the fuel capacity/rate fields (#24) are coupled to fuel_enabled: they only
             * move the car once their master switch is on. The dedicated coupled probes
             * prove them across the coupled extremes, so the one-field perturb is exempted
             * rather than reported. */
            if (strcmp(param->name, "drive.max_clutch_torque") == 0 ||
                strcmp(param->name, "drive.shift_duration") == 0 ||
                strcmp(param->name, "drive.fuel_tank_capacity") == 0 ||
                strcmp(param->name, "drive.fuel_rate") == 0)
                continue;
            check(false, "'%s' is classified physics but changed nothing", param->name);
            unproven++;
        }
    }

    check(unproven == 0, "every physics parameter demonstrably moves the car");
    check(leaked == 0, "no appearance or inactive parameter reaches the simulation");
}

static void param_audit_check_typed_fields(const VehicleSpec *defaults,
                                           const TrackDefinition *track)
{
    const DevSpecFieldAudit *gearCount = dev_spec_field_audit_at(0);
    const DevSpecFieldAudit *loadTransfer = dev_spec_field_audit_at(1);

    check(gearCount != NULL && strcmp(gearCount->name, "drive.gear_count") == 0 &&
              gearCount->offset == offsetof(VehicleSpec, gearCount) &&
              gearCount->size == sizeof(defaults->gearCount) &&
              gearCount->classification == DEV_CLASS_PHYSICS_INPUT &&
              gearCount->owner == DEV_OWNER_SETUP,
          "gearCount has one typed physics/setup audit entry");
    check(loadTransfer != NULL &&
              strcmp(loadTransfer->name, "physics.lateral_load_transfer_enabled") == 0 &&
              loadTransfer->offset == offsetof(VehicleSpec, lateralLoadTransferEnabled) &&
              loadTransfer->size == sizeof(defaults->lateralLoadTransferEnabled) &&
              loadTransfer->classification == DEV_CLASS_PHYSICS_INPUT &&
              loadTransfer->owner == DEV_OWNER_SESSION_RULES,
          "lateralLoadTransferEnabled has one typed physics/session-rules audit entry");
    check(defaults->gearCount == GEAR_COUNT && defaults->lateralLoadTransferEnabled,
          "typed-field defaults match their documented sources");

    VehicleSpec fewerGears = *defaults;
    fewerGears.gearCount = GEAR_COUNT - 1;
    check(vehicle_spec_is_valid(&fewerGears), "a reduced-gear typed-field probe is valid");
    check(param_audit_signature(&fewerGears, track) != param_audit_signature(defaults, track),
          "gearCount demonstrably changes the simulated trajectory");

    VehicleDefinition definition;
    check(vehicle_definition_init(&definition, "audit/typed", "audit/typed", 1u, defaults),
          "the typed-field owner probe definition validates");
    VehicleSetup setup;
    vehicle_setup_set_default(&definition, &setup);
    setup.gearCount = GEAR_COUNT - 1;
    VehicleInstance instance;
    memset(&instance, 0, sizeof(instance));
    check(vehicle_instance_derive(&instance, &definition, &setup) &&
              instance.spec.gearCount == GEAR_COUNT - 1,
          "gearCount is frozen from VehicleSetup when the instance is derived");

    VehicleSpec noLoadTransfer = *defaults;
    noLoadTransfer.lateralLoadTransferEnabled = false;
    check(param_audit_signature(&noLoadTransfer, track) !=
              param_audit_signature(defaults, track),
          "lateralLoadTransferEnabled demonstrably changes the simulated trajectory");
    VehicleDefinition noLoadTransferDefinition;
    check(vehicle_definition_init(&noLoadTransferDefinition, "audit/no-transfer",
                                  "audit/no-transfer", 1u, &noLoadTransfer) &&
              noLoadTransferDefinition.contentHash != definition.contentHash,
          "the current definition hash covers lateralLoadTransferEnabled until session rules "
          "own it");
}

/* The committed table is generated from the registry, so it cannot describe a parameter the
 * registry does not have — but it can go stale. Check the class word of every row. */
static void param_audit_check_document(void)
{
    const char *path = "docs/VEHICLE_PARAMETERS.md";
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        check(false, "%s exists (regenerate with `circuit_tests --dump-params %s`)", path,
              path);
        return;
    }
    fseek(file, 0, SEEK_END);
    const long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    char *text = (size > 0) ? (char *)malloc((size_t)size + 1u) : NULL;
    if (text == NULL) {
        fclose(file);
        check(false, "%s could be read", path);
        return;
    }
    const size_t read = fread(text, 1, (size_t)size, file);
    fclose(file);
    text[read] = '\0';

    const int count = dev_params_count();
    int missing = 0;
    int stale = 0;
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        char row[128];
        snprintf(row, sizeof(row), "| `%s` | `%s` |", param->name,
                 dev_param_class_name(param->classification));
        if (strstr(text, row) != NULL) continue;
        char anyRow[96];
        snprintf(anyRow, sizeof(anyRow), "| `%s` |", param->name);
        if (strstr(text, anyRow) == NULL)
            missing++;
        else
            stale++;
    }

    for (int i = 0; i < dev_spec_field_audit_count(); i++) {
        const DevSpecFieldAudit *field = dev_spec_field_audit_at(i);
        char row[160];
        snprintf(row, sizeof(row), "| `%s` | `%s` | `%s` | `%s` |", field->name, field->cType,
                 dev_param_class_name(field->classification),
                 dev_param_owner_name(field->owner));
        if (strstr(text, row) == NULL) stale++;
    }
    free(text);

    check(missing == 0, "%s documents every registry parameter (%d missing)", path, missing);
    check(stale == 0, "%s states the current class for every parameter (%d stale)", path,
          stale);
}

/* The dynamic-engine triplet (#23) is coupled: clutch torque and shift duration only matter
 * once engineInertiaKgM2 > 0, so the one-field-at-a-time perturb cannot prove them. This
 * probe enables the dynamic engine and compares drive trajectories across the coupled
 * extremes instead. */
static void param_audit_check_dynamic_engine(const VehicleSpec *defaults)
{
    VehicleSpec kin = *defaults;
    kin.engineInertiaKgM2 = 0.0f;
    const uint32_t kinSig = param_audit_drive_signature(&kin, NULL);

    VehicleSpec dyn = *defaults;
    dyn.engineInertiaKgM2 = 1.0f;
    const uint32_t dynSig = param_audit_drive_signature(&dyn, NULL);
    check(dynSig != kinSig, "dynamic engine changes the drive trajectory vs kinematic");

    VehicleSpec clutchLow = dyn;
    clutchLow.maxClutchTorqueNm = 100.0f;
    check(param_audit_drive_signature(&clutchLow, NULL) != dynSig,
          "clutch torque capacity changes the trajectory");

    VehicleSpec shiftFast = dyn;
    shiftFast.shiftDurationS = 0.05f;
    check(param_audit_drive_signature(&shiftFast, NULL) != dynSig,
          "short shift duration changes the trajectory");

    VehicleSpec shiftSlow = dyn;
    shiftSlow.shiftDurationS = 0.5f;
    check(param_audit_drive_signature(&shiftSlow, NULL) != dynSig,
          "long shift duration changes the trajectory");
}

/* The fuel triplet (#24) is coupled: tank capacity and rate only matter once fuel_enabled is
 * on. This probe enables the fuel model and compares drive trajectories across the extremes,
 * like the dynamic-engine probe does for the powertrain triplet. */
static void param_audit_check_fuel(const VehicleSpec *defaults)
{
    VehicleSpec off = *defaults;
    off.fuelEnabled = 0.0f;
    const uint32_t offSig = param_audit_drive_signature(&off, NULL);

    VehicleSpec on = *defaults;
    on.fuelEnabled = 1.0f;
    on.fuelConsumptionRateKgPerWS = 1e-5f; /* fast enough to burn visibly in the run */
    const uint32_t onSig = param_audit_drive_signature(&on, NULL);
    check(onSig != offSig, "fuel model changes the drive trajectory vs disabled");

    VehicleSpec thirsty = on;
    thirsty.fuelConsumptionRateKgPerWS = 5e-5f;
    check(param_audit_drive_signature(&thirsty, NULL) != onSig,
          "fuel rate changes the trajectory");

    /* Tank capacity is a refueling bound (issue #24), not a trajectory input: it only bites
     * through the service hook. Prove it bounds the service. */
    {
        VehicleInstance inst;
        memset(&inst, 0, sizeof(inst));
        inst.spec = on;
        inst.fuelKg = 0.0f;
        vehicle_refuel(&inst, 999.0f); /* far beyond any capacity */
        const float capacityKg = on.fuelTankCapacityL * FUEL_DENSITY_KG_PER_L;
        check(inst.fuelKg <= capacityKg + 1e-3f,
              "refuel respects tank capacity (%.2f <= %.2f kg)", (double)inst.fuelKg,
              (double)capacityKg);
        check(inst.fuelKg > 0.0f, "refuel actually adds fuel (%.2f kg)", (double)inst.fuelKg);
    }
}

static void scenario_param_audit(void)
{
    VehicleSpec defaults;
    vehicle_spec_set_default(&defaults);

    TrackDefinition track;
    memset(&track, 0, sizeof(track));
    track_init(&track);

    param_audit_check_coverage();
    param_audit_check_derived(&defaults);
    param_audit_check_owner(&defaults);
    param_audit_check_effect(&defaults, &track);
    param_audit_check_typed_fields(&defaults, &track);
    param_audit_check_dynamic_engine(&defaults);
    param_audit_check_fuel(&defaults);
    param_audit_check_document();

    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: devreplay — durable timelines and the inspector's event markers               */
/* ------------------------------------------------------------------------------------- */

static void scenario_dev_replay(void)
{
    static ReplayBuffer source;
    static ReplayBuffer restored;

    replay_begin_recording(&source, 100u);
    VehicleDefinition replayDefinition;
    VehicleSetup replaySetup;
    VehicleInstance replayInstance;
    vehicle_definition_set_default(&replayDefinition);
    vehicle_setup_set_default(&replayDefinition, &replaySetup);
    check(vehicle_instance_init(&replayInstance, &replayDefinition, &replaySetup),
          "the durable replay fixture initializes its vehicle");
    replaySetup.brakeBiasFront -= 0.02f;
    check(vehicle_instance_derive(&replayInstance, &replayDefinition, &replaySetup),
          "the durable replay fixture applies its setup");
    replayInstance.vehicle.positionM = (Vector2){ 7.0f, -2.0f };
    replayInstance.vehicle.velocityLongitudinalMps = 12.5f;
    replayInstance.vehicleControls.throttle = 0.6f;
    replayInstance.autoTrans.driveState = AUTO_REVERSE;
    replayInstance.fuelKg = 3.5f;
    replayInstance.tireState[WHEEL_REAR_RIGHT].wear = 0.15f;
    replayInstance.damage = 0.25f;
    replay_capture_initial_vehicle(&source, &replayDefinition, &replaySetup, &replayInstance);
    for (int i = 0; i < 240; i++) {
        Input in;
        input_zero(&in);
        in.throttle = (i >= 20 && i < 120) ? 1.0f : 0.0f;
        in.brake = (i >= 150 && i < 180) ? 1.0f : 0.0f;
        in.handbrake = (i >= 200) ? 1.0f : 0.0f;
        in.steer = (i < 60) ? 0.5f : -0.5f;
        if (i == 90) in.shiftUpPressed = true;
        if (i == 130) in.resetPressed = true;
        replay_record(&source, &in);
    }
    check(source.count == 240, "the source timeline holds 240 frames");

    check(telemetry_ensure_dir("artifacts"), "the artifacts directory exists or was created");
    const char *path = "artifacts/_test_replay.bin";
    check(dev_replay_save(&source, path, "unit-test", 4242u, 0xdeadbeefu),
          "a timeline writes to disk");

    DevReplayInfo info;
    memset(&info, 0, sizeof(info));
    check(dev_replay_load(&restored, path, &info), "the timeline loads back");
    check(info.frameCount == 240u, "the header records 240 frames (got %u)", info.frameCount);
    check(info.firstTick == 100u, "the header records the first tick");
    check(info.seed == 4242u, "the header records the seed");
    check(info.finalChecksum == 0xdeadbeefu, "the header records the final checksum");
    check(strcmp(info.label, "unit-test") == 0, "the header records the label");
    check(restored.count == source.count, "every frame came back");
    VehicleSetup restoredSetup;
    VehicleInstance restoredInstance;
    memset(&restoredSetup, 0, sizeof(restoredSetup));
    memset(&restoredInstance, 0, sizeof(restoredInstance));
    check(replay_restore_initial_vehicle(&restored, &replayDefinition, &restoredSetup,
                                         &restoredInstance),
          "the durable replay restores its definition-referenced vehicle snapshot");
    check(restoredSetup.brakeBiasFront == replaySetup.brakeBiasFront &&
              restoredInstance.vehicle.positionM.x == replayInstance.vehicle.positionM.x &&
              restoredInstance.vehicle.velocityLongitudinalMps ==
                  replayInstance.vehicle.velocityLongitudinalMps &&
              restoredInstance.vehicleControls.throttle ==
                  replayInstance.vehicleControls.throttle &&
              restoredInstance.autoTrans.driveState == replayInstance.autoTrans.driveState &&
              restoredInstance.fuelKg == replayInstance.fuelKg &&
              restoredInstance.tireState[WHEEL_REAR_RIGHT].wear ==
                  replayInstance.tireState[WHEEL_REAR_RIGHT].wear &&
              restoredInstance.damage == replayInstance.damage,
          "durable replay vehicle state is field-identical after save/load");

    int differences = 0;
    for (int i = 0; i < source.count; i++) {
        const ReplayFrame *a = dev_replay_frame_at(&source, i);
        const ReplayFrame *b = dev_replay_frame_at(&restored, i);
        if (a == NULL || b == NULL || memcmp(a, b, sizeof(ReplayFrame)) != 0) differences++;
    }
    check(differences == 0, "the round trip is bit-identical");

    /* Malformed input is rejected rather than trusted. */
    unsigned char small[16];
    memset(small, 0, sizeof(small));
    check(!dev_replay_parse(small, sizeof(small), &restored, NULL),
          "a buffer shorter than the header is rejected");

    static unsigned char blob[64 + 20 * 8];
    memset(blob, 0xA5, sizeof(blob));
    check(!dev_replay_parse(blob, sizeof(blob), &restored, NULL),
          "a buffer with the wrong magic is rejected");

    FILE *file = fopen(path, "rb");
    static unsigned char truncated[4096];
    const size_t read = (file != NULL) ? fread(truncated, 1, sizeof(truncated), file) : 0;
    if (file != NULL) fclose(file);
    check(read > 64, "the saved file is larger than its header");
    check(!dev_replay_parse(truncated, read, &restored, NULL),
          "a file truncated mid-timeline is rejected");
    remove(path);

    /* Event markers: the inspector draws exactly what this returns. */
    static DevReplayEvent events[64];
    const int eventCount = dev_replay_collect_events(&source, events, 64);
    check(eventCount > 0, "the timeline produces event markers (%d)", eventCount);

    int throttle = 0, brake = 0, handbrake = 0, shiftUp = 0, reset = 0, reversal = 0;
    for (int i = 0; i < eventCount; i++) {
        switch (events[i].kind) {
            case DEV_REPLAY_EVENT_THROTTLE: throttle++; break;
            case DEV_REPLAY_EVENT_BRAKE: brake++; break;
            case DEV_REPLAY_EVENT_HANDBRAKE: handbrake++; break;
            case DEV_REPLAY_EVENT_SHIFT_UP: shiftUp++; break;
            case DEV_REPLAY_EVENT_RESET: reset++; break;
            case DEV_REPLAY_EVENT_STEER_REVERSAL: reversal++; break;
            default: break;
        }
    }
    check(throttle == 2, "both throttle edges are marked (%d)", throttle);
    check(brake == 2, "both brake edges are marked (%d)", brake);
    check(handbrake == 1, "the handbrake pull is marked (%d)", handbrake);
    check(shiftUp == 1, "the shift is marked (%d)", shiftUp);
    check(reset == 1, "the reset is marked (%d)", reset);
    check(reversal == 1, "the steering reversal is marked (%d)", reversal);

    const int firstThrottle = events[0].index;
    check(events[0].tick == source.firstTick + (uint64_t)firstThrottle,
          "event ticks are absolute, not window-relative");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: auto-transmission — gear selection and arcade reverse state machine         */
/* ------------------------------------------------------------------------------------- */

static void scenario_auto_transmission(void)
{
    VehicleSpec spec;
    VehicleState vs;
    VehicleDerived derived;
    ControllerOutput io;
    AutoTransmission at;

    vehicle_spec_set_default(&spec);
    memset(&vs, 0, sizeof(vs));
    memset(&derived, 0, sizeof(derived));
    controller_output_zero(&io);
    memset(&at, 0, sizeof(at));

    /* Disabled: nothing changes. */
    at.enabled = false;
    at.driveState = AUTO_DRIVE;
    vs.selectedGear = 2;
    vs.engineRpm = spec.engineRedlineRpm;
    io.throttle = 1.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check(vs.selectedGear == 2, "disabled auto transmission does not change gear");
    check(at.driveState == AUTO_DRIVE,
          "disabled auto transmission does not change drive state");

    /* Upshift near redline. */
    at.enabled = true;
    at.driveState = AUTO_DRIVE;
    at.neutralTimer = 0.0f;
    vs.selectedGear = 1;
    vs.engineRpm = spec.engineRedlineRpm * 0.90f; /* above 0.85 upshift factor */
    derived.speedMps = 10.0f;
    io.throttle = 1.0f;
    io.brake = 0.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check(vs.selectedGear == 2, "high RPM upshifts from 1 to 2 (got %d)", vs.selectedGear);

    /* Downshift at low RPM. */
    vs.selectedGear = 3;
    vs.engineRpm = spec.engineRedlineRpm * 0.20f; /* below 0.35 downshift factor */
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check(vs.selectedGear == 2, "low RPM downshifts from 3 to 2 (got %d)", vs.selectedGear);

    /* Brake-to-stop enters neutral. */
    vs.selectedGear = 2;
    vs.engineRpm = 2000.0f;
    derived.speedMps = 0.2f; /* below AUTO_STOP_THRESHOLD_MPS 0.5 */
    io.throttle = 0.0f;
    io.brake = 1.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check(at.driveState == AUTO_NEUTRAL, "brake-to-stop enters AUTO_NEUTRAL");
    check(vs.selectedGear == 0, "brake-to-stop selects neutral gear");

    /* Neutral delay + throttle returns to drive. */
    at.driveState = AUTO_NEUTRAL;
    at.neutralTimer = 0.0f;
    vs.selectedGear = 0;
    derived.speedMps = 0.0f;
    io.throttle = 1.0f;
    io.brake = 0.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, 0.10f);
    check(at.driveState == AUTO_NEUTRAL, "neutral waits for AUTO_NEUTRAL_DELAY_S");
    check_near((double)io.throttle, 0.0, 1e-6, "neutral zeroes throttle after the check");
    io.throttle = 1.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, 0.10f); /* total 0.20 > 0.15 */
    check(at.driveState == AUTO_DRIVE, "neutral + throttle returns to AUTO_DRIVE");
    check(vs.selectedGear == 1, "neutral + throttle selects first gear");

    /* Neutral delay + brake enters reverse. */
    at.driveState = AUTO_NEUTRAL;
    at.neutralTimer = 0.20f;
    vs.selectedGear = 0;
    io.throttle = 0.0f;
    io.brake = 1.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check(at.driveState == AUTO_REVERSE, "neutral + brake enters AUTO_REVERSE");
    check(vs.selectedGear == -1, "neutral + brake selects reverse gear");

    /* Reverse swaps throttle/brake pedals. */
    at.driveState = AUTO_REVERSE;
    vs.selectedGear = -1;
    derived.speedMps = 3.0f;
    io.throttle = 0.25f; /* original up-pedal = brake in reverse */
    io.brake = 0.80f;    /* original down-pedal = throttle in reverse */
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check_near((double)io.throttle, 0.80, 1e-6, "reverse maps brake pedal to throttle");
    check_near((double)io.brake, 0.25, 1e-6, "reverse maps throttle pedal to brake");
    check(at.driveState == AUTO_REVERSE, "moving reverse does not exit on swapped pedals");

    /* Reverse brake-to-stop uses the original throttle pedal before the swap. */
    at.driveState = AUTO_REVERSE;
    vs.selectedGear = -1;
    derived.speedMps = 0.1f;
    io.throttle = 1.0f; /* original up-pedal while nearly stopped */
    io.brake = 0.0f;
    auto_transmission_update(&at, &vs, &spec, &derived, &io, FIXED_DT_S);
    check(at.driveState == AUTO_NEUTRAL, "reverse brake-to-stop returns to AUTO_NEUTRAL");
    check(vs.selectedGear == 0, "reverse brake-to-stop selects neutral gear");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: presets — registry shape and deterministic apply                            */
/* ------------------------------------------------------------------------------------- */

static void scenario_presets(void)
{
    const int count = dev_preset_count();
    check(count == 10, "there are exactly ten driving presets (got %d)", count);

    const DevPreset *stock = dev_preset_at(0);
    check(stock != NULL, "preset 0 exists");
    check(strcmp(stock->name, "Stock Baseline") == 0, "preset 0 is Stock Baseline");
    check(dev_preset_at(-1) == NULL, "negative preset index is rejected");
    check(dev_preset_at(count) == NULL, "out-of-range preset index is rejected");
    check(dev_preset_apply(NULL, 1) == 0, "NULL spec apply is a no-op");

    VehicleSpec dirty;
    vehicle_spec_set_default(&dirty);
    dirty.massKg = 2000.0f;
    dirty.engineRedlineRpm = 5000.0f;
    const int stockApplied = dev_preset_apply(&dirty, 0);
    check(stockApplied == 0, "Stock Baseline applies zero overrides (got %d)", stockApplied);
    VehicleSpec defaults;
    vehicle_spec_set_default(&defaults);
    check_near((double)dirty.massKg, (double)defaults.massKg, 1e-4,
               "Stock Baseline restores massKg to the default");
    check_near((double)dirty.engineRedlineRpm, (double)defaults.engineRedlineRpm, 1e-4,
               "Stock Baseline restores engineRedlineRpm to the default");

    VehicleSpec a, b;
    vehicle_spec_set_default(&a);
    vehicle_spec_set_default(&b);
    a.massKg = 1800.0f;
    b.massKg = 700.0f;
    const int appliedA = dev_preset_apply(&a, 1);
    const int appliedB = dev_preset_apply(&b, 1);
    check(appliedA > 0, "Touge Hero applies a non-empty override set");
    check(appliedA == appliedB,
          "Touge Hero applies the same override count from any prior state");
    check_near((double)a.massKg, 950.0, 1e-3, "Touge Hero sets massKg to 950");
    check_near((double)b.massKg, 950.0, 1e-3, "Touge Hero mass is independent of prior tuning");
    check_near((double)a.engineRedlineRpm, 8000.0, 1e-3, "Touge Hero sets redline to 8000");
    check_near((double)a.massKg, (double)b.massKg, 1e-6,
               "preset apply is deterministic across dirty starting specs");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: dev-state — scope recording, markers, scenario control, invariant latch     */
/* ------------------------------------------------------------------------------------- */

static void scenario_dev_state(void)
{
    Game *game = alloc_game();
    game_init(game);

    check(game->dev.scopeCount == 0, "dev_state_init starts with an empty scope");
    check(!game->dev.invariantFailed, "dev_state_init clears the invariant latch");
    check(game->dev.markerCount == 0, "dev_state_init starts with no markers");

    /* A normal fixed update records scope history and applied input. */
    game->input.throttle = 0.55f;
    game_fixed_update(game, FIXED_DT_S);
    check(game->dev.scopeCount >= 1, "dev_state_record retains at least one scope sample");
    check_near((double)game->dev.appliedInput.throttle, 0.55, 1e-6,
               "dev_state_record stores the applied throttle");

    /* Throttle edge should produce a marker. */
    bool sawThrottleOn = false;
    for (int i = 0; i < game->dev.markerCount; i++) {
        if (game->dev.markers[i].kind == DEV_MARKER_THROTTLE_ON) sawThrottleOn = true;
    }
    check(sawThrottleOn, "throttle rising edge records DEV_MARKER_THROTTLE_ON");

    /* Scenario start resets sim state and begins a reproducible run. */
    const int accel = dev_scenario_find("accel");
    check(accel > 0, "the accel scenario is registered");
    game->vehicle.positionM.x = 42.0f;
    game->dev.invariantFailed = true;
    game->dev.invariantCount = 3;
    snprintf(game->dev.invariantText, sizeof(game->dev.invariantText), "stale");
    dev_state_scenario_start(game, accel);
    check(game->dev.scenarioRunning, "scenario start marks the scenario as running");
    check(game->dev.scenario == accel, "scenario start stores the scenario index");
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "scenario start resets the simulation");
    check(!game->dev.invariantFailed, "scenario start clears a latched invariant");
    check(game->dev.invariantCount == 0, "scenario start zeroes the invariant count");
    check(game->dev.scopeCount == 0, "scenario start clears scope history");

    dev_state_scenario_stop(game);
    check(!game->dev.scenarioRunning, "scenario stop ends the scripted run");

    /* Force a clear invariant violation through the public recorder. */
    ControllerOutput applied;
    controller_output_zero(&applied);
    game->derived.speedMps = MAX_SAFE_SPEED_MPS + 25.0f;
    dev_state_record(game, &applied);
    check(game->dev.invariantFailed, "overspeed latches invariantFailed");
    check(game->dev.invariantCount >= 1, "overspeed increments invariantCount");
    /* physics_state_is_valid() rejects overspeed first; evaluate_invariants surfaces that. */
    check(strstr(game->dev.invariantText, "physics_state_is_valid") != NULL,
          "invariant text names physics_state_is_valid (got '%s')", game->dev.invariantText);

    bool sawInvariant = false;
    for (int i = 0; i < game->dev.markerCount; i++) {
        if (game->dev.markers[i].kind == DEV_MARKER_INVARIANT) sawInvariant = true;
    }
    check(sawInvariant, "first invariant violation pushes DEV_MARKER_INVARIANT");

    const uint64_t firstTick = game->dev.invariantTick;
    const int countAfterFirst = game->dev.invariantCount;
    game->derived.speedMps = MAX_SAFE_SPEED_MPS + 40.0f;
    dev_state_record(game, &applied);
    check(game->dev.invariantTick == firstTick, "later violations keep the first latch tick");
    check(game->dev.invariantCount == countAfterFirst + 1,
          "later violations still increment the cumulative count");

    dev_state_clear_invariants(&game->dev);
    check(!game->dev.invariantFailed, "clear_invariants drops the latch");
    check(game->dev.invariantCount == 0, "clear_invariants zeroes the cumulative count");

    free(game);
}

/*
 * scenario_cross_spec_invariant_reproducibility — runs a simple acceleration
 * maneuver against a small subset of corpus specs and asserts structural
 * invariants hold for each, using game_apply_spec() to safely switch specs
 * after game_init().
 */
static void scenario_cross_spec_invariant_reproducibility(void)
{
    /* Pick a few corpus indices spanning archetypes and sweeps */
    const int indices[] = { 0, 3, 7, 12, 20 };
    const int n = sizeof(indices) / sizeof(indices[0]);
    int tested = 0;

    for (int k = 0; k < n; k++) {
        VehicleSpec spec;
        if (!car_corpus_spec(indices[k], &spec)) continue;

        Game *game = alloc_game();
        game_init(game);
        game_apply_spec(game, &spec);

        /* Simple maneuver: straight-line acceleration from rest */
        bool finite = true;
        float peakUsage = 0.0f, peakSpeed = 0.0f;
        for (int i = 0; i < 240; i++) {
            game->input.throttle = 0.80f;
            game_fixed_update(game, FIXED_DT_S);
            if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived))
                finite = false;
            for (int w = 0; w < WHEEL_COUNT; w++) {
                if (game->vehicle.wheels[w].frictionUsage > peakUsage)
                    peakUsage = game->vehicle.wheels[w].frictionUsage;
            }
            if (game->derived.speedMps > peakSpeed) peakSpeed = game->derived.speedMps;
        }

        check(finite, "cross-spec idx %d: state finite throughout maneuver", indices[k]);
        check(peakUsage <= 1.0f + FRICTION_TOLERANCE,
              "cross-spec idx %d: friction within budget (peak %.3f)", indices[k],
              (double)peakUsage);
        check(peakSpeed > 1.0f, "cross-spec idx %d: achieves measurable speed (%.1f m/s)",
              indices[k], (double)peakSpeed);

        free(game);
        tested++;
    }

    check(tested > 0, "cross-spec: tested %d corpus specs", tested);
}

/*
 * scenario_low_speed_tight_turn_slip — full-lock, near-zero-speed U-turn maneuvering (the
 * kind the game's parking-lot track mode is built for, per Track.isParkingLot in
 * world/track.h), asserting the model's lateral velocity/slip stays close to the low-speed
 * zero-slip assumption other scenarios rely on, rather than drifting away from it the way a
 * naive kinematic-bicycle blend might at very tight radii.
 *
 * NOT a duplicate of `low-speed` (physics_tests.c): that scenario checks continuity of the
 * kinematic/dynamic blend along a single acceleration trajectory, not full-lock steering at
 * near-zero forward speed. It also is not a duplicate of `rest` (stationary steering with no
 * forward motion at all). This scenario is the missing middle case: slow but moving, at the
 * steering angle limit, the exact regime the reference paper identifies as where the
 * zero-slip assumption starts to measurably break down.
 *
 * Reference: Diener, Kalkkuhl & Enzweiler, "Lateral Velocity Model for Vehicle Parking
 * Applications" (arXiv:2511.01369) — identifies a "systematic deviation from the zero-slip
 * assumption" during low-speed parking-style maneuvers that the common zero-rear-slip model
 * misses. Circuit's arcade model is not held to that paper's accuracy bar, but the finding
 * motivates a scenario that actually measures the deviation instead of assuming it away.
 *
 * WHAT IT DOES. Starts just above LOW_SPEED_BEGIN_MPS so the kinematic/dynamic blend is
 * active, then holds full steering lock and light throttle for two seconds. It records rear
 * slip angle and yaw rate across the maneuver and asserts the state stays finite, that the run
 * actually traverses the blend (rather than sitting on one side of it and proving nothing),
 * that yaw response is continuous through the blend, and that rear slip stays inside the
 * small-angle bound the blend's zero-slip assumption relies on.
 */
static void scenario_low_speed_tight_turn_slip(void)
{
    Game *game = alloc_game();
    game_init(game);
    /* Start just above the kinematic threshold so the blend is active. */
    set_vehicle_rolling_speed(game, LOW_SPEED_BEGIN_MPS);

    check(vehicle_spec_is_valid(&game->spec), "spec is valid before low-speed-tight-turn-slip");

    float peakRearSlip = 0.0f, peakYawRate = 0.0f, maxYawJump = 0.0f;
    float previousYaw = 0.0f;
    bool allFinite = true, crossedBlend = false;

    for (int i = 0; i < 240; i++) {
        game->input.steer = 1.0f;
        game->input.throttle = 0.30f;
        game->input.brake = 0.0f;

        game_fixed_update(game, FIXED_DT_S);

        const float jump = fabsf(game->vehicle.yawRateRadS - previousYaw);
        if (jump > maxYawJump) maxYawJump = jump;
        previousYaw = game->vehicle.yawRateRadS;

        if (game->derived.lowSpeedBlend > 0.0f && game->derived.lowSpeedBlend < 1.0f)
            crossedBlend = true;

        const float rs = fabsf(game->derived.rearSlipAngleRad);
        const float yr = fabsf(game->vehicle.yawRateRadS);
        if (rs > peakRearSlip) peakRearSlip = rs;
        if (yr > peakYawRate) peakYawRate = yr;
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived))
            allFinite = false;
    }

    check(allFinite, "tight-turn: state remains finite at full lock low speed");
    check(crossedBlend, "tight-turn: traverses the kinematic/dynamic blend at full lock");
    check(maxYawJump < 0.1f, "tight-turn: yaw response continuous through blend (jump %.4f)",
          (double)maxYawJump);
    check(peakYawRate > 0.01f, "tight-turn: full lock produces measurable yaw (%.4f rad/s)",
          (double)peakYawRate);
    check(peakRearSlip < 0.15f,
          "tight-turn: rear slip stays within blend bound (peak %.4f rad)",
          (double)peakRearSlip);

    free(game);
}

/*
 * scenario_peak_friction_slip_sweep — sweeps longitudinal slip ratio across a wide range at a
 * fixed load and surface, locating where the tire's mu-slip curve peaks, and asserts the
 * curve is unimodal (rises to one peak, then falls) rather than having a spurious second peak
 * or a peak outside the physically expected slip range.
 *
 * NOT a duplicate of `tire` (physics_tests.c): that scenario checks the nonlinear curve shape
 * and combined-friction ellipse at a handful of fixed sample points as unit-level checks on
 * tire_force(). This scenario instead sweeps the *whole* slip-ratio domain and asserts a
 * global shape property (unimodality, peak location) that a handful of fixed samples cannot
 * establish on their own.
 *
 * Reference: Liang, Zhou, Huang & Li, "High-Slip-Ratio Control for Peak Tire-Road Friction
 * Estimation Using Automated Vehicles" (arXiv:2603.09073) — the whole premise of the paper is
 * that naturalistic (mild) driving under-excites slip and never reaches the peak-friction
 * region, so peak mu can only be found by deliberately sweeping to high slip ratios. This
 * scenario is the test-side equivalent: deliberately sweep the model's slip-ratio domain
 * rather than only sampling the mild-driving region the other physics scenarios stay in.
 *
 * WHAT IT DOES. Sweeps slip ratio from 0 to 3.0 in 0.002 steps at a fixed load and mu,
 * asserting the longitudinal force rises to a single interior maximum and then falls
 * monotonically — a second peak, plateau, or oscillation is a model defect — and that the peak
 * lands in the slip range tire.h documents. The lateral curve gets the same treatment over
 * slip angle.
 */
static void scenario_peak_friction_slip_sweep(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);

    check(vehicle_spec_is_valid(&spec), "spec is valid before peak-friction-slip-sweep");

    /* ---------- longitudinal: uni-modal mu-slip curve ----------
     * Sweep slip ratio from 0 to well past the expected peak, holding load
     * and surface fixed. The force must rise monotonically (non-decreasing)
     * to a single interior peak and then fall monotonically (non-increasing).
     * A second peak, plateau, or oscillation indicates a model defect. */
    {
        const float loadN = 4000.0f;
        const float mu = 1.0f;
        const float b = TIRE_B_LONG;
        const float c = TIRE_C_LONG;
        const float step = 0.002f;
        const float maxSlip = 3.0f;

        float prevForce = 0.0f;
        float peakForce = 0.0f;
        float peakSlip = 0.0f;
        bool pastPeak = false;
        int directionChanges = 0;

        for (float slip = step; slip <= maxSlip; slip += step) {
            const float force = fabsf(tire_longitudinal_force_n(slip, loadN, b, c, mu));

            if (!pastPeak) {
                if (force >= prevForce - 1e-6f) {
                    /* still rising or plateau — track peak */
                    if (force > peakForce) {
                        peakForce = force;
                        peakSlip = slip;
                    }
                } else {
                    /* first decrease after the peak */
                    pastPeak = true;
                    directionChanges++;
                }
            } else {
                /* past the peak, must be non-increasing (monotonic fall) */
                check(force <= prevForce + 1e-6f,
                      "longitudinal force falls monotonically past peak "
                      "(slip %.3f: force %.1f > prev %.1f)",
                      (double)slip, (double)force, (double)prevForce);
            }
            prevForce = force;
        }

        check(pastPeak, "longitudinal force has a finite interior peak");
        check(directionChanges == 1,
              "longitudinal force has exactly one peak (direction changes: %d)",
              directionChanges);
        check(peakSlip > 0.04f && peakSlip < 0.40f,
              "longitudinal peak slip is in the physically expected range (%.3f)",
              (double)peakSlip);
        check_near(peakForce, mu * loadN, mu * loadN * 0.15,
                   "longitudinal peak force is approximately mu*Fz");
    }

    /* ---------- lateral: uni-modal force-vs-slip-angle curve ---------- */
    {
        const float loadN = 4000.0f;
        const float mu = 1.0f;
        const float b = TIRE_B_LAT_REAR;
        const float c = TIRE_C_LAT_REAR;
        const float step = 0.002f;
        const float maxSlip = 3.0f;

        float prevForce = 0.0f;
        float peakForce = 0.0f;
        float peakSlip = 0.0f;
        bool pastPeak = false;
        int directionChanges = 0;

        for (float slip = step; slip <= maxSlip; slip += step) {
            /* lateral force opposes slip — negative for positive slip */
            const float force = fabsf(tire_lateral_force_n(slip, loadN, b, c, mu));

            if (!pastPeak) {
                if (force >= prevForce - 1e-6f) {
                    if (force > peakForce) {
                        peakForce = force;
                        peakSlip = slip;
                    }
                } else {
                    pastPeak = true;
                    directionChanges++;
                }
            } else {
                check(force <= prevForce + 1e-6f,
                      "lateral force falls monotonically past peak "
                      "(slip %.3f: force %.1f > prev %.1f)",
                      (double)slip, (double)force, (double)prevForce);
            }
            prevForce = force;
        }

        check(pastPeak, "lateral force has a finite interior peak");
        check(directionChanges == 1,
              "lateral force has exactly one peak (direction changes: %d)", directionChanges);
        check(peakSlip > 0.03f && peakSlip < 0.40f,
              "lateral peak slip is in the physically expected range (%.3f)", (double)peakSlip);
        check_near(peakForce, mu * loadN, mu * loadN * 0.15,
                   "lateral peak force is approximately mu*Fz");
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: drivetrain-layout                                                              */
/* ------------------------------------------------------------------------------------- */

/* Same spec, same power-oversteer script, three layouts. Asserts that the layout parameter
 * actually changes how torque is routed and how the car behaves under power. */
static void scenario_drivetrain_layout(void)
{
    VehicleSpec baseSpec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&baseSpec, &state, &derived, &renderState);

    /* Make the default car oversteer-prone: lower rear mu and raise engine torque so that
     * drive torque to the rear breaks traction. LSD allows wheel-speed differentiation. The
     * stock steering geometry (0.45 rad max) is left alone so 12 m/s is a sane cornering
     * speed for steer input 0.20. */
    baseSpec.tireMuLatRear = 0.80f;
    baseSpec.engineTorqueCurveNm[3] = 450.0f;
    baseSpec.engineTorqueCurveNm[4] = 440.0f;
    baseSpec.engineTorqueCurveNm[5] = 410.0f;
    baseSpec.engineTorqueCurveNm[6] = 380.0f;
    baseSpec.differentialMode = (float)DIFF_LSD;
    baseSpec.differentialBiasRatio = 2.0f;
    baseSpec.differentialPreloadNm = 60.0f;
    vehicle_spec_refresh_derived(&baseSpec);

    const float restOmega[WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };
    const float noReaction[WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

    /* --- Torque routing: which axle receives drive torque? --- */
    for (int layout = DRIVE_LAYOUT_RWD; layout <= DRIVE_LAYOUT_AWD; layout++) {
        VehicleSpec spec = baseSpec;
        spec.drivetrainLayout = (float)layout;
        if (layout == DRIVE_LAYOUT_AWD) spec.frontTorqueSplit = 0.5f;

        DrivetrainTorques t =
            drivetrain_calculate_torques(&spec, 1, restOmega, noReaction, 1.0f, 0.0f, 0.0f);
        const float frontDrive =
            t.driveTorqueNm[WHEEL_FRONT_LEFT] + t.driveTorqueNm[WHEEL_FRONT_RIGHT];
        const float rearDrive =
            t.driveTorqueNm[WHEEL_REAR_LEFT] + t.driveTorqueNm[WHEEL_REAR_RIGHT];

        if (layout == DRIVE_LAYOUT_RWD) {
            check(frontDrive == 0.0f, "RWD: front wheels receive zero drive torque");
            check(rearDrive > 0.0f, "RWD: rear wheels receive drive torque");
        } else if (layout == DRIVE_LAYOUT_FWD) {
            check(frontDrive > 0.0f, "FWD: front wheels receive drive torque");
            check(rearDrive == 0.0f, "FWD: rear wheels receive zero drive torque");
        } else { /* AWD */
            check(frontDrive > 0.0f, "AWD: front wheels receive drive torque");
            check(rearDrive > 0.0f, "AWD: rear wheels receive drive torque");
        }
    }

    /* --- Slip behaviour under power: driven axle shows higher longitudinal slip ---.
     * Slip ratio directly reflects where the drive torque goes: the driven wheels spin
     * faster (positive slip ratio) while undriven wheels are dragged (near-zero). This is
     * more robust than slip-angle differential, which depends on yaw-rate dynamics that can
     * invert at extreme power levels (RWD wheelspin drops force below the tire peak). */
    float rearSlipRatio[3] = { 0.0f, 0.0f, 0.0f };
    float frontSlipRatio[3] = { 0.0f, 0.0f, 0.0f };
    for (int layout = DRIVE_LAYOUT_RWD; layout <= DRIVE_LAYOUT_AWD; layout++) {
        VehicleSpec spec = baseSpec;
        spec.drivetrainLayout = (float)layout;
        if (layout == DRIVE_LAYOUT_AWD) spec.frontTorqueSplit = 0.5f;
        vehicle_spec_refresh_derived(&spec);
        vehicle_state_reset(&spec, &state, &derived, &renderState);
        state.velocityLongitudinalMps = 12.0f;

        ControllerOutput input;
        controller_output_zero(&input);
        input.steer = 0.20f;
        input.throttle = 0.15f;
        for (int i = 0; i < 120; i++)
            physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL,
                                 &input, FIXED_DT_S);

        input.throttle = 1.0f;
        for (int i = 0; i < 180; i++) {
            physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL,
                                 &input, FIXED_DT_S);
            rearSlipRatio[layout] =
                fmaxf(rearSlipRatio[layout], state.wheels[WHEEL_REAR_LEFT].slipRatio);
            frontSlipRatio[layout] =
                fmaxf(frontSlipRatio[layout], state.wheels[WHEEL_FRONT_LEFT].slipRatio);
        }
    }

    /* RWD: rear slip ratio >> front (all drive goes rear). */
    check(rearSlipRatio[DRIVE_LAYOUT_RWD] > frontSlipRatio[DRIVE_LAYOUT_RWD] + 0.01f,
          "RWD: rear slip ratio (%.4f) exceeds front (%.4f) under power",
          (double)rearSlipRatio[DRIVE_LAYOUT_RWD], (double)frontSlipRatio[DRIVE_LAYOUT_RWD]);
    /* FWD: front slip ratio >> rear (all drive goes front). */
    check(frontSlipRatio[DRIVE_LAYOUT_FWD] > rearSlipRatio[DRIVE_LAYOUT_FWD] + 0.01f,
          "FWD: front slip ratio (%.4f) exceeds rear (%.4f) under power",
          (double)frontSlipRatio[DRIVE_LAYOUT_FWD], (double)rearSlipRatio[DRIVE_LAYOUT_FWD]);
    /* AWD: both axles show nonzero slip (both receive drive torque). */
    check(rearSlipRatio[DRIVE_LAYOUT_AWD] > 0.01f && frontSlipRatio[DRIVE_LAYOUT_AWD] > 0.01f,
          "AWD: both axles show drive-induced slip (rear %.4f, front %.4f)",
          (double)rearSlipRatio[DRIVE_LAYOUT_AWD], (double)frontSlipRatio[DRIVE_LAYOUT_AWD]);
    /* Rear-minus-front slip-ratio differential: RWD is largest, FWD is smallest (negative). */
    const float diff[3] = {
        rearSlipRatio[DRIVE_LAYOUT_RWD] - frontSlipRatio[DRIVE_LAYOUT_RWD],
        rearSlipRatio[DRIVE_LAYOUT_FWD] - frontSlipRatio[DRIVE_LAYOUT_FWD],
        rearSlipRatio[DRIVE_LAYOUT_AWD] - frontSlipRatio[DRIVE_LAYOUT_AWD],
    };
    check(diff[DRIVE_LAYOUT_RWD] >= diff[DRIVE_LAYOUT_AWD],
          "RWD has the largest rear-minus-front slip-ratio differential (RWD %.4f >= AWD %.4f)",
          (double)diff[DRIVE_LAYOUT_RWD], (double)diff[DRIVE_LAYOUT_AWD]);
    check(diff[DRIVE_LAYOUT_AWD] >= diff[DRIVE_LAYOUT_FWD],
          "AWD slip-ratio differential is between RWD and FWD (AWD %.4f >= FWD %.4f)",
          (double)diff[DRIVE_LAYOUT_AWD], (double)diff[DRIVE_LAYOUT_FWD]);

    /* --- LOCKED FWD through a turn: the undriven rear axle stays independent. Before the
     * driven-axle gating, the state invariant demanded rear lockstep for every LOCKED diff,
     * so a front-drive locked car failed validation mid-corner and physics_fixed_update
     * rolled every turning step back — the car froze. Yaw rate growing past zero is the
     * observable proof that the steps are accepted now. */
    {
        VehicleSpec spec = baseSpec;
        spec.drivetrainLayout = (float)DRIVE_LAYOUT_FWD;
        spec.differentialMode = (float)DIFF_LOCKED;
        vehicle_spec_refresh_derived(&spec);
        vehicle_state_reset(&spec, &state, &derived, &renderState);
        state.velocityLongitudinalMps = 12.0f;

        ControllerOutput input;
        controller_output_zero(&input);
        input.steer = 0.20f;
        input.throttle = 0.15f;
        for (int i = 0; i < 180; i++)
            physics_fixed_update(&spec, &state, &derived, &renderState, NULL, NULL, NULL,
                                 &input, FIXED_DT_S);

        check(fabsf(state.yawRateRadS) > 0.01f,
              "FWD LOCKED: turning steps are accepted, no validation rollback (yaw %.4f rad/s)",
              (double)state.yawRateRadS);
        check(fabsf(state.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS -
                    state.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS) <= 1e-5f,
              "FWD LOCKED: driven front wheels share one omega");
        /* The wheel integrator reaches equilibrium against body vx, so the invariant is what
         * separates a locked driven axle from an independent undriven one. Probe it directly:
         * the undriven rear axle may diverge (cornering, asymmetric surfaces), the driven
         * front axle must not. Before the driven-axle gating the first probe was rejected and
         * physics_fixed_update rolled the step back. */
        {
            VehicleState probe = state;
            probe.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS =
                probe.wheels[WHEEL_REAR_LEFT].angularVelocityRadS + 1.0f;
            check(physics_state_is_valid(&spec, &probe, &derived),
                  "FWD LOCKED: invariant accepts divergent undriven rear wheels");
            probe = state;
            probe.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS =
                probe.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS + 1.0f;
            check(!physics_state_is_valid(&spec, &probe, &derived),
                  "FWD LOCKED: invariant still rejects divergent driven front wheels");
        }
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: roster                                                                         */
/* ------------------------------------------------------------------------------------- */

/* Every roster entry is valid, uniquely identified, and its exported profile round-trips
 * back to the same spec. */
static void scenario_roster(void)
{
    const int count = car_roster_count();
    check(count == 6, "the roster has exactly six cars (got %d)", count);

    char ids[16][128];
    for (int i = 0; i < count && i < 16; i++) {
        VehicleSpec spec;
        check(car_roster_spec(i, &spec), "roster entry %d builds successfully", i);
        check(vehicle_spec_is_valid(&spec), "roster entry %d is a valid vehicle spec", i);

        car_roster_id(i, ids[i], sizeof(ids[i]));
        check(strlen(ids[i]) > 0, "roster entry %d has a non-empty id", i);

        /* Filesystem-safe: only lowercase, digits, and underscores. */
        for (size_t c = 0; ids[i][c] != '\0'; c++) {
            char ch = ids[i][c];
            check((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_',
                  "roster id '%s' is filesystem-safe (char '%c')", ids[i], ch);
        }

        /* car_roster_find resolves the id back to this index. */
        check(car_roster_find(ids[i]) == i, "car_roster_find('%s') returns index %d", ids[i],
              i);

        /* The spec hash is stable and nonzero. */
        const uint32_t hash = car_roster_spec_hash(i);
        check(hash != 0, "roster entry %d has a nonzero spec hash", i);
    }

    /* Unique ids. */
    for (int i = 0; i < count && i < 16; i++) {
        for (int j = i + 1; j < count && j < 16; j++) {
            check(strcmp(ids[i], ids[j]) != 0, "roster ids are unique ('%s' != '%s')", ids[i],
                  ids[j]);
        }
    }

    /* Layout names are correct. */
    const int rwdIdx = car_roster_find("rwd_grip");
    const int fwdIdx = car_roster_find("fwd_light");
    const int awdIdx = car_roster_find("awd_gt");
    check(rwdIdx >= 0 && strcmp(car_roster_layout_name(rwdIdx), "RWD") == 0, "rwd_grip is RWD");
    check(fwdIdx >= 0 && strcmp(car_roster_layout_name(fwdIdx), "FWD") == 0,
          "fwd_light is FWD");
    check(awdIdx >= 0 && strcmp(car_roster_layout_name(awdIdx), "AWD") == 0, "awd_gt is AWD");

    /* Removing or querying a non-existent vehicle ID returns -1 (missing-ID error). */
    check(car_roster_find("non_existent_car_id_xyz") == -1,
          "car_roster_find returns -1 for missing vehicle ID");
    /* Profile round-trip: save each spec to a temp file, load it onto a fresh stock spec,
     * and verify every primary parameter survives. The directory is ensured here so the
     * scenario stands alone — it must not depend on scenario_telemetry having run first. */
    check(telemetry_ensure_dir(TELEMETRY_DIR), "the telemetry directory exists or was created");
    for (int i = 0; i < count && i < 16; i++) {
        VehicleSpec original;
        if (!car_roster_spec(i, &original)) continue;

        const char *tmpPath = TELEMETRY_DIR "/_roster_roundtrip.txt";
        check(dev_params_save(&original, tmpPath), "roster entry %d profile saves successfully",
              i);

        VehicleSpec roundtrip;
        vehicle_spec_set_default(&roundtrip);
        int applied = 0;
        const bool ok = dev_params_load(&roundtrip, tmpPath, &applied, NULL, NULL);
        check(ok && applied > 0, "roster entry %d profile round-trips (%d applied)", i,
              applied);
        if (ok) {
            check(vehicle_spec_is_valid(&roundtrip),
                  "roster entry %d round-trip produces a valid spec", i);
            /* The whole primary parameter set must survive, not just the layout: a save/load
             * regression in gearing, engine torque, or brake bias is exactly what this
             * scenario exists to catch. The profile format prints %.6f, so values agree to
             * within the format's quantum rather than bit-exactly. */
            for (int p = 0; p < dev_params_count(); p++) {
                const DevParameter *param = dev_param_at(p);
                if (param->derived) continue;
                const float expected = dev_param_get(&original, param);
                const float got = dev_param_get(&roundtrip, param);
                check(fabsf(got - expected) <= 1e-6f * fmaxf(1.0f, fabsf(expected)),
                      "roster entry %d param %s survives profile round-trip (%g vs %g)", i,
                      param->name, (double)got, (double)expected);
            }
        }
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: vehicle ownership isolation                                                   */
/* ------------------------------------------------------------------------------------- */

static void scenario_vehicle_instance_isolation(void)
{
    VehicleDefinition definition = { 0 };
    vehicle_definition_set_default(&definition);
    const VehicleDefinition originalDefinition = definition;
    check(strcmp(definition.id, "builtin/default") == 0 && definition.contentVersion == 1u &&
              definition.contentHash != 0u,
          "definition has a stable content identity and hash");

    VehicleSpec paddedA = definition.spec;
    VehicleSpec paddedB = definition.spec;
    const size_t paddingStart = offsetof(VehicleSpec, lateralLoadTransferEnabled) +
                                sizeof(paddedA.lateralLoadTransferEnabled);
    memset((unsigned char *)&paddedA + paddingStart, 0xa5, sizeof(paddedA) - paddingStart);
    memset((unsigned char *)&paddedB + paddingStart, 0x5a, sizeof(paddedB) - paddingStart);
    VehicleDefinition definitionA;
    VehicleDefinition definitionB;
    check(vehicle_definition_init(&definitionA, "padding-test", "padding-test", 1u, &paddedA) &&
              vehicle_definition_init(&definitionB, "padding-test", "padding-test", 1u,
                                      &paddedB) &&
              definitionA.contentHash == definitionB.contentHash,
          "content hash ignores structure padding and hashes canonical fields");

    VehicleSetup setupA = { 0 };
    VehicleSetup setupB;
    vehicle_setup_set_default(&definition, &setupA);
    setupB = setupA;
    setupB.tirePressureFrontKpa += 8.0f;
    setupB.brakeBiasFront -= 0.03f;
    setupB.finalDriveRatio += 0.15f;
    check(vehicle_setup_is_valid(&definition, &setupA) &&
              vehicle_setup_is_valid(&definition, &setupB),
          "two distinct setups validate against one definition");

    VehicleInstance instanceA = { 0 };
    VehicleInstance instanceB = { 0 };
    check(vehicle_instance_init(&instanceA, &definition, &setupA) &&
              vehicle_instance_init(&instanceB, &definition, &setupB),
          "two instances initialize from the shared definition");
    check(instanceA.spec.brakeBiasFront != instanceB.spec.brakeBiasFront &&
              instanceA.spec.finalDriveRatio != instanceB.spec.finalDriveRatio &&
              instanceA.tireState[WHEEL_FRONT_LEFT].pressureKpa !=
                  instanceB.tireState[WHEEL_FRONT_LEFT].pressureKpa,
          "each setup produces an entrant-local compiled spec and tire state");

    instanceA.vehicle.positionM = (Vector2){ 12.0f, -3.0f };
    instanceA.vehicleControls.throttle = 0.75f;
    instanceA.autoTrans.driveState = AUTO_REVERSE;
    instanceA.fuelKg = 4.0f;
    instanceA.tireState[WHEEL_REAR_LEFT].temperatureC = 91.0f;
    instanceA.tireState[WHEEL_REAR_LEFT].wear = 0.2f;
    instanceA.damage = 0.35f;
    instanceA.crashLockoutTimerS = 0.4f;
    check(instanceB.vehicle.positionM.x == 0.0f && instanceB.vehicleControls.throttle == 0.0f &&
              instanceB.autoTrans.driveState == AUTO_DRIVE &&
              instanceB.fuelKg == definition.spec.massFuelKg &&
              instanceB.tireState[WHEEL_REAR_LEFT].temperatureC == 20.0f &&
              instanceB.damage == 0.0f && instanceB.crashLockoutTimerS == 0.0f,
          "runtime pose, controls, drivetrain, fuel, tire, and damage state do not cross-talk");
    check(memcmp(&definition, &originalDefinition, sizeof(definition)) == 0,
          "runtime and setup changes never write shared definition memory");

    ReplayBuffer *replay = (ReplayBuffer *)calloc(1, sizeof(*replay));
    check(replay != NULL, "isolation replay buffer allocated");
    if (replay != NULL) {
        replay_begin_recording(replay, 0u);
        replay_capture_initial_vehicle(replay, &definition, &setupA, &instanceA);
        check(replay->initialVehicle.valid &&
                  replay->initialVehicle.definitionHash == definition.contentHash &&
                  memcmp(&replay->initialVehicle.setup, &setupA, sizeof(setupA)) == 0,
              "replay captures setup and instance state beside a definition identity/hash");

        VehicleSetup restoredSetup;
        VehicleInstance restoredInstance;
        memset(&restoredSetup, 0, sizeof(restoredSetup));
        memset(&restoredInstance, 0, sizeof(restoredInstance));
        check(replay_restore_initial_vehicle(replay, &definition, &restoredSetup,
                                             &restoredInstance),
              "matching immutable content restores the replay snapshot");
        check(memcmp(&restoredSetup, &setupA, sizeof(setupA)) == 0 &&
                  memcmp(&restoredInstance.vehicle, &instanceA.vehicle,
                         sizeof(instanceA.vehicle)) == 0 &&
                  restoredInstance.fuelKg == instanceA.fuelKg &&
                  restoredInstance.damage == instanceA.damage &&
                  restoredInstance.tireState[WHEEL_REAR_LEFT].wear ==
                      instanceA.tireState[WHEEL_REAR_LEFT].wear,
              "replay restores setup and authoritative mutable instance values");

        VehicleDefinition wrongDefinition = definition;
        wrongDefinition.contentHash++;
        check(!replay_restore_initial_vehicle(replay, &wrongDefinition, &restoredSetup,
                                              &restoredInstance),
              "replay rejects a mismatched immutable definition without embedding its blob");
        free(replay);
    }

    Game *defaultGame = alloc_game();
    Game *setupGame = alloc_game();
    game_init(defaultGame);
    game_init(setupGame);
    setupGame->vehicleSetup = setupB;
    check(vehicle_instance_init(&setupGame->vehicleInstance, &setupGame->vehicleDefinition,
                                &setupGame->vehicleSetup),
          "game compatibility storage accepts a distinct validated setup");
    const uint32_t defaultChecksum = game_state_checksum(defaultGame);
    const uint32_t setupChecksum = game_state_checksum(setupGame);
    check(defaultChecksum != setupChecksum,
          "rolling checksum includes setup and instance state (%08x vs %08x)", defaultChecksum,
          setupChecksum);
    defaultGame->vehicleDefinition.contentHash++;
    check(game_state_checksum(defaultGame) == defaultChecksum,
          "rolling checksum references no immutable definition blob");
    free(setupGame);
    free(defaultGame);
}

/*
 * tire-thermal — issue #21: bulk carcass temperature/pressure model.
 *
 * With tireThermalEnabled=1 the tires heat from slip/rolling work, cool toward ambient, and
 * live pressure tracks temperature; the grip multiplier is 1.0 at the optimal temperature and
 * falls off away from it. With the default 0 the model is off and temperature stays at
 * ambient (bit-identical baseline).
 */
static void scenario_tire_thermal(void)
{
    /* ---- 1. Enabled: slip work heats the driven wheels above ambient. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.tireThermalEnabled = 1.0f;

        /* Launch script: full throttle from rest spins the rear wheels (wheelspin slip
         * generates heat). */
        game->input.throttle = 1.0f;
        game->input.steer = 0.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game, FIXED_DT_S);

        const float frontTemp = game->vehicleInstance.tireState[WHEEL_FRONT_LEFT].temperatureC;
        const float rearTemp = game->vehicleInstance.tireState[WHEEL_REAR_LEFT].temperatureC;
        check(frontTemp > TIRE_AMBIENT_TEMP_C - 0.5f,
              "front tire temperature stays at or above ambient (%.1f C)", (double)frontTemp);
        check(rearTemp > TIRE_AMBIENT_TEMP_C,
              "wheelspin heated the driven rear tire above ambient (%.1f C)", (double)rearTemp);

        /* Live pressure follows the ideal-gas linearisation exactly: p = nominal * (1 +
         * k*(T - T_optimal)), clamped at the floor. */
        {
            const VehicleTireState *ts = &game->vehicleInstance.tireState[WHEEL_REAR_LEFT];
            const float expected =
                game->spec.tirePressureRearKpa *
                (1.0f + TIRE_PRESSURE_TEMP_COEFF * (ts->temperatureC - TIRE_OPTIMAL_TEMP_C));
            const float clamped =
                expected < TIRE_MIN_PRESSURE_KPA ? TIRE_MIN_PRESSURE_KPA : expected;
            check_near((double)ts->pressureKpa, (double)clamped, 0.01,
                       "live pressure matches the ideal-gas linearisation");
            check(ts->pressureKpa >= TIRE_MIN_PRESSURE_KPA,
                  "live pressure stays above the floor");
        }

        /* The grip multiplier is a bounded diagnostic, finite and in range. */
        const float m = game->derived.tireTemperatureGripMultiplier[WHEEL_REAR_LEFT];
        check(isfinite(m) && m >= TIRE_TEMP_MIN_MULT && m <= TIRE_TEMP_MAX_MULT,
              "temperature grip multiplier stays bounded (%.3f)", (double)m);

        /* ---- 2. Cooling: coast with no throttle -> temperature falls back toward ambient. */
        const float hotRear = rearTemp;
        game->input.throttle = 0.0f;
        game->input.brake = 0.0f;
        game->input.handbrake = 0.0f;
        for (int i = 0; i < 1200; i++) game_fixed_update(game, FIXED_DT_S);
        const float cooledRear = game->vehicleInstance.tireState[WHEEL_REAR_LEFT].temperatureC;
        check(cooledRear < hotRear, "coasting cools the rear tire (%.1f < %.1f C)",
              (double)cooledRear, (double)hotRear);

        /* Long run stays finite with sane pressure. */
        for (int i = 0; i < 3000; i++) game_fixed_update(game, FIXED_DT_S);
        bool sane = true;
        for (int w = 0; w < WHEEL_COUNT; w++) {
            const VehicleTireState *ts = &game->vehicleInstance.tireState[w];
            if (!isfinite(ts->temperatureC) || !isfinite(ts->pressureKpa) ||
                ts->pressureKpa < TIRE_MIN_PRESSURE_KPA || ts->temperatureC < TIRE_MIN_TEMP_C ||
                ts->temperatureC > TIRE_MAX_TEMP_C)
                sane = false;
        }
        check(sane, "5000+ tick thermal run stays finite with bounded pressure/temperature");
        free(game);
    }

    /* ---- 3. Disabled default: temperature never moves off ambient. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        check(game->spec.tireThermalEnabled == 0.0f, "thermal model defaults to off");
        game->input.throttle = 1.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game, FIXED_DT_S);
        for (int w = 0; w < WHEEL_COUNT; w++) {
            check_near((double)game->vehicleInstance.tireState[w].temperatureC,
                       (double)TIRE_AMBIENT_TEMP_C, 1e-6,
                       "disabled model: tire temperature pinned at ambient");
            check_near((double)game->derived.tireTemperatureGripMultiplier[w], 1.0, 1e-6,
                       "disabled model: grip multiplier pinned at 1.0");
        }
        free(game);
    }
}

/*
 * tire-wear — issue #22: wear accumulates monotonically from slip abuse, degrades grip
 * continuously to a bounded floor, and resets only via the service hook.
 */
static void scenario_tire_wear(void)
{
    /* ---- 1. Enabled: locked braking wears faster than normal driving. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.tireWearEnabled = 1.0f;

        /* Normal cruise: already at speed, gentle throttle -> minimal slip, little wear. */
        set_vehicle_rolling_speed(game, 15.0f);
        game->input.throttle = 0.1f;
        game->input.steer = 0.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game, FIXED_DT_S);
        const float wearCruise = game->vehicleInstance.tireState[WHEEL_REAR_LEFT].wear;

        /* Hard locked braking from speed: locked wheels, big slip -> much faster wear. */
        Game *game2 = alloc_game();
        game_init(game2);
        game2->spec.tireWearEnabled = 1.0f;
        set_vehicle_rolling_speed(game2, 25.0f);
        game2->input.brake = 1.0f;
        game2->input.throttle = 0.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game2, FIXED_DT_S);
        const float wearBrake = game2->vehicleInstance.tireState[WHEEL_REAR_LEFT].wear;

        check(wearCruise >= 0.0f, "cruise wear is non-negative (%.6f)", (double)wearCruise);
        check(wearBrake > wearCruise, "locked braking wears faster than cruising (%.6f > %.6f)",
              (double)wearBrake, (double)wearCruise);
        check(wearBrake <= 1.0f, "wear stays bounded at 1.0 (got %.6f)", (double)wearBrake);

        /* Grip degradation is continuous, bounded, and floor-guarded. */
        const float wearMult = game2->derived.tireWearGripMultiplier[WHEEL_REAR_LEFT];
        const float floor = 1.0f - TIRE_WEAR_FULL_DEGRADE;
        check(isfinite(wearMult) && wearMult >= floor - 1e-4f && wearMult <= 1.0f + 1e-4f,
              "wear grip multiplier stays within [%.3f, 1] (got %.4f)", (double)floor,
              (double)wearMult);

        /* Wear is monotonic: a further run never decreases it. */
        const float before = game2->vehicleInstance.tireState[WHEEL_REAR_LEFT].wear;
        for (int i = 0; i < 300; i++) game_fixed_update(game2, FIXED_DT_S);
        const float after = game2->vehicleInstance.tireState[WHEEL_REAR_LEFT].wear;
        check(after >= before - 1e-7f, "wear is monotonic (%.6f -> %.6f)", (double)before,
              (double)after);

        /* ---- 2. Service hook: replace resets wear; re-accumulates afterwards. ---- */
        vehicle_tire_service(&game2->vehicleInstance, true);
        check(game2->vehicleInstance.tireState[WHEEL_REAR_LEFT].wear == 0.0f,
              "tire service resets wear to zero");
        for (int i = 0; i < 300; i++) game_fixed_update(game2, FIXED_DT_S);
        check(game2->vehicleInstance.tireState[WHEEL_REAR_LEFT].wear > 0.0f,
              "wear re-accumulates after service");
        free(game2);
        free(game);
    }

    /* ---- 3. Disabled default: wear never moves, multiplier pinned at 1. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        check(game->spec.tireWearEnabled == 0.0f, "wear model defaults to off");
        set_vehicle_rolling_speed(game, 25.0f);
        game->input.brake = 1.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game, FIXED_DT_S);
        for (int w = 0; w < WHEEL_COUNT; w++) {
            check_near((double)game->vehicleInstance.tireState[w].wear, 0.0, 1e-9,
                       "disabled model: wear pinned at zero");
            check_near((double)game->derived.tireWearGripMultiplier[w], 1.0, 1e-6,
                       "disabled model: wear grip multiplier pinned at 1.0");
        }
        free(game);
    }
}

/*
 * dynamic-engine — issue #23: engine inertia, clutch coupling, and phased shifts.
 *
 * The dynamic engine (engineInertiaKgM2 > 0) revs from torque/inertia in neutral, slips the
 * clutch on launch and shifts with a torque interruption over shiftDurationS. The kinematic
 * engine (inertia 0) is the unchanged baseline; the whole suite already proves that.
 */
static void scenario_dynamic_engine(void)
{
    /* ---- 1. Free-rev: neutral + throttle winds the engine toward the limiter. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.engineInertiaKgM2 = 0.5f;
        game->autoTrans.enabled = false; /* keep it in neutral: no auto shifts */
        game->vehicle.selectedGear = 0;  /* neutral */
        game->input.throttle = 1.0f;
        float peakRpm = 0.0f;
        for (int i = 0; i < 1800; i++) {
            game_fixed_update(game, FIXED_DT_S);
            if (game->vehicle.engineRpm > peakRpm) peakRpm = game->vehicle.engineRpm;
        }
        check(peakRpm > game->spec.engineIdleRpm + 1000.0f,
              "neutral free-rev rises well above idle (peak %.0f rpm)", (double)peakRpm);
        check(peakRpm <= game->spec.engineRedlineRpm * 1.06f,
              "free-rev is bounded by the limiter (peak %.0f rpm)", (double)peakRpm);
        check(isfinite(game->vehicle.engineRpm), "engine rpm stays finite");
        free(game);
    }

    /* ---- 2. Clutch launch: from rest in first, the clutch slips then the car moves. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.engineInertiaKgM2 = 0.5f;
        game->vehicle.selectedGear = 1;
        game->input.throttle = 1.0f;
        for (int i = 0; i < 1200; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->derived.speedMps > 5.0f,
              "dynamic-engine launch accelerates the car (%.1f m/s)",
              (double)game->derived.speedMps);
        check(isfinite(game->vehicle.engineRpm) && game->vehicle.engineRpm > 0.0f,
              "engine is running after launch");
        free(game);
    }

    /* ---- 3. Phased shift: torque interruption over shiftDurationS, gear at midpoint. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.engineInertiaKgM2 = 0.5f;
        /* A clutch smaller than the engine torque makes the interruption visible: the
         * driveline can only carry maxClutch * engagement. Auto shifting is off so the
         * box cannot race the manual shift with its own request. */
        game->spec.maxClutchTorqueNm = 150.0f;
        game->autoTrans.enabled = false;
        game->vehicle.selectedGear = 2;
        set_vehicle_rolling_speed(game, 15.0f);
        game->input.throttle = 0.8f;

        /* Settle the driveline so derived drive torque is meaningful. */
        for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);

        const float halfTicks = game->spec.shiftDurationS / FIXED_DT_S / 2.0f;
        const float preShiftTorque = game->derived.driveTorqueNm[WHEEL_REAR_LEFT];
        check(preShiftTorque > 1.0f, "settled drive torque is meaningful (%.1f N*m)",
              (double)preShiftTorque);
        check(drivetrain_request_shift(&game->vehicle, 3), "shift request accepted");

        /* Through the cutting phase (one tick before the boundary): gear unchanged, drive
         * torque drops to the clutch's closing capacity. */
        for (int i = 0; i < (int)halfTicks - 1; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->vehicle.shiftPhase == 1 && game->vehicle.selectedGear == 2,
              "cutting phase holds the old gear");
        check(game->derived.driveTorqueNm[WHEEL_REAR_LEFT] < preShiftTorque * 0.2f,
              "the clutch cut interrupts drive torque");

        /* At the midpoint: gear applied, phase engaging. */
        game_fixed_update(game, FIXED_DT_S);
        check(game->vehicle.selectedGear == 3, "gear swapped at the shift midpoint");
        check(game->vehicle.shiftPhase == 2, "phase moved to engaging");

        /* After the full window: shift complete and the clutch re-locked. */
        for (int i = 0; i < (int)halfTicks + 2; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->vehicle.shiftPhase == 0, "shift completes");
        check(game->vehicle.shiftTimerS == 0.0f, "shift timer resets");
        free(game);
    }

    /* ---- 4. Engine braking: off-throttle at speed lets the rpm fall under load. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.engineInertiaKgM2 = 0.5f;
        game->vehicle.selectedGear = 3;
        set_vehicle_rolling_speed(game, 25.0f);
        game->input.throttle = 0.0f;
        game->input.brake = 0.0f;
        for (int i = 0; i < 900; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->derived.speedMps < 25.0f, "engine braking decelerates the car (%.1f m/s)",
              (double)game->derived.speedMps);
        free(game);
    }

    /* ---- 5. Determinism: identical dynamic-engine runs reproduce byte-identically. ---- */
    {
        Game *a = alloc_game();
        Game *b = alloc_game();
        game_init(a);
        game_init(b);
        a->spec.engineInertiaKgM2 = b->spec.engineInertiaKgM2 = 0.5f;
        a->input.throttle = b->input.throttle = 1.0f;
        bool same = true;
        for (int i = 0; i < 900; i++) {
            game_fixed_update(a, FIXED_DT_S);
            game_fixed_update(b, FIXED_DT_S);
            if (game_state_checksum(a) != game_state_checksum(b)) same = false;
        }
        check(same, "dynamic-engine runs are deterministic over 900 ticks");
        free(a);
        free(b);
    }
}

/*
 * fuel-model — issue #24: consumption from engine work, dynamic mass/CG, starvation, refuel.
 *
 * The fuel model (fuelEnabled=1) burns fuel proportional to engine work, feeds the live fuel
 * mass back into mass/CG/inertia, and fades drive torque as the tank starves. Disabled
 * (default) pins fuel and keeps the baseline exact.
 */
static void scenario_fuel_model(void)
{
    /* ---- 1. Disabled: fuel pinned, mass unchanged. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        check(game->spec.fuelEnabled == 0.0f, "fuel model defaults to off");
        game->input.throttle = 1.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game, FIXED_DT_S);
        check_near((double)game->vehicleInstance.fuelKg, (double)game->spec.massFuelKg, 1e-6,
                   "disabled model: fuel pinned at the initial load");
        free(game);
    }

    /* ---- 2. Enabled: fuel burns monotonically under load and never goes negative. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.fuelEnabled = 1.0f;
        game->spec.fuelConsumptionRateKgPerWS = 2.0e-6f; /* burn visibly */
        const float initial = game->vehicleInstance.fuelKg;
        const float initialMass = game->spec.massKg;
        game->input.throttle = 1.0f;
        float last = initial;
        bool monotonic = true;
        for (int i = 0; i < 1800; i++) {
            game_fixed_update(game, FIXED_DT_S);
            const float fuel = game->vehicleInstance.fuelKg;
            if (fuel > last + 1e-6f) monotonic = false;
            last = fuel;
        }
        check(last < initial, "fuel decreased under full load (%.2f -> %.2f kg)",
              (double)initial, (double)last);
        check(monotonic, "fuel consumption is monotonic");
        check(last >= 0.0f, "fuel never goes negative");

        /* Mass follows the fuel: the car is lighter now. */
        check(game->spec.massKg < initialMass - 1.0f,
              "mass tracks the live fuel load (%.1f -> %.1f kg)", (double)initialMass,
              (double)game->spec.massKg);
        free(game);
    }

    /* ---- 3. Full load burns faster than idle. ---- */
    {
        Game *idle = alloc_game();
        game_init(idle);
        idle->spec.fuelEnabled = 1.0f;
        idle->spec.fuelConsumptionRateKgPerWS = 2.0e-6f;
        for (int i = 0; i < 1200; i++) game_fixed_update(idle, FIXED_DT_S);
        const float idleFuel = idle->vehicleInstance.fuelKg;

        Game *load = alloc_game();
        game_init(load);
        load->spec.fuelEnabled = 1.0f;
        load->spec.fuelConsumptionRateKgPerWS = 2.0e-6f;
        load->input.throttle = 1.0f;
        for (int i = 0; i < 1200; i++) game_fixed_update(load, FIXED_DT_S);
        const float loadFuel = load->vehicleInstance.fuelKg;

        check(loadFuel < idleFuel, "full load burns more than idle (%.2f < %.2f kg)",
              (double)loadFuel, (double)idleFuel);
        free(idle);
        free(load);
    }

    /* ---- 4. Starvation: a nearly-empty tank fades drive torque and the car cannot pull. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->spec.fuelEnabled = 1.0f;
        game->spec.fuelConsumptionRateKgPerWS = 0.0f; /* no burn; start starved instead */
        game->vehicleInstance.fuelKg = 0.0f;          /* empty tank: zero drive torque */
        game->input.throttle = 1.0f;
        for (int i = 0; i < 600; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->derived.speedMps < 0.5f,
              "an empty tank produces no drive (%.2f m/s after 5 s of full throttle)",
              (double)game->derived.speedMps);
        free(game);
    }

    /* ---- 5. Refuel service respects capacity. ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->vehicleInstance.fuelKg = 0.0f;
        vehicle_refuel(&game->vehicleInstance, 10.0f);
        check_near((double)game->vehicleInstance.fuelKg,
                   (double)(10.0f * FUEL_DENSITY_KG_PER_L), 1e-4,
                   "refuel adds the requested litres");
        vehicle_refuel(&game->vehicleInstance, 999.0f);
        check(game->vehicleInstance.fuelKg <=
                  game->spec.fuelTankCapacityL * FUEL_DENSITY_KG_PER_L + 1e-3f,
              "refuel cannot exceed capacity");
        free(game);
    }

    /* ---- 6. Determinism: identical fuel runs reproduce. ---- */
    {
        Game *a = alloc_game();
        Game *b = alloc_game();
        game_init(a);
        game_init(b);
        a->spec.fuelEnabled = b->spec.fuelEnabled = 1.0f;
        a->spec.fuelConsumptionRateKgPerWS = b->spec.fuelConsumptionRateKgPerWS = 2.0e-6f;
        a->input.throttle = b->input.throttle = 1.0f;
        bool same = true;
        for (int i = 0; i < 900; i++) {
            game_fixed_update(a, FIXED_DT_S);
            game_fixed_update(b, FIXED_DT_S);
            if (game_state_checksum(a) != game_state_checksum(b)) same = false;
        }
        check(same, "fuel-model runs are deterministic over 900 ticks");
        free(a);
        free(b);
    }
}

static const TestScenario kPhysicsScenarios[] = {
    { "telemetry", "CSV writer: stable header, row count, failure handling",
      scenario_telemetry },
    { "vehicle", "canonical structures, steering, contact velocity, render",
      scenario_vehicle_units },
    { "tire", "nonlinear curves, slip ratio, and combined-friction ellipse", scenario_tire },
    { "tire-thermal", "issue #21: bulk temperature/pressure model heats, cools, stays bounded",
      scenario_tire_thermal },
    { "tire-wear", "issue #22: monotonic wear, bounded grip degradation, service reset",
      scenario_tire_wear },
    { "dynamic-engine",
      "issue #23: inertia free-rev, clutch launch, phased shift, engine braking, determinism",
      scenario_dynamic_engine },
    { "fuel-model",
      "issue #24: consumption from engine work, dynamic mass, starvation, refuel, determinism",
      scenario_fuel_model },
    { "solver-stages",
      "staged solver: prefix runs, stage contracts, rollback and failure report",
      scenario_solver_stages },
    { "drivetrain", "engine curve, gearing, torque splits, wheel lock and release",
      scenario_drivetrain },
    { "accel-filter", "previous-step load-transfer acceleration filter",
      scenario_accel_filter },
    { "load-transfer", "static split, dynamic transfer, clamping, wheel loads",
      scenario_load_transfer },
    { "resistance", "aerodynamic drag and per-wheel rolling resistance", scenario_resistance },
    { "aero-loads", "issue #17: speed-squared aerodynamic vertical load, balance, and clamping",
      scenario_aero_loads },
    { "rest", "rest stability and stationary steering", scenario_rest },
    { "launch-stop", "straight launch, braking, and zero-speed stability",
      scenario_launch_stop },
    { "coast-down", "throttle lift, engine braking, and separated resistance",
      scenario_coast_down },
    { "brake-corner", "service braking consumes lateral friction budget",
      scenario_braking_cornering },
    { "power-oversteer", "rear drive saturation and throttle-lift recovery",
      scenario_power_oversteer },
    { "handbrake", "rear lockup, combined-slip yaw, and release recovery",
      scenario_handbrake_entry },
    { "low-speed", "kinematic/dynamic blend continuity", scenario_low_speed },
    { "reverse", "explicit reverse launch and stopping", scenario_reverse },
    { "steer-sign", "left-positive force, torque, yaw, and heading", scenario_steering_sign },
    { "lever-arm", "front/rear slip and yaw depend on distinct lever arms",
      scenario_lever_arm },
    { "integration", "semi-implicit order and heading wrap", scenario_integration },
    { "fixed-rate", "direct stepping matches accumulator stepping", scenario_fixed_rate },
    { "params", "tunable registry, clamping, and tuning-profile round trip", scenario_params },
    { "param-audit",
      "every VehicleSpec field is classified, and each class is proved by behaviour",
      scenario_param_audit },
    { "presets", "driving presets: count, bounds, and deterministic apply", scenario_presets },
    { "auto-trans", "automatic transmission shifts and arcade reverse swap",
      scenario_auto_transmission },
    { "dev-state", "dev scope, markers, scenario control, invariant latch",
      scenario_dev_state },
    { "devreplay", "durable replay timelines, malformed input, event markers",
      scenario_dev_replay },
    { "cross-spec-invariant-reproducibility",
      "corpus spec sweep: structural invariants hold across 5 diverse specs",
      scenario_cross_spec_invariant_reproducibility },
    { "low-speed-tight-turn-slip",
      "full-lock low-speed turn: rear slip stays within blend-safety bound",
      scenario_low_speed_tight_turn_slip },
    { "peak-friction-slip-sweep",
      "uni-modal mu-slip curve: one interior peak, monotonic rise and fall",
      scenario_peak_friction_slip_sweep },
    { "drivetrain-layout", "RWD/FWD/AWD torque routing and slip-ratio differential ordering",
      scenario_drivetrain_layout },
    { "roster", "6-car roster: validity, unique ids, profile round-trip", scenario_roster },
    { "vehicle-instance-isolation",
      "shared immutable definition with isolated setup and mutable runtime state",
      scenario_vehicle_instance_isolation },
    { "run-report", "validation metrics reduction and run.json output contract",
      scenario_run_report },
};

TestScenarioGroup test_physics_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kPhysicsScenarios;
    group.count = sizeof(kPhysicsScenarios) / sizeof(kPhysicsScenarios[0]);
    return group;
}

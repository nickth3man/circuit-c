#include "dev/dev_lab.h"

#if defined(CIRCUIT_DEV_TOOLS) && !defined(CIRCUIT_HEADLESS)

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"

/* Vendored third-party header: compiled once, here. The diagnostic push/pop keeps this
 * project's warning flags applied to this project's code without drowning the build in
 * warnings from a 6000-line single-header library we do not maintain. */
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_presets.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "dev/failure_bundle.h"
#include "game/game.h"
#include "core/math_utils.h"
#include "physics/physics.h"
#include "game/telemetry.h"
#include "core/units.h"

/* ------------------------------------------------------------------------------ layout -- */

#define LAB_X 8.0f
#define LAB_WIDTH 376.0f
#define ROW_HEIGHT 22.0f
#define ROW_GAP 4.0f
#define SCOPE_WIDTH 430.0f
#define SCOPE_ROW_HEIGHT 42.0f
#define INSPECTOR_HEIGHT 132.0f
#define MAX_EVENTS 256

#define PROFILE_DIR "data/vehicles"
#define REPLAY_DIR "artifacts/replays"
#define ARTIFACTS_DIR "artifacts"

/*
 * Transient interface state: raygui edit-mode flags, text buffers, and the derived event
 * cache. Deliberately module-static rather than part of Game — none of it is worth
 * preserving across a hot reload, and keeping it out of Game keeps that struct free of
 * anything the UI happens to need this frame.
 */
static bool g_scenarioDropdownOpen = false;
static bool g_presetDropdownOpen = false;
static int g_activePreset = 0; /* last-applied preset index */
static bool g_profileNameEditing = false;
static char g_profileName[32] = "default";
static char g_activeProfile[32] = "default";
static bool g_replayNameEditing = false;
static char g_replayName[32] = "run";
static DevReplayEvent g_events[MAX_EVENTS];
static int g_eventCount = 0;
static bool g_styleReady = false;

static const Color COLOR_PANEL = { 18, 20, 25, 232 };
static const Color COLOR_TEXT = { 226, 228, 233, 255 };
static const Color COLOR_DIM = { 150, 156, 166, 255 };
static const Color COLOR_ACCENT = { 120, 200, 255, 255 };
static const Color COLOR_GHOST = { 120, 124, 134, 200 };
static const Color COLOR_WARNING = { 208, 62, 52, 255 };

void dev_lab_reload_style(void)
{
    GuiLoadStyleDefault();
    GuiSetStyle(DEFAULT, TEXT_SIZE, 12);
    GuiSetStyle(DEFAULT, BACKGROUND_COLOR, ColorToInt(COLOR_PANEL));
    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, ColorToInt(COLOR_TEXT));
    GuiSetStyle(SLIDER, SLIDER_PADDING, 1);
    g_styleReady = true;
}

static void ensure_style(void)
{
    if (!g_styleReady) dev_lab_reload_style();
}

/* A simple top-to-bottom cursor: every control advances it, so inserting a row never means
 * renumbering the ones below it. */
typedef struct {
    float x;
    float y;
    float width;
} Cursor;

static Rectangle row(Cursor *cursor, float height)
{
    const Rectangle bounds = { cursor->x, cursor->y, cursor->width, height };
    cursor->y += height + ROW_GAP;
    return bounds;
}

/* GuiCheckBox's bounds are the box itself, not the box plus its label, so a full-width
 * rectangle would draw an enormous square. Shrink to a real checkbox and let the label run
 * into the rest of the column. */
static void checkbox(Rectangle cell, const char *text, bool *value)
{
    const Rectangle box = { cell.x, cell.y + 1.0f, 13.0f, 13.0f };
    GuiCheckBox(box, text, value);
}

static Rectangle column(Rectangle bounds, int index, int count, float gap)
{
    const float width = (bounds.width - gap * (float)(count - 1)) / (float)count;
    return (Rectangle){ bounds.x + (width + gap) * (float)index, bounds.y, width,
                        bounds.height };
}

/* ------------------------------------------------------------------- keyboard shortcuts -- */

void dev_lab_update(struct Game *game)
{
    if (game == NULL) return;
    DevState *dev = &game->dev;

    if (dev->statusTimerS > 0.0f) {
        dev->statusTimerS = fmaxf(0.0f, dev->statusTimerS - GetFrameTime());
    }

    if (IsKeyPressed(KEY_F2)) {
        dev->labVisible = !dev->labVisible;
        ensure_style();
    }
    if (IsKeyPressed(KEY_F3)) dev->inspectorVisible = !dev->inspectorVisible;
    if (IsKeyPressed(KEY_F5)) {
        dev->paused = !dev->paused;
        dev_state_set_status(dev, false, dev->paused ? "paused" : "running");
    }
    if (IsKeyPressed(KEY_F6)) {
        dev->paused = true;
        dev->stepTicks += (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 10 : 1;
    }
    if (IsKeyPressed(KEY_F7)) {
        static const float scales[] = { 1.0f, 0.5f, 0.25f, 0.1f, 2.0f, 4.0f };
        int next = 0;
        for (int i = 0; i < (int)(sizeof(scales) / sizeof(scales[0])); i++) {
            if (fabsf(dev->timeScale - scales[i]) < 1e-4f) {
                next = (i + 1) % (int)(sizeof(scales) / sizeof(scales[0]));
                break;
            }
        }
        dev->timeScale = scales[next];
        dev_state_set_status(dev, false, "time scale %.2fx", (double)dev->timeScale);
    }
    if (IsKeyPressed(KEY_F8)) {
        dev_state_capture_ghost(dev);
        dev_state_set_status(dev, false, "baseline captured (%d points)", dev->ghostCount);
    }
}

/* ------------------------------------------------------------------------ world overlays -- */

static Vector2 body_to_world(Vector2 bodyPointM, Vector2 positionM, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){ positionM.x + bodyPointM.x * c - bodyPointM.y * s,
                      positionM.y + bodyPointM.x * s + bodyPointM.y * c };
}

static Vector2 rotate_to_world(Vector2 bodyVector, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){ bodyVector.x * c - bodyVector.y * s,
                      bodyVector.x * s + bodyVector.y * c };
}

static void draw_arrow(Vector2 fromM, Vector2 deltaM, float ppm, Color color)
{
    if (deltaM.x * deltaM.x + deltaM.y * deltaM.y < 1e-4f) return;
    const Vector2 toM = { fromM.x + deltaM.x, fromM.y + deltaM.y };
    const Vector2 a = units_world_to_render_px(fromM, ppm);
    const Vector2 b = units_world_to_render_px(toM, ppm);
    DrawLineEx(a, b, 2.0f, color);
    DrawCircleV(b, 3.0f, color);
}

/* The ghost is stored oldest-first in a plain array; the live path is a ring. */
static void draw_ghost_polyline(const DevState *dev, float ppm, Color color)
{
    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i < dev->ghostCount; i++) {
        const Vector2 pixel = units_world_to_render_px(dev->ghost[i], ppm);
        if (i > 0) DrawLineEx(previous, pixel, 1.5f, color);
        previous = pixel;
    }
}

static void draw_path_polyline(const DevState *dev, float ppm, Color color)
{
    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i < dev->pathCount; i++) {
        const Vector2 pixel = units_world_to_render_px(dev_state_path_point(dev, i), ppm);
        if (i > 0) DrawLineEx(previous, pixel, 2.0f, color);
        previous = pixel;
    }
}

void dev_lab_draw_world(const struct Game *game, const VehicleDrawState *draw)
{
    if (game == NULL || draw == NULL || !game->dev.labVisible) return;

    const DevState *dev = &game->dev;
    const float ppm = game->renderPixelsPerMeter;

    if (dev->showGhost && dev->ghostValid) draw_ghost_polyline(dev, ppm, COLOR_GHOST);
    if (dev->showPath) draw_path_polyline(dev, ppm, (Color){ 90, 190, 140, 220 });

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &game->vehicle.wheels[i];
        const Vector2 contactM =
            body_to_world(wheel->localPositionM, draw->positionM, draw->headingRad);

        if (dev->showContacts) {
            const Vector2 pixel = units_world_to_render_px(contactM, ppm);
            DrawCircleV(pixel, 3.0f, wheel->locked ? COLOR_WARNING : COLOR_ACCENT);
        }
        if (dev->showLoads) {
            /* Radius encodes normal load: 1 kN -> 6 px, so the front/rear split is visible
             * at a glance without reading a number. A second, dimmer ring marks the static
             * load, so the transfer is the visible gap between the rings. */
            const float radius = fmaxf(2.0f, wheel->normalLoadN * 0.006f);
            const float staticLoadN = (i <= WHEEL_FRONT_RIGHT)
                                          ? game->derived.staticFrontLoadN * 0.5f
                                          : game->derived.staticRearLoadN * 0.5f;
            const Vector2 pixel = units_world_to_render_px(contactM, ppm);
            DrawCircleLinesV(pixel, fmaxf(2.0f, staticLoadN * 0.006f),
                             (Color){ 240, 200, 90, 70 });
            DrawCircleLinesV(pixel, radius, (Color){ 240, 200, 90, 200 });
        }
        if (dev->showForces) {
            const Vector2 wheelForceN = { wheel->forceLongitudinalN, wheel->forceLateralN };
            const Vector2 bodyForceN =
                physics_rotate_wheel_force_to_body(wheelForceN, wheel->steerAngleRad);
            /* 8 kN maps to one metre of arrow. */
            const Vector2 scaled = { bodyForceN.x * 0.000125f, bodyForceN.y * 0.000125f };
            draw_arrow(contactM, rotate_to_world(scaled, draw->headingRad), ppm,
                       (Color){ 232, 96, 84, 255 });
        }
        if (dev->showSlip) {
            /* Slip angle drawn as a short line off the wheel's own heading. */
            const float wheelHeading = draw->headingRad + wheel->steerAngleRad;
            const Vector2 slipDirection = { cosf(wheelHeading - wheel->slipAngleRad) * 0.9f,
                                            sinf(wheelHeading - wheel->slipAngleRad) * 0.9f };
            draw_arrow(contactM, slipDirection, ppm, (Color){ 150, 130, 240, 220 });
        }
    }

    if (dev->showVelocity) {
        const Vector2 velocityBody = { game->vehicle.velocityLongitudinalMps * 0.3f,
                                       game->vehicle.velocityLateralMps * 0.3f };
        draw_arrow(draw->positionM, rotate_to_world(velocityBody, draw->headingRad), ppm,
                   (Color){ 110, 220, 130, 255 });
    }

    if (dev->showResistance) {
        /* Aerodynamic drag and rolling resistance from the CG, at 1 kN per metre of arrow.
         * Drag must visibly oppose the velocity arrow, not the nose. */
        const Vector2 dragScaledM = { game->derived.aeroDragBodyN.x * 0.001f,
                                      game->derived.aeroDragBodyN.y * 0.001f };
        const Vector2 rollingScaledM = { game->derived.rollingResistanceBodyN.x * 0.001f,
                                         game->derived.rollingResistanceBodyN.y * 0.001f };
        draw_arrow(draw->positionM, rotate_to_world(dragScaledM, draw->headingRad), ppm,
                   (Color){ 96, 170, 240, 255 });
        draw_arrow(draw->positionM, rotate_to_world(rollingScaledM, draw->headingRad), ppm,
                   (Color){ 200, 160, 90, 255 });
    }
}

/* --------------------------------------------------------------------------------- scope -- */

static void draw_scope_channel(Rectangle bounds, const DevState *dev, DevScopeChannel channel)
{
    DrawRectangleRec(bounds, (Color){ 14, 16, 20, 220 });
    DrawRectangleLinesEx(bounds, 1.0f, (Color){ 62, 66, 74, 255 });

    const int count = dev->scopeCount;
    if (count < 2) {
        DrawText(dev_state_scope_name(channel), (int)bounds.x + 6, (int)bounds.y + 4, 10,
                 COLOR_DIM);
        return;
    }

    float low = dev_state_scope_value(dev, channel, 0);
    float high = low;
    for (int i = 1; i < count; i++) {
        const float value = dev_state_scope_value(dev, channel, i);
        if (value < low) low = value;
        if (value > high) high = value;
    }
    for (int i = 0; i < dev->ghostScopeCount; i++) {
        const float value = dev_state_ghost_scope_value(dev, channel, i);
        if (value < low) low = value;
        if (value > high) high = value;
    }
    if (high - low < 1e-3f) {
        high += 0.5f;
        low -= 0.5f;
    }
    const float span = high - low;

    /* Zero line, when zero is inside the visible range. */
    if (low < 0.0f && high > 0.0f) {
        const float zeroY = bounds.y + bounds.height * (high / span);
        DrawLineV((Vector2){ bounds.x, zeroY }, (Vector2){ bounds.x + bounds.width, zeroY },
                  (Color){ 58, 62, 70, 255 });
    }

    const float stepX = bounds.width / (float)(count - 1);

    if (dev->showGhost && dev->ghostScopeCount > 1) {
        const float ghostStepX = bounds.width / (float)(dev->ghostScopeCount - 1);
        Vector2 previous = { 0.0f, 0.0f };
        for (int i = 0; i < dev->ghostScopeCount; i++) {
            const float value = dev_state_ghost_scope_value(dev, channel, i);
            const Vector2 point = { bounds.x + ghostStepX * (float)i,
                                    bounds.y + bounds.height * ((high - value) / span) };
            if (i > 0) DrawLineV(previous, point, COLOR_GHOST);
            previous = point;
        }
    }

    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i < count; i++) {
        const float value = dev_state_scope_value(dev, channel, i);
        const Vector2 point = { bounds.x + stepX * (float)i,
                                bounds.y + bounds.height * ((high - value) / span) };
        if (i > 0) DrawLineV(previous, point, COLOR_ACCENT);
        previous = point;
    }

    const float current = dev_state_scope_value(dev, channel, count - 1);
    DrawText(TextFormat("%s  %+.3f %s", dev_state_scope_name(channel), (double)current,
                        dev_state_scope_unit(channel)),
             (int)bounds.x + 6, (int)bounds.y + 3, 10, COLOR_TEXT);
    DrawText(TextFormat("%+.2f", (double)high), (int)(bounds.x + bounds.width) - 34,
             (int)bounds.y + 3, 10, COLOR_DIM);
    DrawText(TextFormat("%+.2f", (double)low), (int)(bounds.x + bounds.width) - 34,
             (int)(bounds.y + bounds.height) - 12, 10, COLOR_DIM);
}

static void draw_scope(DevState *dev)
{
    const float x = (float)GetScreenWidth() - SCOPE_WIDTH - 8.0f;
    float y = 8.0f;

    /* Preset selector: every channel is recorded, eight are drawn. Switching presets mid-run
     * shows the newly selected channels' full retained history, not a blank trace. */
    {
        const Rectangle bar = { x, y, SCOPE_WIDTH, 18.0f };
        for (int p = 0; p < DEV_SCOPE_PRESET_COUNT; p++) {
            const bool wasActive = (dev->scopePreset == p);
            bool active = wasActive;
            /* raygui's GuiToggle returns 0 unconditionally — it only flips *active — so the
             * standard pattern is to detect a 0->1 edge on the local bool. */
            GuiToggle(column(bar, p, DEV_SCOPE_PRESET_COUNT, 4.0f),
                      dev_scope_preset_name((DevScopePreset)p), &active);
            if (active && !wasActive) {
                dev->scopePreset = p;
            }
        }
        y += 21.0f;
    }

    for (int slot = 0; slot < DEV_SCOPE_VISIBLE; slot++) {
        const Rectangle bounds = { x, y, SCOPE_WIDTH, SCOPE_ROW_HEIGHT };
        draw_scope_channel(bounds, dev,
                           dev_scope_preset_channel((DevScopePreset)dev->scopePreset, slot));
        y += SCOPE_ROW_HEIGHT + 3.0f;
    }
}

/* ----------------------------------------------------------------------------- inspector -- */

static Color event_color(DevReplayEventKind kind)
{
    switch (kind) {
        case DEV_REPLAY_EVENT_THROTTLE: return (Color){ 110, 220, 130, 255 };
        case DEV_REPLAY_EVENT_BRAKE: return (Color){ 232, 120, 96, 255 };
        case DEV_REPLAY_EVENT_HANDBRAKE: return (Color){ 240, 180, 70, 255 };
        case DEV_REPLAY_EVENT_SHIFT_UP: return (Color){ 120, 200, 255, 255 };
        case DEV_REPLAY_EVENT_SHIFT_DOWN: return (Color){ 90, 150, 210, 255 };
        case DEV_REPLAY_EVENT_RESET: return (Color){ 220, 220, 220, 255 };
        case DEV_REPLAY_EVENT_STEER_REVERSAL: return (Color){ 170, 140, 240, 255 };
        case DEV_REPLAY_EVENT_COUNT: break;
    }
    return COLOR_DIM;
}

static void export_failure_bundle(struct Game *game, const char *reason)
{
    FailureBundle bundle;
    memset(&bundle, 0, sizeof(bundle));
    bundle.scenario = dev_scenario_at(game->dev.scenario) != NULL
                          ? dev_scenario_at(game->dev.scenario)->name
                          : "manual";
    bundle.failureText = reason;
    bundle.replay = &game->replay;
    bundle.spec = &game->spec;
    bundle.activeProfile = g_activeProfile;
    bundle.failingTick = game->dev.invariantFailed ? game->dev.invariantTick : game->sim.tick;
    bundle.checksum = game->stateChecksum;
    bundle.seed = game->dev.seed;
    bundle.checksRun = 0;
    bundle.checksFailed = game->dev.invariantCount;

    char directory[512];
    if (failure_bundle_write(ARTIFACTS_DIR, &bundle, directory, sizeof(directory))) {
        dev_state_set_status(&game->dev, false, "bundle written to %s", directory);
    } else {
        dev_state_set_status(&game->dev, true, "could not write a failure bundle");
    }
}

static void draw_inspector(struct Game *game, Rectangle bounds)
{
    DevState *dev = &game->dev;
    const ReplayBuffer *replay = &game->replay;

    GuiPanel(bounds, "REPLAY INSPECTOR");
    Cursor cursor = { bounds.x + 8.0f, bounds.y + 28.0f, bounds.width - 16.0f };

    const int frameCount = replay->count;
    if (dev->inspectorFollowsLive) dev->inspectorCursor = (frameCount > 0) ? frameCount - 1 : 0;
    if (dev->inspectorCursor > frameCount - 1) dev->inspectorCursor = frameCount - 1;
    if (dev->inspectorCursor < 0) dev->inspectorCursor = 0;

    /* Timeline with event markers: input-derived events across the full height, and the
     * state-derived Phase 3 markers (saturation, peak transfer, invariant, recovery) as
     * brighter notches in the lower half — the ones only the simulation knows about. */
    const Rectangle timeline = row(&cursor, 26.0f);
    DrawRectangleRec(timeline, (Color){ 14, 16, 20, 255 });
    DrawRectangleLinesEx(timeline, 1.0f, (Color){ 62, 66, 74, 255 });
    if (frameCount > 1) {
        for (int e = 0; e < g_eventCount; e++) {
            const float t = (float)g_events[e].index / (float)(frameCount - 1);
            const float x = timeline.x + t * timeline.width;
            DrawLineEx((Vector2){ x, timeline.y + 2.0f },
                       (Vector2){ x, timeline.y + timeline.height - 2.0f }, 1.0f,
                       event_color(g_events[e].kind));
        }
        for (int m = 0; m < dev->markerCount; m++) {
            const DevMarker *marker = &dev->markers[m];
            if (marker->tick < replay->firstTick) continue;
            const uint64_t offset = marker->tick - replay->firstTick;
            if (offset >= (uint64_t)frameCount) continue;
            const float t = (float)offset / (float)(frameCount - 1);
            const float x = timeline.x + t * timeline.width;
            const Color color =
                (marker->kind == DEV_MARKER_INVARIANT)       ? COLOR_WARNING
                : (marker->kind == DEV_MARKER_SATURATION)    ? (Color){ 255, 150, 60, 255 }
                : (marker->kind == DEV_MARKER_PEAK_TRANSFER) ? (Color){ 250, 220, 90, 255 }
                : (marker->kind == DEV_MARKER_RECOVERY)      ? (Color){ 120, 235, 150, 255 }
                                                             : COLOR_DIM;
            DrawLineEx((Vector2){ x, timeline.y + timeline.height * 0.5f },
                       (Vector2){ x, timeline.y + timeline.height - 2.0f }, 2.0f, color);
        }
        const float cursorT = (float)dev->inspectorCursor / (float)(frameCount - 1);
        const float cursorX = timeline.x + cursorT * timeline.width;
        DrawLineEx((Vector2){ cursorX, timeline.y },
                   (Vector2){ cursorX, timeline.y + timeline.height }, 2.0f, COLOR_ACCENT);
    }

    float scrubber = (float)dev->inspectorCursor;
    const Rectangle scrubberBounds = row(&cursor, 16.0f);
    if (GuiSlider(scrubberBounds, "", "", &scrubber, 0.0f,
                  (float)((frameCount > 0) ? frameCount - 1 : 0))) {
        dev->inspectorCursor = (int)(scrubber + 0.5f);
        dev->inspectorFollowsLive = false;
    }

    const Rectangle buttons = row(&cursor, ROW_HEIGHT);
    if (GuiButton(column(buttons, 0, 6, 4.0f), "|<")) {
        dev->inspectorCursor = 0;
        dev->inspectorFollowsLive = false;
    }
    if (GuiButton(column(buttons, 1, 6, 4.0f), "<")) {
        if (dev->inspectorCursor > 0) dev->inspectorCursor--;
        dev->inspectorFollowsLive = false;
    }
    if (GuiButton(column(buttons, 2, 6, 4.0f), ">")) {
        if (dev->inspectorCursor < frameCount - 1) dev->inspectorCursor++;
        dev->inspectorFollowsLive = false;
    }
    if (GuiButton(column(buttons, 3, 6, 4.0f), ">|")) {
        dev->inspectorFollowsLive = true;
    }
    if (GuiButton(column(buttons, 4, 6, 4.0f), "to fail")) {
        if (dev->invariantFailed && dev->invariantTick >= replay->firstTick) {
            dev->inspectorCursor = (int)(dev->invariantTick - replay->firstTick);
            dev->inspectorFollowsLive = false;
        } else {
            dev_state_set_status(dev, true, "no invariant violation recorded");
        }
    }
    if (GuiButton(column(buttons, 5, 6, 4.0f), "bundle")) {
        export_failure_bundle(game, dev->invariantFailed ? dev->invariantText
                                                         : "exported by hand from the lab\n");
    }

    /* Frame contents at the cursor, plus the live-vs-baseline delta when one exists. */
    const ReplayFrame *frame = dev_replay_frame_at(replay, dev->inspectorCursor);
    const Rectangle frameRow = row(&cursor, 16.0f);
    if (frame != NULL) {
        GuiLabel(
            frameRow,
            TextFormat("tick %llu   steer %+.2f  throttle %.2f  brake %.2f  hb %.2f",
                       (unsigned long long)(replay->firstTick + (uint64_t)dev->inspectorCursor),
                       (double)frame->steer, (double)frame->throttle, (double)frame->brake,
                       (double)frame->handbrake));
    } else {
        GuiLabel(frameRow, "no frames recorded yet");
    }

    const Rectangle delta = row(&cursor, 16.0f);
    if (dev->ghostScopeCount > 0 && dev->scopeCount > 0) {
        const int index = (dev->inspectorCursor < dev->scopeCount) ? dev->inspectorCursor
                                                                   : dev->scopeCount - 1;
        const float liveSideslip = dev_state_scope_value(dev, DEV_SCOPE_SIDESLIP, index);
        const float baseSideslip = dev_state_ghost_scope_value(dev, DEV_SCOPE_SIDESLIP, index);
        const float liveYaw = dev_state_scope_value(dev, DEV_SCOPE_YAW_RATE, index);
        const float baseYaw = dev_state_ghost_scope_value(dev, DEV_SCOPE_YAW_RATE, index);
        GuiLabel(delta, TextFormat("vs baseline: sideslip %+.4f rad   yaw rate %+.4f rad/s",
                                   (double)(liveSideslip - baseSideslip),
                                   (double)(liveYaw - baseYaw)));
    } else {
        GuiLabel(delta, "no baseline captured (F8 captures the current run)");
    }

    const Rectangle replayRow = row(&cursor, ROW_HEIGHT);
    if (GuiTextBox(column(replayRow, 0, 4, 4.0f), g_replayName, sizeof(g_replayName),
                   g_replayNameEditing)) {
        g_replayNameEditing = !g_replayNameEditing;
    }
    if (GuiButton(column(replayRow, 1, 4, 4.0f), "save")) {
        char path[256];
        telemetry_ensure_dir(REPLAY_DIR);
        snprintf(path, sizeof(path), "%s/%s.bin", REPLAY_DIR, g_replayName);
        const bool ok =
            dev_replay_save(replay, path, g_replayName, dev->seed, game->stateChecksum);
        dev_state_set_status(dev, !ok, ok ? "replay saved to %s" : "could not save %s", path);
    }
    if (GuiButton(column(replayRow, 2, 4, 4.0f), "load")) {
        char path[256];
        snprintf(path, sizeof(path), "%s/%s.bin", REPLAY_DIR, g_replayName);
        DevReplayInfo info;
        const bool ok = dev_replay_load(&game->replay, path, &info);
        if (ok) {
            dev->seed = info.seed;
            dev_state_set_status(dev, false, "loaded %u frames from %s", info.frameCount, path);
        } else {
            dev_state_set_status(dev, true, "could not load %s", path);
        }
    }
    if (GuiButton(column(replayRow, 3, 4, 4.0f), "play")) {
        game_reset_sim(game);
        dev_state_clear_history(dev);
        if (replay_begin_playback(&game->replay)) {
            dev->paused = false;
            dev->scenarioRunning = false;
            dev_state_set_status(dev, false, "replaying %d frames", frameCount);
        } else {
            dev_state_set_status(dev, true, "nothing recorded to replay");
        }
    }
}

/* ------------------------------------------------------------------------ parameter panel -- */

static void draw_parameter_rows(struct Game *game, Rectangle view, float scrollY,
                                const char *group)
{
    const int count = dev_params_count();
    float y = view.y + 4.0f + scrollY;

    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        if (strcmp(param->group, group) != 0) continue;
        if (param->tier > game->dev.labTierFilter) continue;

        const Rectangle labelBounds = { view.x + 6.0f, y, view.width - 12.0f, 14.0f };
        const Rectangle sliderBounds = { view.x + 6.0f, y + 15.0f, view.width - 66.0f, 14.0f };
        const Rectangle resetBounds = { view.x + view.width - 54.0f, y + 15.0f, 48.0f, 14.0f };

        /* Skip rows scrolled out of view: with ~100 parameters this matters. */
        if (y + 30.0f >= view.y && y <= view.y + view.height) {
            const bool modified = !param->derived && !dev_param_is_default(&game->spec, param);
            /* The class word is part of the label, not a tooltip: an inactive or
             * appearance-only parameter beside a live slider is exactly the confusion
             * issue 12 exists to remove. */
            GuiLabel(labelBounds,
                     TextFormat("%s%s  [%s]  default %g  (%s)", modified ? "* " : "",
                                param->name, param->unit[0] != '\0' ? param->unit : "-",
                                (double)param->defaultValue,
                                dev_param_class_name(param->classification)));

            float value = dev_param_get(&game->spec, param);
            if (param->derived) {
                GuiLabel(sliderBounds, TextFormat("%.3f", (double)value));
            } else if (GuiSlider(sliderBounds, "", TextFormat("%.3f", (double)value), &value,
                                 param->minimum, param->maximum)) {
                /* Snap to the declared step so a slider drag produces a value a human can
                 * type back into a profile. */
                if (param->step > 0.0f) {
                    value = param->step * roundf(value / param->step);
                }
                dev_param_set(&game->spec, param, value);
                if (param->requiresRestart) {
                    dev_state_set_status(&game->dev, false,
                                         "%s needs a reset (R) to move the contact points",
                                         param->name);
                }
            }
            if (!param->derived && modified && GuiButton(resetBounds, "reset")) {
                dev_param_reset(&game->spec, param);
            }
        }

        y += 34.0f;
    }
}

/* --------------------------------------------------------------------------------- panel -- */

static void draw_lab_panel(struct Game *game)
{
    DevState *dev = &game->dev;
    const float height = (float)GetScreenHeight() - 16.0f -
                         (dev->inspectorVisible ? (INSPECTOR_HEIGHT + 8.0f) : 0.0f);
    const Rectangle panel = { LAB_X, 8.0f, LAB_WIDTH, height };
    GuiPanel(panel, "PHYSICS LAB   F2 hide  F5 pause  F6 step  F7 speed  F8 baseline");

    Cursor cursor = { panel.x + 8.0f, panel.y + 28.0f, panel.width - 16.0f };

    /* Scenario row. The dropdown itself is drawn last so its open list is not painted over;
     * here we only reserve its rectangle. */
    const Rectangle scenarioRow = row(&cursor, ROW_HEIGHT);
    const Rectangle scenarioDropdown = column(scenarioRow, 0, 2, 4.0f);
    const Rectangle scenarioButtons = column(scenarioRow, 1, 2, 4.0f);
    if (GuiButton(column(scenarioButtons, 0, 2, 4.0f), "start")) {
        dev_state_scenario_start(game, dev->scenario);
    }
    if (GuiButton(column(scenarioButtons, 1, 2, 4.0f), "stop")) {
        dev_state_scenario_stop(game);
    }

    /* Time control. */
    const Rectangle timeRow = row(&cursor, ROW_HEIGHT);
    if (GuiButton(column(timeRow, 0, 4, 4.0f), dev->paused ? "resume" : "pause")) {
        dev->paused = !dev->paused;
    }
    if (GuiButton(column(timeRow, 1, 4, 4.0f), "step 1")) {
        dev->paused = true;
        dev->stepTicks += 1;
    }
    if (GuiButton(column(timeRow, 2, 4, 4.0f), "step 10")) {
        dev->paused = true;
        dev->stepTicks += 10;
    }
    if (GuiButton(column(timeRow, 3, 4, 4.0f), "reset")) {
        game_reset_sim(game);
        dev_state_clear_history(dev);
        dev_state_clear_invariants(dev);
    }

    const Rectangle speedRow = row(&cursor, 16.0f);
    GuiSlider(column(speedRow, 0, 1, 0.0f), "", TextFormat("%.2fx", (double)dev->timeScale),
              &dev->timeScale, 0.05f, 4.0f);

    /* Overlays. */
    GuiLine(row(&cursor, 12.0f), "overlays");
    const Rectangle overlayRowA = row(&cursor, 16.0f);
    checkbox(column(overlayRowA, 0, 4, 4.0f), "forces", &dev->showForces);
    checkbox(column(overlayRowA, 1, 4, 4.0f), "vel", &dev->showVelocity);
    checkbox(column(overlayRowA, 2, 4, 4.0f), "slip", &dev->showSlip);
    checkbox(column(overlayRowA, 3, 4, 4.0f), "loads", &dev->showLoads);
    const Rectangle overlayRowB = row(&cursor, 16.0f);
    checkbox(column(overlayRowB, 0, 5, 4.0f), "contact", &dev->showContacts);
    checkbox(column(overlayRowB, 1, 5, 4.0f), "path", &dev->showPath);
    checkbox(column(overlayRowB, 2, 5, 4.0f), "ghost", &dev->showGhost);
    checkbox(column(overlayRowB, 3, 5, 4.0f), "scope", &dev->showScope);
    checkbox(column(overlayRowB, 4, 5, 4.0f), "resist", &dev->showResistance);

    /* Parameter groups. */
    GuiLine(row(&cursor, 12.0f), "tuning");
    const int groupCount = dev_params_group_count();
    const int perRow = 3;
    for (int base = 0; base < groupCount; base += perRow) {
        const Rectangle groupRow = row(&cursor, 18.0f);
        for (int i = 0; i < perRow && base + i < groupCount; i++) {
            const int groupIndex = base + i;
            const bool wasActive = (dev->activeGroup == groupIndex);
            bool active = wasActive;
            /* raygui's GuiToggle returns 0 unconditionally — it only flips *active — so the
             * standard pattern is to detect a 0->1 edge on the local bool. */
            GuiToggle(column(groupRow, i, perRow, 4.0f), dev_params_group_name(groupIndex),
                      &active);
            if (active && !wasActive) {
                dev->activeGroup = groupIndex;
            }
        }
    }

    /* Presets. A dropdown of hardcoded driving profiles (see dev_presets.c) plus an
     * apply button. The dropdown rectangle is reserved here and drawn last, the same
     * pattern as the scenario dropdown, so the open list is not painted over. */
    GuiLine(row(&cursor, 12.0f), "presets");
    const Rectangle presetRow = row(&cursor, ROW_HEIGHT);
    const Rectangle presetDropdown = column(presetRow, 0, 2, 4.0f);
    if (GuiButton(column(presetRow, 1, 2, 4.0f), "apply")) {
        const int applied = dev_preset_apply(&game->spec, g_activePreset);
        const DevPreset *preset = dev_preset_at(g_activePreset);
        dev_state_set_status(dev, false, "%s — %d params set, press R to reset",
                             preset != NULL ? preset->name : "?", applied);
    }

    /* Parameter list. The remaining vertical space belongs to it, minus the footer rows. */
    const float footerHeight = 3.0f * (ROW_HEIGHT + ROW_GAP) + 46.0f + 36.0f +
                               (12.0f + ROW_GAP) + (ROW_HEIGHT + ROW_GAP); /* presets block */
    const float listHeight = fmaxf(80.0f, panel.y + panel.height - cursor.y - footerHeight);
    const Rectangle listBounds = { cursor.x, cursor.y, cursor.width, listHeight };
    cursor.y += listHeight + ROW_GAP;

    const char *group = dev_params_group_name(dev->activeGroup);
    int groupRows = 0;
    for (int i = 0; i < dev_params_count(); i++) {
        if (strcmp(dev_param_at(i)->group, group) == 0) groupRows++;
    }
    const Rectangle content = { 0.0f, 0.0f, listBounds.width - 14.0f,
                                (float)groupRows * 34.0f + 8.0f };
    Vector2 scroll = { 0.0f, dev->paramScrollY };
    Rectangle view = { 0 };
    GuiScrollPanel(listBounds, NULL, content, &scroll, &view);
    dev->paramScrollY = scroll.y;

    BeginScissorMode((int)view.x, (int)view.y, (int)view.width, (int)view.height);
    draw_parameter_rows(game, view, scroll.y, group);
    EndScissorMode();

    /* Profiles. */
    const Rectangle profileRow = row(&cursor, ROW_HEIGHT);
    if (GuiTextBox(column(profileRow, 0, 4, 4.0f), g_profileName, sizeof(g_profileName),
                   g_profileNameEditing)) {
        g_profileNameEditing = !g_profileNameEditing;
    }
    if (GuiButton(column(profileRow, 1, 4, 4.0f), "save")) {
        char path[256];
        telemetry_ensure_dir(PROFILE_DIR);
        snprintf(path, sizeof(path), "%s/%s.txt", PROFILE_DIR, g_profileName);
        const bool ok = dev_params_save(&game->spec, path);
        if (ok) snprintf(g_activeProfile, sizeof(g_activeProfile), "%s", g_profileName);
        dev_state_set_status(dev, !ok, ok ? "profile saved to %s" : "could not save %s", path);
    }
    if (GuiButton(column(profileRow, 2, 4, 4.0f), "load")) {
        char path[256];
        snprintf(path, sizeof(path), "%s/%s.txt", PROFILE_DIR, g_profileName);
        int applied = 0, unknown = 0, rejected = 0;
        if (dev_params_load(&game->spec, path, &applied, &unknown, &rejected)) {
            snprintf(g_activeProfile, sizeof(g_activeProfile), "%s", g_profileName);
            dev_state_set_status(dev, false, "%d applied, %d unknown, %d rejected", applied,
                                 unknown, rejected);
        } else {
            dev_state_set_status(dev, true, "could not load %s", path);
        }
    }
    if (GuiButton(column(profileRow, 3, 4, 4.0f), "defaults")) {
        dev_params_reset_all(&game->spec);
        snprintf(g_activeProfile, sizeof(g_activeProfile), "default");
        dev_state_set_status(dev, false, "every tunable restored to its default");
    }

    /* Phase 3: the load-transfer chain, previous -> filtered -> solved, then where the
     * load went and what resistance is doing. This is the panel to watch during a lift. */
    const Rectangle loadRow = row(&cursor, 16.0f);
    GuiLabel(loadRow, TextFormat("Fz %.0f/%.0f N (st %.0f/%.0f)  xfer %+.0f N",
                                 (double)game->derived.normalLoadFrontN,
                                 (double)game->derived.normalLoadRearN,
                                 (double)game->derived.staticFrontLoadN,
                                 (double)game->derived.staticRearLoadN,
                                 (double)game->derived.loadTransferN));
    const Rectangle accelRow = row(&cursor, 16.0f);
    GuiLabel(accelRow, TextFormat("ax %+.2f>%+.2f>%+.2f  drag %.0f  roll %.0f N",
                                  (double)game->derived.previousLongAccelMps2,
                                  (double)game->derived.filteredLongAccelMps2,
                                  (double)game->derived.solvedLongAccelMps2,
                                  (double)game->derived.aeroDragMagnitudeN,
                                  (double)game->derived.rollingResistanceMagnitudeN));

    /* Run identity. */
    const Rectangle seedRow = row(&cursor, 16.0f);
    const DevScenario *scenario = dev_scenario_at(dev->scenario);
    GuiLabel(seedRow, TextFormat("seed %u   tick %llu   checksum %08x   modified %d", dev->seed,
                                 (unsigned long long)game->sim.tick, game->stateChecksum,
                                 dev_params_modified_count(&game->spec)));

    /* Status line. */
    const Rectangle statusRow = row(&cursor, 16.0f);
    if (dev->statusTimerS > 0.0f) {
        DrawText(dev->status, (int)statusRow.x, (int)statusRow.y, 11,
                 dev->statusIsError ? COLOR_WARNING : COLOR_ACCENT);
    } else if (scenario != NULL) {
        DrawText(TextFormat("scenario: %s - %s", scenario->name, scenario->description),
                 (int)statusRow.x, (int)statusRow.y, 11, COLOR_DIM);
    }

    /* Invariant warning. Deliberately loud and deliberately last, so it is drawn over
     * whatever else wanted that space. */
    const Rectangle warning = { panel.x + 8.0f, panel.y + panel.height - 44.0f,
                                panel.width - 16.0f, 38.0f };
    if (dev->invariantFailed) {
        DrawRectangleRec(warning, (Color){ 90, 22, 18, 245 });
        DrawRectangleLinesEx(warning, 2.0f, COLOR_WARNING);
        DrawText(TextFormat("INVARIANT VIOLATED at tick %llu (%d total)",
                            (unsigned long long)dev->invariantTick, dev->invariantCount),
                 (int)warning.x + 6, (int)warning.y + 5, 11, (Color){ 255, 220, 210, 255 });
        DrawText(dev->invariantText, (int)warning.x + 6, (int)warning.y + 20, 10,
                 (Color){ 255, 200, 190, 255 });
    } else {
        DrawRectangleRec(warning, (Color){ 20, 40, 26, 200 });
        DrawRectangleLinesEx(warning, 1.0f, (Color){ 70, 130, 90, 255 });
        DrawText("invariants holding", (int)warning.x + 6, (int)warning.y + 12, 11,
                 (Color){ 150, 210, 170, 255 });
    }

    /* The dropdowns, last so their open lists are not painted over. The two are kept
     * mutually exclusive so the scenario list and the preset list can never overlap. */
    char scenarioList[512];
    scenarioList[0] = '\0';
    for (int i = 0; i < dev_scenario_count(); i++) {
        if (i > 0) strncat(scenarioList, ";", sizeof(scenarioList) - strlen(scenarioList) - 1u);
        strncat(scenarioList, dev_scenario_at(i)->name,
                sizeof(scenarioList) - strlen(scenarioList) - 1u);
    }
    if (GuiDropdownBox(scenarioDropdown, scenarioList, &dev->scenario,
                       g_scenarioDropdownOpen)) {
        g_scenarioDropdownOpen = !g_scenarioDropdownOpen;
        if (g_scenarioDropdownOpen) g_presetDropdownOpen = false;
    }

    char presetList[256];
    presetList[0] = '\0';
    for (int i = 0; i < dev_preset_count(); i++) {
        if (i > 0) strncat(presetList, ";", sizeof(presetList) - strlen(presetList) - 1u);
        strncat(presetList, dev_preset_at(i)->name,
                sizeof(presetList) - strlen(presetList) - 1u);
    }
    if (GuiDropdownBox(presetDropdown, presetList, &g_activePreset, g_presetDropdownOpen)) {
        g_presetDropdownOpen = !g_presetDropdownOpen;
        if (g_presetDropdownOpen) g_scenarioDropdownOpen = false;
    }
}

void dev_lab_draw_ui(struct Game *game)
{
    if (game == NULL || !game->dev.labVisible) return;
    ensure_style();

    g_eventCount = dev_replay_collect_events(&game->replay, g_events, MAX_EVENTS);

    /* Locked controls draw in their normal state regardless of the pointer, which is what
     * makes --capture-scene reproducible. */
    if (game->dev.uiDeterministic) GuiLock();

    draw_lab_panel(game);
    if (game->dev.showScope) draw_scope(&game->dev);
    if (game->dev.inspectorVisible) {
        const Rectangle bounds = { LAB_X, (float)GetScreenHeight() - INSPECTOR_HEIGHT - 8.0f,
                                   LAB_WIDTH + 420.0f, INSPECTOR_HEIGHT };
        draw_inspector(game, bounds);
    }

    if (game->dev.uiDeterministic) GuiUnlock();
}

#endif /* CIRCUIT_DEV_TOOLS && !CIRCUIT_HEADLESS */

#include "render.h"

#include <math.h>
#include <string.h>

#include "dev_lab.h"
#include "game.h"
#include "math_utils.h"
#include "profile.h"
#include "tire.h"
#include "units.h"

VehicleDrawState render_interpolate_vehicle(const VehicleRenderState *state, float alpha)
{
    VehicleDrawState out;
    memset(&out, 0, sizeof(out));
    if (state == NULL) return out;
    const float t = clampf(alpha, 0.0f, 1.0f);
    out.positionM.x = lerpf(state->prevPositionM.x, state->currPositionM.x, t);
    out.positionM.y = lerpf(state->prevPositionM.y, state->currPositionM.y, t);
    out.headingRad = lerp_angle(state->prevHeadingRad, state->currHeadingRad, t);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        out.wheelAngleRad[i] = lerp_angle(state->prevWheelAngleRad[i],
                                         state->currWheelAngleRad[i], t);
    }
    return out;
}

#if defined(DRIFTY_HEADLESS)
void render_draw_game(struct Game *game, float interpolationAlpha)
{
    (void)game;
    (void)interpolationAlpha;
}
#else

#include "raylib.h"

static Vector2 body_point_to_world(Vector2 bodyPointM, Vector2 positionM, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){
        positionM.x + bodyPointM.x * c - bodyPointM.y * s,
        positionM.y + bodyPointM.x * s + bodyPointM.y * c
    };
}

static Vector2 body_vector_to_world(Vector2 bodyVector, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){
        bodyVector.x * c - bodyVector.y * s,
        bodyVector.x * s + bodyVector.y * c
    };
}

static void draw_world_vector(Vector2 startM, Vector2 vectorM, float ppm,
                              Color color, const char *label)
{
    if (vectorM.x * vectorM.x + vectorM.y * vectorM.y < 0.0004f) return;
    const Vector2 endM = { startM.x + vectorM.x, startM.y + vectorM.y };
    const Vector2 startPx = units_world_to_render_px(startM, ppm);
    const Vector2 endPx = units_world_to_render_px(endM, ppm);
    DrawLineEx(startPx, endPx, 2.0f, color);
    DrawCircleV(endPx, 3.0f, color);
    if (label != NULL && label[0] != '\0') {
        DrawText(label, (int)endPx.x + 4, (int)endPx.y - 8, 12, color);
    }
}

static void draw_vehicle(const Game *game, const VehicleDrawState *draw)
{
    const float ppm = game->renderPixelsPerMeter;
    const Vector2 bodyCenterPx = units_world_to_render_px(draw->positionM, ppm);
    const Rectangle body = {
        bodyCenterPx.x, bodyCenterPx.y,
        units_meters_to_pixels(4.2f, ppm),
        units_meters_to_pixels(1.82f, ppm)
    };
    DrawRectanglePro(body, (Vector2){ body.width * 0.5f, body.height * 0.5f },
                     units_heading_to_rotation_deg(draw->headingRad),
                     (Color){ 214, 76, 58, 255 });

    const Vector2 noseM = body_point_to_world((Vector2){ 2.25f, 0.0f },
                                              draw->positionM, draw->headingRad);
    DrawCircleV(units_world_to_render_px(noseM, ppm), 5.0f,
                (Color){ 255, 220, 100, 255 });

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Vector2 wheelWorldM = body_point_to_world(
            game->vehicle.wheels[i].localPositionM, draw->positionM, draw->headingRad);
        const Vector2 wheelPx = units_world_to_render_px(wheelWorldM, ppm);
        const Rectangle wheel = {
            wheelPx.x, wheelPx.y,
            units_meters_to_pixels(0.72f, ppm),
            units_meters_to_pixels(0.24f, ppm)
        };
        DrawRectanglePro(wheel, (Vector2){ wheel.width * 0.5f, wheel.height * 0.5f },
                         units_heading_to_rotation_deg(draw->headingRad +
                                                       draw->wheelAngleRad[i]),
                         (Color){ 35, 38, 43, 255 });
    }
}

static void draw_debug_vectors(const Game *game, const VehicleDrawState *draw)
{
    const float ppm = game->renderPixelsPerMeter;
    const float heading = draw->headingRad;
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 2.0f, 0.0f }, heading),
                      ppm, YELLOW, "");
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 0.0f, 1.5f }, heading),
                      ppm, SKYBLUE, "");
    const Vector2 bodyVelocity = {
        game->vehicle.velocityLongitudinalMps * 0.30f,
        game->vehicle.velocityLateralMps * 0.30f
    };
    draw_world_vector(draw->positionM, body_vector_to_world(bodyVelocity, heading),
                      ppm, GREEN, "velocity");

    const Vector2 frontM = body_point_to_world(
        (Vector2){ game->spec.cgToFrontM, 0.0f }, draw->positionM, heading);
    const Vector2 rearM = body_point_to_world(
        (Vector2){ -game->spec.cgToRearM, 0.0f }, draw->positionM, heading);
    draw_world_vector(frontM, body_vector_to_world(
        (Vector2){ game->derived.frontAxleContactVelocityBodyMps.x * 0.20f,
                   game->derived.frontAxleContactVelocityBodyMps.y * 0.20f }, heading),
        ppm, ORANGE, "front v");
    draw_world_vector(rearM, body_vector_to_world(
        (Vector2){ game->derived.rearAxleContactVelocityBodyMps.x * 0.20f,
                   game->derived.rearAxleContactVelocityBodyMps.y * 0.20f }, heading),
        ppm, PURPLE, "rear v");
    draw_world_vector(frontM, body_vector_to_world(
        (Vector2){ cosf(draw->wheelAngleRad[WHEEL_FRONT_LEFT]) * 1.4f,
                   sinf(draw->wheelAngleRad[WHEEL_FRONT_LEFT]) * 1.4f }, heading),
        ppm, GOLD, "front heading");
    draw_world_vector(frontM, body_vector_to_world(
        (Vector2){ game->derived.frontBodyForceN.x * 0.00008f,
                   game->derived.frontBodyForceN.y * 0.00008f }, heading),
        ppm, RED, "front force");
    draw_world_vector(rearM, body_vector_to_world(
        (Vector2){ game->derived.rearBodyForceN.x * 0.00008f,
                   game->derived.rearBodyForceN.y * 0.00008f }, heading),
        ppm, MAROON, "rear force");
}

static void hud_line(int x, int *y, const char *text, Color color)
{
    DrawText(text, x, *y, 16, color);
    *y += 19;
}

static Vector2 plot_point(Rectangle bounds, float x, float xMin, float xMax,
                          float y, float yMin, float yMax)
{
    return (Vector2){
        bounds.x + (x - xMin) / (xMax - xMin) * bounds.width,
        bounds.y + bounds.height - (y - yMin) / (yMax - yMin) * bounds.height
    };
}

static void draw_curve_axes(Rectangle bounds, float xMin, float xMax,
                            float yMin, float yMax)
{
    DrawRectangleRec(bounds, (Color){ 16, 18, 22, 235 });
    DrawRectangleLinesEx(bounds, 1.0f, (Color){ 100, 106, 116, 255 });
    DrawLineV(plot_point(bounds, xMin, xMin, xMax, 0.0f, yMin, yMax),
              plot_point(bounds, xMax, xMin, xMax, 0.0f, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
    DrawLineV(plot_point(bounds, 0.0f, xMin, xMax, yMin, yMin, yMax),
              plot_point(bounds, 0.0f, xMin, xMax, yMax, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
}

static void draw_tire_curve_panel(const Game *game)
{
    const Rectangle lateral = { SCREEN_W - 390.0f, 26.0f, 365.0f, 145.0f };
    const Rectangle longitudinal = { SCREEN_W - 390.0f, 205.0f, 365.0f, 145.0f };
    const float latMin = -0.55f;
    const float latMax = 0.55f;
    const float forceMin = -1.4f;
    const float forceMax = 1.4f;
    draw_curve_axes(lateral, latMin, latMax, forceMin, forceMax);
    DrawText("LATERAL  normalized force / wheel load", (int)lateral.x,
             (int)lateral.y - 19, 14, RAYWHITE);
    DrawText("front", (int)lateral.x + 6, (int)lateral.y + 5, 12, ORANGE);
    DrawText("rear", (int)lateral.x + 52, (int)lateral.y + 5, 12, SKYBLUE);
    Vector2 prevFront = { 0.0f, 0.0f };
    Vector2 prevRear = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(latMin, latMax, (float)i / 120.0f);
        const float front = -game->spec.tireMuLatFront * tire_normalized_curve(
            game->spec.tireBLatFront, game->spec.tireCLatFront, slip);
        const float rear = -game->spec.tireMuLatRear * tire_normalized_curve(
            game->spec.tireBLatRear, game->spec.tireCLatRear, slip);
        const Vector2 pFront = plot_point(
            lateral, slip, latMin, latMax, front, forceMin, forceMax);
        const Vector2 pRear = plot_point(
            lateral, slip, latMin, latMax, rear, forceMin, forceMax);
        if (i > 0) {
            DrawLineV(prevFront, pFront, ORANGE);
            DrawLineV(prevRear, pRear, SKYBLUE);
        }
        prevFront = pFront;
        prevRear = pRear;
    }
    const float currentFront = -game->spec.tireMuLatFront * tire_normalized_curve(
        game->spec.tireBLatFront, game->spec.tireCLatFront,
        game->derived.frontSlipAngleRad);
    const float currentRear = -game->spec.tireMuLatRear * tire_normalized_curve(
        game->spec.tireBLatRear, game->spec.tireCLatRear,
        game->derived.rearSlipAngleRad);
    DrawCircleV(plot_point(lateral, game->derived.frontSlipAngleRad, latMin, latMax,
                           currentFront, forceMin, forceMax), 4.0f, ORANGE);
    DrawCircleV(plot_point(lateral, game->derived.rearSlipAngleRad, latMin, latMax,
                           currentRear, forceMin, forceMax), 4.0f, SKYBLUE);

    const float longMin = -1.25f;
    const float longMax = 1.25f;
    draw_curve_axes(longitudinal, longMin, longMax, -1.1f, 1.1f);
    DrawText("LONGITUDINAL  normalized force / wheel load",
             (int)longitudinal.x, (int)longitudinal.y - 19, 14, RAYWHITE);
    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(longMin, longMax, (float)i / 120.0f);
        const float force = game->spec.tireMuLongScale * tire_normalized_curve(
            game->spec.tireBLong, game->spec.tireCLong, slip);
        const Vector2 point = plot_point(
            longitudinal, slip, longMin, longMax, force, -1.1f, 1.1f);
        if (i > 0) DrawLineV(previous, point, LIME);
        previous = point;
    }
    const float rearSlip = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
    const float rearLong = game->spec.tireMuLongScale * tire_normalized_curve(
        game->spec.tireBLong, game->spec.tireCLong, rearSlip);
    DrawCircleV(plot_point(longitudinal, rearSlip, longMin, longMax,
                           rearLong, -1.1f, 1.1f), 4.0f, YELLOW);
    DrawText("zero axes; curve peaks are the configured friction references",
             (int)longitudinal.x, (int)longitudinal.y + (int)longitudinal.height + 5,
             11, (Color){ 155, 160, 170, 255 });
}

static const char *gear_label(int selectedGear)
{
    if (selectedGear < 0) return "R";
    if (selectedGear == 0) return "N";
    static const char *labels[MAX_GEARS] = { "1", "2", "3", "4", "5", "6", "7", "8" };
    if (selectedGear <= MAX_GEARS) return labels[selectedGear - 1];
    return "?";
}

void render_draw_game(struct Game *game, float interpolationAlpha)
{
    if (game == NULL) return;

    /* Development shortcuts and the status-line timer, once per render frame. Compiles to
     * nothing when the dev tools are not built in. */
    dev_lab_update(game);

    DRIFTY_ZONE_BEGIN(render, "Render");
    const float renderDt = GetFrameTime();
    if (game->reloadFlashTimerS > 0.0f) {
        game->reloadFlashTimerS = fmaxf(0.0f, game->reloadFlashTimerS - renderDt);
    }
    const float alpha = clampf(interpolationAlpha, 0.0f, 1.0f);
    const VehicleDrawState draw = render_interpolate_vehicle(&game->renderState, alpha);
    game->camera.target = units_world_to_render_px(draw.positionM, game->renderPixelsPerMeter);

    BeginDrawing();
    ClearBackground((Color){ 22, 24, 28, 255 });
    BeginMode2D(game->camera);
    DrawLine(-10000, 0, 10000, 0, (Color){ 55, 59, 66, 255 });
    DrawLine(0, -10000, 0, 10000, (Color){ 55, 59, 66, 255 });
    draw_vehicle(game, &draw);
    if (game->debugOverlay) draw_debug_vectors(game, &draw);
    dev_lab_draw_world(game, &draw);
    EndMode2D();

    const Color label = (Color){ 235, 235, 235, 255 };
    const Color dim = (Color){ 155, 160, 170, 255 };
    int y = 12;
    hud_line(14, &y, "DRIFTY  Phase 3 - load transfer, drag, and rolling resistance", label);
    hud_line(14, &y, TextFormat("pos (%+.2f,%+.2f) m  heading %+.3f rad",
             (double)game->vehicle.positionM.x, (double)game->vehicle.positionM.y,
             (double)game->vehicle.headingRad), dim);
    hud_line(14, &y, TextFormat("vx %+.3f m/s  vy %+.3f m/s  speed %.3f m/s  yaw %+.3f rad/s",
             (double)game->vehicle.velocityLongitudinalMps,
             (double)game->vehicle.velocityLateralMps, (double)game->derived.speedMps,
             (double)game->vehicle.yawRateRadS), label);
    hud_line(14, &y, TextFormat("steer %+.3f  slip F/R %+.3f / %+.3f rad  sideslip %+.3f",
             (double)game->vehicle.frontRoadWheelAngleRad,
             (double)game->derived.frontSlipAngleRad,
             (double)game->derived.rearSlipAngleRad,
             (double)game->derived.bodySideslipRad), dim);
    hud_line(14, &y, TextFormat("gear %s  engine %.0f rpm  rear drive %+.1f Nm",
             gear_label(game->vehicle.selectedGear), (double)game->vehicle.engineRpm,
             (double)game->derived.drivelineTorqueNm), label);
    hud_line(14, &y, TextFormat("omega FL/FR/R %.2f / %.2f / %.2f rad/s  surface R %+.2f m/s",
             (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS,
             (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS,
             (double)game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS,
             (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS *
                      game->spec.wheelRadiusM)), dim);
    hud_line(14, &y, TextFormat("kappa FL/FR/RL/RR %+.3f %+.3f %+.3f %+.3f",
             (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio,
             (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio,
             (double)game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio,
             (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio), dim);
    hud_line(14, &y, TextFormat("load F/R %.1f / %.1f N  transfer %+.1f N  lateral F/R %+.1f / %+.1f N",
             (double)game->derived.normalLoadFrontN,
             (double)game->derived.normalLoadRearN,
             (double)game->derived.loadTransferN,
             (double)game->derived.frontLateralForceN,
             (double)game->derived.rearLateralForceN), dim);
    hud_line(14, &y, TextFormat("body force (%+.1f,%+.1f) N  yaw torque %+.1f Nm  blend %.3f",
             (double)game->derived.totalBodyForceN.x,
             (double)game->derived.totalBodyForceN.y,
             (double)game->derived.totalYawTorqueNm,
             (double)game->derived.lowSpeedBlend), dim);
    hud_line(14, &y, TextFormat("substeps %d  backlog %d  alpha %.3f  tick %llu  checksum %08x",
             game->lastSubstepCount, game->physicsBacklogDrops, (double)alpha,
             (unsigned long long)game->sim.tick, game->stateChecksum), label);
    hud_line(14, &y, TextFormat("reloads %d%s  gear %s  render %.1f px/m",
             game->reloadCount, game->reloadFlashTimerS > 0.0f ? " (state preserved)" : "",
             gear_label(game->vehicle.selectedGear),
             (double)game->renderPixelsPerMeter), dim);

    if (game->debugOverlay) {
        hud_line(14, &y, TextFormat("pure Fx F/R %+.0f/%+.0f  pure Fy F/R %+.0f/%+.0f N",
                 (double)(game->derived.pureLongitudinalForceN[WHEEL_FRONT_LEFT] +
                          game->derived.pureLongitudinalForceN[WHEEL_FRONT_RIGHT]),
                 (double)(game->derived.pureLongitudinalForceN[WHEEL_REAR_LEFT] +
                          game->derived.pureLongitudinalForceN[WHEEL_REAR_RIGHT]),
                 (double)(game->derived.pureLateralForceN[WHEEL_FRONT_LEFT] +
                          game->derived.pureLateralForceN[WHEEL_FRONT_RIGHT]),
                 (double)(game->derived.pureLateralForceN[WHEEL_REAR_LEFT] +
                          game->derived.pureLateralForceN[WHEEL_REAR_RIGHT])), dim);
        hud_line(14, &y, TextFormat("limited Fx F/R %+.0f/%+.0f  limited Fy F/R %+.0f/%+.0f N",
                 (double)(game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                          game->vehicle.wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN),
                 (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                          game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN),
                 (double)game->derived.frontLateralForceN,
                 (double)game->derived.rearLateralForceN), dim);
        hud_line(14, &y, TextFormat("usage FL/FR/RL/RR %.2f %.2f %.2f %.2f  lock %d%d%d%d",
                 (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                 (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage,
                 (double)game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                 (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage,
                 game->vehicle.wheels[WHEEL_FRONT_LEFT].locked,
                 game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked,
                 game->vehicle.wheels[WHEEL_REAR_LEFT].locked,
                 game->vehicle.wheels[WHEEL_REAR_RIGHT].locked), dim);
        hud_line(14, &y, TextFormat("brake F/R %.0f/%.0f Nm  handbrake R %.0f Nm",
                 (double)(game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                          game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT]),
                 (double)(game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                          game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT]),
                 (double)(game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
                          game->derived.handbrakeTorqueNm[WHEEL_REAR_RIGHT])), dim);
        hud_line(14, &y, TextFormat("ax prev/filt/solved %+.2f/%+.2f/%+.2f m/s^2  "
                 "drag %.0f N  rolling %.0f N",
                 (double)game->derived.previousLongAccelMps2,
                 (double)game->derived.filteredLongAccelMps2,
                 (double)game->derived.solvedLongAccelMps2,
                 (double)game->derived.aeroDragMagnitudeN,
                 (double)game->derived.rollingResistanceMagnitudeN), dim);
        draw_tire_curve_panel(game);
    }

    DrawText("W throttle  S brake  Space handbrake  Q/E shift  A/D steer  R reset  "
             "F1 diagnostics  F2 physics lab",
             14, SCREEN_H - 28, 17, dim);

    /* The lab paints over the HUD deliberately: when it is open it is what you are reading. */
    DRIFTY_ZONE_BEGIN(lab, "PhysicsLab");
    dev_lab_draw_ui(game);
    DRIFTY_ZONE_END(lab);

    EndDrawing();
    DRIFTY_ZONE_END(render);
    DRIFTY_FRAME_MARK();
}
#endif

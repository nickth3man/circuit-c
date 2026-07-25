#include "render.h"

#include <math.h>
#include <string.h>

#include "game.h"
#include "math_utils.h"
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
    const Vector2 endM = { startM.x + vectorM.x, startM.y + vectorM.y };
    const Vector2 startPx = units_world_to_render_px(startM, ppm);
    const Vector2 endPx = units_world_to_render_px(endM, ppm);
    DrawLineEx(startPx, endPx, 2.0f, color);
    DrawCircleV(endPx, 3.0f, color);
    DrawText(label, (int)endPx.x + 4, (int)endPx.y - 8, 12, color);
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
                      ppm, YELLOW, "body +X");
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 0.0f, 1.5f }, heading),
                      ppm, SKYBLUE, "body +Y");
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
    draw_world_vector(rearM, body_vector_to_world((Vector2){ 1.4f, 0.0f }, heading),
                      ppm, BEIGE, "rear heading");
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

void render_draw_game(struct Game *game, float interpolationAlpha)
{
    if (game == NULL) return;
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
    EndMode2D();

    const Color label = (Color){ 235, 235, 235, 255 };
    const Color dim = (Color){ 155, 160, 170, 255 };
    int y = 12;
    hud_line(14, &y, "DRIFTY  Phase 1 - rigid-body vehicle", label);
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
    hud_line(14, &y, TextFormat("load F/R %.1f / %.1f N  lateral F/R %+.1f / %+.1f N",
             (double)game->derived.normalLoadFrontN,
             (double)game->derived.normalLoadRearN,
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
    hud_line(14, &y, TextFormat("reloads %d%s  direction %s  render %.1f px/m",
             game->reloadCount, game->reloadFlashTimerS > 0.0f ? " (state preserved)" : "",
             game->vehicle.selectedGear < 0 ? "reverse" : "forward",
             (double)game->renderPixelsPerMeter), dim);

    DrawText("W throttle  S brake  Q reverse  E forward  A/D steer  R reset  F1 vectors",
             14, SCREEN_H - 28, 17, dim);
    EndDrawing();
}
#endif

/*
 * units.h — the single boundary between simulation meters and render pixels.
 *
 * COORDINATE AND SIGN CONVENTION (docs/SPEC.md, "Coordinate and sign convention").
 * Every equation in this project obeys it:
 *
 *   Body X            forward (nose direction)
 *   Body Y            left
 *   World X / Y       right-handed simulation plane; +Y is "up" on screen after the flip
 *   Heading           angle of body X measured from world X, counterclockwise positive
 *   Yaw rate          counterclockwise positive
 *   Steering angle    left positive
 *   Slip angle        positive when contact-point velocity points left of wheel heading
 *   Lateral force     always opposes the slip angle
 *   Slip ratio        positive when wheel surface speed exceeds ground speed
 *
 * World meters -> render pixel space, which has +Y DOWN:
 *
 *     screen.x =  world_m.x * pixelsPerMeter
 *     screen.y = -world_m.y * pixelsPerMeter
 *
 * Because the render layer negates Y, a counterclockwise world heading still reads as
 * counterclockwise on screen. raylib's rotation arguments are degrees, positive clockwise
 * in screen space, hence the negation in units_heading_to_rotation_deg().
 *
 * The scale is an explicit parameter rather than a hard-wired macro read: simulation code
 * has no way to reach it by accident, and the headless harness can run the same simulation
 * at two different scales and compare checksums. PIXELS_PER_METER in config.h supplies the
 * default, held at runtime in Game.renderPixelsPerMeter.
 *
 * This header includes raylib.h for the Vector2 type only. It calls no raylib function, so
 * translation units that include it still link without a window.
 */
#ifndef DRIFTY_UNITS_H
#define DRIFTY_UNITS_H

#include <math.h>

#include "raylib.h"

#include "core/config.h"
#include "core/math_utils.h"

/* Scalar length: world meters -> render pixels. */
static inline float units_meters_to_pixels(float meters, float pixelsPerMeter)
{
    return meters * pixelsPerMeter;
}

/* Scalar length: render pixels -> world meters. A non-positive scale is meaningless, so
 * it yields 0 rather than an infinity that would propagate into a draw call. */
static inline float units_pixels_to_meters(float pixels, float pixelsPerMeter)
{
    if (pixelsPerMeter <= 0.0f) return 0.0f;
    return pixels / pixelsPerMeter;
}

/* World position (meters, +Y up) -> render pixel space (+Y down). */
static inline Vector2 units_world_to_render_px(Vector2 worldM, float pixelsPerMeter)
{
    Vector2 px;
    px.x =  worldM.x * pixelsPerMeter;
    px.y = -worldM.y * pixelsPerMeter;
    return px;
}

/* Render pixel space (+Y down) -> world position (meters, +Y up). */
static inline Vector2 units_render_px_to_world(Vector2 px, float pixelsPerMeter)
{
    Vector2 worldM;
    if (pixelsPerMeter <= 0.0f) {
        worldM.x = 0.0f;
        worldM.y = 0.0f;
        return worldM;
    }
    worldM.x =  px.x / pixelsPerMeter;
    worldM.y = -px.y / pixelsPerMeter;
    return worldM;
}

/* World heading (radians, counterclockwise positive) -> raylib rotation
 * (degrees, clockwise positive in render space). */
static inline float units_heading_to_rotation_deg(float headingRad)
{
    return -headingRad * DRIFTY_RAD2DEG;
}

/* Inverse of units_heading_to_rotation_deg(). */
static inline float units_rotation_deg_to_heading(float rotationDeg)
{
    return -rotationDeg * DRIFTY_DEG2RAD;
}

/* ---------------------------------------------------------------- pixel-grid snapping --
 *
 * A raylib Camera2D maps world to screen as (world - target) * zoom + offset. When the world
 * is rendered into a low-resolution target and then enlarged by an integer factor with
 * nearest-neighbour filtering, a FRACTIONAL camera translation makes every world pixel land
 * between two target pixels — and as the camera follows the car, the whole grid crawls a
 * pixel at a time. That shimmer is the single most visible way to get pixel art wrong.
 *
 * Given an integral base offset, this returns the offset that makes the translation
 * (-target * zoom + offset) land exactly on a whole target pixel. The cost is sub-pixel
 * camera smoothness, which nobody can see; the gain is a stable grid, which everybody can.
 *
 * Pure and header-only so the property is reachable from the headless test binary — the
 * shimmer itself needs a moving camera and an eye, but the arithmetic underneath does not.
 */
static inline float units_snap_camera_offset_axis(float offset, float target, float zoom)
{
    const float translated = target * zoom;
    return offset + (translated - floorf(translated));
}

/* The resulting translation, which callers assert is integral. */
static inline float units_camera_translation_axis(float offset, float target, float zoom)
{
    return offset - target * zoom;
}

#endif /* DRIFTY_UNITS_H */

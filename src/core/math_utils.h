/*
 * math_utils.h — the small set of scalar helpers raylib's raymath.h does not provide.
 *
 * These are pure functions of their arguments. They never read window, render, or global
 * state, so the headless test executable links them without raylib.
 */
#ifndef DRIFTY_MATH_UTILS_H
#define DRIFTY_MATH_UTILS_H

#ifndef DRIFTY_PI
#define DRIFTY_PI 3.14159265358979323846f
#endif
#ifndef DRIFTY_TWO_PI
#define DRIFTY_TWO_PI (2.0f * DRIFTY_PI)
#endif
#ifndef DRIFTY_RAD2DEG
#define DRIFTY_RAD2DEG (180.0f / DRIFTY_PI)
#endif
#ifndef DRIFTY_DEG2RAD
#define DRIFTY_DEG2RAD (DRIFTY_PI / 180.0f)
#endif

/* Clamp v into [lo, hi]. If hi < lo the bounds are swapped, so the result is always
 * within the interval the caller named. */
float clampf(float v, float lo, float hi);

/* Linear interpolation. t is not clamped: t < 0 or t > 1 extrapolates. */
float lerpf(float a, float b, float t);
/* Maximum/minimum of two floats. */
float maxf(float a, float b);
float minf(float a, float b);

/* Frame-rate independent exponential approach of current toward target.
 * rateHz is the reciprocal time constant in 1/seconds; dt is in seconds.
 * dt <= 0 or rateHz <= 0 returns current unchanged. */
float smooth_to(float current, float target, float rateHz, float dt);

/* Wrap an angle into the canonical range [-PI, +PI).
 * +PI wraps to -PI; the interval is closed at the bottom and open at the top. */
float wrap_angle(float angleRad);

/* Hermite smoothstep. Returns 0 at or below edge0, 1 at or above edge1, and
 * 3t^2 - 2t^3 between them.
 * Degenerate bounds (edge1 <= edge0, or either edge NaN) collapse to a hard step at
 * edge0: 0 below it, 1 at or above it. The result is always finite and in [0, 1]. */
float smoothstep(float edge0, float edge1, float x);

/* Interpolate between two angles along the shortest wrapped path, so a heading crossing
 * +-PI does not spin through a full rotation. The result is wrapped into [-PI, +PI). */
float lerp_angle(float aRad, float bRad, float t);

#endif /* DRIFTY_MATH_UTILS_H */

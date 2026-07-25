#include "math_utils.h"

#include <math.h>

float clampf(float v, float lo, float hi)
{
    if (hi < lo) {
        const float swap = lo;
        lo = hi;
        hi = swap;
    }
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

float smooth_to(float current, float target, float rateHz, float dt)
{
    if (dt <= 0.0f || rateHz <= 0.0f) return current;
    const float blend = 1.0f - expf(-rateHz * dt);
    return current + (target - current) * blend;
}

float wrap_angle(float angleRad)
{
    /* fmodf's magnitude is strictly less than its divisor, so the shifted remainder lies
     * in [0, 2PI) and the returned value lies in [-PI, +PI). */
    float shifted = fmodf(angleRad + DRIFTY_PI, DRIFTY_TWO_PI);
    if (shifted < 0.0f) shifted += DRIFTY_TWO_PI;
    return shifted - DRIFTY_PI;
}

float smoothstep(float edge0, float edge1, float x)
{
    /* Written as a negated comparison so a NaN edge takes the degenerate branch rather
     * than dividing by NaN. */
    if (!(edge1 > edge0)) return (x < edge0) ? 0.0f : 1.0f;

    const float t = clampf((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float lerp_angle(float aRad, float bRad, float t)
{
    const float delta = wrap_angle(bRad - aRad);
    return wrap_angle(aRad + delta * t);
}

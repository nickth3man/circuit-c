/*
 * appearance_metrics.c — the shared canvas, label rasterization, signature distances, and the
 * primary-parameter difference count.
 */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "appearance_metrics.h"

#include "dev/car_corpus.h"
#include "dev/dev_params.h"

/* Every car is rasterized into ONE canvas with the CG at a fixed point, so label maps are
 * directly comparable and an alignment difference cannot masquerade as a shape difference. */
CarRasterInfo test_car_shared_canvas(float pxPerM)
{
    float left = 1.0f, right = 1.0f, up = 1.0f, down = 1.0f;

    for (int i = 0; i < car_corpus_count(); i++) {
        VehicleSpec spec;
        CarVisual visual;
        if (!car_corpus_spec(i, &spec)) continue;
        car_visual_derive(&spec, &visual);

        const CarRasterInfo info = car_raster_info(&visual, pxPerM, 2);
        const float l = info.originXPx;
        const float r = (float)info.width - info.originXPx;
        const float u = info.originYPx;
        const float d = (float)info.height - info.originYPx;
        if (l > left)  left  = l;
        if (r > right) right = r;
        if (u > up)    up    = u;
        if (d > down)  down  = d;
    }

    CarRasterInfo shared;
    memset(&shared, 0, sizeof(shared));
    shared.pxPerM    = pxPerM;
    shared.width     = (int)ceilf(left + right);
    shared.height    = (int)ceilf(up + down);
    shared.originXPx = left;
    shared.originYPx = up;
    return shared;
}

bool test_car_labels_for_spec(const VehicleSpec *spec, CarRasterInfo canvas,
                               unsigned char *labels, size_t bytes)
{
    CarVisual visual;
    car_visual_derive(spec, &visual);
    return car_raster_draw_labels(&visual, canvas, labels, bytes);
}

/* Largest single-component gap between two signature vectors, and which component it was. */
float test_car_signature_linf(const CarVisual *a, const CarVisual *b, int *worstOut)
{
    float sa[CAR_SIGNATURE_MAX], sb[CAR_SIGNATURE_MAX];
    const int n = car_visual_signature_count();
    if (n > (int)(sizeof(sa) / sizeof(sa[0]))) return 0.0f;
    if (car_visual_signature(a, sa, n) != n) return 0.0f;
    if (car_visual_signature(b, sb, n) != n) return 0.0f;

    float worst = 0.0f;
    int worstIndex = 0;
    for (int i = 0; i < n; i++) {
        const float d = fabsf(sa[i] - sb[i]);
        if (d > worst) { worst = d; worstIndex = i; }
    }
    if (worstOut != NULL) *worstOut = worstIndex;
    return worst;
}

/* Euclidean distance between two signature vectors, in the same "visible metres" currency. */
float test_car_signature_l2(const CarVisual *a, const CarVisual *b)
{
    float sa[CAR_SIGNATURE_MAX], sb[CAR_SIGNATURE_MAX];
    const int n = car_visual_signature_count();
    if (n > (int)(sizeof(sa) / sizeof(sa[0]))) return 0.0f;
    if (car_visual_signature(a, sa, n) != n) return 0.0f;
    if (car_visual_signature(b, sb, n) != n) return 0.0f;

    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        const float d = sa[i] - sb[i];
        sum += d * d;
    }
    return sqrtf(sum);
}

/* Every primary (non-derived) registry key that differs between two specs. Derived rows are
 * skipped because they are consequences, not choices: a sweep of body.wheelbase legitimately
 * moves cgToFront, yaw inertia and frontal area with it. */
int test_car_primary_diff_count(const VehicleSpec *a, const VehicleSpec *b,
                                 const char **firstName)
{
    int diffs = 0;
    for (int p = 0; p < dev_params_count(); p++) {
        const DevParameter *param = dev_param_at(p);
        if (param->derived) continue;
        const float va = dev_param_get(a, param);
        const float vb = dev_param_get(b, param);
        const float tol = (param->step > 0.0f) ? (0.25f * param->step) : 1e-5f;
        if (fabsf(va - vb) > tol) {
            if (diffs == 0 && firstName != NULL) *firstName = param->name;
            diffs++;
        }
    }
    return diffs;
}

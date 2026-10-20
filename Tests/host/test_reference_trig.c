#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_reference_trig.h"

int main(void)
{
    const int terms[] = {3, 5, 7, 9, 12};
    unsigned family, order, index;
    for (family = 0U; family < 3U; ++family) {
        float span = family == 0U ? 0.785398163f : family == 1U ? 3.141592654f : 64.0f;
        for (order = 0U; order < sizeof(terms) / sizeof(terms[0]); ++order) {
            float maximum = 0.0f;
            unsigned failures = 0U;
            for (index = 0U; index <= 32768U; ++index) {
                float angle = -span + 2.0f * span * (float)index / 32768.0f;
                float sine, cosine, error;
                Motor_ReferenceTrigRaw(angle, terms[order], &sine, &cosine);
                error = fmaxf(fabsf(sine - sinf(angle)), fabsf(cosine - cosf(angle)));
                if (!isfinite(sine) || !isfinite(cosine) || error > 0.000025f) failures++;
                if (error > maximum) maximum = error;
            }
            printf("Reference trig span=%g n=%d max_error=%g over_2.5e-5=%u/32769\n",
                   span, terms[order], maximum, failures);
        }
    }
    {
        float sine, cosine;
        Motor_ReferenceTrigRaw(3.141592654f, 3, &sine, &cosine);
        assert(fabsf(sine) > 0.1f && fabsf(cosine + 1.0f) > 0.1f);
    }
    return 0;
}

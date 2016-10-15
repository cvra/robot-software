#include <math.h>
#include "math_helpers.h"

float angle_delta(float start, float end)
{
    float res = fmodf(end - start + M_PI, 2 * M_PI) - M_PI;
    float res_conj = 2 * M_PI - res;

    if (fabsf(res) < fabsf(res_conj)) {
        return res;
    } else {
        return res_conj;
    }
}

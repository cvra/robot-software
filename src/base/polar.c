#include <math.h>
#include "polar.h"

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar)
{
    polar->distance = (wheels.right + wheels.left) / 2;
    polar->angle    = (wheels.right - wheels.left) / 2;
}

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

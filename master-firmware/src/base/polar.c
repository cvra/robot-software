#include <math.h>
#include "polar.h"

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar)
{
    polar->distance = (wheels.right + wheels.left) / 2;
    polar->angle    = (wheels.right - wheels.left) / 2;
}

void polar_get_wheels_from_polar(const polar_t polar, wheels_t *wheels)
{
    wheels->right = polar.distance + polar.angle;
    wheels->left = polar.distance - polar.angle;
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

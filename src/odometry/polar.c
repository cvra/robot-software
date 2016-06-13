#include <math.h>
#include "polar.h"

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar)
{
    polar->distance = (wheels.right + wheels.left) / 2;
    polar->angle    = (wheels.right - wheels.left) / 2;
}

float angle_wrap(float angle)
{
    float res = fmodf(angle + M_PI, 2 * M_PI);

    if (res < 0) {
        res += (2 * M_PI);
    }

    return (res - M_PI);
}

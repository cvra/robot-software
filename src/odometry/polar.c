#include "polar.h"

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar)
{
    polar->distance = (wheels.right + wheels.left) / 2;
    polar->angle    = (wheels.right - wheels.left) / 2;
}

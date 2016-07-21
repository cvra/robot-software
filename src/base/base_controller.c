#include <math.h>
#include "polar.h"
#include "base_controller.h"

void base_controller_init(base_controller_t *base)
{
    pid_init(&(base->distance_pid));
    pid_init(&(base->heading_pid));
}

void base_controller_compute_error(polar_t *error, pose2d_t desired, pose2d_t measured)
{
    error->distance = sqrtf(powf(desired.x - measured.x, 2)
                            + powf(desired.y - measured.y, 2));

    error->angle = angle_delta(measured.heading, desired.heading);
}

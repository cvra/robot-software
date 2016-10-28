#include <math.h>

#include "beacon_helpers.h"


float beacon_get_angle(float start_angle, float signal_length)
{
    float angle = start_angle + signal_length / 2;

    if (angle > M_PI) {
        angle -= (M_PI * 2.);
    } else if (angle < - M_PI) {
        angle += (M_PI * 2.);
    }

    return angle;
}

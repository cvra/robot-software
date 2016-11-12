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

float beacon_cartesian_convert(struct robot_position* robot_pos, float distance, float angle, float* x, float* y)
{
    *x = distance * cosf(position_get_a_rad_float(robot_pos) + angle) + position_get_x_float(robot_pos);
    *y = distance * sinf(position_get_a_rad_float(robot_pos) + angle) + position_get_y_float(robot_pos);
}

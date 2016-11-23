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

void beacon_cartesian_convert(struct robot_position* robot_pos, float distance, float angle, float* x, float* y)
{
    *x = distance * cosf(position_get_a_rad_float(robot_pos) + angle) + position_get_x_float(robot_pos);
    *y = distance * sinf(position_get_a_rad_float(robot_pos) + angle) + position_get_y_float(robot_pos);
}

void beacon_set_opponent_obstacle(poly_t* opponent, int x, int y, int opponent_size, int robot_size)
{
    opponent->pts[3].x = x - (opponent_size + robot_size) / 2;
    opponent->pts[3].y = y - (opponent_size + robot_size) / 2;

    opponent->pts[2].x = x - (opponent_size + robot_size) / 2;
    opponent->pts[2].y = y + (opponent_size + robot_size) / 2;

    opponent->pts[1].x = x + (opponent_size + robot_size) / 2;
    opponent->pts[1].y = y + (opponent_size + robot_size) / 2;

    opponent->pts[0].x = x + (opponent_size + robot_size) / 2;
    opponent->pts[0].y = y - (opponent_size + robot_size) / 2;
}

void beacon_create_opponent_obstacle(beacon_opponent_obstacle_t* opponent, int x, int y, int opponent_size, int robot_size)
{
    opponent->polygon.pts = opponent->points;
    opponent->polygon.l = BEACON_OPPONENT_NUM_EDGES;

    beacon_set_opponent_obstacle(&opponent->polygon, x, y, opponent_size, robot_size);
}

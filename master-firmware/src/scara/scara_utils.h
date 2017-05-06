#ifndef SCARA_UTILS_H
#define SCARA_UTILS_H

// #include <platform.h>
#include <aversive/math/vect2/vect2.h>
#include <aversive/math/geometry/polygon.h>

#include "scara_waypoint.h"

point_t scara_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle);
point_t scara_coordinate_arm2robot(point_t arm_point, vect2_cart offset_xy, float offset_angle);

point_t scara_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad);
point_t scara_coordinate_robot2table(point_t robot_point, point_t robot_pos, float robot_a_rad);

float scara_heading_robot2arm(float robot_angle, float offset_angle);
float scara_heading_arm2robot(float arm_angle, float offset_angle);

float scara_heading_table2robot(float table_angle, float robot_a_rad);
float scara_heading_robot2table(float robot_angle, float robot_a_rad);

#endif /* SCARA_UTILS_H */

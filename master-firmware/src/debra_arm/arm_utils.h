#ifndef _ARM_UTILS_H_
#define _ARM_UTILS_H_

// #include <platform.h>
#include <aversive/math/vect2/vect2.h>
#include <aversive/math/geometry/polygon.h>

#include "keyframe.h"

point_t arm_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle);

point_t arm_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad);

#endif

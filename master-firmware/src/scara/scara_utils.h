#ifndef SCARA_UTILS_H
#define SCARA_UTILS_H

// #include <platform.h>
#include <aversive/math/vect2/vect2.h>
#include <aversive/math/geometry/polygon.h>

#include "scara_waypoint.h"

/* 2D pose */
typedef struct _pose2d_t {
    point_t translation;
    float heading;
} pose2d_t;


point_t scara_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle);
pose2d_t scara_pose_robot2arm(pose2d_t target, vect2_cart offset_xy, float offset_angle);

point_t scara_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad);
pose2d_t scara_pose_table2robot(pose2d_t target, point_t robot_pos, float robot_a_rad);


#endif /* SCARA_UTILS_H */

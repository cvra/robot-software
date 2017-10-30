#include "scara_utils.h"
#include "lie_groups.h"


point_t scara_coordinate_robot2arm(point_t robot_point, vect2_cart offset_xy, float offset_angle)
{
    point_t arm_point = {
        .x = robot_point.x - offset_xy.x,
        .y = robot_point.y - offset_xy.y
    };

    arm_point = so2_rotate(so2_create(- offset_angle), arm_point);

    return arm_point;
}

point_t scara_coordinate_arm2robot(point_t arm_point, vect2_cart offset_xy, float offset_angle)
{
    arm_point = so2_rotate(so2_create(offset_angle), arm_point);

    point_t robot_point = {
        .x = arm_point.x + offset_xy.x,
        .y = arm_point.y + offset_xy.y
    };

    return robot_point;
}

point_t scara_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad)
{
    point_t robot_point = {
        .x = target_point.x - robot_pos.x,
        .y = target_point.y - robot_pos.y
    };

    robot_point = so2_rotate(so2_create(- robot_a_rad), robot_point);

    return robot_point;
}

point_t scara_coordinate_robot2table(point_t robot_point, point_t robot_pos, float robot_a_rad)
{
    robot_point = so2_rotate(so2_create(robot_a_rad), robot_point);

    point_t table_point = {
        .x = robot_point.x + robot_pos.x,
        .y = robot_point.y + robot_pos.y
    };

    return table_point;
}

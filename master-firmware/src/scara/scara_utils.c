#include "scara_utils.h"
#include "math/lie_groups.h"

point_t scara_coordinate_robot2arm(point_t robot_point, vect2_cart offset_xy, float offset_angle)
{
    se2_t arm2robot = se2_create(offset_angle, translation_2d(offset_xy.x, offset_xy.y));
    return se2_inverse_transform(arm2robot, robot_point);
}

point_t scara_coordinate_arm2robot(point_t arm_point, vect2_cart offset_xy, float offset_angle)
{
    se2_t arm2robot = se2_create(offset_angle, translation_2d(offset_xy.x, offset_xy.y));
    return se2_transform(arm2robot, arm_point);
}

point_t scara_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad)
{
    se2_t robot2table = se2_create(robot_a_rad, translation_2d(robot_pos.x, robot_pos.y));
    return se2_inverse_transform(robot2table, target_point);
}

point_t scara_coordinate_robot2table(point_t robot_point, point_t robot_pos, float robot_a_rad)
{
    se2_t robot2table = se2_create(robot_a_rad, translation_2d(robot_pos.x, robot_pos.y));
    return se2_transform(robot2table, robot_point);
}

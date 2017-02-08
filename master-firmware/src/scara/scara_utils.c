#include "scara_utils.h"


point_t scara_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle)
{
    vect2_cart target;
    vect2_pol target_pol;
    target.x = target_point.x;
    target.y = target_point.y;

    vect2_sub_cart(&target, &offset_xy, &target);
    vect2_cart2pol(&target, &target_pol);

    target_pol.theta -= offset_angle;

    vect2_pol2cart(&target_pol, &target);

    target_point.x = target.x;
    target_point.y = target.y;

    return target_point;
}

point_t scara_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad)
{
    vect2_cart target;
    vect2_pol target_pol;

    target.x = target_point.x - robot_pos.x;
    target.y = target_point.y - robot_pos.y;

    vect2_cart2pol(&target, &target_pol);
    /* XXX Not sure if it is -= or += here. */
    target_pol.theta -= robot_a_rad;
    vect2_pol2cart(&target_pol, &target);

    target_point.x = target.x;
    target_point.y = target.y;

    return target_point;
}

pose2d_t scara_pose_robot2arm(pose2d_t target, vect2_cart offset_xy, float offset_angle)
{
    pose2d_t result;

    result.translation = scara_coordinate_robot2arm(target.translation, offset_xy, offset_angle);
    result.heading = target.heading - offset_angle;

    return result;
}

pose2d_t scara_pose_table2robot(pose2d_t target, point_t robot_pos, float robot_a_rad)
{
    pose2d_t result;

    result.translation = scara_coordinate_table2robot(target.translation, robot_pos, robot_a_rad);
    result.heading = target.heading - robot_a_rad;

    return result;
}

#include "arm_utils.h"


point_t arm_coordinate_robot2arm(point_t target_point, vect2_cart offset_xy, float offset_angle)
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

point_t arm_coordinate_table2robot(point_t target_point, point_t robot_pos, float robot_a_rad)
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

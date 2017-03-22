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

point_t scara_coordinate_arm2robot(point_t arm_point, vect2_cart offset_xy, float offset_angle)
{
    point_t robot_point;
    vect2_cart robot_cart;
    vect2_pol robot_pol;

    robot_cart.x = arm_point.x;
    robot_cart.y = arm_point.y;

    vect2_cart2pol(&robot_cart, &robot_pol);
    robot_pol.theta += offset_angle;
    vect2_pol2cart(&robot_pol, &robot_cart);

    vect2_add_cart(&robot_cart, &offset_xy, &robot_cart);

    robot_point.x = robot_cart.x;
    robot_point.y = robot_cart.y;

    return robot_point;
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

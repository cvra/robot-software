#include "hand_utils.h"

float hand_heading_arm2hand(float arm_angle, float arm_offset)
{
    return arm_angle - arm_offset;
}

float hand_heading_hand2arm(float hand_angle, float arm_offset)
{
    return hand_angle + arm_offset;
}

float hand_heading_robot2arm(float robot_angle, float offset_angle)
{
    return robot_angle - offset_angle;
}

float hand_heading_arm2robot(float arm_angle, float offset_angle)
{
    return arm_angle + offset_angle;
}

float hand_heading_table2robot(float table_angle, float robot_a_rad)
{
    return table_angle - robot_a_rad;
}

float hand_heading_robot2table(float robot_angle, float robot_a_rad)
{
    return robot_angle + robot_a_rad;
}

#include <string.h>

#include "hand.h"


void hand_init(hand_t* hand)
{
    memset(hand, 0, sizeof(hand_t));

    for (size_t i = 0; i < 4; i++) {
        hand->fingers_open[i] = FINGER_RETRACTED;
    }

    hand->last_waypoint.heading = 0.;
    hand->last_waypoint.coordinate_system = HAND_COORDINATE_HAND;
}

void hand_set_wrist_callbacks(hand_t* hand, void (*set_wrist_position)(void*, float), float (*get_wrist_position)(void*), void* wrist_args)
{
    hand->set_wrist_position = set_wrist_position;
    hand->get_wrist_position = get_wrist_position;
    hand->wrist_args = wrist_args;
}

void hand_set_fingers_callbacks(hand_t* hand, void (*set_fingers)(finger_state_t*))
{
    hand->set_fingers = set_fingers;
}

void hand_goto(hand_t* hand, float heading, hand_coordinate_t system)
{
    /* Set new waypoint */
    hand->last_waypoint.heading = heading;
    hand->last_waypoint.coordinate_system = system;
}

void hand_manage(hand_t* hand)
{
    /* Convert waypoint heading to hand coordinate system */
    float heading = hand_convert_waypoint_coordinate(hand, hand->last_waypoint);

    /* Send new control reference to wrist motor */
    hand->set_wrist_position(hand->wrist_args, heading);
}

void hand_set_finger(hand_t* hand, int index, finger_state_t state)
{
    hand->fingers_open[index % 4] = state;
    hand->set_fingers(hand->fingers_open);
}

void hand_set_related_robot_pos(hand_t *hand, struct robot_position *pos)
{
    hand->robot_pos = pos;
}

void hand_set_related_arm(hand_t *hand, scara_t *arm)
{
    hand->arm = arm;
}


float hand_convert_waypoint_coordinate(hand_t *hand, hand_waypoint_t waypoint)
{
    float heading = waypoint.heading;

    if (waypoint.coordinate_system == HAND_COORDINATE_TABLE) {
        heading = hand_heading_table2robot(heading, position_get_a_rad_float(hand->robot_pos));
        heading = hand_heading_robot2arm(heading, hand->arm->offset_rotation);
        heading = hand_heading_arm2hand(heading, hand->arm->shoulder_pos + hand->arm->elbow_pos);
    } else if (waypoint.coordinate_system == HAND_COORDINATE_ROBOT) {
        heading = hand_heading_robot2arm(heading, hand->arm->offset_rotation);
        heading = hand_heading_arm2hand(heading, hand->arm->shoulder_pos + hand->arm->elbow_pos);
    } else if (waypoint.coordinate_system == HAND_COORDINATE_ARM) {
        heading = hand_heading_arm2hand(heading, hand->arm->shoulder_pos + hand->arm->elbow_pos);
    }

    return heading;
}

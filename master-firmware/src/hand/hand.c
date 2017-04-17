#include <string.h>

#include "hand.h"


void hand_init(hand_t* hand)
{
    memset(hand, 0, sizeof(hand_t));

    for (size_t i = 0; i < 4; i++) {
        hand->fingers_open[i] = FINGER_RETRACTED;
    }
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
    float wrist_angle = heading;

    if (system == HAND_COORDINATE_TABLE) {
        wrist_angle = hand_heading_table2robot(wrist_angle, position_get_a_rad_float(hand->robot_pos));
        wrist_angle = hand_heading_robot2arm(wrist_angle, hand->arm->offset_rotation);
        wrist_angle = hand_heading_arm2hand(wrist_angle, hand->arm->shoulder_pos + hand->arm->elbow_pos);
    } else if (system == HAND_COORDINATE_ROBOT) {
        wrist_angle = hand_heading_robot2arm(wrist_angle, hand->arm->offset_rotation);
        wrist_angle = hand_heading_arm2hand(wrist_angle, hand->arm->shoulder_pos + hand->arm->elbow_pos);
    } else if (system == HAND_COORDINATE_ARM) {
        wrist_angle = hand_heading_arm2hand(wrist_angle, hand->arm->shoulder_pos + hand->arm->elbow_pos);
    }

    hand->set_wrist_position(hand->wrist_args, wrist_angle);
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

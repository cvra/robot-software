#include "CppUTest/TestHarness.h"
#include <CppUTestExt/MockSupport.h>

#include "hand/hand.h"


void set_hand_motor_pos(void *m, float value)
{
    *(float *)m = value;
}

float get_hand_motor_pos(void *m)
{
    return *(float *)m;
}


TEST_GROUP(HandTestGroup)
{
    hand_t hand;
    scara_t arm;
    struct robot_position robot_pos;
    float wrist_angle;

    void setup()
    {
        hand_init(&hand);

        wrist_angle = 0.;
        hand_set_wrist_callbacks(&hand, set_hand_motor_pos, get_hand_motor_pos, &wrist_angle);

        arm.shoulder_pos = M_PI / 6.;
        arm.elbow_pos = M_PI / 6.;
        hand_set_related_arm(&hand, &arm);

        arm.offset_rotation = -M_PI / 2.;

        robot_pos.pos_d.a = M_PI / 12.;
        hand_set_related_robot_pos(&hand, &robot_pos);
    }

    void teardown()
    {
    }
};

TEST(HandTestGroup, HandChangesConsign)
{
    hand_goto(&hand, 1., HAND_COORDINATE_HAND);
    hand_manage(&hand);

    DOUBLES_EQUAL(1., wrist_angle, 1e-2);
}

TEST(HandTestGroup, HandSetHeadingInArmCoordinateSystem)
{
    hand_goto(&hand, M_PI / 2., HAND_COORDINATE_ARM);
    hand_manage(&hand);

    DOUBLES_EQUAL(M_PI / 6., wrist_angle, 1e-2);
}

TEST(HandTestGroup, HandSetHeadingInRobotCoordinateSystem)
{
    hand_goto(&hand, 0, HAND_COORDINATE_ROBOT);
    hand_manage(&hand);

    DOUBLES_EQUAL(M_PI / 6., wrist_angle, 1e-2);
}

TEST(HandTestGroup, HandSetHeadingInTableCoordinateSystem)
{
    hand_goto(&hand, M_PI / 6., HAND_COORDINATE_TABLE);
    hand_manage(&hand);

    DOUBLES_EQUAL(3 * M_PI / 12., wrist_angle, 1e-2);
}

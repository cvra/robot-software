#include <CppUTest/TestHarness.h>

extern "C" {
#include <base/base_helpers.h>
}

TEST_GROUP (ABasePositionGetter) {
    const int ARBITRARY_TRACK_LENGTH_MM = 100;
    const int ARBITRARY_ENCODER_TICKS_PER_MM = 10000;
    struct robot_position robot_pos;

    void setup()
    {
        position_init(&robot_pos);
        position_set_physical_params(&robot_pos, ARBITRARY_TRACK_LENGTH_MM, ARBITRARY_ENCODER_TICKS_PER_MM);
    }

    void teardown()
    {
    }
};

TEST(ABasePositionGetter, returnsPoseOfRobot)
{
    position_set(&robot_pos, 10, 10, 180);

    se2_t pose = base_get_robot_pose(&robot_pos);

    DOUBLES_EQUAL(pose.translation.x, 10, 0.1);
    DOUBLES_EQUAL(pose.translation.y, 10, 0.1);
    DOUBLES_EQUAL(pose.rotation.angle, 3.14, 0.1);
}

#include <CppUTest/TestHarness.h>

#include "ch.h"
#include "robot_helpers/trajectory_helpers.h"


TEST_GROUP(TrajectorSetAligningMode)
{
    const int ARBITRARY_TRACK_LENGTH_MM = 100;
    const int ARBITRARY_ENCODER_TICKS_PER_MM = 10000;
    const int ARBITRARY_FREQUENCY = 10;

    enum board_mode_t robot_mode;
    struct robot_position robot_pos;
    struct trajectory robot_traj;
    struct blocking_detection distance_blocking;
    struct blocking_detection angle_blocking;

    void setup(void)
    {
        position_set_physical_params(&robot_pos, ARBITRARY_TRACK_LENGTH_MM, ARBITRARY_ENCODER_TICKS_PER_MM);
        trajectory_manager_init(&robot_traj, ARBITRARY_FREQUENCY);
        trajectory_set_robot_params(&robot_traj, NULL, &robot_pos);

        trajectory_set_mode_aligning(&robot_mode, &robot_traj, &distance_blocking, &angle_blocking);
    }
};

TEST(TrajectorSetAligningMode, ConfiguresSpeed)
{
    CHECK_TRUE(robot_traj.a_speed > 0);
    CHECK_TRUE(robot_traj.d_speed > 0);
}

TEST(TrajectorSetAligningMode, ConfiguresAcceleration)
{
    CHECK_TRUE(robot_traj.a_acc > 0);
    CHECK_TRUE(robot_traj.d_acc > 0);
}

TEST(TrajectorSetAligningMode, ConfiguresDistanceBlockingManager)
{
    CHECK_TRUE(distance_blocking.cpt_thres > 0);
    CHECK_TRUE(distance_blocking.err_thres > 0);
}

TEST(TrajectorSetAligningMode, DoesntChangeAngleBlockingManager)
{
    CHECK_TRUE(angle_blocking.cpt_thres == 0);
    CHECK_TRUE(angle_blocking.err_thres == 0);
}

TEST(TrajectorSetAligningMode, DisablesAngleControl)
{
    CHECK_TRUE(robot_mode == BOARD_MODE_DISTANCE_ONLY);
}

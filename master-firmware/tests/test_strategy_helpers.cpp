#include <CppUTest/TestHarness.h>

extern "C" {
#include "scara/scara_kinematics.h"
#include "scara/scara_trajectories.h"
#include "scara/scara_port.h"
#include "scara/scara.h"
}
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/strategy_helpers.h"

extern void scara_time_set(int32_t time);

TEST_GROUP(ArmSetTrajectory)
{
    float alpha, beta;
    point_t p1, p2;
    int status;

    scara_trajectory_t traj;
    scara_t arm;
    float arbitraryLengths[3] = {100, 50, 20};

    struct robot_position pos;

    arm_waypoint_t trajectory[3] = {
        {.x=0, .y=0, .z=0, .a=0, .p=-90, .coord=COORDINATE_TABLE, .dt=1000, .l3=20},
        {.x=100, .y=0, .z=0, .a=90, .p=0, .coord=COORDINATE_TABLE, .dt=1000, .l3=20},
        {.x=100, .y=100, .z=0, .a=180, .p=90, .coord=COORDINATE_TABLE, .dt=1000, .l3=20},
    };

    void setup()
    {
        scara_init(&arm);
        scara_set_physical_parameters(&arm, arbitraryLengths[0], arbitraryLengths[1], arbitraryLengths[2]);
        arm.offset_rotation = M_PI / 2;

        scara_set_related_robot_pos(&arm, &pos);

    }

    void teardown()
    {
        scara_time_set(0);
        scara_trajectory_delete(&traj);
        scara_trajectory_delete(&arm.trajectory);
    }
};

TEST(ArmSetTrajectory, ComputesDurationOfTrajectory)
{
    unsigned duration = strategy_set_arm_trajectory(&arm, &trajectory[0], sizeof(trajectory) / sizeof(arm_waypoint_t));

    CHECK_EQUAL(3000, duration);
}

TEST(ArmSetTrajectory, SetsGivenPointInTrajectory)
{
    strategy_set_arm_trajectory(&arm, &trajectory[0], sizeof(trajectory) / sizeof(arm_waypoint_t));

    CHECK_EQUAL(100, arm.trajectory.frames[1].position[0]);
    CHECK_EQUAL(0, arm.trajectory.frames[1].position[1]);
    DOUBLES_EQUAL(M_PI/2, arm.trajectory.frames[1].hand_angle, 1e-2);

    CHECK_EQUAL(100, arm.trajectory.frames[2].position[0]);
    CHECK_EQUAL(100, arm.trajectory.frames[2].position[1]);
    DOUBLES_EQUAL(M_PI, arm.trajectory.frames[2].hand_angle, 1e-2);
}

TEST(ArmSetTrajectory, SetsTimeCorrectly)
{
    strategy_set_arm_trajectory(&arm, &trajectory[0], sizeof(trajectory) / sizeof(arm_waypoint_t));

    CHECK_EQUAL(0, arm.trajectory.frames[0].date);
    CHECK_EQUAL(1000000, arm.trajectory.frames[1].date);
    CHECK_EQUAL(2000000, arm.trajectory.frames[2].date);
}

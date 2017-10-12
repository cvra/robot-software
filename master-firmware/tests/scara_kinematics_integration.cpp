#include "CppUTest/TestHarness.h"
#include <cmath>

extern "C" {
#include "scara/scara_kinematics.h"
#include "scara/scara_trajectories.h"
#include "scara/scara_port.h"
#include "scara/scara.h"
}

extern void scara_time_set(int32_t time);


#define RAD(x) ((x/180.)*M_PI)

TEST_GROUP(kinematicsTestGroup)
{
    float alpha, beta;
    point_t p1, p2;
    int status;

    scara_trajectory_t traj;
    scara_t arm;
    float arbitraryLengths[3] = {100, 50, 20};

    void setup()
    {
        scara_init(&arm);
        scara_set_physical_parameters(&arm, arbitraryLengths[0], arbitraryLengths[1], arbitraryLengths[2]);
        arm.offset_rotation = M_PI / 2;
        scara_trajectory_init(&traj);
    }

    void teardown()
    {
        scara_time_set(0);
        scara_trajectory_delete(&traj);
        scara_trajectory_delete(&arm.trajectory);
    }
};

TEST(kinematicsTestGroup, FindsASolution)
{
    point_t target = {100., 100.};
    status = scara_num_possible_elbow_positions(target, 100., 100., &p1, &p2);
    CHECK_EQUAL(2, status);
}

TEST(kinematicsTestGroup, FailsWhenTooFar)
{
    point_t target = {100., 100.};
    status = scara_num_possible_elbow_positions(target, 10., 10., &p1, &p2);
    CHECK_EQUAL(0, status);
}

TEST(kinematicsTestGroup, ForwardKinematicsTrivialCase)
{
    point_t result;
    float length[] = {100., 100.};
    result = scara_forward_kinematics(0., 0., length);
    DOUBLES_EQUAL(result.x, 200, 1e-2);
    DOUBLES_EQUAL(result.y, 0, 1e-2);
}

TEST(kinematicsTestGroup, ForwardKinematicsTrivialCaseBis)
{
    point_t result;
    float length[] = {100., 100.};
    result = scara_forward_kinematics(M_PI/2, 0., length);

    DOUBLES_EQUAL(result.x, 0, 1e-2);
    DOUBLES_EQUAL(result.y, 200, 1e-2);
}

TEST(kinematicsTestGroup, ForwardKinematicsNegativeAnglesToo)
{
    point_t result;
    float length[] = {100., 100.};
    result = scara_forward_kinematics(-M_PI/2, 0., length);

    DOUBLES_EQUAL(result.x, 0, 1e-2);
    DOUBLES_EQUAL(result.y, -200, 1e-2);
}

TEST(kinematicsTestGroup, DoesNotOscillateAroundZero)
{
    scara_waypoint_t frame;

    point_t target;
    int position_count;

    scara_trajectory_append_point(&traj, 100,  1, 10, COORDINATE_ARM, 1., arbitraryLengths);
    scara_trajectory_append_point(&traj, 100, -1, 10, COORDINATE_ARM, 10., arbitraryLengths);
    scara_do_trajectory(&arm, &traj);

    while (scara_time_get() < traj.frames[traj.frame_count-1].date) {
        frame = scara_position_for_date(&arm, scara_time_get());

        target.x = frame.position[0];
        target.y = frame.position[1];

        position_count = scara_num_possible_elbow_positions(target,
                                 frame.length[0], frame.length[1],
                                 &p1, &p2);

        CHECK_EQUAL(2, position_count);
        scara_time_set(scara_time_get() + 2*1000);
    }
}

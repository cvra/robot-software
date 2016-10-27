#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

extern "C" {
#include "trajectory_manager/trajectory_manager.h"
#include "trajectory_manager/trajectory_manager_core.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "control_system_manager/control_system_manager.h"
#include "quadramp/quadramp.h"
#include "position_manager/position_manager.h"
}

TEST_GROUP(TrajectoryManagerTestGroup)
{
    struct trajectory traj;
    struct cs distance_cs, angle_cs;
    struct quadramp_filter distance_qr, angle_qr;
    struct robot_position pos;
    struct robot_system rs;

    const int arbitrary_max_speed = 10;

    void setup()
    {
        quadramp_init(&angle_qr);
        quadramp_init(&distance_qr);

        cs_init(&distance_cs);
        cs_init(&angle_cs);
        cs_set_consign_filter(&distance_cs, quadramp_do_filter, &distance_qr);
        cs_set_consign_filter(&angle_cs, quadramp_do_filter, &angle_qr);

        rs_init(&rs);
        position_init(&pos);
        position_set(&pos, 0, 0, 0);

        trajectory_manager_init(&traj, 20);
        trajectory_set_cs(&traj, &distance_cs, &angle_cs);
        trajectory_set_robot_params(&traj, &rs, &pos);

        auto angle_start_deg = 10;
        auto angle_window_deg = 1;
        auto distance_window_mm = 10;

        trajectory_set_windows(&traj, distance_window_mm,
                               angle_window_deg, angle_start_deg);

        trajectory_set_acc(&traj, 10, 10);
        trajectory_set_speed(&traj, arbitrary_max_speed, arbitrary_max_speed);

        // Finally go to a point
        trajectory_goto_forward_xy_abs(&traj, 200, 200);
    }
};

TEST(TrajectoryManagerTestGroup, SchedulesTrajectoryEventFirst)
{
    // Checks that the trajectory manager is running, and turning to align
    // itself with the target point.
    CHECK_TRUE(traj.scheduled);
    CHECK_EQUAL(RUNNING_XY_F_START, traj.state);
}

TEST(TrajectoryManagerTestGroup, ChangesToInPlaceRotation)
{
    // Checks that the robot starts rotating in place
    trajectory_manager_xy_event(&traj);

    CHECK_EQUAL(RUNNING_XY_F_ANGLE, traj.state);

    // Check that we are rotating in place
    CHECK_EQUAL(0, get_quadramp_distance_speed(&traj));
    CHECK_TRUE(get_quadramp_angle_speed(&traj) > 0);
}

TEST(TrajectoryManagerTestGroup, ChangesToDriving)
{
    /* The robot turned enough, now check that we are moving. */
    position_set(&pos, 0, 0, 45);

    trajectory_manager_xy_event(&traj);
    trajectory_manager_xy_event(&traj);
    trajectory_manager_xy_event(&traj);

    CHECK_EQUAL(RUNNING_XY_F_ANGLE_OK, traj.state);

    // Since we are aligned to the target, we expect to go full speed
    DOUBLES_EQUAL(arbitrary_max_speed, get_quadramp_angle_speed(&traj), 0.01);
    DOUBLES_EQUAL(arbitrary_max_speed, get_quadramp_distance_speed(&traj), 0.01);
}

TEST(TrajectoryManagerTestGroup, RemovesEventWhenInWindow)
{
    trajectory_manager_xy_event(&traj);
    trajectory_manager_xy_event(&traj);

    // Moves the robot in the destination window, which means it should not
    // update the trajectory
    position_set(&pos, 199, 199, 45);
    trajectory_manager_xy_event(&traj);

    CHECK_FALSE(traj.scheduled);
}

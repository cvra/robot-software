#include <CppUTest/TestHarness.h>

extern "C" {
#include "trajectory_manager/trajectory_manager.h"
#include "trajectory_manager/trajectory_manager_core.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "control_system_manager/control_system_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "quadramp/quadramp.h"
#include "position_manager/position_manager.h"
}

#include "robot_helpers/trajectory_helpers.h"


TEST_GROUP(TrajectorySetAligningMode)
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

TEST(TrajectorySetAligningMode, ConfiguresSpeed)
{
    CHECK_TRUE(robot_traj.a_speed > 0);
    CHECK_TRUE(robot_traj.d_speed > 0);
}

TEST(TrajectorySetAligningMode, ConfiguresAcceleration)
{
    CHECK_TRUE(robot_traj.a_acc > 0);
    CHECK_TRUE(robot_traj.d_acc > 0);
}

TEST(TrajectorySetAligningMode, ConfiguresDistanceBlockingManager)
{
    CHECK_TRUE(distance_blocking.cpt_thres > 0);
    CHECK_TRUE(distance_blocking.err_thres > 0);
}

TEST(TrajectorySetAligningMode, DoesntChangeAngleBlockingManager)
{
    CHECK_TRUE(angle_blocking.cpt_thres == 0);
    CHECK_TRUE(angle_blocking.err_thres == 0);
}

TEST(TrajectorySetAligningMode, DisablesAngleControl)
{
    CHECK_TRUE(robot_mode == BOARD_MODE_DISTANCE_ONLY);
}


TEST_GROUP(TrajectorySetGameMode)
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

        trajectory_set_mode_game(&robot_mode, &robot_traj, &distance_blocking, &angle_blocking);
    }
};

TEST(TrajectorySetGameMode, ConfiguresSpeed)
{
    CHECK_TRUE(robot_traj.a_speed > 0);
    CHECK_TRUE(robot_traj.d_speed > 0);
}

TEST(TrajectorySetGameMode, ConfiguresAcceleration)
{
    CHECK_TRUE(robot_traj.a_acc > 0);
    CHECK_TRUE(robot_traj.d_acc > 0);
}

TEST(TrajectorySetGameMode, ConfiguresDistanceBlockingManager)
{
    CHECK_TRUE(distance_blocking.cpt_thres > 0);
    CHECK_TRUE(distance_blocking.err_thres > 0);
}

TEST(TrajectorySetGameMode, ConfiguresAngleBlockingManager)
{
    CHECK_TRUE(angle_blocking.cpt_thres > 0);
    CHECK_TRUE(angle_blocking.err_thres > 0);
}

TEST(TrajectorySetGameMode, EnablesSimultaneousAngleAndDistanceControl)
{
    CHECK_TRUE(robot_mode == BOARD_MODE_ANGLE_DISTANCE);
}


TEST_GROUP(TrajectoryHasEnded)
{
    struct _robot robot;

    const int arbitrary_max_speed = 10;
    const int arbitrary_max_acc = 10;
    const int arbitrary_goal_x = 200;
    const int arbitrary_goal_y = 200;
    const int arbitrary_end_angle = 45;

    void setup()
    {
        pid_init(&robot.distance_pid.pid);
        pid_init(&robot.angle_pid.pid);

        quadramp_init(&robot.distance_qr);
        quadramp_init(&robot.angle_qr);

        cs_init(&robot.distance_cs);
        cs_init(&robot.angle_cs);
        cs_set_consign_filter(&robot.distance_cs, quadramp_do_filter, &robot.distance_qr);
        cs_set_consign_filter(&robot.angle_cs, quadramp_do_filter, &robot.angle_qr);

        bd_init(&robot.distance_bd, &robot.distance_cs);
        bd_init(&robot.angle_bd, &robot.angle_cs);
        bd_set_thresholds(&robot.distance_bd, 1, 1);
        bd_set_thresholds(&robot.angle_bd, 1, 1);

        rs_init(&robot.rs);
        position_init(&robot.pos);
        position_set(&robot.pos, 0, 0, 0);

        int arbitrary_track = 100;
        int arbitrary_impulses_per_mm = 1;
        position_set_physical_params(&robot.pos, arbitrary_track, arbitrary_impulses_per_mm);

        trajectory_manager_init(&robot.traj, 10);
        trajectory_set_cs(&robot.traj, &robot.distance_cs, &robot.angle_cs);
        trajectory_set_robot_params(&robot.traj, &robot.rs, &robot.pos);

        auto angle_start_deg = 10;
        auto angle_window_deg = 1;
        auto distance_window_mm = 10;

        trajectory_set_windows(&robot.traj, distance_window_mm,
                               angle_window_deg, angle_start_deg);

        trajectory_set_acc(&robot.traj, 10, 1);
        trajectory_set_speed(&robot.traj, 10, 1);

        // Finally go to a point
        trajectory_goto_forward_xy_abs(&robot.traj, arbitrary_goal_x, arbitrary_goal_y);
        robot_manage();
    }

    void robot_manage()
    {
        cs_manage(&robot.distance_cs);
        cs_manage(&robot.angle_cs);
        bd_manage(&robot.distance_bd);
        bd_manage(&robot.angle_bd);
        trajectory_manager_xy_event(&robot.traj);
    }
};

TEST(TrajectoryHasEnded, ReturnsZeroWhenTrajectoryHasNotEndedYet)
{
    int traj_end_reason = trajectory_has_ended(&robot, 0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsGoalIsReached)
{
    /* Reach goal (close enough) */
    position_set(&robot.pos, arbitrary_goal_x-1, arbitrary_goal_y-1, arbitrary_end_angle);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(&robot, TRAJ_END_GOAL_REACHED);

    CHECK_EQUAL(TRAJ_END_GOAL_REACHED, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfGoalReached)
{
    /* Reach goal */
    position_set(&robot.pos, arbitrary_goal_x, arbitrary_goal_y, arbitrary_end_angle);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(&robot, 0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsCollisionInDistance)
{
    /* Align angle */
    position_set(&robot.pos, 0, 0, 45);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(&robot, TRAJ_END_COLLISION);

    CHECK_EQUAL(TRAJ_END_COLLISION, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsCollisionInAngle)
{
    robot_manage();
    robot_manage();

    CHECK_EQUAL(RUNNING_XY_F_ANGLE, robot.traj.state);

    int traj_end_reason = trajectory_has_ended(&robot, TRAJ_END_COLLISION);

    CHECK_EQUAL(TRAJ_END_COLLISION, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfCollisionInDistance)
{
    /* Align angle */
    position_set(&robot.pos, 0, 0, 45);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(&robot, 0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfCollisionInAngle)
{
    robot_manage();
    robot_manage();

    CHECK_EQUAL(RUNNING_XY_F_ANGLE, robot.traj.state);

    int traj_end_reason = trajectory_has_ended(&robot, 0);

    CHECK_EQUAL(0, traj_end_reason);
}

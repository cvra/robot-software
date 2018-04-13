#include <CppUTest/TestHarness.h>

#include <msgbus/messagebus.h>
#include <timestamp/timestamp.h>
#include "../../lib/msgbus/examples/posix/port.h"

extern "C" {
#include "trajectory_manager/trajectory_manager.h"
#include "trajectory_manager/trajectory_manager_core.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "control_system_manager/control_system_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "quadramp/quadramp.h"
#include "position_manager/position_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
}

#include "uwb_position.h"
#include "base/map.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"

extern "C" {
    // TODO: Should this live here?
    struct _robot robot;
    messagebus_t bus;
}


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


TEST_GROUP(CurrentTrajectoryCheck)
{
    struct _robot robot;
    poly_t *opponent;

    const int arbitrary_max_speed = 10;
    const int arbitrary_max_acc = 10;
    const int arbitrary_goal_x = 200;
    const int arbitrary_goal_y = 200;
    const int arbitrary_end_angle = 45;

    const int arbitrary_robot_size = 100;

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

        rs_init(&robot.rs);
        position_init(&robot.pos);
        position_set(&robot.pos, 1, 1, 45);

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

        robot.robot_size = arbitrary_robot_size;
        robot.opponent_size = 0;

        // Opponent obstacle
        oa_init();
        opponent = oa_new_poly(4);

        // Finally go to a point
        trajectory_goto_forward_xy_abs(&robot.traj, arbitrary_goal_x, arbitrary_goal_y);
        robot_manage();
        robot_manage();
    }

    void robot_manage()
    {
        cs_manage(&robot.distance_cs);
        cs_manage(&robot.angle_cs);
        trajectory_manager_xy_event(&robot.traj);
    }

    void set_opponent_position(int x, int y)
    {
        map_set_rectangular_obstacle(opponent, x, y, robot.opponent_size, robot.opponent_size, robot.robot_size);
    }
};

TEST(CurrentTrajectoryCheck, CurrentPathCrossesWithObstacle)
{
    robot.opponent_size = 100;
    set_opponent_position(400, 400);

    point_t intersection;
    bool res = trajectory_crosses_obstacle(opponent, &intersection);

    CHECK_FALSE(res);
}

TEST(CurrentTrajectoryCheck, CurrentPathDoesntCrossWithObstacle)
{
    robot.opponent_size = 100;
    set_opponent_position(240, 250);

    point_t intersection;
    bool res = trajectory_crosses_obstacle(opponent, &intersection);

    CHECK_TRUE(res);
}

TEST(CurrentTrajectoryCheck, DetectsPathCrossingIfPathInsideObstacle)
{
    robot.opponent_size = 400;
    set_opponent_position(250, 250);

    point_t intersection;
    bool res = trajectory_crosses_obstacle(opponent, &intersection);

    CHECK_TRUE(res);
}

TEST(CurrentTrajectoryCheck, IsNotOnCollisionPathWithObstacle)
{
    robot.opponent_size = 100;
    bool res = trajectory_is_on_collision_path(400, 400);

    CHECK_FALSE(res);
}

TEST(CurrentTrajectoryCheck, IsOnCollisionPathWithObstacle)
{
    robot.opponent_size = 100;
    bool res = trajectory_is_on_collision_path(240, 250);

    CHECK_TRUE(res);
}

TEST(CurrentTrajectoryCheck, DetectsCollisionIfPathIsCompletelyInsideObstacle)
{
    robot.opponent_size = 400;
    bool res = trajectory_is_on_collision_path(250, 250);

    CHECK_TRUE(res);
}


TEST_GROUP(TrajectoryHasEnded)
{
    messagebus_topic_t proximity_beacon_topic;
    messagebus_topic_t ally_position_topic;

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

        bd_init(&robot.distance_bd);
        bd_init(&robot.angle_bd);
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

        robot.opponent_size = 200;
        robot.robot_size = 200;

        msgbus_setup();
        msgbus_advertise_beacon();
        msgbus_advertise_ally();

        // Finally go to a point
        trajectory_goto_forward_xy_abs(&robot.traj, arbitrary_goal_x, arbitrary_goal_y);
        robot_manage();
    }

    void robot_manage()
    {
        cs_manage(&robot.distance_cs);
        cs_manage(&robot.angle_cs);
        bd_manage(&robot.distance_bd, abs(cs_get_error(&robot.distance_cs)));
        bd_manage(&robot.angle_bd, abs(cs_get_error(&robot.angle_cs)));
        trajectory_manager_xy_event(&robot.traj);
    }

    void msgbus_setup()
    {
        condvar_wrapper_t bus_sync = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};
        messagebus_init(&bus, &bus_sync, &bus_sync);
    }

    void msgbus_advertise_beacon()
    {
        condvar_wrapper_t proximity_beacon_topic_wrapper = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};
        static float proximity_beacon_topic_buffer[3];
        messagebus_topic_init(&proximity_beacon_topic,
                              &proximity_beacon_topic_wrapper,
                              &proximity_beacon_topic_wrapper,
                              &proximity_beacon_topic_buffer,
                              sizeof(proximity_beacon_topic_buffer));
        messagebus_advertise_topic(&bus, &proximity_beacon_topic, "/proximity_beacon");
    }

    void msgbus_advertise_ally()
    {
        static allied_position_t pos;
        messagebus_topic_init(&ally_position_topic, nullptr, nullptr, &pos,
                              sizeof(pos));

        messagebus_advertise_topic(&bus, &ally_position_topic, "/allied_position");
    }

    void publish_allied_position(uint32_t timestamp, point_t pos)
    {
        allied_position_t msg = {pos, timestamp};
        messagebus_topic_publish(&ally_position_topic, &msg, sizeof(msg));
    }
};

uint32_t timestamp_now = 42424242;

timestamp_t timestamp_get(void)
{
    return (timestamp_t)timestamp_now;
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenTrajectoryHasNotEndedYet)
{
    int traj_end_reason = trajectory_has_ended(0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsGoalIsReached)
{
    /* Reach goal (close enough) */
    position_set(&robot.pos, arbitrary_goal_x-1, arbitrary_goal_y-1, arbitrary_end_angle);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(TRAJ_END_GOAL_REACHED);

    CHECK_EQUAL(TRAJ_END_GOAL_REACHED, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfGoalReached)
{
    /* Reach goal */
    position_set(&robot.pos, arbitrary_goal_x, arbitrary_goal_y, arbitrary_end_angle);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsCollisionInDistance)
{
    /* Align angle */
    position_set(&robot.pos, 0, 0, 45);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(TRAJ_END_COLLISION);

    CHECK_EQUAL(TRAJ_END_COLLISION, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsCollisionInAngle)
{
    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(TRAJ_END_COLLISION);

    CHECK_EQUAL(TRAJ_END_COLLISION, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfCollisionInDistance)
{
    /* Align angle */
    position_set(&robot.pos, 0, 0, 45);

    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfCollisionInAngle)
{
    robot_manage();
    robot_manage();

    int traj_end_reason = trajectory_has_ended(0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsFutureCollisionWhenGoalIsInsideOpponentPolygon)
{
    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.3, RADIANS(10)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(TRAJ_END_OPPONENT_NEAR, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsObstacleNearWhenCurrentPosIsInsideOpponentPolygon)
{
    /* Inflate opoonent size to be so big, it includes the current position of the robot */
    robot.opponent_size = 400;

    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.3, RADIANS(170)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(TRAJ_END_OPPONENT_NEAR, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsWhenOpponentTooClose)
{
    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.3, RADIANS(10)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(TRAJ_END_OPPONENT_NEAR, traj_end_reason);
}

TEST(TrajectoryHasEnded, IgnoresOpponentIfOutOfTheWayEvenIfTooClose)
{
    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.3, RADIANS(170)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DetectsWhenOpponentTooCloseAndInDirectionOfMotionWhenGoingBack)
{
    /* Rotate and force backwards motion */
    position_set(&robot.pos, 0, 0, -135);
    trajectory_goto_backward_xy_abs(&robot.traj, arbitrary_goal_x, arbitrary_goal_y);
    robot_manage();
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.3, RADIANS(170)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(TRAJ_END_OPPONENT_NEAR, traj_end_reason);
}

TEST(TrajectoryHasEnded, ReturnsZeroWhenNoReasonSpecifiedEvenIfOpponentGetsNear)
{
    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.3, RADIANS(170)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(0);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, IgnoresOpponentTooCloseWhenBeaconSignalTooOld)
{
    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    const float arbitrary_old_timestamp = 0;
    float data[3] = {arbitrary_old_timestamp, 0.3, RADIANS(10)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, IgnoresOpponentIfOutOfTheWayEvenIfTooBig)
{
    /* Inflate opoonent size to be so big, it includes the goal position */
    robot.opponent_size = 400;

    /* Start forward motion */
    position_set(&robot.pos, 0, 0, 45);
    robot_manage();
    robot_manage();

    /* Simulate opponent */
    float data[3] = {(float)timestamp_now, 0.7, RADIANS(10)};
    messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

    int traj_end_reason = trajectory_has_ended(TRAJ_END_OPPONENT_NEAR);

    CHECK_EQUAL(0, traj_end_reason);
}

TEST(TrajectoryHasEnded, DoesStopWhenAllyIsOnTheWay)
{
    publish_allied_position(timestamp_now, {100, 100});
    auto reason = trajectory_has_ended(TRAJ_END_ALLY_NEAR);
    CHECK_EQUAL(TRAJ_END_ALLY_NEAR, reason);
}

TEST(TrajectoryHasEnded, DoesNotStopWhenAllyIsNotOnPath)
{
    publish_allied_position(timestamp_now, {1000, 1000});
    auto reason = trajectory_has_ended(TRAJ_END_ALLY_NEAR);
    CHECK_EQUAL(0, reason);
}

TEST(TrajectoryHasEnded, DoesNotStopWhenAllyPositionIsTooOld)
{
    // Timestamp 20 seconds in the past
    const auto age = 20;
    const auto timestamp = timestamp_now - age * 1000000;
    publish_allied_position(timestamp, {100, 100});
    auto reason = trajectory_has_ended(TRAJ_END_ALLY_NEAR);
    CHECK_EQUAL(0, reason);
}

TEST(TrajectoryHasEnded, ByDefaultCheckForAllyPosition)
{
    CHECK_TRUE(TRAJ_FLAGS_ALL & TRAJ_END_ALLY_NEAR);
}

TEST(TrajectoryHasEnded, DetectsTimeIsUp)
{
    trajectory_game_timer_reset();
    timestamp_now += GAME_DURATION * 1000000;

    int traj_end_reason = trajectory_has_ended(TRAJ_FLAGS_ALL);

    CHECK_EQUAL(TRAJ_END_TIMER, traj_end_reason);
}


TEST_GROUP(ATimeWatcher)
{
    void setup(void)
    {
        trajectory_game_timer_reset();
    }
};

TEST(ATimeWatcher, DetectsGameIsStillGoing)
{
    bool time_is_up = trajectory_game_has_ended();

    CHECK_FALSE(time_is_up);
}

TEST(ATimeWatcher, DetectsTimeIsUp)
{
    timestamp_now += GAME_DURATION * 1000000;

    bool time_is_up = trajectory_game_has_ended();

    CHECK_TRUE(time_is_up);
}

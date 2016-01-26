#include <CppUTest/TestHarness.h>
#include "../src/setpoint.h"
#include "../src/setpoint.c"


#define FLOAT_TOLERANCE     1e-6


TEST_GROUP(Setpoint)
{
    struct setpoint_s setpoint;
    setpoint_interpolator_t interpolator;

    float acc_limit = 10;
    float vel_limit = 2;

    void setup(void)
    {
        setpoint_init(&interpolator);
        setpoint_set_acceleration_limit(&interpolator, acc_limit);
        setpoint_set_velocity_limit(&interpolator, vel_limit);
    }
};

TEST(Setpoint, Init)
{
    setpoint_init(&interpolator);
    CHECK_EQUAL(SETPT_MODE_TORQUE, interpolator.setpt_mode);
    DOUBLES_EQUAL(0, interpolator.setpt_torque, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0, interpolator.acc_limit, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(0, interpolator.vel_limit, FLOAT_TOLERANCE);
}

TEST(Setpoint, AccLimit)
{
    DOUBLES_EQUAL(acc_limit, interpolator.acc_limit, FLOAT_TOLERANCE);
}

TEST(Setpoint, VelLimit)
{
    DOUBLES_EQUAL(vel_limit, interpolator.vel_limit, FLOAT_TOLERANCE);
}

TEST(Setpoint, UpdatePositionAfterInit)
{
    float target_pos = 2;
    float current_pos = 1;
    float current_vel = 0.5;

    CHECK_EQUAL(SETPT_MODE_TORQUE, interpolator.setpt_mode);

    setpoint_update_position(&interpolator, target_pos, current_pos, current_vel, false);

    CHECK_EQUAL(SETPT_MODE_POS, interpolator.setpt_mode);
    DOUBLES_EQUAL(target_pos, interpolator.target_pos, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(current_pos, interpolator.setpt_pos, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(current_vel, interpolator.setpt_vel, FLOAT_TOLERANCE);
}

TEST(Setpoint, UpdateVelocityAfterInit)
{
    float target_vel = 1;
    float current_vel = 0.5;

    CHECK_EQUAL(SETPT_MODE_TORQUE, interpolator.setpt_mode);

    setpoint_update_velocity(&interpolator, target_vel, current_vel);

    CHECK_EQUAL(SETPT_MODE_VEL, interpolator.setpt_mode);
    DOUBLES_EQUAL(target_vel, interpolator.target_vel, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(current_vel, interpolator.setpt_vel, FLOAT_TOLERANCE);
}

TEST(Setpoint, UpdateTorque)
{
    float target_torque = 0.2;

    setpoint_update_torque(&interpolator, target_torque);
    CHECK_EQUAL(SETPT_MODE_TORQUE, interpolator.setpt_mode);
    DOUBLES_EQUAL(target_torque, interpolator.setpt_torque, FLOAT_TOLERANCE);
}

TEST(Setpoint, UpdateTrajectory)
{
    float pos = 2;
    float vel = 1;
    float acc = 0.1;
    float torque = 0.2;
    timestamp_t timestamp = 10000;

    setpoint_update_trajectory(&interpolator, pos, vel, acc, torque, timestamp);

    DOUBLES_EQUAL(pos, interpolator.setpt_pos, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(vel, interpolator.setpt_vel, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(acc, interpolator.traj_acc, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(torque, interpolator.setpt_torque, FLOAT_TOLERANCE);
    CHECK_EQUAL(timestamp, interpolator.setpt_ts);
}

TEST(Setpoint, UpdatePositionAfterVelCtrl)
{
    // TODO
}

TEST(Setpoint, UpdatePosition)
{
    // TODO
}

TEST_GROUP(SetpointInterpolation)
{

};

TEST(SetpointInterpolation, PositionConstant)
{
    float pos = 1;
    float vel = 0;
    float acc = 0;
    float delta_t = 0.1;

    DOUBLES_EQUAL(pos,
                  pos_setpt_interpolation(pos, vel, acc, delta_t),
                  FLOAT_TOLERANCE);
}

TEST(SetpointInterpolation, PositionConstantVel)
{
    float pos = 1;
    float vel = 2;
    float acc = 0;
    float delta_t = 0.1;

    DOUBLES_EQUAL(1.2, pos + vel * delta_t, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(pos + vel * delta_t,
                  pos_setpt_interpolation(pos, vel, acc, delta_t),
                  FLOAT_TOLERANCE);
}

TEST(SetpointInterpolation, PositionStart)
{
    float pos = 1;
    float vel = 0;
    float acc = 2;
    float delta_t = 0.1;

    DOUBLES_EQUAL(1.01, pos + acc * delta_t * delta_t / 2, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(pos + acc * delta_t * delta_t / 2,
                  pos_setpt_interpolation(pos, vel, acc, delta_t),
                  FLOAT_TOLERANCE);
}

TEST(SetpointInterpolation, Position)
{
    float pos = 1;
    float vel = 2;
    float acc = 2;
    float delta_t = 0.1;

    DOUBLES_EQUAL(1.21,
                  pos + vel * delta_t + acc * delta_t * delta_t / 2,
                  FLOAT_TOLERANCE);
    DOUBLES_EQUAL(pos + vel * delta_t + acc * delta_t * delta_t / 2,
                  pos_setpt_interpolation(pos, vel, acc, delta_t),
                  FLOAT_TOLERANCE);
}

TEST(SetpointInterpolation, Velocity)
{
    float vel = 1;
    float acc = 2;
    float delta_t = 0.1;


    DOUBLES_EQUAL(1.2, vel + acc * delta_t, FLOAT_TOLERANCE);
    DOUBLES_EQUAL(vel + acc * delta_t,
                  vel_setpt_interpolation(vel, acc, delta_t),
                  FLOAT_TOLERANCE);
}

TEST_GROUP(SetpointVelocityRamp)
{

};

TEST(SetpointVelocityRamp, Static)
{
    float pos = 1;
    float vel = 0;
    float target_pos = pos;
    float delta_t = 0.1;
    float max_vel = 3;
    float max_acc = 2;

    DOUBLES_EQUAL(0,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

TEST(SetpointVelocityRamp, Cruising)
{
    float pos = 1;
    float vel = 3;
    float target_pos = 4;
    float delta_t = 0.1;
    float max_vel = 3;
    float max_acc = 2;

    DOUBLES_EQUAL(0,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

TEST(SetpointVelocityRamp, Start)
{
    float pos = 1;
    float vel = 0;
    float target_pos = 2;
    float delta_t = 0.1;
    float max_vel = 3;
    float max_acc = 2;

    DOUBLES_EQUAL(max_acc,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

TEST(SetpointVelocityRamp, Stopping)
{
    float pos = 1.9;
    float vel = 2;
    float target_pos = 2;
    float delta_t = 0.1;
    float max_vel = 3;
    float max_acc = 2;

    DOUBLES_EQUAL(-max_acc,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

TEST(SetpointVelocityRamp, VeryClose)
{
    float pos = 0;
    float delta_t = 0.1;
    float max_acc = 2;
    float vel = max_acc * delta_t / 2;
    float target_pos = pos + max_acc * delta_t * delta_t / 2;
    float max_vel = 3;

    DOUBLES_EQUAL(- max_acc / 2,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

TEST(SetpointVelocityRamp, GoingTheWrongWay)
{
    float pos = 1;
    float vel = -2;
    float target_pos = 2;
    float delta_t = 0.1;
    float max_vel = 3;
    float max_acc = 2;

    DOUBLES_EQUAL(max_acc,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

TEST(SetpointVelocityRamp, Overshot)
{
    float target_pos = 1;
    float delta_t = 0.1;
    float max_acc = 2;
    float pos = target_pos + max_acc * delta_t * delta_t / 2 * 0.9;
    float vel = max_acc * delta_t / 2;
    float max_vel = 3;

    DOUBLES_EQUAL(- max_acc / 2,
                  vel_ramp(pos, vel, target_pos, delta_t, max_vel, max_acc, false),
                  FLOAT_TOLERANCE);
}

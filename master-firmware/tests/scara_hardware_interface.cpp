#include "CppUTest/TestHarness.h"

extern "C" {
#include "scara/scara_hardware_interface.h"
}

namespace {
    typedef struct {
        float pos;
        float vel;
    } joint_state_t;

    void set_pos(void *m, float value)
    {
        joint_state_t * state = (joint_state_t *)m;
        state->pos = value;
    }

    void set_vel(void *m, float value)
    {
        joint_state_t * state = (joint_state_t *)m;
        state->vel = value;
    }

    float get_pos(void *m)
    {
        joint_state_t * state = (joint_state_t *)m;
        return state->pos;
    }
}

TEST_GROUP(AScaraHWInterface)
{
    scara_hardware_interface_t hw_interface;
    joint_state_t z_state {0, 0};
    joint_state_t shoulder_state {0, 0};
    joint_state_t elbow_state {0, 0};

    void setup()
    {
        joint_set_callbacks(&(hw_interface.z_joint), set_pos, set_vel, get_pos, &z_state);
        joint_set_callbacks(&(hw_interface.shoulder_joint), set_pos, set_vel, get_pos, &shoulder_state);
        joint_set_callbacks(&(hw_interface.elbow_joint), set_pos, set_vel, get_pos, &elbow_state);
    }

    void teardown()
    {
    }

    void set_non_trivial_joint_states()
    {
        shoulder_state = {1, 2};
        elbow_state = {3, 4};
        z_state = {42, 73};
    }
};

TEST(AScaraHWInterface, ReadsJointPositions)
{
    set_non_trivial_joint_states();

    const auto joint_positions = scara_hw_read_joint_positions(&hw_interface);

    CHECK_EQUAL(shoulder_state.pos, joint_positions.shoulder);
    CHECK_EQUAL(elbow_state.pos, joint_positions.elbow);
    CHECK_EQUAL(z_state.pos, joint_positions.z);
}

TEST(AScaraHWInterface, StopsJointsWhenAskedToShutdown)
{
    set_non_trivial_joint_states();

    scara_hw_shutdown_joints(&hw_interface);

    CHECK_EQUAL(0, shoulder_state.vel);
    CHECK_EQUAL(0, elbow_state.vel);
    CHECK_EQUAL(0, z_state.vel);
}

TEST(AScaraHWInterface, SetsJointPosition)
{
    joint_set(&hw_interface.z_joint, {POSITION, 7});

    CHECK_EQUAL(7, z_state.pos);
    CHECK_EQUAL(0, z_state.vel);
}

TEST(AScaraHWInterface, SetsJointVelocity)
{
    joint_set(&hw_interface.z_joint, {VELOCITY, 7});

    CHECK_EQUAL(7, z_state.vel);
    CHECK_EQUAL(0, z_state.pos);
}

TEST(AScaraHWInterface, SetsAllRequestedJointStates)
{
    scara_hw_set_joints(&hw_interface, {.z={POSITION, 7}, .shoulder={VELOCITY, 5}, .elbow={VELOCITY, 3}});

    CHECK_EQUAL(7, z_state.pos);
    CHECK_EQUAL(5, shoulder_state.vel);
    CHECK_EQUAL(3, elbow_state.vel);
}

#include "CppUTest/TestHarness.h"

extern "C" {
#include "scara/scara_hardware_interface.h"
}

namespace {
    void set_motor_pos(void *m, float value)
    {
        *(float *)m = value;
    }

    void set_motor_vel(void *m, float value)
    {
        *(float *)m = value;
    }

    float get_motor_pos(void *m)
    {
        return *(float *)m;
    }
}

TEST_GROUP(AScaraHWInterface)
{
    scara_hardware_interface_t hw_interface;
    float z_pos {0};
    float shoulder_angle {0};
    float elbow_angle {0};

    void setup()
    {
        joint_set_callbacks(&(hw_interface.z_joint), set_motor_pos, set_motor_vel, get_motor_pos, &z_pos);
        joint_set_callbacks(&(hw_interface.shoulder_joint), set_motor_pos, set_motor_vel, get_motor_pos, &shoulder_angle);
        joint_set_callbacks(&(hw_interface.elbow_joint), set_motor_pos, set_motor_vel, get_motor_pos, &elbow_angle);
    }

    void teardown()
    {
    }

    void set_non_trivial_joint_states()
    {
        shoulder_angle = 2;
        elbow_angle = 3;
        z_pos = 42;
    }
};

TEST(AScaraHWInterface, ReadsJointPositions)
{
    set_non_trivial_joint_states();

    const auto joint_positions = scara_hw_read_joint_positions(&hw_interface);

    CHECK_EQUAL(shoulder_angle, joint_positions.shoulder);
    CHECK_EQUAL(elbow_angle, joint_positions.elbow);
    CHECK_EQUAL(z_pos, joint_positions.z);
}

TEST(AScaraHWInterface, StopsJointsWhenAskedToShutdown)
{
    set_non_trivial_joint_states();

    scara_hw_shutdown_joints(&hw_interface);

    CHECK_EQUAL(0, shoulder_angle);
    CHECK_EQUAL(0, elbow_angle);
    CHECK_EQUAL(0, z_pos);
}

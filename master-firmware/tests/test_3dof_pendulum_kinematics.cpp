#include "CppUTest/TestHarness.h"

#include "manipulator/kinematics.h"

#include <math.h>

using namespace manipulator;

namespace {
#define RAD(x) ((x / 180.) * M_PI)
void ANGLES_EQUAL(const Angles& lhs, const Angles rhs)
{
    DOUBLES_EQUAL(lhs[0], rhs[0], 1e-3);
    DOUBLES_EQUAL(lhs[1], rhs[1], 1e-3);
    DOUBLES_EQUAL(lhs[2], rhs[2], 1e-3);
}
} // namespace

TEST_GROUP (APendulumWith3Joints) {
    ArmLengths lengths{1.0f, 0.5, 0.2f};
};

TEST(APendulumWith3Joints, IsFlat)
{
    const Angles original_angles = {RAD(0.f), RAD(0.f), RAD(0.f)};

    const auto angles = inverse_kinematics(lengths, forward_kinematics(lengths, original_angles));

    ANGLES_EQUAL(original_angles, angles);
}

TEST(APendulumWith3Joints, IsFlatAt90Degrees)
{
    const Angles original_angles = {RAD(90.f), RAD(0.f), RAD(0.f)};

    const auto angles = inverse_kinematics(lengths, forward_kinematics(lengths, original_angles));

    ANGLES_EQUAL(original_angles, angles);
}

TEST(APendulumWith3Joints, IsPositivelyCurvedAtExpectedAngles)
{
    const Angles original_angles = {RAD(10.f), RAD(20.f), RAD(30.f)};

    const auto angles = inverse_kinematics(lengths, forward_kinematics(lengths, original_angles));

    ANGLES_EQUAL(original_angles, angles);
}

TEST(APendulumWith3Joints, IsNegativelyCurvedAtExpectedAngles)
{
    const Angles original_angles = {RAD(-10.f), RAD(-20.f), RAD(-30.f)};

    const auto angles = inverse_kinematics(lengths, forward_kinematics(lengths, original_angles));

    ANGLES_EQUAL(original_angles, angles);
}

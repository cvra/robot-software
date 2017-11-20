#include "CppUTest/TestHarness.h"
#include <math.h>

extern "C" {
#include "scara/scara_kinematics.h"
}

#define RAD(x) ((x/180.)*M_PI)

TEST_GROUP(GetAngleFromArmPosTestGroup)
{
};

TEST(GetAngleFromArmPosTestGroup, SimpleShoulderAngle)
{
    float angle;
    point_t elbow, hand;
    elbow.x = 100;
    elbow.y = 0;
    angle = scara_compute_shoulder_angle(elbow, hand);

    DOUBLES_EQUAL(RAD(0), angle, 1e-3);
}

TEST(GetAngleFromArmPosTestGroup, SimpleElbowAngle)
{
    float angle;
    point_t elbow = {10, 10};
    point_t hand =  {20, 20};

    angle = scara_compute_elbow_angle(elbow, hand);
    DOUBLES_EQUAL(RAD(45), angle, 1e-3);
}

TEST_GROUP(ChooseElbowPositionTestGroup)
{
};

TEST(ChooseElbowPositionTestGroup, ModeMirrorIdentity)
{
    shoulder_mode_t m = SHOULDER_FRONT;
    float angle = 1;
    CHECK_EQUAL(m, scara_orientation_mode(m, angle));
}

TEST(ChooseElbowPositionTestGroup, ModeMirrorInvert)
{
    shoulder_mode_t m = SHOULDER_FRONT;
    float angle = -1;
    CHECK_EQUAL(SHOULDER_BACK, scara_orientation_mode(m, angle));
}

TEST(ChooseElbowPositionTestGroup, ModeMirrorInvertBack)
{
    shoulder_mode_t m = SHOULDER_BACK;
    float angle = -1;
    CHECK_EQUAL(SHOULDER_FRONT, scara_orientation_mode(m, angle));
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowSolutionOutsideRobot)
{
    // First test case : target_x < 0 ('inside' the robot)
    point_t elbow1 = {10, 10};
    point_t elbow2 = {-10, 10};
    point_t target = {-10, 20};
    point_t chosen;

    chosen = scara_shoulder_solve(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow1.x, chosen.x);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowSolutionOutsideRobotBis)
{
    // First test case : target_x < 0 ('inside' the robot)
    point_t elbow1 = {-10, 10};
    point_t elbow2 = {10, 10};
    point_t target = {-10, 20};
    point_t chosen;

    chosen = scara_shoulder_solve(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow2.x, chosen.x);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowBackward)
{
    point_t elbow1 = {10, 10};
    point_t elbow2 = {10, -10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = scara_shoulder_solve(target, elbow1, elbow2, SHOULDER_BACK);
    CHECK_EQUAL(elbow1.y, chosen.y);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowBackwardBis)
{
    point_t elbow1 = {10, -10};
    point_t elbow2 = {10, 10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = scara_shoulder_solve(target, elbow1, elbow2, SHOULDER_BACK);
    CHECK_EQUAL(elbow2.y, chosen.y);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowForward)
{
    point_t elbow1 = {10, -10};
    point_t elbow2 = {10, 10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = scara_shoulder_solve(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow1.y, chosen.y);
}

TEST(ChooseElbowPositionTestGroup, ChooseElbowForwardBis)
{
    point_t elbow1 = {10, 10};
    point_t elbow2 = {10, -10};
    point_t target = {10, 0};
    point_t chosen;

    chosen = scara_shoulder_solve(target, elbow1, elbow2, SHOULDER_FRONT);
    CHECK_EQUAL(elbow2.y, chosen.y);
}


TEST_GROUP(AScaraJointAnglesComputer)
{
    float alpha, beta;
    shoulder_mode_t mode = SHOULDER_BACK;
    float length[2] = {100, 50};

    void setup()
    {
    }

    position_3d_t target(float x, float y)
    {
        return {x, y, 0};
    }
};

TEST(AScaraJointAnglesComputer, failsWhenNoSolutionFound)
{
    position_3d_t unreachable_target = target(200, 0);

    bool solution_found = scara_compute_joint_angles(unreachable_target, mode,
                                                     length, &alpha, &beta);

    CHECK_FALSE(solution_found);
}

TEST(AScaraJointAnglesComputer, choosesASolutionWhenMoreThanOnePossibility)
{
    position_3d_t ambiguous_target = target(120, 0);

    bool solution_found = scara_compute_joint_angles(ambiguous_target, mode,
                                                     length, &alpha, &beta);

    CHECK_TRUE(solution_found);
    DOUBLES_EQUAL(0.5, alpha, 0.1);
    DOUBLES_EQUAL(-1.379634, beta, 0.1);
}

TEST(AScaraJointAnglesComputer, wrapsBetaWhenLowerThanMinusPi)
{
    position_3d_t valid_target = target(-length[0], length[1]);

    bool solution_found = scara_compute_joint_angles(valid_target, mode,
                                                     length, &alpha, &beta);

    CHECK_TRUE(solution_found);
    DOUBLES_EQUAL(0.5 * M_PI, beta, 0.1);
}

TEST(AScaraJointAnglesComputer, wrapsBetaWhenHigherThanMinusPi)
{
    position_3d_t valid_target = target(-length[0], -length[1]);

    bool solution_found = scara_compute_joint_angles(valid_target, mode,
                                                     length, &alpha, &beta);

    CHECK_TRUE(solution_found);
    DOUBLES_EQUAL(- 0.5 * M_PI, beta, 0.1);
}

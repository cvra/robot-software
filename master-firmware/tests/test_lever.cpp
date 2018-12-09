#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <lever/lever.h>
#include <robot_helpers/math_helpers.h>

void set_servo_pos(void* s, float value)
{
    *(float*)s = value;
}

void set_pump_voltage(void* p, float value)
{
    *(float*)p = value;
}

TEST_GROUP (ALever) {
    lever_t lever;
    float lever_servo_pos;
    float lever_pump1_voltage, lever_pump2_voltage;
    se2_t _ = se2_create_xya(0, 0, 0);

    void setup()
    {
        lever_init(&lever);
        lever_set_callbacks(&lever, &set_servo_pos, &lever_servo_pos);
        lever_set_servo_range(&lever, 0.001, 0.002);
        lever_pump_set_callbacks(&lever, &set_pump_voltage, &lever_pump1_voltage, &lever_pump2_voltage);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }

    void POSE_EQUAL(se2_t lhs, se2_t rhs, double tolerance = 0.01)
    {
        DOUBLES_EQUAL(lhs.translation.x, rhs.translation.x, tolerance);
        DOUBLES_EQUAL(lhs.translation.y, rhs.translation.y, tolerance);
        DOUBLES_EQUAL(lhs.rotation.angle, rhs.rotation.angle, tolerance);
    }
};

TEST(ALever, initializesInDisabledState)
{
    CHECK_EQUAL(LEVER_DISABLED, lever.state);
    CHECK_EQUAL(LEVER_PUMP_OFF, lever.pump_state);
}

TEST(ALever, deploys)
{
    lever_deploy(&lever);

    CHECK_EQUAL(LEVER_DEPLOYED, lever.state);
    CHECK(lever_servo_pos != 0.0f);
}

TEST(ALever, retracts)
{
    lever_retract(&lever);

    CHECK_EQUAL(LEVER_RETRACTED, lever.state);
    CHECK(lever_servo_pos != 0.0f);
}

TEST(ALever, pumpsWhenPickingUpBlocks)
{
    se2_t robot_pose = se2_create_xya(10, 20, 180);
    se2_t block_pose = se2_create_xya(30, 40, 0);

    lever_pickup(&lever, robot_pose, block_pose);

    CHECK_EQUAL(LEVER_PUMP_ON, lever.pump_state);
    CHECK(lever_pump1_voltage != 0.0f);
    CHECK(lever_pump2_voltage != 0.0f);
}

TEST(ALever, stopsPumpingWhenDepositingBlocks)
{
    se2_t robot_pose = se2_create_xya(10, 20, RADIANS(180));

    lever_deposit(&lever, robot_pose);

    CHECK_EQUAL(LEVER_PUMP_OFF, lever.pump_state);
    CHECK_EQUAL(lever_pump1_voltage, 0.0);
    CHECK_EQUAL(lever_pump2_voltage, 0.0);
}

TEST(ALever, depositsBlocksAtSamePositionAsPickupIfNotMoved)
{
    se2_t robot_pose = se2_create_xya(10, 20, RADIANS(180));
    se2_t block_pickup_pose = se2_create_xya(30, 40, RADIANS(0));

    lever_pickup(&lever, robot_pose, block_pickup_pose);
    se2_t block_deposit_pose = lever_deposit(&lever, robot_pose);

    POSE_EQUAL(block_deposit_pose, block_pickup_pose, 0.01);
}

TEST(ALever, depositsBlocksRotatedWhenDepositedFromMirroredPickupPose)
{
    se2_t robot_pickup_pose = se2_create_xya(10, 20, RADIANS(180));
    se2_t block_pickup_pose = se2_create_xya(30, 40, RADIANS(0));
    se2_t robot_deposit_pose = se2_create_xya(50, 60, RADIANS(0));

    lever_pickup(&lever, robot_pickup_pose, block_pickup_pose);
    se2_t block_deposit_pose = lever_deposit(&lever, robot_deposit_pose);

    POSE_EQUAL(block_deposit_pose, se2_create_xya(30, 40, RADIANS(-180)), 0.01);
}

TEST(ALever, depositsBlocksTranslatedWhenDepositedAfterTranslationOnly)
{
    se2_t robot_pickup_pose = se2_create_xya(10, 20, RADIANS(180));
    se2_t block_pickup_pose = se2_create_xya(30, 40, RADIANS(0));
    se2_t robot_deposit_pose = se2_create_xya(50, 60, RADIANS(180));

    lever_pickup(&lever, robot_pickup_pose, block_pickup_pose);
    se2_t block_deposit_pose = lever_deposit(&lever, robot_deposit_pose);

    POSE_EQUAL(block_deposit_pose, se2_create_xya(70, 80, RADIANS(0)), 0.01);
}

TEST(ALever, depositsBlocksTranslatedWhenNonTrivialRotation)
{
    se2_t robot_pickup_pose = se2_create_xya(100, 100, RADIANS(45));
    se2_t block_pickup_pose = se2_create_xya(150, 150, RADIANS(0));
    se2_t robot_deposit_pose = se2_create_xya(200, 50, RADIANS(45 + 180));

    lever_pickup(&lever, robot_pickup_pose, block_pickup_pose);
    se2_t block_deposit_pose = lever_deposit(&lever, robot_deposit_pose);

    POSE_EQUAL(se2_create_xya(150, 0, RADIANS(180)), block_deposit_pose, 0.01);
}

TEST(ALever, depositsBlocksTranslatedWhenNonTrivialRotationBis)
{
    se2_t robot_pickup_pose = se2_create_xya(0, 0, RADIANS(0));
    se2_t block_pickup_pose = se2_create_xya(150, 150, RADIANS(0));
    se2_t robot_deposit_pose = se2_create_xya(200, 50, RADIANS(180));

    lever_pickup(&lever, robot_pickup_pose, block_pickup_pose);
    se2_t block_deposit_pose = lever_deposit(&lever, robot_deposit_pose);

    POSE_EQUAL(se2_create_xya(50, -100, RADIANS(180)), block_deposit_pose, 0.01);
}

TEST(ALever, locksCorrectlyWhenSettingServoRange)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &lever.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &lever.lock);

    lever_set_servo_range(&lever, 0.001, 0.002);
}

TEST(ALever, locksCorrectlyWhenDeploying)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &lever.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &lever.lock);

    lever_deploy(&lever);
}

TEST(ALever, locksCorrectlyWhenRetracting)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &lever.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &lever.lock);

    lever_retract(&lever);
}

TEST(ALever, locksCorrectlyOnPickup)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &lever.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &lever.lock);

    lever_pickup(&lever, _, _);
}

TEST(ALever, locksCorrectlyOnDeposit)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &lever.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &lever.lock);

    lever_deposit(&lever, _);
}

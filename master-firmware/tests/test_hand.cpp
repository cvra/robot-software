#include "CppUTest/TestHarness.h"
#include <CppUTestExt/MockSupport.h>

#include "hand/hand.h"


TEST_GROUP(HandTestGroup)
{
    hand_t hand;
    scara_t arm;
    struct robot_position robot_pos;
    float wrist_angle;

    void setup()
    {
        hand_init(&hand);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }
};

TEST(HandTestGroup, HandManagerIsAtomic)
{
    lock_mocks_enable(true);
    mock().expectOneCall("chMtxLock").withPointerParameter("lock", &hand.lock);
    mock().expectOneCall("chMtxUnlock").withPointerParameter("lock", &hand.lock);
    hand_manage(&hand);
}

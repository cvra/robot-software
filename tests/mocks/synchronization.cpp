#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../../messagebus.h"

static bool lock_enabled = false;

void messagebus_lock_acquire(void *lock)
{
    if (lock_enabled) {
        mock().actualCall("messagebus_lock_acquire")
            .withPointerParameter("lock", lock);
    }
}

void messagebus_lock_release(void *lock)
{
    if (lock_enabled) {
        mock().actualCall("messagebus_lock_release")
            .withPointerParameter("lock", lock);
    }
}

void lock_mocks_enable(bool enabled)
{
    lock_enabled = enabled;
}

TEST_GROUP(LockTestGroup)
{
    int lock;

    void setup()
    {
        lock_mocks_enable(true);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }
};

TEST(LockTestGroup, CanLock)
{
    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", &lock);

    messagebus_lock_acquire(&lock);
}

TEST(LockTestGroup, CanUnlock)
{
    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", &lock);

    messagebus_lock_release(&lock);
}

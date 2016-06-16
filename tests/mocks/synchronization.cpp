#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../../messagebus.h"

static bool lock_enabled = false;
static bool condvar_enabled = false;

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

void messagebus_condvar_broadcast(void *var)
{
    if (condvar_enabled) {
        mock().actualCall("messagebus_condvar_broadcast")
              .withPointerParameter("var", var);
    }
}

void messagebus_condvar_wait(void *var)
{
    if (condvar_enabled) {
        mock().actualCall("messagebus_condvar_wait")
              .withPointerParameter("var", var);
    }
}

void lock_mocks_enable(bool enabled)
{
    lock_enabled = enabled;
}

void condvar_mocks_enable(bool enabled)
{
    condvar_enabled = enabled;
}

TEST_GROUP(LockTestGroup)
{
    int lock;

    void setup()
    {
        lock_mocks_enable(true);
        condvar_mocks_enable(true);
    }

    void teardown()
    {
        lock_mocks_enable(false);
        condvar_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
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

TEST(LockTestGroup, CanBroadcastCondVar)
{
    int var;
    mock().expectOneCall("messagebus_condvar_broadcast")
          .withPointerParameter("var", &var);

    messagebus_condvar_broadcast(&var);
}

TEST(LockTestGroup, CanWaitCondVar)
{
    int var;

    mock().expectOneCall("messagebus_condvar_wait")
          .withPointerParameter("var", &var);


    messagebus_condvar_wait(&var);
}

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <ch.h>

static bool lock_enabled = false;

void chMtxLock(mutex_t *lock)
{
    if (lock_enabled) {
        mock().actualCall("chMtxLock").withPointerParameter("lock", lock);
    }
}

void chMtxUnlock(mutex_t *lock)
{
    if (lock_enabled) {
        mock().actualCall("chMtxUnlock").withPointerParameter("lock", lock);
    }
}

void lock_mocks_enable(bool enabled)
{
    lock_enabled = enabled;
}

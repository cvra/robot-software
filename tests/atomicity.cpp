#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
#include "mocks/synchronization.hpp"

TEST_GROUP(MessageBusAtomicityTestGroup)
{
    messagebus_t bus;
    int bus_lock;
    topic_t topic;
    uint8_t buffer[128];

    void setup()
    {
        mock().strictOrder();

        messagebus_init(&bus, &bus_lock);
        topic_init(&topic, buffer, sizeof buffer);
    }

    void teardown()
    {
        lock_mocks_enable(false);
    }
};

TEST(MessageBusAtomicityTestGroup, AdvertiseIsLockedProperly)
{
    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", bus.lock);
    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", bus.lock);

    lock_mocks_enable(true);
    messagebus_advertise_topic(&bus, &topic, "topic");
}

TEST(MessageBusAtomicityTestGroup, FindNoneIsLockedProperly)
{
    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", bus.lock);
    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", bus.lock);

    lock_mocks_enable(true);
    messagebus_find_topic(&bus, "topic");
}

TEST(MessageBusAtomicityTestGroup, FindExistingTopicIsLockedProperly)
{
    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", bus.lock);
    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", bus.lock);

    messagebus_advertise_topic(&bus, &topic, "topic");
    lock_mocks_enable(true);
    messagebus_find_topic(&bus, "topic");

}

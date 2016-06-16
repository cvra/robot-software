#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
#include "mocks/synchronization.hpp"

TEST_GROUP(SignalingTestGroup)
{
    messagebus_t bus;
    int bus_lock, bus_condvar;
    messagebus_topic_t topic;
    uint8_t buffer[128];
    int topic_lock;
    int topic_condvar;

    void setup()
    {
        mock().strictOrder();
        messagebus_init(&bus, &bus_lock, &bus_condvar);
        messagebus_topic_init(&topic, &topic_lock, &topic_condvar, buffer, sizeof buffer);
    }

    void teardown()
    {
        lock_mocks_enable(false);
        condvar_mocks_enable(false);
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(SignalingTestGroup, TopicPublish)
{
    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", topic.lock);

    mock().expectOneCall("messagebus_condvar_broadcast")
          .withPointerParameter("var", topic.condvar);

    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", topic.lock);

    messagebus_topic_publish(&topic, buffer, sizeof(buffer));
}

TEST(SignalingTestGroup, TopicWait)
{
    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", topic.lock);

    mock().expectOneCall("messagebus_condvar_wait")
          .withPointerParameter("var", topic.condvar);

    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", topic.lock);

    messagebus_topic_wait(&topic, buffer, sizeof(buffer));
}

TEST(SignalingTestGroup, Advertise)
{
    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().expectOneCall("messagebus_lock_acquire")
          .withPointerParameter("lock", bus.lock);

    mock().expectOneCall("messagebus_condvar_broadcast")
          .withPointerParameter("var", bus.condvar);

    mock().expectOneCall("messagebus_lock_release")
          .withPointerParameter("lock", bus.lock);

    messagebus_advertise_topic(&bus, &topic, "topic");
}

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"

TEST_GROUP(MessageBusTestGroup)
{
    messagebus_t bus;
    int bus_lock;
    topic_t topic;
    uint8_t buffer[128];
    int topic_lock;

    void setup()
    {
        messagebus_init(&bus, &bus_lock);
        topic_init(&topic, &topic_lock, buffer, sizeof buffer);
    }
};

TEST(MessageBusTestGroup, CanCreateTopicWithBuffer)
{
    POINTERS_EQUAL(buffer, topic.buffer);
    POINTERS_EQUAL(&topic_lock, topic.lock);
    CHECK_EQUAL(topic.buffer_len, sizeof(buffer));
}

TEST(MessageBusTestGroup, CanCreateBus)
{
    POINTERS_EQUAL(NULL, bus.topics.head);
    POINTERS_EQUAL(&bus_lock, bus.lock);
}

TEST(MessageBusTestGroup, AdvertiseTopicName)
{
    messagebus_advertise_topic(&bus, &topic, "/imu/raw");

    STRCMP_EQUAL("/imu/raw", topic.name);
}

TEST(MessageBusTestGroup, FirstTopicGoesToHead)
{
    messagebus_advertise_topic(&bus, &topic, "/imu/raw");

    POINTERS_EQUAL(&topic, bus.topics.head);
}

TEST(MessageBusTestGroup, NextofListIsOkToo)
{
    topic_t second_topic;
    topic_init(&second_topic, NULL, NULL, 0);

    messagebus_advertise_topic(&bus, &topic, "first");
    messagebus_advertise_topic(&bus, &second_topic, "second");

    POINTERS_EQUAL(&second_topic, bus.topics.head);
    POINTERS_EQUAL(&topic, bus.topics.head->next);
}

TEST(MessageBusTestGroup, TopicNotFound)
{
    messagebus_find_topic(&bus, "topic");
    POINTERS_EQUAL(NULL, messagebus_find_topic(&bus, "topic"));
}

TEST(MessageBusTestGroup, TopicFound)
{
    messagebus_advertise_topic(&bus, &topic, "topic");
    POINTERS_EQUAL(&topic, messagebus_find_topic(&bus, "topic"));
}

TEST(MessageBusTestGroup, CanScanBus)
{
    topic_t second_topic;
    topic_init(&second_topic, NULL, NULL, 0);

    messagebus_advertise_topic(&bus, &topic, "first");
    messagebus_advertise_topic(&bus, &second_topic, "second");

    POINTERS_EQUAL(&topic, messagebus_find_topic(&bus, "first"));
    POINTERS_EQUAL(&second_topic, messagebus_find_topic(&bus, "second"));
}

TEST(MessageBusTestGroup, CanPublish)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    res = messagebus_publish(&topic, data, sizeof(data));

    MEMCMP_EQUAL(topic.buffer, data, sizeof(data));
    CHECK_TRUE(res);
}

TEST(MessageBusTestGroup, WontPublishTooBigMessage)
{
    uint8_t data[] = {1, 2, 3};
    bool res;

    topic.buffer_len = 1;
    res = messagebus_publish(&topic, data, sizeof(data));

    CHECK_FALSE(res);
}

TEST(MessageBusTestGroup, CanRead)
{
    int tx=42, rx;
    bool res;

    messagebus_publish(&topic, &tx, sizeof(int));
    res = messagebus_read(&topic, &rx, sizeof(int));

    CHECK_TRUE(res);
    CHECK_EQUAL(tx, rx);
}

TEST(MessageBusTestGroup, WontReadUnpublishedtopic)
{
    int rx;
    bool res;

    res = messagebus_read(&topic, &rx, sizeof(int));
    CHECK_FALSE(res);
}


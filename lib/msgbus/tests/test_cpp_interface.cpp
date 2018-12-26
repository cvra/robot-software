#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"


TEST_GROUP(MessagebusCppInterface)
{
    messagebus_topic_t raw_topic;
    int topic_content;
    messagebus_t bus;
    messagebus::TopicWrapper<int> topic{&raw_topic};
    void setup(void)
    {
        messagebus_topic_init(&raw_topic, nullptr, nullptr, &topic_content, sizeof(topic_content));
        messagebus_init(&bus, nullptr, nullptr);
        messagebus_advertise_topic(&bus, &raw_topic, "/foo");
    }
};

TEST(MessagebusCppInterface, CanPublish)
{
    topic.publish(42);

    int read_msg;
    bool res = messagebus_topic_read(&raw_topic, &read_msg, sizeof(read_msg));

    CHECK_TRUE(res);
    CHECK_EQUAL(42, read_msg);
}

TEST(MessagebusCppInterface, CanRead)
{
    int msg = 42;
    messagebus_topic_publish(&raw_topic, &msg, sizeof(msg));

    int read_msg;
    bool res = topic.read(read_msg);

    CHECK_TRUE(res);
    CHECK_EQUAL(msg, read_msg);
}

TEST(MessagebusCppInterface, CanWait)
{
    int msg = 42;
    messagebus_topic_publish(&raw_topic, &msg, sizeof(msg));

    CHECK_EQUAL(42, topic.wait());
}

TEST(MessagebusCppInterface, CanCheckIfTopicExists)
{
    CHECK_TRUE(messagebus::TopicWrapper<int>(&raw_topic));
    CHECK_FALSE(messagebus::TopicWrapper<int>(nullptr));
}

TEST(MessagebusCppInterface, CanFindTopic)
{
    auto topic = messagebus::find_topic<int>(bus, "/foo");
    CHECK_TRUE(topic);
}

TEST(MessagebusCppInterface, CanFindTopicBlocking)
{
    auto topic = messagebus::find_topic_blocking<int>(bus, "/foo");
    CHECK_TRUE(topic);
}

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"

TEST_GROUP (MessagebusStats) {
    messagebus_topic_t topic;

    void setup()
    {
        messagebus_topic_init(&topic, nullptr, nullptr, nullptr, 0);
    }
};

TEST(MessagebusStats, StatsAreZeroedOnInit)
{
    messagebus_topic_stats_t stats;
    messagebus_topic_stats_get(&topic, &stats);
    CHECK_EQUAL(0, stats.messages);
}

TEST(MessagebusStats, MessagesAreIncrementedOnPublish)
{
    messagebus_topic_stats_t stats;

    messagebus_topic_publish(&topic, nullptr, 0);
    messagebus_topic_stats_get(&topic, &stats);
    CHECK_EQUAL(1, stats.messages);
}

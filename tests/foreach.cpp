#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include "../messagebus.h"

TEST_GROUP(MsgBusForeachTestGroup)
{
   messagebus_t bus;
   messagebus_topic_t foo, bar;

   void setup()
   {
       messagebus_init(&bus, NULL, NULL);
       messagebus_topic_init(&foo, NULL, NULL, NULL, 0);
       messagebus_topic_init(&bar, NULL, NULL, NULL, 0);
       messagebus_advertise_topic(&bus, &foo, "foo");
       messagebus_advertise_topic(&bus, &bar, "bar");
   }
};

TEST(MsgBusForeachTestGroup, CanIterate)
{
    mock().expectOneCall("loop").withParameter("topic", &foo);
    mock().expectOneCall("loop").withParameter("topic", &bar);

    MESSAGEBUS_TOPIC_FOREACH(&bus, topic) {
        mock().actualCall("loop").withParameter("topic", topic);
    }
}

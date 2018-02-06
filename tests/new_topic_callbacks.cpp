#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"

TEST_GROUP(NewTopicCallback)
{
    messagebus_t bus;
    messagebus_topic_t topic;
    messagebus_new_topic_cb_t cb;

    void setup()
    {
        messagebus_init(&bus, NULL, NULL);
        messagebus_topic_init(&topic, NULL, NULL, NULL, 0);
    }
};

void my_cb(messagebus_t *bus, messagebus_topic_t *topic, void *arg)
{
    mock().actualCall("my_cb")
        .withPointerParameter("bus", bus)
        .withPointerParameter("topic", topic)
        .withPointerParameter("arg", arg);
}

TEST(NewTopicCallback, CanRegisterCallback)
{
    messagebus_new_topic_callback_register(&bus, &cb, my_cb, (void *)0x1234);

    POINTERS_EQUAL(my_cb, cb.callback);
    POINTERS_EQUAL((void *)0x1234, cb.callback_arg);
    POINTERS_EQUAL(&cb, bus.new_topic_callback_list);
    POINTERS_EQUAL(NULL, cb.next);
}


TEST(NewTopicCallback, CallbackFiresSeveralTime)
{
    messagebus_new_topic_cb_t cb2;
    messagebus_new_topic_callback_register(&bus, &cb, my_cb, (void *)0x1234);
    messagebus_new_topic_callback_register(&bus, &cb2, my_cb, (void *)0x4321);

    POINTERS_EQUAL(&cb, cb2.next);

    mock().expectOneCall("my_cb")
        .withPointerParameter("bus", &bus)
        .withPointerParameter("topic", &topic)
        .withPointerParameter("arg", (void *)0x1234);
    mock().expectOneCall("my_cb")
        .withPointerParameter("bus", &bus)
        .withPointerParameter("topic", &topic)
        .withPointerParameter("arg", (void *)0x4321);
    messagebus_advertise_topic(&bus, &topic, "/foo");
}

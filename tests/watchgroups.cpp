#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
#include "mocks/synchronization.hpp"

TEST_GROUP(Watchgroups)
{
    messagebus_watchgroup_t group;
    messagebus_topic_t topic;
    int lock, condvar;

    void setup()
    {
        messagebus_topic_init(&topic, NULL, NULL, NULL, 0);
        messagebus_watchgroup_init(&group, &lock, &condvar);
    }

    void teardown()
    {
        lock_mocks_enable(false);
        condvar_mocks_enable(false);
    }
};

TEST(Watchgroups, CanInitWatchGroup)
{
    POINTERS_EQUAL(&lock, group.lock);
    POINTERS_EQUAL(&condvar, group.condvar);
}

TEST(Watchgroups, CanAddTopicToGroup)
{
    messagebus_watchgroup_watch(&group, &topic);
    POINTERS_EQUAL(&group, topic.groups);
}

TEST(Watchgroups, CanAddTopicToDifferentGroups)
{
    messagebus_watchgroup_t second_group;
    messagebus_watchgroup_init(&second_group, NULL, NULL);

    messagebus_watchgroup_watch(&group, &topic);
    messagebus_watchgroup_watch(&second_group, &topic);

    POINTERS_EQUAL(&second_group, topic.groups);
    POINTERS_EQUAL(&group, topic.groups->next);
}

TEST(Watchgroups, CanWaitOnGroup)
{
    group.published_topic = &topic;

    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    mock().strictOrder();
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", group.lock);
    mock().expectOneCall("messagebus_condvar_wait").withPointerParameter("var", group.condvar);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", group.lock);

    auto res = messagebus_watchgroup_wait(&group);
    POINTERS_EQUAL(&topic, res);
}

TEST(Watchgroups, GroupIsWokeUpOnPublish)
{
    messagebus_watchgroup_watch(&group, &topic);

    mock().strictOrder();

    /* Normal publish on bus */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", topic.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", topic.condvar);

    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", group.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", group.condvar);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", group.lock);


    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", topic.lock);

    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    messagebus_topic_publish(&topic, NULL, 0);
}

TEST(Watchgroups, GroupHasTopic)
{
    messagebus_watchgroup_watch(&group, &topic);
    messagebus_topic_publish(&topic, NULL, 0);
    POINTERS_EQUAL(&topic, group.published_topic);
}

TEST(Watchgroups, AllGroupsAreSignaled)
{
    messagebus_watchgroup_t group2;
    int lock2, var2;

    messagebus_watchgroup_init(&group, &lock2, &var2);
    messagebus_watchgroup_watch(&group2, &topic);
    messagebus_watchgroup_watch(&group, &topic);

    /* Normal publish on bus */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", topic.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", topic.condvar);


    /* First group */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", group.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", group.condvar);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", group.lock);

    /* Second group */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", group2.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", group2.condvar);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", group2.lock);

    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", topic.lock);

    lock_mocks_enable(true);
    condvar_mocks_enable(true);

    messagebus_topic_publish(&topic, NULL, 0);

}

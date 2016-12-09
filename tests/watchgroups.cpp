#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../messagebus.h"
#include "mocks/synchronization.hpp"

TEST_GROUP(Watchgroups)
{
    messagebus_watchgroup_t group;
    messagebus_topic_t topic;
    int lock, condvar;
    messagebus_watcher_t watcher;

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
    messagebus_watchgroup_watch(&watcher, &group, &topic);
    POINTERS_EQUAL(&group, topic.watchers->group);
}

TEST(Watchgroups, CanAddTopicToDifferentGroups)
{
    messagebus_watcher_t w2;
    messagebus_watchgroup_t second_group;
    messagebus_watchgroup_init(&second_group, NULL, NULL);

    messagebus_watchgroup_watch(&watcher, &group, &topic);
    messagebus_watchgroup_watch(&w2, &second_group, &topic);

    POINTERS_EQUAL(&second_group, topic.watchers->group);
    POINTERS_EQUAL(&group, topic.watchers->next->group);
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
    messagebus_watchgroup_watch(&watcher, &group, &topic);

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
    messagebus_watchgroup_watch(&watcher, &group, &topic);
    messagebus_topic_publish(&topic, NULL, 0);
    POINTERS_EQUAL(&topic, group.published_topic);
}

TEST(Watchgroups, AllGroupsAreSignaled)
{
    messagebus_watchgroup_t group2;
    int lock2, var2;
    messagebus_watcher_t w2;

    messagebus_watchgroup_watch(&watcher, &group, &topic);

    messagebus_watchgroup_init(&group, &lock2, &var2);
    messagebus_watchgroup_watch(&w2, &group2, &topic);

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

/* This test checks that we can have several group containing the same topics. */
TEST(Watchgroups, GroupCanBeLinkedInMultipleTopics)
{
    messagebus_topic_t topic2;
    messagebus_topic_init(&topic2, NULL, NULL, NULL, 0);

    messagebus_watchgroup_t group2, group3;
    int lock2, var2;

    messagebus_watchgroup_init(&group2, &lock2, &var2);
    messagebus_watchgroup_init(&group3, NULL, NULL);


    /* Adds the first topic to two groups. */
    messagebus_watcher_t watcher2;
    messagebus_watchgroup_watch(&watcher, &group, &topic);
    messagebus_watchgroup_watch(&watcher2, &group2, &topic);

    /* Adds the second topic to two groups as well. */
    messagebus_watcher_t watcher3, watcher1_2;
    messagebus_watchgroup_watch(&watcher3, &group3, &topic2);
    messagebus_watchgroup_watch(&watcher1_2, &group, &topic2);

    /* Normal publish on first topic */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", topic.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", topic.condvar);


    /* Published topic belongs to groups 1 & 2, lets start with group1. */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", group.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", group.condvar);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", group.lock);

    /* Then group 2. */
    mock().expectOneCall("messagebus_lock_acquire").withPointerParameter("lock", group2.lock);
    mock().expectOneCall("messagebus_condvar_broadcast").withPointerParameter("var", group2.condvar);
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", group2.lock);

    /* Also a part of the normal publish cycle */
    mock().expectOneCall("messagebus_lock_release").withPointerParameter("lock", topic.lock);

    /* Finally, do the actuall publish. */
    lock_mocks_enable(true);
    condvar_mocks_enable(true);
    messagebus_topic_publish(&topic, NULL, 0);
}

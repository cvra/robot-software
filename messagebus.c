#include "messagebus.h"
#include <string.h>

static messagebus_topic_t *topic_by_name(messagebus_t *bus, const char *name)
{
    messagebus_topic_t *t;
    for (t = bus->topics.head; t != NULL; t = t->next) {
        if (!strcmp(name, t->name)) {
            return t;
        }
    }

    return NULL;
}

void messagebus_init(messagebus_t *bus, void *lock, void *condvar)
{
    memset(bus, 0, sizeof(messagebus_t));
    bus->lock = lock;
    bus->condvar = condvar;
}

void messagebus_topic_init(messagebus_topic_t *topic, void *topic_lock, void *topic_condvar,
                           void *buffer, size_t buffer_len)
{
    memset(topic, 0, sizeof(messagebus_topic_t));
    topic->buffer = buffer;
    topic->buffer_len = buffer_len;
    topic->lock = topic_lock;
    topic->condvar = topic_condvar;
}

void messagebus_advertise_topic(messagebus_t *bus, messagebus_topic_t *topic, const char *name)
{
    memset(topic->name, 0, sizeof(topic->name));
    strncpy(topic->name, name, TOPIC_NAME_MAX_LENGTH);

    messagebus_lock_acquire(bus->lock);

    if (bus->topics.head != NULL) {
        topic->next = bus->topics.head;
    }
    bus->topics.head = topic;

    messagebus_condvar_broadcast(bus->condvar);

    messagebus_lock_release(bus->lock);
}

messagebus_topic_t *messagebus_find_topic(messagebus_t *bus, const char *name)
{
    messagebus_topic_t *res;

    messagebus_lock_acquire(bus->lock);

    res = topic_by_name(bus, name);

    messagebus_lock_release(bus->lock);

    return res;
}

messagebus_topic_t *messagebus_find_topic_blocking(messagebus_t *bus, const char *name)
{
    messagebus_topic_t *res = NULL;

    messagebus_lock_acquire(bus->lock);

    while (res == NULL) {
        res = topic_by_name(bus, name);

        if (res == NULL) {
            messagebus_condvar_wait(bus->condvar);
        }
    }

    messagebus_lock_release(bus->lock);

    return res;
}

bool messagebus_topic_publish(messagebus_topic_t *topic, void *buf, size_t buf_len)
{
    if (topic->buffer_len < buf_len) {
        return false;
    }

    messagebus_lock_acquire(topic->lock);

    memcpy(topic->buffer, buf, buf_len);
    topic->published = true;
    messagebus_condvar_broadcast(topic->condvar);

    messagebus_watcher_t *w;
    for (w = topic->watchers; w != NULL; w = w->next) {
        messagebus_lock_acquire(w->group->lock);
        w->group->published_topic = topic;
        messagebus_condvar_broadcast(w->group->condvar);
        messagebus_lock_release(w->group->lock);
    }

    messagebus_lock_release(topic->lock);

    return true;
}

bool messagebus_topic_read(messagebus_topic_t *topic, void *buf, size_t buf_len)
{
    bool success = false;
    messagebus_lock_acquire(topic->lock);

    if (topic->published) {
        success = true;
        memcpy(buf, topic->buffer, buf_len);
    }

    messagebus_lock_release(topic->lock);

    return success;
}

void messagebus_topic_wait(messagebus_topic_t *topic, void *buf, size_t buf_len)
{
    messagebus_lock_acquire(topic->lock);
    messagebus_condvar_wait(topic->condvar);

    memcpy(buf, topic->buffer, buf_len);

    messagebus_lock_release(topic->lock);
}

void messagebus_watchgroup_init(messagebus_watchgroup_t *group, void *lock,
                                void *condvar)
{
    group->lock = lock;
    group->condvar = condvar;
}

void messagebus_watchgroup_watch(messagebus_watcher_t *watcher,
                                 messagebus_watchgroup_t *group,
                                 messagebus_topic_t *topic)
{
    messagebus_lock_acquire(topic->lock);
    messagebus_lock_acquire(group->lock);

    watcher->group = group;

    if (topic->watchers != NULL) {
        watcher->next = topic->watchers;
    }
    topic->watchers = watcher;

    messagebus_lock_release(group->lock);
    messagebus_lock_release(topic->lock);
}

messagebus_topic_t *messagebus_watchgroup_wait(messagebus_watchgroup_t *group)
{
    messagebus_topic_t *res;

    messagebus_lock_acquire(group->lock);
    messagebus_condvar_wait(group->condvar);

    res = group->published_topic;

    messagebus_lock_release(group->lock);

    return res;
}

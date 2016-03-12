#include "messagebus.h"
#include <string.h>

void messagebus_init(messagebus_t *bus, void *bus_lock)
{
    memset(bus, 0, sizeof(messagebus_t));
    bus->lock = bus_lock;
}

void topic_init(topic_t *topic, void *topic_lock, void *buffer, size_t buffer_len)
{
    memset(topic, 0, sizeof(topic_t));
    topic->buffer = buffer;
    topic->buffer_len = buffer_len;
    topic->lock = topic_lock;
}

void messagebus_advertise_topic(messagebus_t *bus, topic_t *topic, const char *name)
{
    memset(topic->name, 0, sizeof(topic->name));
    strncpy(topic->name, name, TOPIC_NAME_MAX_LENGTH);

    messagebus_lock_acquire(bus->lock);

    if (bus->topics.head != NULL) {
        topic->next = bus->topics.head;
    }
    bus->topics.head = topic;

    messagebus_lock_release(bus->lock);
}

topic_t *messagebus_find_topic(messagebus_t *bus, const char *name)
{
    topic_t *t, *res=NULL;

    messagebus_lock_acquire(bus->lock);

    for (t=bus->topics.head; t!=NULL; t=t->next) {
        if (!strcmp(name, t->name)) {
            res = t;
            break;
        }
    }

    messagebus_lock_release(bus->lock);

    return res;
}

bool messagebus_publish(topic_t *topic, void *buf, size_t buf_len)
{
    if (topic->buffer_len < buf_len) {
        return false;
    }

    messagebus_lock_acquire(topic->lock);
    memcpy(topic->buffer, buf, buf_len);
    topic->published = true;
    messagebus_lock_release(topic->lock);

    return true;
}

bool messagebus_read(topic_t *topic, void *buf, size_t buf_len)
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

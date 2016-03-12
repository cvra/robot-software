#include "messagebus.h"
#include <string.h>

void messagebus_init(messagebus_t *bus)
{
    memset(bus, 0, sizeof(messagebus_t));
}

void topic_init(topic_t *topic, void *buffer, size_t buffer_len)
{
    memset(topic, 0, sizeof(topic_t));
    topic->buffer = buffer;
    topic->buffer_len = buffer_len;
}

void messagebus_advertise_topic(messagebus_t *bus, topic_t *topic, const char *name)
{
    memset(topic->name, 0, sizeof(topic->name));
    strncpy(topic->name, name, TOPIC_NAME_MAX_LENGTH);

    if (bus->topics.head != NULL) {
        topic->next = bus->topics.head;
    }
    bus->topics.head = topic;
}

topic_t *messagebus_find_topic(messagebus_t *bus, const char *name)
{
    topic_t *t;

    for (t=bus->topics.head; t!=NULL; t=t->next) {
        if (!strcmp(name, t->name)) {
            return t;
        }
    }

    return NULL;
}

bool messagebus_publish(topic_t *topic, void *buf, size_t buf_len)
{
    if (topic->buffer_len < buf_len) {
        return false;
    }

    memcpy(topic->buffer, buf, buf_len);
    topic->published = true;

    return true;
}

bool messagebus_read(topic_t *topic, void *buf, size_t buf_len)
{
    if (!topic->published) {
        return false;
    }

    memcpy(buf, topic->buffer, buf_len);

    return true;
}

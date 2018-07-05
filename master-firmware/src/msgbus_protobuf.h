#ifndef MSGBUS_PROTOBUF_H
#define MSGBUS_PROTOBUF_H

#define PB_MSGID 1

#include <pb.h>
#include <msgbus/messagebus.h>

#include <unistd.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const pb_field_t *fields;
    uint32_t msgid;
    messagebus_watcher_t udp_watcher;
    messagebus_watcher_t sdcard_watcher;
} topic_metadata_t;

#define TOPIC_DECL(name, type)                                                                     \
    struct {                                                                                       \
        messagebus_topic_t topic;                                                                  \
        mutex_t lock;                                                                              \
        condition_variable_t condvar;                                                              \
        type value;                                                                                \
        topic_metadata_t metadata;                                                                 \
    } name = {                                                                                     \
        _MESSAGEBUS_TOPIC_DATA(name.topic,                                                         \
                               name.lock,                                                          \
                               name.condvar,                                                       \
                               &name.value,                                                        \
                               sizeof(type),                                                       \
                               name.metadata),                                                     \
        _MUTEX_DATA(name.lock),                                                                    \
        _CONDVAR_DATA(name.condvar),                                                               \
        type##_init_default,                                                                       \
        {                                                                                          \
            type##_fields,                                                                         \
            type##_msgid,                                                                          \
            {0, 0},                                                                                \
            {0, 0},                                                                                \
        },                                                                                         \
    }

#define _MESSAGEBUS_TOPIC_DATA(topic, lock, condvar, buffer, buffer_size, metadata)                \
    {                                                                                              \
        buffer, buffer_size, &lock, &condvar, "", 0, NULL, NULL, &metadata,                        \
    }

/* Wraps the topic information in a header (in protobuf format) to be sent over
 * UDP or logged to disk or whatever.
 *
 * @return The message size in bytes
 */
size_t messagebus_encode_topic_message(
    messagebus_topic_t *topic, uint8_t *buf, size_t buf_len, uint8_t *obj_buf, size_t obj_buf_len);

/** Takes a topic information with a header and injects it into the
 * corresponding topic.
 *
 * @warning This functions is not thread safe due to the use of a static buffer.
 */
void messagebus_inject_encoded_message(messagebus_t *bus, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
#endif

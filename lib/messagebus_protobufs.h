#ifndef MESSAGEBUS_PROTOBUFS_H
#define MESSAGEBUS_PROTOBUFS_H

#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#define MESSAGEBUS_PB_PUBLISH(topic, msg, type) do { \
        uint8_t buffer[type##_size]; \
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer)); \
        pb_encode(&stream, type##_fields, msg); \
        messagebus_topic_publish(topic, buffer, sizeof(buffer)); \
    } while (0);

#define MESSAGEBUS_PB_WAIT(topic, msg, type) do { \
        uint8_t buffer[type##_size]; \
        messagebus_topic_wait(topic, &buffer, sizeof buffer); \
        pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer)); \
        pb_decode(&stream, type##_fields, msg); \
    } while(0);

#define MESSAGEBUS_PB_READ(topic, msg, type) do { \
        uint8_t buffer[type##_size]; \
        messagebus_topic_read(topic, &buffer, sizeof buffer); \
        pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer)); \
        pb_decode(&stream, type##_fields, msg); \
    } while(0);

#endif

#include "msgbus_protobuf.h"
#include "protobuf/MessageSize.pb.h"
#include "protobuf/TopicHeader.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>

size_t messagebus_encode_topic_message(
    messagebus_topic_t *topic, uint8_t *buf, size_t buf_len, uint8_t *obj_buf, size_t obj_buf_len)
{
    size_t offset;
    int32_t max_len;

    topic_metadata_t *metadata = (topic_metadata_t *)topic->metadata;

    TopicHeader header;
    MessageSize header_size, msg_size;

    /* First populate information in the header */
    strncpy(header.name, topic->name, sizeof(header.name) - 1);
    header.msgid = metadata->msgid;

    pb_ostream_t stream;
    offset = MessageSize_size;
    max_len = buf_len - 2 * MessageSize_size;
    if (max_len <= 0) {
        return 0;
    }

    stream = pb_ostream_from_buffer(&buf[offset], max_len);
    if (!pb_encode(&stream, TopicHeader_fields, &header)) {
        return 0;
    }

    header_size.bytes = stream.bytes_written;

    offset = 0;
    max_len = MessageSize_size;
    stream = pb_ostream_from_buffer(&buf[offset], max_len);

    if (!pb_encode(&stream, MessageSize_fields, &header_size)) {
        return 0;
    }

    /* Read topic content */
    if (obj_buf_len < topic->buffer_len) {
        return 0;
    }
    messagebus_topic_read(topic, obj_buf, topic->buffer_len);

    offset = 2 * MessageSize_size + header_size.bytes;
    max_len = buf_len - (offset);

    if (max_len <= 0) {
        return 0;
    }

    stream = pb_ostream_from_buffer(&buf[offset], max_len);
    if (!pb_encode(&stream, metadata->fields, obj_buf)) {
        return 0;
    }

    msg_size.bytes = stream.bytes_written;
    offset = header_size.bytes + MessageSize_size;
    stream = pb_ostream_from_buffer(&buf[offset], MessageSize_size);
    if (!pb_encode(&stream, MessageSize_fields, &msg_size)) {
        return 0;
    }

    return msg_size.bytes + header_size.bytes + 2 * MessageSize_size;
}

void messagebus_inject_encoded_message(messagebus_t *bus, uint8_t *buf, size_t len)
{
    size_t offset = 0;
    pb_istream_t istream;
    messagebus_topic_t *topic;

    /* TODO check for out of bounds access */
    (void)len;

    /* Get header size */
    istream = pb_istream_from_buffer(buf + offset, MessageSize_size);
    MessageSize header_size;
    if (!pb_decode(&istream, MessageSize_fields, &header_size)) {
        return;
    }

    /* Get header */
    TopicHeader header;
    offset = MessageSize_size;
    istream = pb_istream_from_buffer(buf + offset, header_size.bytes);
    if (!pb_decode(&istream, TopicHeader_fields, &header)) {
        return;
    }

    topic = messagebus_find_topic(bus, header.name);
    if (topic == NULL) {
        return;
    }

    /* Read message size */
    MessageSize msg_size;
    offset = MessageSize_size + header_size.bytes;
    istream = pb_istream_from_buffer(buf + offset, MessageSize_size);
    if (!pb_decode(&istream, MessageSize_fields, &msg_size)) {
        return;
    }

    /* Read message */
    offset = 2 * MessageSize_size + header_size.bytes;
    istream = pb_istream_from_buffer(buf + offset, msg_size.bytes);

    /* TODO better approach than just a static buffer. */
    static uint8_t obj_buffer[1024];
    topic_metadata_t *metadata = (topic_metadata_t *)topic->metadata;
    if (!pb_decode(&istream, metadata->fields, obj_buffer)) {
        return;
    }

    messagebus_topic_publish(topic, obj_buffer, topic->buffer_len);
}

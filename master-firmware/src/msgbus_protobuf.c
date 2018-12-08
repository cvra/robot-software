#include "msgbus_protobuf.h"
#include "protobuf/protocol.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>

/** Encodes a given topic's header in the buffer and returns the size.
 *
 * @note returns zero if there was an error.
 */
static size_t encode_topic_header(const messagebus_topic_t* topic, uint8_t* buf, size_t buf_len);

/** Encode a given's topic body in the buffer and returns the size. Uses the
 * provided scratch buffer to hold the topic content while it is being
 * processed.
 *
 * @returns encoded size or zero if there was an error.
 */
static size_t encode_topic_body(messagebus_topic_t* topic,
                                uint8_t* buf,
                                size_t buf_len,
                                uint8_t* scratch,
                                size_t scratch_len);

size_t messagebus_encode_topic_message(messagebus_topic_t* topic,
                                       uint8_t* buf,
                                       size_t buf_len,
                                       uint8_t* scratch,
                                       size_t scratch_len)
{
    size_t header_len, body_len;

    header_len = encode_topic_header(topic, buf, buf_len);

    if (!header_len) {
        return 0;
    }

    body_len = encode_topic_body(topic, &buf[header_len], buf_len - header_len,
                                 scratch, scratch_len);

    if (!body_len) {
        return 0;
    }

    return header_len + body_len;
}

void messagebus_inject_encoded_message(messagebus_t* bus, uint8_t* buf, size_t len)
{
    size_t offset = 0;
    pb_istream_t istream;
    messagebus_topic_t* topic;

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
    topic_metadata_t* metadata = (topic_metadata_t*)topic->metadata;
    if (!pb_decode(&istream, metadata->fields, obj_buffer)) {
        return;
    }

    messagebus_topic_publish(topic, obj_buffer, topic->buffer_len);
}

static size_t encode_topic_header(const messagebus_topic_t* topic, uint8_t* buf, size_t buf_len)
{
    topic_metadata_t* metadata = topic->metadata;
    size_t offset;
    size_t max_len;

    TopicHeader header;
    MessageSize header_size;

    /* First populate information in the header */
    strncpy(header.name, topic->name, sizeof(header.name) - 1);
    header.msgid = metadata->msgid;

    /* Then encode header, skipping enough bytes to store the message size before. */
    pb_ostream_t stream;
    offset = MessageSize_size;
    max_len = buf_len - MessageSize_size;
    if (buf_len < MessageSize_size) {
        return 0;
    }

    offset = MessageSize_size;
    stream = pb_ostream_from_buffer(&buf[offset], max_len);

    if (!pb_encode(&stream, TopicHeader_fields, &header)) {
        return 0;
    }

    /* Then prepend the header length message */
    header_size.bytes = stream.bytes_written;
    stream = pb_ostream_from_buffer(buf, MessageSize_size);

    if (!pb_encode(&stream, MessageSize_fields, &header_size)) {
        return 0;
    }

    return MessageSize_size + header_size.bytes;
}

static size_t encode_topic_body(messagebus_topic_t* topic,
                                uint8_t* buf,
                                size_t buf_len,
                                uint8_t* scratch,
                                size_t scratch_len)
{
    pb_ostream_t stream;
    MessageSize msg_size;
    bool was_posted_once;
    topic_metadata_t* metadata = topic->metadata;

    if (scratch_len < topic->buffer_len) {
        return 0;
    }

    if (buf_len < MessageSize_size) {
        return 0;
    }

    was_posted_once = messagebus_topic_read(topic, scratch, scratch_len);
    if (!was_posted_once) {
        return 0;
    }

    /* Encode while leaving enough room to write the message length */
    stream = pb_ostream_from_buffer(&buf[MessageSize_size], buf_len - MessageSize_size);
    if (!pb_encode(&stream, metadata->fields, scratch)) {
        return 0;
    }

    /* Prepend the encoded topic len. */
    msg_size.bytes = stream.bytes_written;
    stream = pb_ostream_from_buffer(buf, MessageSize_size);
    if (!pb_encode(&stream, MessageSize_fields, &msg_size)) {
        return 0;
    }

    return msg_size.bytes + MessageSize_size;
}

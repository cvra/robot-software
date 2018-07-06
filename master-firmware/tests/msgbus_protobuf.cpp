#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <cstdio>
#include <array>

// Mock types, must be before msgbus_protobuf.h
typedef int mutex_t;
typedef int condition_variable_t;
#define _MUTEX_DATA(name) 0
#define _CONDVAR_DATA(name) 0

#include "msgbus_protobuf.h"

// Message types
#include "protobuf/Timestamp.pb.h"
#include "protobuf/MessageSize.pb.h"
#include "protobuf/TopicHeader.pb.h"

TEST_GROUP (MessagebusProtobufIntegration) {
    messagebus_t bus;
    int bus_lock;
    int bus_condvar;
    messagebus_topic_t mytopic;
    Timestamp topic_content;
    topic_metadata_t metadata;

    void setup()
    {
        messagebus_init(&bus, &bus_lock, &bus_condvar);
        messagebus_topic_init(&mytopic,
                              nullptr, nullptr,
                              &topic_content, sizeof(topic_content));
        mytopic.metadata = &metadata;
        metadata.fields = Timestamp_fields;
        metadata.msgid = Timestamp_msgid;

        messagebus_advertise_topic(&bus, &mytopic, "mytopic");
        Timestamp msg = Timestamp_init_default;
        messagebus_topic_publish(&mytopic, &msg, sizeof(msg));
    }
};

TEST (MessagebusProtobufIntegration, CanCreateTopic) {
    TOPIC_DECL(topic, Timestamp);

    POINTERS_EQUAL(&topic.lock, topic.topic.lock);
    POINTERS_EQUAL(&topic.condvar, topic.topic.condvar);
    POINTERS_EQUAL(&topic.value, topic.topic.buffer);
    CHECK_EQUAL(sizeof(Timestamp), topic.topic.buffer_len);

    POINTERS_EQUAL(&topic.metadata, topic.topic.metadata);
    POINTERS_EQUAL(Timestamp_fields, topic.metadata.fields);
    CHECK_EQUAL(Timestamp_msgid, topic.metadata.msgid);
}

TEST (MessagebusProtobufIntegration, CanPublishThenEncodeData) {
    Timestamp foo;
    foo.us = 1000;

    messagebus_topic_publish(&mytopic, &foo, sizeof(foo));

    messagebus_topic_t *topic = messagebus_find_topic(&bus, "mytopic");

    // Encode the data using introspection
    uint8_t encoded_buffer[128];
    memset(encoded_buffer, 0, sizeof(encoded_buffer));
    {
        pb_ostream_t stream = pb_ostream_from_buffer(encoded_buffer, sizeof(encoded_buffer));

        uint8_t msg_buffer[128];
        CHECK_TRUE(sizeof(msg_buffer) >= topic->buffer_len); // assert
        messagebus_topic_read(topic, msg_buffer, topic->buffer_len);
        topic_metadata_t *metadata = (topic_metadata_t *)topic->metadata;
        auto res = pb_encode(&stream, metadata->fields, msg_buffer);

        CHECK_TRUE(res);
    }

    // Now decode the stream. We assume the message type was transmitted out of
    // band
    {
        Timestamp message;
        pb_istream_t stream = pb_istream_from_buffer(encoded_buffer, Timestamp_size);
        auto status = pb_decode(&stream, Timestamp_fields, &message);

        if (!status) {
            FAIL(PB_GET_ERROR(&stream));
        }

        CHECK_EQUAL(1000, message.us);
    }
}

TEST (MessagebusProtobufIntegration, EncodeMessageWithHeader) {
    uint8_t buffer[128];
    uint8_t obj_buffer[128];

    auto res = messagebus_encode_topic_message(&mytopic,
                                               buffer,
                                               sizeof(buffer),
                                               obj_buffer,
                                               sizeof(obj_buffer));

    CHECK_TRUE(res > 0)
    pb_istream_t stream;
    size_t offset = 0;

    // Check that we first have a size object
    MessageSize size;
    stream = pb_istream_from_buffer(&buffer[offset], MessageSize_size);
    CHECK_TRUE(pb_decode(&stream, MessageSize_fields, &size));
    CHECK_TRUE(size.bytes > 0);

    offset += MessageSize_size;

    // Check that we then have a header object
    TopicHeader header;
    stream = pb_istream_from_buffer(&buffer[offset], size.bytes);
    CHECK_TRUE(pb_decode(&stream, TopicHeader_fields, &header));
    offset += size.bytes;

    STRCMP_EQUAL("mytopic", header.name);
    CHECK_EQUAL(Timestamp_msgid, header.msgid);

    // Now we should again have a size object
    stream = pb_istream_from_buffer(&buffer[offset], MessageSize_size);
    CHECK_TRUE(pb_decode(&stream, MessageSize_fields, &size));
    CHECK_TRUE(size.bytes > 0);
    offset += MessageSize_size;

    // Finally we should have a timestamp object
    Timestamp ts;
    stream = pb_istream_from_buffer(&buffer[offset], size.bytes);
    CHECK_TRUE(pb_decode(&stream, Timestamp_fields, &ts));
}

TEST (MessagebusProtobufIntegration, NotEnoughRoomForMessageHeader) {
    uint8_t buffer[1];
    uint8_t obj_buffer[128];
    auto res = messagebus_encode_topic_message(&mytopic,
                                               buffer,
                                               sizeof(buffer),
                                               obj_buffer,
                                               sizeof(obj_buffer));

    CHECK_EQUAL(0, res);
}

TEST (MessagebusProtobufIntegration, NotEnoughRoomForMessageBody) {
    uint8_t buffer[256];
    uint8_t obj_buffer[128];
    auto res = messagebus_encode_topic_message(&mytopic,
                                               buffer,
                                               18,
                                               obj_buffer,
                                               sizeof(obj_buffer));

    CHECK_EQUAL(0, res);
}

TEST (MessagebusProtobufIntegration, NotEnoughRoomForObject) {
    uint8_t buffer[256];
    uint8_t obj_buffer[2];
    auto res = messagebus_encode_topic_message(&mytopic,
                                               buffer,
                                               sizeof(buffer),
                                               obj_buffer,
                                               sizeof(obj_buffer));

    CHECK_EQUAL(0, res);
}

TEST_GROUP (MessagebusProtobufMessageInjection) {
    messagebus_t bus;
    using EncodedMessage = std::array<uint8_t, 128>;

    EncodedMessage prepare_message(const std::string &name, Timestamp value)
    {
        // Encode the message using a dummy bus
        messagebus_t bus;
        messagebus_init(&bus, nullptr, nullptr);
        TOPIC_DECL(mytopic, Timestamp);
        messagebus_advertise_topic(&bus, &mytopic.topic, name.c_str());
        messagebus_topic_publish(&mytopic.topic, &value, sizeof(Timestamp));

        EncodedMessage encoded_message;

        uint8_t obj_buffer[128];

        messagebus_encode_topic_message(&mytopic.topic,
                                        encoded_message.data(),
                                        encoded_message.size(),
                                        obj_buffer,
                                        sizeof(obj_buffer));

        return encoded_message;
    }

    void setup()
    {
        messagebus_init(&bus, nullptr, nullptr);
    }
};

TEST (MessagebusProtobufMessageInjection, CanInjectExternalMessageIntoMessageBus) {
    // Create a serialized object message
    TOPIC_DECL(mytopic, Timestamp);
    messagebus_advertise_topic(&bus, &mytopic.topic, "mytopic");

    // Prepare an encoded message and injects it into the bus
    {
        Timestamp value;
        value.us = 100;

        auto msg = prepare_message("mytopic", value);

        messagebus_inject_encoded_message(&bus, msg.data(), msg.size());
    }

    // Now check that the value was indeed injected into the bus
    {
        Timestamp value;
        auto was_published = messagebus_topic_read(&mytopic.topic, &value, sizeof(value));
        CHECK_TRUE(was_published);
        CHECK_EQUAL(100, value.us);
    }
}

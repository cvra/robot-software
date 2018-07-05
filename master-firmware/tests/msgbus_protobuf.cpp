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

    void setup()
    {
        messagebus_init(&bus, &bus_lock, &bus_condvar);
    }
};

TEST (MessagebusProtobufIntegration, CanCreateTopic) {
    TOPIC_DECL(mytopic, Timestamp);

    POINTERS_EQUAL(&mytopic.lock, mytopic.topic.lock);
    POINTERS_EQUAL(&mytopic.condvar, mytopic.topic.condvar);
    POINTERS_EQUAL(&mytopic.value, mytopic.topic.buffer);
    CHECK_EQUAL(sizeof(Timestamp), mytopic.topic.buffer_len);

    POINTERS_EQUAL(&mytopic.metadata, mytopic.topic.metadata);
    POINTERS_EQUAL(Timestamp_fields, mytopic.metadata.fields);
    CHECK_EQUAL(Timestamp_msgid, mytopic.metadata.msgid);
}

TEST (MessagebusProtobufIntegration, CanPublishThenEncodeData) {
    TOPIC_DECL(mytopic, Timestamp);
    messagebus_advertise_topic(&bus, &mytopic.topic, "mytopic");

    Timestamp foo;
    foo.us = 1000;

    messagebus_topic_publish(&mytopic.topic, &foo, sizeof(foo));

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

#if 0
        // Write the message in a file to be decoded via protoc --decode
        FILE *out = fopen("example_timestamp.bin", "wb");
        fwrite(encoded_buffer, stream.bytes_written, 1, out);
        fclose(out);
#endif
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
    TOPIC_DECL(mytopic, Timestamp);
    messagebus_advertise_topic(&bus, &mytopic.topic, "mytopic");

    uint8_t buffer[128];
    uint8_t obj_buffer[128];

    auto res = messagebus_encode_topic_message(&mytopic.topic,
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
    TOPIC_DECL(mytopic, Timestamp);
    messagebus_advertise_topic(&bus, &mytopic.topic, "mytopic");

    uint8_t buffer[1];
    uint8_t obj_buffer[128];
    auto res = messagebus_encode_topic_message(&mytopic.topic,
                                               buffer,
                                               sizeof(buffer),
                                               obj_buffer,
                                               sizeof(obj_buffer));

    CHECK_EQUAL(0, res);
}

TEST (MessagebusProtobufIntegration, NotEnoughRoomForMessageBody) {
    TOPIC_DECL(mytopic, Timestamp);
    messagebus_advertise_topic(&bus, &mytopic.topic, "mytopic");

    uint8_t buffer[256];
    uint8_t obj_buffer[128];
    auto res = messagebus_encode_topic_message(&mytopic.topic,
                                               buffer,
                                               18,
                                               obj_buffer,
                                               sizeof(obj_buffer));

    CHECK_EQUAL(0, res);
}

TEST (MessagebusProtobufIntegration, NotEnoughRoomForObject) {
    TOPIC_DECL(mytopic, Timestamp);
    messagebus_advertise_topic(&bus, &mytopic.topic, "mytopic");

    uint8_t buffer[256];
    uint8_t obj_buffer[2];
    auto res = messagebus_encode_topic_message(&mytopic.topic,
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

#pragma once

#include <memory>
#include <absl/strings/string_view.h>
#include <absl/types/optional.h>
#include <unordered_map>
#include <can/bus_enumerator.h>
#include <msgbus/messagebus.h>
#include <msgbus_protobuf.h>
#include <msgbus/posix/port.h>
#include <pb.h>
#include <uavcan/uavcan.hpp>

// This class is responsible for forwarding messages coming in on the UAVCAN
// bus to the software message bus.
// It will create one topic per board that sends messages, and put messages
// coming from this board on the corresponding topic.
// This proxy can be useful when building integration with a new sensor or a new
// board.
// It is templated over the types of the messages (both on the UAVCAN side, as
// well as on the messagebus side), as well as some metadata required for
// protobuf.
template <
    typename UavcanMessage,
    typename TopicMessage,
    const pb_field_t TopicFields[],
    uint32_t TopicMsgId>
class UavcanToMessagebusProxy {
    struct TopicData {
        // we store a pointer to data, as TopicData itself might be moved as we
        // add more topics, but we do not want to move the data buffer, as it
        // is also referenced in other places
        std::unique_ptr<TopicMessage> data;
        condvar_wrapper_t var;
        messagebus_topic_t topic;
    };

    bus_enumerator_t* bus_enumerator;
    messagebus_t* msgbus;

    topic_metadata_t metadata;

    std::unique_ptr<uavcan::Subscriber<UavcanMessage>> subscriber;
    std::unordered_map<std::string, TopicData> topic_map;

public:
    // Constructor. Takes a bus enumerator that will be used to gather the
    // sender's board name from received messages, as well as the messagebus to
    // send messages to.
    UavcanToMessagebusProxy(bus_enumerator_t* be, messagebus_t* bus_)
        : bus_enumerator(be)
        , msgbus(bus_)
    {
        metadata.fields = TopicFields;
        metadata.msgid = TopicMsgId;
    };

    // Transforms a board name into a topic name. For example, a user
    // implementing some distance sensor integration might want to transform
    // messages coming from a board named "front-left" into
    // "/distance/front-left"
    // This must be implemented by users of this class.
    virtual std::string topic_name(absl::string_view board_name) = 0;

    // Returns the message translated to messagebus format, or empty if the
    // message should not be forwarded to the bus.
    // This must be implemented by users of this class.
    virtual absl::optional<TopicMessage> translate(const UavcanMessage& in) = 0;

    // Starts a listener for the provided UAVCAN type on the node, and starts
    // forwarding messages.
    int start(uavcan::INode& node)
    {
        subscriber = std::make_unique<uavcan::Subscriber<UavcanMessage>>(node);
        return subscriber->start([&](const uavcan::ReceivedDataStructure<UavcanMessage>& msg) {
            process(msg, msg.getSrcNodeID().get());
        });
    }

    virtual ~UavcanToMessagebusProxy()
    {
    }

protected:
    void process(const UavcanMessage& msg, uint8_t src_id)
    {
        // Finds out the name of the sender from the bus enumerator and abors
        // if that sender is not known yet.
        const char* board_name = bus_enumerator_get_str_id(bus_enumerator, src_id);

        if (!board_name) {
            // TODO(antoinealb): Maybe we want to log that failure
            return;
        }

        messagebus_topic_t* topic = find_or_create_topic(topic_name(board_name));

        // Sends the message to messagebus if the user was able to translate it
        auto topic_msg_opt = translate(msg);
        if (topic_msg_opt) {
            messagebus_topic_publish(topic, &(*topic_msg_opt),
                                     sizeof(TopicMessage));
        }
    }

    messagebus_topic_t* find_or_create_topic(std::string topic_name)
    {
        auto elem = topic_map.find(topic_name);
        if (elem != topic_map.end()) {
            return &elem->second.topic;
        }

        // Otherwise create it from scratch and store it in the map
        topic_map.insert({topic_name, TopicData()});
        topic_map[topic_name].data = std::make_unique<TopicMessage>();

        messagebus_topic_init(&topic_map[topic_name].topic,
                              &topic_map[topic_name].var,
                              &topic_map[topic_name].var,
                              topic_map[topic_name].data.get(), sizeof(TopicMessage));
        topic_map[topic_name].topic.metadata = &metadata;
        messagebus_advertise_topic(msgbus, &topic_map[topic_name].topic, topic_name.c_str());

        return &topic_map[topic_name].topic;
    }
};

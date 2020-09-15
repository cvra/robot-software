#include <absl/strings/str_cat.h>
#include <uavcan_linux/uavcan_linux.hpp>
#include "can/UavcanToMessagebusProxy.hpp"
#include <msgbus/messagebus.h>

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include <protobuf/beacons.pb.h>
#include <cvra/proximity_beacon/Signal.hpp>

using ProxyBase = UavcanToMessagebusProxy<cvra::proximity_beacon::Signal, BeaconSignal, BeaconSignal_fields, BeaconSignal_msgid>;
struct Proxy : public ProxyBase {
    Proxy(bus_enumerator_t* be, messagebus_t* bus)
        : ProxyBase(be, bus)
    {
    }

    std::string topic_name(absl::string_view board_name) override
    {
        return absl::StrCat("/", board_name);
    }

    absl::optional<BeaconSignal> translate(const cvra::proximity_beacon::Signal& in) override
    {
        // Example translation function. In general you would have a more
        // complicated translation function here
        BeaconSignal out;
        out.range.range.distance = in.length;
        return {out};
    }

    // for test only, expose process as a method, which would usually be called
    // from UAVCAN
    void process(const cvra::proximity_beacon::Signal& msg, uint8_t src_id)
    {
        ProxyBase::process(msg, src_id);
    }
};

TEST_GROUP (ProxyTestGroup) {
    bus_enumerator_t be;
    bus_enumerator_entry_allocator bea[10];
    messagebus_t bus;
    Proxy proxy{&be, &bus};
    cvra::proximity_beacon::Signal signal;

    void setup() override
    {
        bus_enumerator_init(&be, bea, 10);
        bus_enumerator_add_node(&be, "myboard", nullptr);
        messagebus_init(&bus, nullptr, nullptr);
    }
};

TEST(ProxyTestGroup, DoesNothingWhenUnknownBoardIsThere)
{
    proxy.process(signal, 42);
}

TEST(ProxyTestGroup, CreatesTopicForKnownBoards)
{
    bus_enumerator_update_node_info(&be, "myboard", 42);
    proxy.process(signal, 42);

    auto* topic = messagebus_find_topic(&bus, "/myboard");
    CHECK_TRUE_TEXT(topic, "A topic should be created matching on received messages");
}

TEST(ProxyTestGroup, DoesNotDoubleTopic)
{
    bus_enumerator_update_node_info(&be, "myboard", 42);
    proxy.process(signal, 42);
    auto* topic1 = messagebus_find_topic(&bus, "/myboard");
    proxy.process(signal, 42);
    auto* topic2 = messagebus_find_topic(&bus, "/myboard");
    CHECK_EQUAL(topic1, topic2);
}

TEST(ProxyTestGroup, ForwardsCANReceivedMessages)
{
    bus_enumerator_update_node_info(&be, "myboard", 42);
    signal.length = 42.;
    proxy.process(signal, 42);
    auto* topic = messagebus_find_topic(&bus, "/myboard");
    BeaconSignal topic_data;
    auto success = messagebus_topic_read(topic, &topic_data, sizeof topic_data);
    CHECK_TRUE_TEXT(success, "Topic should contain proxied data");

    // Finally, check that the translation function was called
    CHECK_EQUAL(topic_data.range.range.distance, signal.length);
}

struct NonForwardingProxy : public Proxy {
    absl::optional<BeaconSignal> translate(const cvra::proximity_beacon::Signal& /* in */) override
    {
        // Do not forward message by returning an empty optional
        return {};
    }

    NonForwardingProxy(bus_enumerator_t* be, messagebus_t* bus)
        : Proxy(be, bus)
    {
    }
};

TEST(ProxyTestGroup, NonForwardingMessageStillCreatesTopic)
{
    NonForwardingProxy nonproxy{&be, &bus};
    bus_enumerator_update_node_info(&be, "myboard", 42);
    signal.length = 42.;
    nonproxy.process(signal, 42);
    auto* topic = messagebus_find_topic(&bus, "/myboard");
    CHECK_TRUE(topic);
    BeaconSignal topic_data;
    auto success = messagebus_topic_read(topic, &topic_data, sizeof topic_data);
    CHECK_FALSE_TEXT(success, "The topic should not have been published");
}

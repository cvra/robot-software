#include <uavcan/uavcan.hpp>
#include <cvra/sensor/DistanceVL6180X.hpp>

#include <absl/strings/string_view.h>
#include <absl/strings/str_cat.h>
#include "can/UavcanToMessagebusProxy.hpp"

#include <error/error.h>
#include "main.h"
#include "protobuf/sensors.pb.h"
#include <msgbus_protobuf.h>

using ProxyBase = UavcanToMessagebusProxy<cvra::sensor::DistanceVL6180X,
                                          Range,
                                          Range_fields,
                                          Range_msgid>;
struct Proxy : public ProxyBase {
    Proxy(bus_enumerator_t* be, messagebus_t* bus_)
        : ProxyBase(be, bus_)
    {
    }

    std::string
    topic_name(absl::string_view board_name) override
    {
        return absl::StrCat("/distance/", board_name);
    }

    absl::optional<Range> translate(const cvra::sensor::DistanceVL6180X& msg) override
    {
        Range dist;
        dist.distance = msg.distance_mm / 1000.f;
        dist.type = Range_RangeType_LASER;

        return dist;
    }
};

static std::unique_ptr<Proxy> range_proxy;

int sensor_handler_init(uavcan::INode& node, bus_enumerator_t* e)
{
    range_proxy = std::make_unique<Proxy>(e, &bus);

    return range_proxy->start(node);
}

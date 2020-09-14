#include <uavcan/uavcan.hpp>
#include <cvra/actuator/Feedback.hpp>

#include <absl/strings/string_view.h>
#include <absl/strings/str_cat.h>
#include <absl/strings/str_replace.h>
#include "can/UavcanToMessagebusProxy.hpp"

#include <error/error.h>
#include "main.h"
#include "protobuf/actuators.pb.h"
#include <msgbus_protobuf.h>

using Feedback = cvra::actuator::Feedback;
using ProxyBase = UavcanToMessagebusProxy<Feedback,
                                          ActuatorFeedback,
                                          ActuatorFeedback_fields,
                                          ActuatorFeedback_msgid>;
struct ActuatorFeedbackProxy : public ProxyBase {
    ActuatorFeedbackProxy(bus_enumerator_t* be, messagebus_t* bus_)
        : ProxyBase(be, bus_)
    {
    }

    std::string
    topic_name(absl::string_view board_name) override
    {
        return absl::StrReplaceAll(board_name, {{"actuator-", "/actuator/"}});
    }

    absl::optional<ActuatorFeedback> translate(const Feedback& msg) override
    {
        ActuatorFeedback feedback;

        feedback.pressure[0] = msg.pressure[0];
        feedback.pressure[1] = msg.pressure[1];
        feedback.digital_input = msg.digital_input;

        return feedback;
    }
};

static std::unique_ptr<ActuatorFeedbackProxy> proxy;

int actuator_handler_init(uavcan::INode& node, bus_enumerator_t* e)
{
    proxy = std::make_unique<ActuatorFeedbackProxy>(e, &bus);

    return proxy->start(node);
}

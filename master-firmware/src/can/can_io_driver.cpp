#include <ch.h>
#include <uavcan/uavcan.hpp>
#include <cvra/io/ServoPWM.hpp>
#include <cvra/io/DigitalInput.hpp>
#include <error/error.h>
#include <msgbus/messagebus.h>
#include <uavcan_node.h>
#include "main.h"
#include "can_io_driver.h"
#include "protobuf/sensors.pb.h"

static uavcan::LazyConstructor<uavcan::Publisher<cvra::io::ServoPWM>> can_io_pwm_pub;

static void io_input_cb(const uavcan::ReceivedDataStructure<cvra::io::DigitalInput>& msg)
{
    (void)msg;
}

int can_io_init(uavcan::INode& node)
{
    if (!can_io_pwm_pub.isConstructed()) {
        can_io_pwm_pub.construct<uavcan::INode&>(node);
    }

    static uavcan::Subscriber<cvra::io::DigitalInput> io_input_sub(node);
    return io_input_sub.start(io_input_cb);
}

extern "C" {

void can_io_set_pwm(const char* can_io_name, int channel, float pwm)
{
    uint8_t node_id = bus_enumerator_get_can_id(&bus_enumerator, can_io_name);
    if (node_id == BUS_ENUMERATOR_CAN_ID_NOT_SET || node_id == BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        WARNING("Hand %s was not identified correctly [node_id: %d]", can_io_name, node_id);
        return;
    }

    cvra::io::ServoPWM pwm_signals;
    pwm_signals.node_id = node_id;
    pwm_signals.servo_pos[channel] = pwm;
    can_io_pwm_pub->broadcast(pwm_signals);
}
}

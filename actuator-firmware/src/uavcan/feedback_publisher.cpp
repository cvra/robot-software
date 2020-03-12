#include <ch.h>
#include <hal.h>
#include "board.h"
#include "feedback_publisher.h"
#include <cvra/actuator/Feedback.hpp>
#include "analog_input.h"

void feedback_publish(uavcan::INode& node)
{
    static uavcan::Publisher<cvra::actuator::Feedback> pub(node);
    float analog[2];

    analog_input_read(analog);

    cvra::actuator::Feedback msg;

    msg.analog_input[0] = analog[0];
    msg.analog_input[1] = analog[1];

    msg.digital_input = board_digital_input_read();

    pub.broadcast(msg);
}

#include <ch.h>
#include <hal.h>
#include "board.h"
#include "feedback_publisher.h"
#include "pressure_sensor.h"
#include "pressure_sensor_interface.h"
#include <cvra/actuator/Feedback.hpp>
#include "analog_input.h"
#include <error/error.h>

void feedback_publish(uavcan::INode& node)
{
    static uavcan::Publisher<cvra::actuator::Feedback> pub(node);
    // float analog[2];

    // analog_input_read(analog);

    cvra::actuator::Feedback msg;

    // msg.analog_input[0] = analog[0];
    // msg.analog_input[1] = analog[1];

    palClearPad(GPIOA, GPIOA_PRESSURE_RST);
    chThdSleepMilliseconds(10);
    palSetPad(GPIOA, GPIOA_PRESSURE_RST);
    chThdSleepMilliseconds(10);

    mpr_start_measurement(&pressure_sensors[1]);
    chThdSleepMilliseconds(5);
    uint8_t status = mpr_read_status(&pressure_sensors[1]);
    chThdSleepMilliseconds(5);
    uint32_t data = mpr_read_data(&pressure_sensors[1]);
    bool ok = mpr_status_is_error(status) == 0;
    DEBUG("p: %lu, %lx, %02x, %s", data, data, status, ok ? "ok" : "error");

    msg.digital_input = board_digital_input_read();

    pub.broadcast(msg);
}

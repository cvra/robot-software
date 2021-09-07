#include <ch.h>
#include <hal.h>
#include "board.h"
#include "feedback_publisher.h"
#include "pressure_sensor.h"
#include "pressure_sensor_interface.h"
#include <cvra/actuator/Feedback.hpp>
#include "analog_input.h"
#include <error/error.h>

/** Read the pressure of given sensor, returns zero on error*/
static uint32_t read_pressure(int sensor)
{
    // TODO(mspieler): Why is a reset cycle required if the addressed sensor changes?
    palClearPad(GPIOA, GPIOA_PRESSURE_RST);
    chThdSleepMilliseconds(3);
    palSetPad(GPIOA, GPIOA_PRESSURE_RST);
    chThdSleepMilliseconds(3);

    mpr_start_measurement(&pressure_sensors[sensor]);
    chThdSleepMilliseconds(5); // Datasheet says wait at least 5m for conversion  (Datasheet p.15)

    // error condition
    const uint8_t status = mpr_read_status(&pressure_sensors[sensor]);
    if (mpr_status_is_error(status) || mpr_status_is_busy(status)) {
        WARNING("Pressure sensor %d status error: 0x%x", sensor, status);
        return 0;
    }

    chThdSleepMilliseconds(1); // insert delay of min 2us between CS selection (Datasheet p.18)
    const uint32_t pressure_raw = mpr_read_data(&pressure_sensors[sensor]);
    return (uint32_t)mpr_pressure_raw_to_pascal(pressure_raw);
}

bool feedback_publish(uavcan::INode& node)
{
    static uavcan::Publisher<cvra::actuator::Feedback> pub(node);

    float analog[2];
    analog_input_read(analog);

    cvra::actuator::Feedback msg;
    msg.analog_input[0] = analog[0];
    msg.analog_input[1] = analog[1];
    msg.pressure[0] = read_pressure(0);
    msg.pressure[1] = read_pressure(1);
    NOTICE("Pressure %d  %d", msg.pressure[0], msg.pressure[1]);

    msg.digital_input = board_digital_input_read();

    pub.broadcast(msg);

    return msg.pressure[0] != 0 && msg.pressure[1] != 0;
}

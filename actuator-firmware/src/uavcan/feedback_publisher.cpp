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
    float analog[2];

    analog_input_read(analog);

    cvra::actuator::Feedback msg;

    msg.analog_input[0] = analog[0];
    msg.analog_input[1] = analog[1];

    for (int i = 0; i < 2; i++) {
        uint8_t status;

        mpr_start_measurement(&pressure_sensors[i]);
        for (int timeout_ms = 10; timeout_ms > 0; timeout_ms--) {
            status = mpr_read_status(&pressure_sensors[i]);
            if (!mpr_status_is_busy(status)) {
                break;
            }
            chThdSleepMilliseconds(1);
        }

        if (mpr_status_is_error(status) || mpr_status_is_busy(status)) {
            WARNING("Pressure sensor %d status error: 0x%x", i, status);
            continue; // discard reading on error
        }

        // TODO(mspieler): Why is another start measurement required?
        mpr_start_measurement(&pressure_sensors[i]);

        uint32_t pressure = mpr_read_data(&pressure_sensors[i]);
        msg.pressure[i] = mpr_pressure_raw_to_pascal(pressure);
    }

    msg.digital_input = board_digital_input_read();

    pub.broadcast(msg);
}

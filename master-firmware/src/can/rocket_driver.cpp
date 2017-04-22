#include <uavcan/uavcan.hpp>
#include "rocket_driver.h"
#include "uavcan_node.h"
#include <cvra/io/ServoPWM.hpp>
#include <main.h>

#include <error/error.h>

/* TODO: Check which channel it is. */
#define ROCKET_PWM_CHANNEL 0
#define ROCKET_BOARD_NAME "rocket"

uavcan::LazyConstructor<uavcan::Publisher<cvra::io::ServoPWM> > pwm_pub;

float rocket_pos = ROCKET_POS_CLOSE;

static void timer_cb(const uavcan::TimerEvent &event)
{
    (void) event;

    DEBUG("Rocket publish timer fired!");

    /* Get the board id */
    uint8_t node_id = bus_enumerator_get_can_id(&bus_enumerator, ROCKET_BOARD_NAME);
    if (node_id == BUS_ENUMERATOR_CAN_ID_NOT_SET ||
        node_id == BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        WARNING("Rocket launcher id was not found correctly [res: %d]", node_id);
        return;
    }

    /* Send the message. */
    cvra::io::ServoPWM pwm;
    pwm.node_id = node_id;
    pwm.servo_pos[ROCKET_PWM_CHANNEL] = rocket_pos;

    pwm_pub->broadcast(pwm);
}


int rocket_init(uavcan::INode &node)
{
    bus_enumerator_add_node(&bus_enumerator, "rocket", NULL);
    /* We broadcast the setpoint periodically in case there is some packet drop. */
    static uavcan::Timer periodic_timer(node);
    periodic_timer.setCallback(timer_cb);
    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));

    pwm_pub.construct<uavcan::INode &>(node);
    int res = pwm_pub->init();
    if (res < 0) {
        return res;
    }

    return 0;
}

void rocket_set_pos(float pos)
{
    DEBUG("Setting rocket position to %.2f", pos);
    rocket_pos = pos;
}

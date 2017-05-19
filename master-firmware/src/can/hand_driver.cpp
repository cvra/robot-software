#include <ch.h>
#include <uavcan/uavcan.hpp>
#include <cvra/io/ServoPWM.hpp>
#include <cvra/io/DigitalInput.hpp>
#include <error/error.h>
#include <msgbus/messagebus.h>
#include "uavcan_node.h"
#include "main.h"
#include "hand_driver.h"

static const float FINGERS_PULSE_OPEN[2][4] = {
    {0.0013, 0.0013, 0.0013, 0.0013}, {0.0013, 0.0013, 0.0013, 0.0013}
};
static const float FINGERS_PULSE_CLOSED[2][4] = {
    {0.0021, 0.0021, 0.0021, 0.0021}, {0.0021, 0.0021, 0.0021, 0.0021}
};

static messagebus_topic_t left_hand_sensors_topic, right_hand_sensors_topic;
static MUTEX_DECL(left_hand_sensors_topic_lock);
static MUTEX_DECL(right_hand_sensors_topic_lock);
static CONDVAR_DECL(left_hand_sensors_topic_condvar);
static CONDVAR_DECL(right_hand_sensors_topic_condvar);
static hand_sensors_t left_hand_sensors_topic_value, right_hand_sensors_topic_value;

static finger_state_t left_status[4], right_status[4];


static uavcan::LazyConstructor<uavcan::Publisher<cvra::io::ServoPWM> > fingers_pub;

static void digital_input_cb(const uavcan::ReceivedDataStructure<cvra::io::DigitalInput>& msg)
{
    hand_sensors_t val;
    for (int i = 0; i < 4; i++) {
        val.object_present[i] = msg.pin[i];
        val.object_color[i] = msg.pin[i + 4];
    }

    uint8_t nodeId = msg.getSrcNodeID().get();
    const char *name = bus_enumerator_get_str_id(&bus_enumerator, nodeId);

    if (name && !strcmp(name, "right-hand")) {
        messagebus_topic_publish(&right_hand_sensors_topic, &val, sizeof(val));
    } else if (name && !strcmp(name, "left-hand")) {
        messagebus_topic_publish(&left_hand_sensors_topic, &val, sizeof(val));
    } else if (name && !strcmp(name, "rocket")) {
        // Dont do anything
    } else {
        DEBUG("Unknown hand board streaming sensors data %d", nodeId);
    }

    DEBUG("Hand %s: Objects: %d %d %d %d Colors: %d %d %d %d",
            bus_enumerator_get_str_id(&bus_enumerator, nodeId),
            (int)val.object_present[0], (int)val.object_present[1],
            (int)val.object_present[2], (int)val.object_present[3],
            (int)val.object_color[0], (int)val.object_color[1],
            (int)val.object_color[2], (int)val.object_color[3]);

}

static void send_fingers_pwm(const char *name, finger_state_t status[4])
{
    cvra::io::ServoPWM pwm_signals;

    uint8_t node_id = bus_enumerator_get_can_id(&bus_enumerator, name);
    if (node_id == BUS_ENUMERATOR_CAN_ID_NOT_SET || node_id == BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        WARNING("Hand %s was not identified correctly [node_id: %d]", name, node_id);
        return;
    }

    pwm_signals.node_id = node_id;

    for (size_t i = 0; i < 4; i++) {
        if (status[i] == FINGER_CLOSED) {
            pwm_signals.servo_pos[i] = FINGERS_PULSE_CLOSED[0][i];
        } else if (status[i] == FINGER_OPEN) {
            pwm_signals.servo_pos[i] = FINGERS_PULSE_OPEN[0][i];
        } else {
            pwm_signals.servo_pos[i] = 0.8 * FINGERS_PULSE_CLOSED[0][i] + 0.2 *
                                       FINGERS_PULSE_OPEN[0][i];
        }
    }

    fingers_pub->broadcast(pwm_signals);

    DEBUG("Sent to hand \"%s\" node %d, signals: %.3f %.3f %.3f %.3f",
           name, node_id,
           pwm_signals.servo_pos[0],
           pwm_signals.servo_pos[1],
           pwm_signals.servo_pos[2],
           pwm_signals.servo_pos[3]);
}

static void timer_cb(const uavcan::TimerEvent &event)
{
    (void) event;
    send_fingers_pwm("left-hand", left_status);
    send_fingers_pwm("right-hand", right_status);
}


int hand_driver_init(uavcan::INode &node)
{

    for (int i = 0; i < 4; i++) {
        left_status[i] = FINGER_CLOSED;
        right_status[i] = FINGER_CLOSED;
    }

    messagebus_topic_init(&left_hand_sensors_topic,
                          &left_hand_sensors_topic_lock,
                          &left_hand_sensors_topic_condvar,
                          &left_hand_sensors_topic_value,
                          sizeof(left_hand_sensors_topic_value));
    messagebus_topic_init(&right_hand_sensors_topic,
                          &right_hand_sensors_topic_lock,
                          &right_hand_sensors_topic_condvar,
                          &right_hand_sensors_topic_value,
                          sizeof(right_hand_sensors_topic_value));

    messagebus_advertise_topic(&bus, &left_hand_sensors_topic, "/hand/sensors/left");
    messagebus_advertise_topic(&bus, &right_hand_sensors_topic, "/hand/sensors/right");
    static uavcan::Subscriber<cvra::io::DigitalInput> digital_input_sub(node);
    int res = digital_input_sub.start(digital_input_cb);

    if (res != 0) {
        return res;
    }

    if (!fingers_pub.isConstructed()) {
        fingers_pub.construct<uavcan::INode &>(node);
    }

    static uavcan::Timer periodic_timer(node);
    periodic_timer.setCallback(timer_cb);
    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));


    return 0;
}

extern "C" {

void hand_driver_set_fingers(const char *hand_id, finger_state_t* status)
{
    int i;
    finger_state_t *dst;
    if (!strcmp(hand_id, "left-hand")) {
        dst = left_status;
    } else if (!strcmp(hand_id, "right-hand")) {
        dst = right_status;
    } else {
        ERROR("Unknown hand \"%s\"", hand_id);
        return;
    }

    for (i = 0; i < 4; i++) {
        dst[i] = status[i];
    }
}

void hand_driver_set_fingers_float(const char *hand_id, float* signal)
{
    cvra::io::ServoPWM pwm_signals;

    uint8_t node_id = bus_enumerator_get_can_id(&bus_enumerator, hand_id);
    if (node_id == BUS_ENUMERATOR_CAN_ID_NOT_SET || node_id == BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        WARNING("Hand %s was not identified correctly [node_id: %d]", hand_id, node_id);
        return;
    }

    pwm_signals.node_id = node_id;
    for (size_t i = 0; i < 4; i++) {
        pwm_signals.servo_pos[i] = signal[i];
    }

    fingers_pub->broadcast(pwm_signals);

    NOTICE("Sent to hand node %d, signals: %.3f %.3f %.3f %.3f",
           node_id,
           signal[0],
           signal[1],
           signal[2],
           signal[3]);
}

void hand_driver_set_right_fingers(finger_state_t* status)
{
    hand_driver_set_fingers("right-hand", status);
}

void hand_driver_set_left_fingers(finger_state_t* status)
{
    hand_driver_set_fingers("left-hand", status);
}

}

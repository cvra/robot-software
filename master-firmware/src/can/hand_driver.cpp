#include <ch.h>
#include <uavcan/uavcan.hpp>
#include <cvra/io/ServoPWM.hpp>
#include <cvra/io/DigitalInput.hpp>
#include <error/error.h>
#include <msgbus/messagebus.h>
#include "uavcan_node.h"
#include "uavcan_node_private.hpp"
#include "main.h"
#include "hand_driver.h"

static const float FINGER_LEFT_PULSE_OPEN[4] = {0.0013, 0.0013, 0.0013, 0.0013};
static const float FINGER_LEFT_PULSE_CLOSED[4] = {0.0021, 0.0021, 0.0021, 0.0021};

using namespace uavcan_node;

static uavcan::LazyConstructor<uavcan::Publisher<cvra::io::ServoPWM> > fingers_pub;


extern "C" {

int hand_driver_init(void)
{
    // todo: right hand

    static messagebus_topic_t left_hand_sensors_topic;
    static MUTEX_DECL(left_hand_sensors_topic_lock);
    static CONDVAR_DECL(left_hand_sensors_topic_condvar);
    static hand_sensors_t left_hand_sensors_topic_value;

    messagebus_topic_init(&left_hand_sensors_topic,
                          &left_hand_sensors_topic_lock,
                          &left_hand_sensors_topic_condvar,
                          &left_hand_sensors_topic_value,
                          sizeof(left_hand_sensors_topic_value));

    messagebus_advertise_topic(&bus, &left_hand_sensors_topic, "/hand_sensors/left");
    static uavcan::Subscriber<cvra::io::DigitalInput> digital_input_sub(getNode());
    int res = digital_input_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::io::DigitalInput>& msg)
        {
            hand_sensors_t val;
            for (int i = 0; i < 4; i++) {
                val.object_present[i] = msg.pin[i];
                val.object_color[i] = msg.pin[i+4];
            }
            messagebus_topic_publish(&left_hand_sensors_topic, &val, sizeof(val));

            DEBUG("Hands: Objects: %d %d %d %d Colors: %d %d %d %d",
                (int)val.object_present[0], (int)val.object_present[1], (int)val.object_present[2], (int)val.object_present[3],
                (int)val.object_color[0], (int)val.object_color[1], (int)val.object_color[2], (int)val.object_color[3]);
        }
    );
    if (res != 0) {
        ERROR("Failed to subscribe to DigitalInput message, reason %d", res);
        return -1;
    }

    if (!fingers_pub.isConstructed()) {
        fingers_pub.construct<Node &>(getNode());
    }

    return 0;
}

void hand_driver_set_fingers(const char *hand_id, bool open_0, bool open_1, bool open_2, bool open_3)
{
    cvra::io::ServoPWM pwm_signals;

    uint8_t node_id = bus_enumerator_get_can_id(&bus_enumerator, hand_id);
    if (node_id == BUS_ENUMERATOR_CAN_ID_NOT_SET || node_id == BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        WARNING("Hand %s was not identified correctly [node_id: %d]", hand_id, node_id);
        return;
    }

    pwm_signals.node_id = node_id;

    pwm_signals.servo_pos[0] = open_0 ? FINGER_LEFT_PULSE_OPEN[0] : FINGER_LEFT_PULSE_CLOSED[0];
    pwm_signals.servo_pos[1] = open_1 ? FINGER_LEFT_PULSE_OPEN[1] : FINGER_LEFT_PULSE_CLOSED[1];
    pwm_signals.servo_pos[2] = open_2 ? FINGER_LEFT_PULSE_OPEN[2] : FINGER_LEFT_PULSE_CLOSED[2];
    pwm_signals.servo_pos[3] = open_3 ? FINGER_LEFT_PULSE_OPEN[3] : FINGER_LEFT_PULSE_CLOSED[3];

    fingers_pub->broadcast(pwm_signals);

    NOTICE("Sent to hand node %d, signals: %.3f %.3f %.3f %.3f", node_id,
           pwm_signals.servo_pos[0], pwm_signals.servo_pos[1], pwm_signals.servo_pos[2], pwm_signals.servo_pos[3]);
}


void hand_driver_set_fingers_float(const char *hand_id, float signal_0, float signal_1, float signal_2, float signal_3)
{
    cvra::io::ServoPWM pwm_signals;

    uint8_t node_id = bus_enumerator_get_can_id(&bus_enumerator, hand_id);
    if (node_id == BUS_ENUMERATOR_CAN_ID_NOT_SET || node_id == BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        WARNING("Hand %s was not identified correctly [node_id: %d]", hand_id, node_id);
        return;
    }

    pwm_signals.node_id = node_id;
    pwm_signals.servo_pos[0] = signal_0;
    pwm_signals.servo_pos[1] = signal_1;
    pwm_signals.servo_pos[2] = signal_2;
    pwm_signals.servo_pos[3] = signal_3;

    fingers_pub->broadcast(pwm_signals);

    NOTICE("Sent to hand node %d, signals: %.3f %.3f %.3f %.3f", node_id, signal_0, signal_1, signal_2, signal_3);
}

}

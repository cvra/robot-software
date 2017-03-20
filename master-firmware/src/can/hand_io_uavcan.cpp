#include <uavcan/uavcan.hpp>
#include <cvra/io/ServoPWM.hpp>
#include <cvra/io/DigitalInput.hpp>
#include "uavcan_node.hpp"

uavcan::Publisher<cvra::io::ServoPWM> finger_pub;

int hand_init(void)
{
    static messagebus_topic_t left_hand_sensors_topic;
    static MUTEX_DECL(left_hand_sensors_topic_lock);
    static CONDVAR_DECL(left_hand_sensors_topic_condvar);
    static hand_sensors_t left_hand_sensors_topic_value;

    messagebus_topic_init(&left_hand_sensors_topic,
                          &left_hand_sensors_topic_lock,
                          &left_hand_sensors_topic_condvar,
                          &left_hand_sensors_topic_value,
                          sizeof(left_hand_sensors_topic_value));

    messagebus_advertise_topic(&bus, &left_hand_sensors_topic, "/left_hand_sensors");

    // todo: right hand

    static uavcan::Subscriber<cvra::io::DigitalInput> digital_input_sub(getNode());
    res = digital_input_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::io::DigitalInput>& msg)
        {
            messagebus_topic_t *topic;
            uint8_t can_id = msg.getSrcNodeID().get();

            topic = (messagebus_topic_t *)bus_enumerator_get_driver_by_can_id(&bus_enumerator, can_id);
            if (topic != NULL) {
                static hand_sensors_t val;
                memset(&val, 0, sizeof(hand_sensors_t));
                int i;
                for (i = 0; i < 4; i++) {
                    if (msg.pin[i]) {
                        val.object_present[i] = true;
                    }
                    if (msg.pin[i+4]) {
                        val.object_color[i] = true;
                    }
                }
                messagebus_topic_publish(topic, &val, sizeof(val));
            }
        }
    );
    if (res != 0) {
        return -1;
    }

    static uavcan::Publisher<cvra::io::ServoPWM> servo_pwm_pub(getNode());
    const int res = servo_pwm_pub.init();
    if (res < 0) {
        return -1;
    }
}
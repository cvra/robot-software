#include <ch.h>
#include <hal.h>
#include <cvra/odometry/WheelEncoder.hpp>
#include <base/rs_port.h>

#include "WheelEncoder_pub.hpp"

int wheel_encoder_init(uavcan::INode& node)
{
    static uavcan::Publisher<cvra::odometry::WheelEncoder> pub(node);
    static uavcan::Timer periodic_timer(node);
    periodic_timer.setCallback(
        [&](const uavcan::TimerEvent& event) {
            (void)event;
            auto msg = cvra::odometry::WheelEncoder();

            msg.right_encoder_raw = rs_encoder_get_right_ext(NULL);
            msg.left_encoder_raw = rs_encoder_get_left_ext(NULL);

            pub.broadcast(msg);
        });
    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(10));
    return 0;
}

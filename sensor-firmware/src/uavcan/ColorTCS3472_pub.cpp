#include <ch.h>
#include <hal.h>
#include "ColorTCS3472_pub.h"
#include <cvra/sensor/ColorTCS3472.hpp>
#include "main.h"

void color_publish(uavcan::INode &node)
{
    static uavcan::Publisher<cvra::sensor::ColorTCS3472> pub(node);

    uint16_t rgbc[4];
    TCS3472_read_color(&color_sensor, rgbc);

    cvra::sensor::ColorTCS3472 msg = cvra::sensor::ColorTCS3472();

    msg.red = rgbc[0];
    msg.green = rgbc[1];
    msg.blue = rgbc[2];
    msg.clear = rgbc[3];

    pub.broadcast(msg);
}

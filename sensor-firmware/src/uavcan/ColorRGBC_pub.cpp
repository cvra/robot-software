#include <ch.h>
#include <hal.h>
#include "ColorRGBC_pub.h"
#include <cvra/sensor/ColorRGBC.hpp>
#include "main.h"

void color_publish(uavcan::INode& node)
{
    static uavcan::Publisher<cvra::sensor::ColorRGBC> pub(node);

    uint16_t rgbc[4];
    TCS3472_read_color(&color_sensor, rgbc);

    cvra::sensor::ColorRGBC msg = cvra::sensor::ColorRGBC();

    msg.red = rgbc[0];
    msg.green = rgbc[1];
    msg.blue = rgbc[2];
    msg.clear = rgbc[3];

    pub.broadcast(msg);
}

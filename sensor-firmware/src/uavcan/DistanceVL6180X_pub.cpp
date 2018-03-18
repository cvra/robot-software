#include <ch.h>
#include <hal.h>
#include "DistanceVL6180X_pub.h"
#include <cvra/sensor/DistanceVL6180X.hpp>
#include "main.h"

void distance_publish(uavcan::INode &node)
{
    static uavcan::Publisher<cvra::sensor::DistanceVL6180X> pub(node);

    uint8_t distance_mm, status;
    status = vl6180x_measure_distance(&vl6180x_dev, &distance_mm);

    cvra::sensor::DistanceVL6180X msg = cvra::sensor::DistanceVL6180X();

    msg.distance_mm = distance_mm;
    msg.status = status;

    pub.broadcast(msg);
}

#include "ball_sensor.h"

enum ball_sensor_state_t ball_sensor_state(bool previous, bool current)
{
    if (previous == true) {
        return (current == true) ? BALL_SENSOR_HIGH : BALL_SENSOR_FALLING;
    } else /* previous == false */ {
        return (current == false) ? BALL_SENSOR_LOW : BALL_SENSOR_RISING;
    }
}

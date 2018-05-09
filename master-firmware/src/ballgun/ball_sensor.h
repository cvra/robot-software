#ifndef BALL_SENSOR_H
#define BALL_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

enum ball_sensor_state_t {
    BALL_SENSOR_LOW = 0,
    BALL_SENSOR_HIGH,
    BALL_SENSOR_RISING,
    BALL_SENSOR_FALLING,
};

enum ball_sensor_state_t ball_sensor_state(bool previous, bool current);

#ifdef __cplusplus
}
#endif

#endif /* BALL_SENSOR_H */

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

typedef struct {
    bool previous;
    unsigned low_count;
    unsigned high_count;
    unsigned ball_count;

    bool ball_detection_signal; /* Measurement value when ball is detected */
    unsigned low_threshold;     /* Low count threshold to filter pulses */
    unsigned high_threshold;    /* High count threshold to filter pulses */
} ball_sensor_t;

enum ball_sensor_state_t ball_sensor_state(bool previous, bool current);

void ball_sensor_init(ball_sensor_t* sensor);
void ball_sensor_update(ball_sensor_t* sensor, bool measurement);
bool ball_sensor_detect_pulse(ball_sensor_t *sensor, unsigned low_threshold,
                              unsigned high_threshold);
void ball_sensor_manage(ball_sensor_t* sensor, bool measurement);
void ball_sensor_configure(ball_sensor_t *sensor, bool ball_detection_signal,
                           unsigned low_threshold, unsigned high_threshold);

#ifdef __cplusplus
}
#endif

#endif /* BALL_SENSOR_H */

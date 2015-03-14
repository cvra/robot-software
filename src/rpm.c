
#include <math.h>
#include "rpm.h"

static timestamp_t last_crossing;
static float period;

void rpm_barrier_crossing(timestamp_t time)
{
    // mutex around this?
    period = timestamp_duration_s(last_crossing, time);
    last_crossing = time;
}

float rpm_get_position(void)
{
    // mutex?
    timestamp_t time = timestamp_get();
    float delta_t = timestamp_duration_s(last_crossing, time);

    if (delta_t < period) {
        // calculate position assuming constant velocity
        return delta_t / period * 2 * M_PI;
    } else {
        // can't handle non-constant speed -> return 0 'till next barrier crossing
        return 0;
    }

}

float rpm_get_velocity(void)
{
    // mutex?
    timestamp_t time = timestamp_get();
    float delta_t = timestamp_duration_s(last_crossing, time);

    if (delta_t < period) {
        // assuming constant velocity could still be true -> return it
        return 1 / period * 2 * M_PI;
    } else {
        // definitely decelerating -> return maximal possible velocity
        return 1 / delta_t * 2 * M_PI;
    }
}

void rpm_get_velocity_and_position(float *velocity, float *position)
{
    // mutex?
    timestamp_t time = timestamp_get();
    float delta_t = timestamp_duration_s(last_crossing, time);

    if (delta_t < period) {
        // assuming constant velocity
        *velocity = 1 / period * 2 * M_PI;
        *position = delta_t / period * 2 * M_PI;
    } else {
        // definitely decelerating
        *velocity = 1 / delta_t * 2 * M_PI;
        *position = 0;
    }
}

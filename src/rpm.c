
#include <math.h>
#include "rpm.h"
#include <rpm_port.h>

static timestamp_t last_last_crossing;
static timestamp_t last_crossing;

void rpm_barrier_crossing(timestamp_t timestamp)
{
    last_last_crossing = last_crossing;
    last_crossing = timestamp;
}

static float get_period(void)
{
    timestamp_t t1 = last_last_crossing;
    timestamp_t t2 = last_crossing;

    return timestamp_duration_s(t1, t2);
}

float rpm_get_position(void)
{
    timestamp_t t = timestamp_get();
    RPM_LOCK();
    float delta_t = timestamp_duration_s(last_crossing, t);
    float period = get_period();
    RPM_UNLOCK();

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
    timestamp_t t = timestamp_get();
    RPM_LOCK();
    float delta_t = timestamp_duration_s(last_crossing, t);
    float period = get_period();
    RPM_UNLOCK();

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
    timestamp_t t = timestamp_get();
    RPM_LOCK();
    float delta_t = timestamp_duration_s(last_crossing, t);
    float period = get_period();
    RPM_UNLOCK();

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

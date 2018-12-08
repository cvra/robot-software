#include <ch.h>
#include <hal.h>
#include <stdbool.h>
#include <math.h>
#include <timestamp/timestamp.h>
#include <timestamp/timestamp_stm32.h>
#include "proximity_beacon.h"

// max 7 signals per tour
#define SIGNAL_BUFFER_SIZE 2 * 7
#define MAX_SPEED 50 * 2 * M_PI

struct proximity_beacon_signal proximity_beacon_array[SIGNAL_BUFFER_SIZE];
msg_t proximity_beacon_buffer[SIGNAL_BUFFER_SIZE];
mailbox_t proximity_beacon_mbox;
memory_pool_t proximity_beacon_pool;

static float start_angle = 0;
static bool signal_active = false;
static timestamp_t last_last_crossing = 0;
static timestamp_t last_crossing = 1;

static void hall_sensor_cb(void* arg)
{
    (void)arg;
    timestamp_t timestamp = timestamp_get();

    chSysLockFromISR();

    last_last_crossing = last_crossing;
    last_crossing = timestamp;

    chSysUnlockFromISR();
}

static void light_sensor_cb(void* arg)
{
    (void)arg;
    timestamp_t timestamp = timestamp_get();

    int pin = palReadPad(GPIOA, GPIOA_GPIO_I);

    // compute position
    float delta_t = timestamp_duration_s(last_crossing, timestamp);
    float period = timestamp_duration_s(last_last_crossing, last_crossing);
    float position;
    if (delta_t < period) {
        position = delta_t / period * 2 * M_PI;
    } else {
        // can't handle non-constant speed -> set to 0 until next barrier crossing
        position = 0;
    }

    if (!pin) {
        // light signal received
        start_angle = position;
        signal_active = true;
    } else if (signal_active) {
        // light signal lost
        float length = position - start_angle;
        if (length < 0) {
            length += 2 * M_PI;
        }
        signal_active = false;

        chSysLockFromISR();
        struct proximity_beacon_signal* sp;
        sp = chPoolAllocI(&proximity_beacon_pool);
        if (sp) {
            sp->start_angle = start_angle;
            sp->length = length;
            chMBPostI(&proximity_beacon_mbox, (msg_t)sp);
        }
        chSysUnlockFromISR();
    }
}

void proximity_beacon_init(void)
{
    chMBObjectInit(&proximity_beacon_mbox, proximity_beacon_buffer, SIGNAL_BUFFER_SIZE);
    chPoolObjectInit(&proximity_beacon_pool, sizeof(struct proximity_beacon_signal), NULL);
    chPoolLoadArray(&proximity_beacon_pool, proximity_beacon_array, SIGNAL_BUFFER_SIZE);

    // Enable event on hall sensor pin, PB4
    palEnableLineEvent(PAL_LINE(GPIOB, 4U), PAL_EVENT_MODE_FALLING_EDGE);
    palSetLineCallback(PAL_LINE(GPIOB, 4U), hall_sensor_cb, NULL);

    // Enable event on light sensor pin, PA12
    palEnableLineEvent(PAL_LINE(GPIOB, 12U), PAL_EVENT_MODE_BOTH_EDGES);
    palSetLineCallback(PAL_LINE(GPIOB, 12U), light_sensor_cb, NULL);
}

struct proximity_beacon_signal* proximity_beacon_signal_get(void)
{
    struct proximity_beacon_signal* sp;
    msg_t m = chMBFetchTimeout(&proximity_beacon_mbox, (msg_t*)&sp, TIME_IMMEDIATE);
    if (m != MSG_OK) {
        return NULL;
    }
    return sp;
}

void proximity_beacon_signal_delete(struct proximity_beacon_signal* sp)
{
    chPoolFree(&proximity_beacon_pool, sp);
}

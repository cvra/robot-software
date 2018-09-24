#include <ch.h>
#include <hal.h>
#include <stddef.h>
#include <stdlib.h>
#include "error/error.h"
#include "main.h"

#define NB_CHANNELS 5

static char *get_line(void *arg)
{
    static char line_buffer[500];
    static size_t pos = 0;
    size_t i;
    for (i = pos; i < sizeof(line_buffer); i++) {
        int c = chnGetTimeout((BaseChannel *)arg, TIME_INFINITE);
        if (c == STM_TIMEOUT) {
            /* no more data, continue */
            pos = i;
            return NULL;
        }
        if (c == '\n' || c == '\r' || c == '\0') {
            /* line found */
            line_buffer[i] = 0;
            pos = 0;
            return line_buffer;
        } else {
            line_buffer[i] = c;
        }
    }

    /* reset */
    pos = 0;
    return NULL;
}

static int parse_channels(char *line, int *channels)
{
    int i;
    for (i = 0; i < NB_CHANNELS; i++) {
        char *next;
        int val = (int)strtol(line, &next, 10);

        // check valid data
        if ((val < 700 || val > 2300) && val != -1) {
            return -1;
        }
        channels[i] = val;

        // check CSV structure
        if ((i < NB_CHANNELS - 1 && *next != ',') ||
            (i == NB_CHANNELS - 1 && *next != '\0')) {
            return -1;
        }
    }
    return 0;
}

static THD_WORKING_AREA(rc_thread, 500);
static THD_FUNCTION(rc_thread_main, arg)
{
    (void)arg;
    chRegSetThreadName("rc");

    // URAT6 RX
    palSetPadMode(GPIOG, 9, PAL_MODE_ALTERNATE(8));

    static const SerialConfig serial_config = {
        115200,
        0,
        USART_CR2_STOP1_BITS,
        0
    };
    sdStart(&SD6, &serial_config);

    static int channels[NB_CHANNELS];
    char *line;
    while (true) {
        if (!(line = get_line(&SD6))) {
            continue;
        }
        if (parse_channels(line, channels) != 0) {
            NOTICE("%d,%d,%d,%d,%d", channels[0], channels[1], channels[2], channels[3], channels[4]);

            float linear_x, angular_z;
            linear_x = angular_z = 0.0f;

            if (channels[0] != -1) {
                linear_x = 3.0f * ((float) channels[0] - 1500.0f) / 500.0f;
            }
            if (channels[1] != -1) {
                angular_z = 3.0f * ((float) channels[1] - 1500.0f) / 500.0f;
            }

            base_set_speed(&rover_base, linear_x, angular_z);
        }
    }
}

void remote_control_start(void)
{
    chThdCreateStatic(rc_thread, sizeof(rc_thread), NORMALPRIO + 1, rc_thread_main, NULL);
}

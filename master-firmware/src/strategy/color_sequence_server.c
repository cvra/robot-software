#include <ch.h>
#include <hal.h>

#include <error/error.h>

#include "main.h"
#include "priorities.h"

#include "robot_helpers/eurobot2018.h"
#include "color_sequence_server.h"

#define COLOR_SEQUENCE_SERVER_STACKSIZE 512

// Bluetooth UART
#define BLUETOOTH_UART_BAUDRATE 19200
static const SerialConfig bt_uart_config = {
    .speed = BLUETOOTH_UART_BAUDRATE,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    .cr3 = 0
};

static THD_FUNCTION(color_sequence_server_thd, arg) {
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    uint8_t buffer[16];
    sdStart(&SD2, &bt_uart_config);

    static enum cube_color colors[5] = {
        CUBE_UNKNOWN, CUBE_UNKNOWN, CUBE_UNKNOWN, CUBE_UNKNOWN, CUBE_UNKNOWN
    };
    static messagebus_topic_t colors_topic;
    static MUTEX_DECL(colors_topic_lock);
    static CONDVAR_DECL(colors_topic_condvar);
    messagebus_topic_init(&colors_topic, &colors_topic_lock, &colors_topic_condvar, &colors[0], sizeof(colors));
    messagebus_advertise_topic(&bus, &colors_topic, "/colors");

    NOTICE("Color sequence server up, listening over BT");

    while (true) {
        size_t len = sdAsynchronousRead(&SD2, buffer, sizeof(buffer));

        if (len < 3) {
            cube_color_from_string((char*)buffer, 3, colors);
            messagebus_topic_publish(&colors_topic, &colors[0], sizeof(colors));

            DEBUG("Received color: %s %s %s %s %s",
                  cube_color_name(colors[0]),
                  cube_color_name(colors[1]),
                  cube_color_name(colors[2]),
                  cube_color_name(colors[3]),
                  cube_color_name(colors[4]));
        }

        chThdSleepMilliseconds(1000 / COLOR_SEQUENCE_SERVER_FREQUENCY);
    }
}

void color_sequence_server_start()
{
    static THD_WORKING_AREA(color_sequence_server_thd_wa,
                            COLOR_SEQUENCE_SERVER_STACKSIZE);
    chThdCreateStatic(
        color_sequence_server_thd_wa, sizeof(color_sequence_server_thd_wa),
        COLOR_SEQUENCE_SERVER_PRIO, color_sequence_server_thd, NULL);
}

#include <ch.h>
#include <hal.h>

#include <error/error.h>

#include "robot_helpers/eurobot2018.h"

#include "priorities.h"
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

static enum cube_color colors[5] = {
    CUBE_UNKNOWN, CUBE_UNKNOWN, CUBE_UNKNOWN, CUBE_UNKNOWN, CUBE_UNKNOWN
};

static THD_FUNCTION(color_sequence_server_thd, arg) {
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    uint8_t buffer[16];
    sdStart(&SD2, &bt_uart_config);

    NOTICE("Color sequence server up, listening over BT");

    while (true) {
        size_t len = sdAsynchronousRead(&SD2, buffer, sizeof(buffer));

        if (len != 5) { continue; }

        cube_color_from_string((char*)buffer, colors);

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

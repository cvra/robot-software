#include <stdio.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <msgbus_protobuf.h>

#include "base/base_controller.h"
#include "protobuf/ally_position.pb.h"
#include "main.h"

#define BT_SERIAL SD2
static BaseSequentialStream* bt_stream = (BaseSequentialStream*)&BT_SERIAL;

static void position_send_thread(void* p)
{
    (void)p;
    chRegSetThreadName(__FUNCTION__);

    while (true) {
        int16_t x = position_get_x_s16(&robot.pos);
        int16_t y = position_get_y_s16(&robot.pos);
        int16_t a = position_get_a_deg_s16(&robot.pos);

        uint8_t check = (uint8_t)(0xff & (x + y + a));

        chprintf(bt_stream, "pos %hd,%hd,%hd,%02hhx\n", x, y, a, check);

        /* Publish at 10 Hz */
        chThdSleepMilliseconds(100);
    }
}

static size_t read_line(BaseSequentialStream *chp, char *line, size_t len)
{
    char c;
    size_t i = 0;
    while (1) {
        c = (char)streamGet(chp);
        line[i] = c;

        if (c == '\n') {
            line[i+1] = '\0';
            return i;
        }

        i += 1;
        if (i >= len-1) {
            i = 0; // reset
        }
    }
}

static TOPIC_DECL(ally_position_topic, AllyPosition);

static void position_receive_thread(void *p)
{
    (void)p;
    chRegSetThreadName(__FUNCTION__);

    messagebus_advertise_topic(&bus, &ally_position_topic.topic, "/ally_pos");
    while (true) {
        AllyPosition pos;
        int16_t x, y, a;
        uint8_t check;
        static char line[32];
        size_t len;

        len = read_line(bt_stream, line, sizeof(line));

        // check length
        if (len < strlen("pos 0,0,0,XX\n")) {
            continue;
        }

        // example "pos 1200,3400,56,XX\n"
        if (sscanf(line, "pos %hd,%hd,%hd,%02hhx\n", &x, &y, &a, &check) != 4) {
            continue;
        }

        if ((uint8_t)(0xff & (x + y + a)) != check) {
            continue;
        }

        pos.x = (float)x;
        pos.y = (float)y;
        pos.a = ((M_PI * 2) / 360.0) * (float)a;

        // verify checksum
        messagebus_topic_publish(&ally_position_topic.topic, &pos, sizeof(pos));
    }
}

void ally_position_bluetooth_start(void)
{
    static const SerialConfig bt_uart_config = {
        .speed = 19200,
        .cr1 = 0,
        .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN,
        .cr3 = 0};
    sdStart(&BT_SERIAL, &bt_uart_config);

    static THD_WORKING_AREA(send_wa, 1000);
    static THD_WORKING_AREA(receive_wa, 1000);
    chThdCreateStatic(send_wa, sizeof(send_wa), NORMALPRIO, position_send_thread, NULL);
    chThdCreateStatic(receive_wa, sizeof(receive_wa), NORMALPRIO, position_receive_thread, NULL);
}

#include <hal.h>
#include <simplerpc/message.h>
#include <lwip/api.h>
#include <string.h>
#include "interface_panel.h"
#include "priorities.h"
#include "chprintf.h"
#include "rpc_server.h"
#include "unix_timestamp.h"

#define INTERFACE_PANEL_STACKSIZE 1024
#define PC_ERROR_STACKSIZE 512
#define BTN_CNT_RESET_VALUE 3
#define MSGPACK_BUF_LEN     40
#define TOPIC_NAME_BUF_LEN 32

typedef enum {UP, DOWN} btn_state_t;

static void debounce(int *count, bool pressed)
{
    if (pressed) {
        *count = *count - 1;
    } else {
        *count = BTN_CNT_RESET_VALUE;
    }
}

static void transmit_button_state(const btn_state_t *state, const char *button_name)
{
    static uint8_t buffer[MSGPACK_BUF_LEN];
    static char topic_name[TOPIC_NAME_BUF_LEN];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;
    BUTTON_PRESS_PUBLISHER(&server);

    strncpy(topic_name, "interface-panel/", TOPIC_NAME_BUF_LEN);
    strncat(topic_name, button_name, TOPIC_NAME_BUF_LEN);
    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
    cmp_write_bool(&ctx, *state == DOWN);

    message_transmit(buffer, cmp_mem_access_get_pos(&mem),
            &server, BUTTON_PRESS_PUBLISHER_PORT);
}

static void handle_state_change(int count, btn_state_t *state)
{
    if (*state == UP && count == 0) {
        *state = DOWN;
    } else if (*state == DOWN && count > 0) {
        *state = UP;
    }
}

THD_WORKING_AREA(wa_interface_panel, INTERFACE_PANEL_STACKSIZE);
void interface_panel_thread(void *p)

{
    chRegSetThreadName("interface_panel");
    (void) p;
    int yellow_btn_cnt = BTN_CNT_RESET_VALUE;
    int green_btn_cnt = BTN_CNT_RESET_VALUE;
    int start_cnt = BTN_CNT_RESET_VALUE;

    btn_state_t yellow_state = UP;
    btn_state_t green_state = UP;
    btn_state_t start_state = UP;

    while (1) {
        debounce(&yellow_btn_cnt, !palReadPad(GPIOF, GPIOF_BTN_YELLOW));
        debounce(&green_btn_cnt, !palReadPad(GPIOF, GPIOF_BTN_GREEN));
        debounce(&start_cnt, palReadPad(GPIOF, GPIOF_START));

        handle_state_change(yellow_btn_cnt, &yellow_state);
        handle_state_change(green_btn_cnt, &green_state);
        handle_state_change(start_cnt, &start_state);

        transmit_button_state(&yellow_state, "yellow-pressed");
        transmit_button_state(&green_state, "green-pressed");
        transmit_button_state(&start_state, "start");

        chThdSleepMilliseconds(200);
    }
}


THD_WORKING_AREA(wa_pc_error_thread, PC_ERROR_STACKSIZE);
void pc_error_led_thread(void *p)
{
    chRegSetThreadName("pc_error");
    (void) p;
    while (1) {
        /* First error condition: time not synced. */
        unix_timestamp_t origin;
        origin = timestamp_local_us_to_unix(0);
        if (origin.s == 0) {
            palWritePad(GPIOF, GPIOF_LED_PC_ERROR, 1);
            chThdSleepMilliseconds(500 / SLOW_BLINK_RATE);
            palWritePad(GPIOF, GPIOF_LED_PC_ERROR, 0);
            chThdSleepMilliseconds(500 / SLOW_BLINK_RATE);

            /* Skip checking less critical errors. */
            continue;
        } else {
            chThdSleepMilliseconds(1000);
        }
    }
}

void interface_panel_init(void)
{
    chThdCreateStatic(wa_interface_panel,
                      INTERFACE_PANEL_STACKSIZE,
                      INTERFACE_PANEL_PRIO,
                      interface_panel_thread,
                      NULL);

    chThdCreateStatic(wa_pc_error_thread,
                      PC_ERROR_STACKSIZE,
                      INTERFACE_PANEL_PRIO,
                      pc_error_led_thread,
                      NULL);
}


#include <hal.h>
#include <simplerpc/service_call.h>
#include <lwip/api.h>
#include "interface_panel.h"
#include "priorities.h"
#include "usbconf.h"
#include "chprintf.h"
#include "rpc_server.h"

#define INTERFACE_PANEL_STACKSIZE 1024
#define BTN_CNT_RESET_VALUE 10

typedef enum {UP, DOWN} btn_state_t;

THD_WORKING_AREA(wa_interface_panel, INTERFACE_PANEL_STACKSIZE);
msg_t interface_panel_thread(void *p)

{
    (void) p;
    int yellow_btn_cnt = BTN_CNT_RESET_VALUE;
    int green_btn_cnt = BTN_CNT_RESET_VALUE;
    int start_cnt = BTN_CNT_RESET_VALUE;

    btn_state_t yellow_state = UP;
    btn_state_t green_state = UP;
    btn_state_t start_state = UP;

    static uint8_t rpc_output[32];
    static cmp_ctx_t cmp;
    static cmp_mem_access_t mem;
    ip_addr_t server;

    LWIP_GATEWAY(&server);

    while (1) {
        if (!palReadPad(GPIOF, GPIOF_BTN_YELLOW)) {
            yellow_btn_cnt --;
        } else {
            yellow_btn_cnt = BTN_CNT_RESET_VALUE;
        }

        if (!palReadPad(GPIOF, GPIOF_BTN_GREEN)) {
            green_btn_cnt --;
        } else {
            green_btn_cnt = BTN_CNT_RESET_VALUE;
        }

        if (!palReadPad(GPIOF, GPIOF_START)) {
            start_cnt --;
        } else {
            start_cnt = BTN_CNT_RESET_VALUE;
        }

        if (yellow_state == UP && yellow_btn_cnt == 0) {
            yellow_state = DOWN;

            service_call_encode(&cmp, &mem,
                                rpc_output, sizeof rpc_output,
                                "button_pressed", 1);
            cmp_write_str(&cmp, "yellow", 6);
            rpc_transmit(rpc_output, cmp_mem_access_get_pos(&mem),
                         NULL, 0, &server, RPC_SERVER_PORT);
        } else if (yellow_state == DOWN && yellow_btn_cnt > 0) {
            yellow_state = UP;
        }

        if (green_state == UP && green_btn_cnt == 0) {
            green_state = DOWN;
            service_call_encode(&cmp, &mem,
                                rpc_output, sizeof rpc_output,
                                "button_pressed", 1);
            cmp_write_str(&cmp, "green", 6);
            rpc_transmit(rpc_output, cmp_mem_access_get_pos(&mem),
                         NULL, 0, &server, RPC_SERVER_PORT);
        } else if (green_state == DOWN && green_btn_cnt > 0) {
            green_state = UP;
        }

        if (start_state == UP && start_cnt == 0) {
            start_state = DOWN;
            service_call_encode(&cmp, &mem,
                                rpc_output, sizeof rpc_output,
                                "button_pressed", 1);
            cmp_write_str(&cmp, "start", 6);
            rpc_transmit(rpc_output, cmp_mem_access_get_pos(&mem),
                         NULL, 0, &server, RPC_SERVER_PORT);
        } else if (start_state == DOWN && start_cnt > 0) {
            start_state = UP;
        }

        if (green_state == DOWN) {
            palWritePad(GPIOF, GPIOF_LED_GREEN_2, 1);
        } else {
            palWritePad(GPIOF, GPIOF_LED_GREEN_2, 0);
        }

        if (start_state == DOWN) {
            palWritePad(GPIOF, GPIOF_LED_READY, 1);
        } else {
            palWritePad(GPIOF, GPIOF_LED_READY, 0);
        }

        if (yellow_state == DOWN) {
            palWritePad(GPIOF, GPIOF_LED_YELLOW_2, 1);
        } else {
            palWritePad(GPIOF, GPIOF_LED_YELLOW_2, 0);
        }

        chThdSleepMilliseconds(10);
    }
}

void interface_panel_init(void)
{
    chThdCreateStatic(wa_interface_panel,
                      INTERFACE_PANEL_STACKSIZE,
                      INTERFACE_PANEL_PRIO,
                      interface_panel_thread,
                      NULL);
}


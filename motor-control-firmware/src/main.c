#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <blocking_uart_driver.h>
#include "motor_pwm.h"
#include "control.h"
#include "setpoint.h"
#include "analog.h"
#include "encoder.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "serial-datagram/serial_datagram.h"
#include "parameter/parameter.h"
#include "parameter/parameter_msgpack.h"
#include <string.h>
#include <math.h>
#include "bootloader_config.h"
#include "uavcan/uavcan_node.h"
#include "timestamp/timestamp_stm32.h"
#include "index.h"
#include "uart_stream.h"

BaseSequentialStream* ch_stdout;
parameter_namespace_t parameter_root_ns;

static void error_cb(void *arg, const char *id, const char *err)
{
    (void)arg;
    (void)id;
    (void)err;
}

static void parameter_decode_cb(const void *dtgrm, size_t len, void *arg)
{
    int ret = parameter_msgpack_read(&parameter_root_ns, (char*)dtgrm, len, error_cb, arg);
    chprintf(ch_stdout, "ok %d\n", ret);
    // parameter_print(&parameter_root_ns);
}

static THD_WORKING_AREA(parameter_listener_wa, 512);
static THD_FUNCTION(parameter_listener, arg)
{
    static char rcv_buf[200];
    static serial_datagram_rcv_handler_t rcv_handler;
    serial_datagram_rcv_handler_init(&rcv_handler, &rcv_buf, sizeof(rcv_buf), parameter_decode_cb, NULL);
    while (1) {
        char c = chSequentialStreamGet((BaseSequentialStream*)arg);
        int ret = serial_datagram_receive(&rcv_handler, &c, 1);
        if (ret != SERIAL_DATAGRAM_RCV_NO_ERROR) {
            chprintf(ch_stdout, "serial datagram error %d\n", ret);
        }
        (void)ret; // ingore errors
    }
}


void panic_hook(const char* reason)
{
    palClearPad(GPIOA, GPIOA_LED);      // turn on LED (active low)
    motor_pwm_disable();
    static BlockingUARTDriver blocking_uart_stream;
    blocking_uart_init(&blocking_uart_stream, USART3, 115200);
    BaseSequentialStream* uart = (BaseSequentialStream*)&blocking_uart_stream;
    int i;
    while(42){
        for(i = 10000000; i>0; i--){
            __asm__ volatile ("nop");
        }
        chprintf(uart, "Panic: %s\n", reason);
    }
}

void __assert_func(const char *_file, int _line, const char *_func, const char *_expr )
{
    (void)_file;
    (void)_line;
    (void)_func;
    (void)_expr;

    chSysHalt("assertion failed");
    while(1);
}

static int error_level = 0;

static THD_WORKING_AREA(led_thread_wa, 128);
static THD_FUNCTION(led_thread, arg)
{
    (void)arg;
    while (1) {
        if (analog_get_battery_voltage() < 12.f) {
            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);
            palSetPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);

            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);
            palSetPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);

            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);
            palSetPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);

            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(40);
            palSetPad(GPIOA, GPIOA_LED);

            chThdSleepMilliseconds(720);
        } else if (error_level) {
            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(80);
            palSetPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(80);
        } else {
            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(80);
            palSetPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(80);
            palClearPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(80);
            palSetPad(GPIOA, GPIOA_LED);
            chThdSleepMilliseconds(760);
        }
    }
}

int main(void) {
    halInit();
    chSysInit();

    static bootloader_config_t config;
    if (!config_get(&config)) {
        chSysHalt("invalid config");
    }

    chSysLock();
    timestamp_stm32_init();
    chSysUnlock();

    sdStart(&SD3, NULL);
    ch_stdout = (BaseSequentialStream*)&SD3;

    parameter_namespace_declare(&parameter_root_ns, NULL, NULL);

    motor_pwm_setup();
    motor_pwm_enable();
    motor_pwm_set(0.0);

    analog_init();
    encoder_init_primary();
    encoder_init_secondary();

    chprintf(ch_stdout, "boot\n");
    chprintf(ch_stdout, "%s: %d\n", config.board_name, config.ID);


    control_init();

    index_init();

    // uart_stream_start(ch_stdout);
    chThdCreateStatic(parameter_listener_wa, sizeof(parameter_listener_wa), LOWPRIO, parameter_listener, &SD3);
    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), LOWPRIO, led_thread, NULL);

    static struct uavcan_node_arg node_arg;
    node_arg.node_id = config.ID;
    node_arg.node_name = config.board_name;
    can_transceiver_activate();
    uavcan_node_start(&node_arg);


    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

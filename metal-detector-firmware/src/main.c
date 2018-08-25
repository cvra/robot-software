#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <blocking_uart_driver.h>
#include "motor_pwm.h"
#include "analog.h"
#include "metal_detector.h"
#include "parameter/parameter.h"
#include "bootloader_config.h"
#include "uavcan/uavcan_node.h"
#include "timestamp/timestamp_stm32.h"
#include "parameter_listener.h"
#include "main.h"
#include <parameter_flash_storage/parameter_flash_storage.h>

BaseSequentialStream* ch_stdout;
parameter_namespace_t parameter_root_ns;

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

void _unhandled_exception(void)
{
    chSysHalt("unhandled exception");

    while (true) {
        /* wait forever */
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
        }else {
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

void _fini(void)
{
    /* empty */
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
    motor_pwm_set(0.01);

    analog_init();
    metal_detector_init();

    chprintf(ch_stdout, "boot\n");
    chprintf(ch_stdout, "%s: %d\n", config.board_name, config.ID);

    // uart_stream_start(ch_stdout);
    parameter_listener_start(ch_stdout);
    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), LOWPRIO, led_thread, NULL);

    static struct uavcan_node_arg node_arg;
    node_arg.node_id = config.ID;
    node_arg.node_name = config.board_name;
    can_transceiver_activate();
    uavcan_node_start(&node_arg);

    /* Wait for all services to boot, then try to load config. */
    chThdSleepMilliseconds(300);

    if(parameter_flash_storage_load(&parameter_root_ns, &_config_start)) {
        uavcan_init_complete();
    }

    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

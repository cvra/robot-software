#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <blocking_uart_driver.h>
#include "motor_pwm.h"
#include "control.h"
#include "analog.h"
#include "encoder.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "serial-datagram/serial_datagram.h"
#include "parameter/parameter.h"
#include "parameter/parameter_msgpack.h"
#include <string.h>

BaseSequentialStream* stdout;
parameter_namespace_t parameter_root_ns;


static void _stream_sndfn(void *arg, const void *p, size_t len)
{
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
    }
}

static THD_WORKING_AREA(stream_task_wa, 256);
static THD_FUNCTION(stream_task, arg)
{
    (void)arg;
    chRegSetThreadName("print data");
    static char dtgrm[200];
    static cmp_mem_access_t mem;
    static cmp_ctx_t cmp;
    while (1) {
        cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
        bool err = false;
        err = err || !cmp_write_map(&cmp, 3);
        const char *current_id = "motor_current";
        err = err || !cmp_write_str(&cmp, current_id, strlen(current_id));
        err = err || !cmp_write_float(&cmp, analog_get_motor_current());
        const char *motor_voltage_id = "motor_voltage";
        err = err || !cmp_write_str(&cmp, motor_voltage_id, strlen(motor_voltage_id));
        err = err || !cmp_write_float(&cmp, control_get_motor_voltage());
        const char *batt_voltage_id = "batt_voltage";
        err = err || !cmp_write_str(&cmp, batt_voltage_id, strlen(batt_voltage_id));
        err = err || !cmp_write_float(&cmp, analog_get_battery_voltage());
        if (!err) {
            serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_sndfn, stdout);
        }
        chThdSleepMilliseconds(10);
    }
    return 0;
}


static void parameter_decode_cb(const void *dtgrm, size_t len)
{
    int ret = parameter_msgpack_read(&parameter_root_ns, (char*)dtgrm, len);
    chprintf(stdout, "ok %d\n", ret);
    // parameter_print(&parameter_root_ns);
}

static THD_WORKING_AREA(parameter_listener_wa, 512);
static THD_FUNCTION(parameter_listener, arg)
{
    static char rcv_buf[200];
    static serial_datagram_rcv_handler_t rcv_handler;
    serial_datagram_rcv_handler_init(&rcv_handler, &rcv_buf, sizeof(rcv_buf), parameter_decode_cb);
    while (1) {
        char c = chSequentialStreamGet((BaseSequentialStream*)arg);
        int ret = serial_datagram_receive(&rcv_handler, &c, 1);
        (void)ret; // ingore errors
    }
    return 0;
}


void panic_hook(const char* reason)
{
    palClearPad(GPIOA, GPIOA_LED);      // turn on LED (active low)
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


static int error_level = 0;

static THD_WORKING_AREA(led_thread_wa, 128);
static THD_FUNCTION(led_thread, arg)
{
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


    sdStart(&SD3, NULL);
    stdout = (BaseSequentialStream*)&SD3;

    motor_pwm_setup();
    motor_pwm_enable();
    motor_pwm_set(0.0);

    analog_init();
    encoder_init_primary();
    control_start();

    chprintf(stdout, "boot\n");

    // chThdCreateStatic(stream_task_wa, sizeof(stream_task_wa), LOWPRIO, stream_task, NULL);
    chThdCreateStatic(parameter_listener_wa, sizeof(parameter_listener_wa), LOWPRIO, parameter_listener, &SD3);
    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa), LOWPRIO, led_thread, NULL);

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(1000);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(1000);
    }
}

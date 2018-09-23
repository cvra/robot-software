#include <ch.h>
#include <hal.h>
#include <chprintf.h>

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palClearPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(500);
        palSetPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(500);
    }
}

int main(void)
{
    halInit();
    chSysInit();

    static const SerialConfig serial_config =
    {
        115200,
        0,
        USART_CR2_STOP1_BITS,
        0
    };
    sdStart(&SD6, &serial_config);
    palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(8)); // UART6 TX
    palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(8)); // UART6 RX

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

    while (true) {
        chprintf((BaseSequentialStream *) &SD6, "hello world\n");
        chThdSleepMilliseconds(500);
    }
}

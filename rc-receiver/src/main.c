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

icucnt_t last_width, last_period;

static void icuwidthcb(ICUDriver *icup)
{
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    last_width = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup)
{
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    last_period = icuGetPeriodX(icup);
}

static ICUConfig icucfg3 = {
    ICU_INPUT_ACTIVE_HIGH,
    1000000,
    icuwidthcb,
    icuperiodcb,
    NULL,
    ICU_CHANNEL_1,
    0
};

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

    palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(2)); // PA6 TIM3_CH1
    icuStart(&ICUD3, &icucfg3);
    icuStartCapture(&ICUD3);
    icuEnableNotifications(&ICUD3);

    while (true) {
        chprintf((BaseSequentialStream *) &SD6, "pwm width %u us, period %u us\n", last_width, last_period);
        chThdSleepMilliseconds(500);
    }
}

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

struct input_channel_s {
    ICUDriver *icup;
    int pulse_width_us;
} input_channels[] = {
    {&ICUD3, -1}, // 1: PA6 TIM3_CH1
    {&ICUD4, -1}, // 2: PB6 TIM4_CH1
    {&ICUD1, -1}, // 3: PA8 TIM1_CH1
    {&ICUD9, -1}, // 4: PA2 TIM9_CH1
    {&ICUD5, -1}, // 5: PA0 TIM5_CH1
};

#define NB_INPUT_CHANNELS (sizeof(input_channels) / sizeof(input_channels[0]))

int servo_pulse_validate(icucnt_t w)
{
    if (w >= 700 && w <= 2300) {
        return w;
    }
    return -1;
}

static void icu_width_cb(ICUDriver *icup)
{
    unsigned i;
    for (i = 0; i < NB_INPUT_CHANNELS; i++) {
        if (icup == input_channels[i].icup) {
            icucnt_t w = icuGetWidthX(icup);
            input_channels[i].pulse_width_us = servo_pulse_validate(w);
            break;
        }
    }
}

static void icu_overflow_cb(ICUDriver *icup)
{
    unsigned i;
    for (i = 0; i < NB_INPUT_CHANNELS; i++) {
        if (icup == input_channels[i].icup) {
            input_channels[i].pulse_width_us = -1; // invalidate measurement
            break;
        }
    }
}

static ICUConfig icucfg = {
    ICU_INPUT_ACTIVE_HIGH,
    1000000,
    icu_width_cb,
    NULL,
    icu_overflow_cb,
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
    BaseSequentialStream *uart = (BaseSequentialStream *)&SD6;

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

    palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_PULLDOWN);
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_PULLDOWN);
    palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1) | PAL_STM32_PUPDR_PULLDOWN);
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(3) | PAL_STM32_PUPDR_PULLDOWN);
    palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_PULLDOWN);

    unsigned i;
    for (i = 0; i < NB_INPUT_CHANNELS; i++) {
        icuStart(input_channels[i].icup, &icucfg);
        icuStartCapture(input_channels[i].icup);
        icuEnableNotifications(input_channels[i].icup);
    }

    while (true) {
        for (i = 0; i < NB_INPUT_CHANNELS - 1; i++) {
            chprintf(uart, "%d,", input_channels[i].pulse_width_us);
        }
        chprintf(uart, "%d\n", input_channels[i].pulse_width_us);
        chThdSleepMilliseconds(50);
    }
}

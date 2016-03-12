#include "ch.h"
#include "hal.h"
#include "../../../messagebus.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static THD_FUNCTION(button_thread, arg)
{

    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    messagebus_topic_t *button_topic;
    button_topic = messagebus_find_topic_blocking(&bus, "/button_pressed");

    while (true) {
        while (palReadPad(GPIOC, GPIOC_BUTTON)) {
            chThdSleepMilliseconds(100);
        }
        messagebus_topic_publish(button_topic, NULL, 0);
        chThdSleepMilliseconds(100);
        while (!palReadPad(GPIOC, GPIOC_BUTTON)) {
            chThdSleepMilliseconds(100);
        }
    }
}

static THD_FUNCTION(led_thread, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);
    messagebus_topic_t *button_topic;

    button_topic = messagebus_find_topic_blocking(&bus, "/button_pressed");
    while (true) {
        messagebus_topic_wait(button_topic, NULL, 0);
        palTogglePad(GPIOB, GPIOB_LED_GREEN);
    }
}

int main(void) {
    halInit();
    chSysInit();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    messagebus_topic_t button_topic;
    MUTEX_DECL(button_topic_lock);
    CONDVAR_DECL(button_topic_condvar);

    messagebus_topic_init(&button_topic,
                          &button_topic_lock, &button_topic_condvar,
                          NULL, 0);

    messagebus_advertise_topic(&bus, &button_topic, "/button_pressed");

    static THD_WORKING_AREA(button_thread_wa, 128);
    chThdCreateStatic(button_thread_wa, sizeof(button_thread_wa),
                      NORMALPRIO, button_thread, NULL);

    static THD_WORKING_AREA(led_thread_wa, 128);
    chThdCreateStatic(led_thread_wa, sizeof(led_thread_wa),
                      NORMALPRIO, led_thread, NULL);

    while (true) {
        chThdSleepMilliseconds(500);
    }
}

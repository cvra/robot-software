#include <ch.h>
#include <hal.h>
#include "analog.h"

#define METAL_DETECTOR_WAKEUP_EVENT 1


adcsample_t* adc_samples;
size_t nb_samples;

void metal_detector_set_adc_samples(adcsample_t* samples, size_t n)
{
    adc_samples = samples;
    nb_samples = n;
}

static THD_FUNCTION(metal_detector_task, arg)
{
    (void)arg;
    chRegSetThreadName("metal detector");

    static event_listener_t analog_event_listener;
    chEvtRegisterMaskWithFlags(&analog_event, &analog_event_listener,
                               (eventmask_t)METAL_DETECTOR_WAKEUP_EVENT,
                               (eventflags_t)ANALOG_EVENT_CONVERSION_DONE);

    while(TRUE){
        chEvtWaitAny(METAL_DETECTOR_WAKEUP_EVENT);
        chEvtGetAndClearFlags(&analog_event_listener);

        // Detect stuff
    }
}

void metal_detector_init(void)
{
    static THD_WORKING_AREA(metal_detector_task_wa, 256);
    chThdCreateStatic(metal_detector_task_wa, sizeof(metal_detector_task_wa), HIGHPRIO, metal_detector_task, NULL);
}

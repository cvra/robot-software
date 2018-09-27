#ifndef ANALOG_H
#define ANALOG_H

#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANALOG_EVENT_CONVERSION_DONE 1
extern event_source_t analog_event;

#define ANALOG_CONVERSION_FREQUENCY 2002 // frequency of the conversion event

float analog_get_motor_current(void);
float analog_get_battery_voltage(void);
float analog_get_auxiliary(void);
void analog_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_H */

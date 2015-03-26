#ifndef ANALOG_H
#define ANALOG_H

#ifdef __cplusplus
extern "C" {
#endif

float analog_get_motor_current(void);
float analog_get_battery_voltage(void);
float analog_get_auxiliary(void);
void analog_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_H */
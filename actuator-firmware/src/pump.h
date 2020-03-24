#ifndef PUMPS_H
#define PUMPS_H

#ifdef __cplusplus
extern "C" {
#endif

void pump_init(void);

void pump_set_pwm(int index, float duty_cycle);

void pump_set_solenoid(int index, int on);

#ifdef __cplusplus
}
#endif
#endif

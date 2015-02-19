#ifndef CONTROL_H
#define CONTROL_H


#ifdef __cplusplus
extern "C" {
#endif

void control_start(void);

void control_set_current(float c);
float control_get_motor_voltage(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */

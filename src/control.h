#ifndef CONTROL_H
#define CONTROL_H


#ifdef __cplusplus
extern "C" {
#endif

void control_start(void);

void control_update_position_setpoint(float pos);
void control_update_velocity_setpoint(float vel);
void control_update_trajectory_setpoint(float pos, float vel, float acc, float torque);

float control_get_motor_voltage(void);
float control_get_current(void);
float control_get_torque(void);
float control_get_velocity(void);
float control_get_position(void);
float control_get_current_error(void);
float control_get_velocity_error(void);
float control_get_position_error(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */

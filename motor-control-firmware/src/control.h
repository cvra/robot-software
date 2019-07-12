#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <timestamp/timestamp.h>
#include "motor_protection.h"
#include "feedback.h"

extern struct feedback_s control_feedback;
extern motor_protection_t control_motor_protection;

void control_init(void);
void control_start(void);
void control_stop(void);

void control_enable(bool en);

void control_update_position_setpoint(float pos);
void control_update_velocity_setpoint(float vel);
void control_update_torque_setpoint(float torque);
void control_update_trajectory_setpoint(float pos, float vel, float acc, float torque, timestamp_t ts);
void control_update_voltage_setpoint(float voltage);

float control_get_motor_voltage(void);
float control_get_vel_ctrl_out(void);
float control_get_pos_ctrl_out(void);
float control_get_current(void);
float control_get_torque(void);
float control_get_velocity(void);
float control_get_position(void);
float control_get_current_error(void);
float control_get_velocity_error(void);
float control_get_position_error(void);
float control_get_current_setpoint(void);
float control_get_velocity_setpoint(void);
float control_get_position_setpoint(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */

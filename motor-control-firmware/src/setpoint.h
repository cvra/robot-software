#ifndef SETPOINT_H
#define SETPOINT_H

#include <stdbool.h>
#include <timestamp/timestamp.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SETPT_MODE_POS 0
#define SETPT_MODE_VEL 1
#define SETPT_MODE_TORQUE 2
#define SETPT_MODE_TRAJ 3
#define SETPT_MODE_VOLT 4

typedef struct {
    int setpt_mode;
    float target_pos; // valid only in position control mode
    float target_vel; // valid only in velocity control mode
    float setpt_voltage; // valid only in voltage control mode
    float setpt_torque; // torque control setpoint / feedforward torque
    float setpt_pos; // actual position setpoint
    float setpt_vel; // actual velocity setpoint
    float traj_acc; // acceleration of the trajectory mode
    timestamp_t setpt_ts; // timestamp of the last setpoint update (traj. mode)
    float acc_limit; // acceleration limit
    float vel_limit; // velocity limit
    bool periodic_actuator;
} setpoint_interpolator_t;

struct setpoint_s {
    bool position_control_enabled;
    bool velocity_control_enabled;
    float position_setpt;
    float velocity_setpt;
    float feedforward_torque;
};

void setpoint_init(setpoint_interpolator_t* ip);

void setpoint_set_acceleration_limit(setpoint_interpolator_t* ip,
                                     float acc_limit);

void setpoint_set_velocity_limit(setpoint_interpolator_t* ip, float vel_limit);

void setpoint_update_position(setpoint_interpolator_t* ip,
                              float pos,
                              float current_pos,
                              float current_vel,
                              bool periodic);

void setpoint_update_velocity(setpoint_interpolator_t* ip,
                              float vel,
                              float current_vel);

void setpoint_update_torque(setpoint_interpolator_t* ip, float torque);

void setpoint_update_trajectory(setpoint_interpolator_t* ip,
                                float pos,
                                float vel,
                                float acc,
                                float torque,
                                timestamp_t ts);

void setpoint_update_voltage(setpoint_interpolator_t* ip, float voltage);

void setpoint_compute(setpoint_interpolator_t* ip,
                      struct setpoint_s* setpts,
                      float delta_t);

#ifdef __cplusplus
}
#endif

#endif /* SETPOINT_H */

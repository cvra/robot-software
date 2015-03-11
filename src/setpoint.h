#ifndef SETPOINT_H
#define SETPOINT_H

#include "timestamp/timestamp.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    int ctrl_mode;
    float target_pos;   // valid only in position control mode
    float target_vel;   // valid only in velocity control mode
    float setpt_torque; // torque control setpoint / feedforward torque
    float setpt_pos;    // actual position setpoint
    float setpt_vel;    // actual velocity setpoint
    float traj_acc;     // acceleration of the trajectory mode
    timestamp_t setpt_ts; // timestamp of the last setpoint update (traj. mode)
} setpoint_t;


void setpoint_init(setpoint_t *spt);
void setpoint_update_position(setpoint_t *spt, float pos);
void setpoint_update_velocity(setpoint_t *spt, float vel);
void setpoint_update_torque(setpoint_t *spt, float torque);
void setpoint_update_trajectory(setpoint_t *spt, float pos, float vel,
                                float acc, float torque, timestamp_t ts);


#ifdef __cplusplus
}
#endif

#endif /* SETPOINT_H */
#include <math.h>
#include "setpoint.h"

#define CTRL_MODE_POS      0
#define CTRL_MODE_VEL      1
#define CTRL_MODE_TORQUE   2
#define CTRL_MODE_TRAJ     3


static float pos_setpt_interpolation(float pos, float vel, float acc, float delta_t)
{
    return pos + vel * delta_t + acc / 2 * delta_t * delta_t;
}

static float vel_setpt_interpolation(float vel, float acc, float delta_t)
{
    return vel + acc * delta_t;
}

// returns acceleration to be applied for the next delta_t
static float vel_ramp(float pos, float vel, float target_pos, float delta_t, float max_vel, float max_acc)
{
    float breaking_dist = vel * vel / 2 / max_acc;  // distance needed to break with max_acc
    float next_error = pos + vel * delta_t + max_acc / 2 / delta_t / delta_t - target_pos;
    float next_error_sign = copysignf(1.0, next_error);

    if (next_error_sign == copysignf(1.0, vel)) {
        if (fabs(next_error) <= breaking_dist) {
            // too close to break (or just close enough)
            return next_error_sign * max_acc;
        } else if (fabs(vel) >= max_vel) {
            // maximal velocity reched -> just cruise
            return 0;
        } else {
            // we can go faster
            return - next_error_sign * max_acc;
        }
    } else {
        // driving away from target position -> turn around
        return - next_error_sign * max_acc;
    }
}


void setpoint_init(setpoint_t *spt)
{
    spt->ctrl_mode = SETPT_MODE_TORQUE
}


void control_update_position_setpoint(float pos)
{
    if (setpt_mode == SETPT_MODE_TORQUE) {
        setpt_pos = pos; // todo replace by current position
        setpt_vel = 0; // todo replace by current velocity
    }
    if (setpt_mode == SETPT_MODE_VEL) {
        setpt_pos = pos; // todo see above
    }
    setpt_mode = SETPT_MODE_POS;
    target_pos = pos;
}

void control_update_velocity_setpoint(float vel)
{
    if (setpt_mode == SETPT_MODE_TORQUE) {
        setpt_vel = 0; // todo see above
    }
    setpt_mode = SETPT_MODE_VEL;
    target_vel = vel;
}

void control_update_torque_setpoint(float torque)
{
    setpt_mode = SETPT_MODE_TORQUE;
    setpt_torque = torque;
}

void control_update_trajectory_setpoint(float pos, float vel, float acc, float torque, timestamp_t ts)
{
    setpt_mode = SETPT_MODE_TRAJ;
    setpt_pos = pos;
    setpt_vel = vel;
    traj_acc = acc;
    setpt_torque = torque;
    setpt_ts = ts;
}


    if (setpt_mode == SETPT_MODE_TRAJ) {
        timestamp_t now = timestamp_get();
        float delta_t = timestamp_duration_s(setpt_ts, now);
        ctrl.position_control_enabled = true;
        ctrl.velocity_control_enabled = true;
        ctrl.position_setpt = pos_setpt_interpolation(setpt_pos, setpt_vel, traj_acc, delta_t);
        ctrl.velocity_setpt = vel_setpt_interpolation(setpt_vel, traj_acc, delta_t);
        ctrl.feedforward_torque = setpt_torque;

    } else if (setpt_mode == SETPT_MODE_TORQUE) {
        ctrl.position_control_enabled = false;
        ctrl.velocity_control_enabled = false;
        ctrl.feedforward_torque = setpt_torque;

    } else if (setpt_mode == SETPT_MODE_VEL) {
        ctrl.position_control_enabled = false;
        ctrl.velocity_control_enabled = true;
        float delta_vel = target_vel - setpt_vel;
        setpt_vel += filter_limit_sym(delta_vel, delta_t * acc_max);
        ctrl.velocity_setpt = setpt_vel;
        ctrl.feedforward_torque = 0;

    } else { // setpt_mode == SETPT_MODE_POS
        ctrl.position_control_enabled = true;
        ctrl.velocity_control_enabled = true;
        float acc = vel_ramp(setpt_pos, setpt_vel, target_pos, delta_t, ctrl.velocity_limit, acc_max);
        float pos = pos_setpt_interpolation(setpt_pos, setpt_vel, acc, delta_t);
        float vel = vel_setpt_interpolation(setpt_vel, acc, delta_t);
        ctrl.position_setpt = setpt_pos = pos;
        ctrl.velocity_setpt = setpt_vel = vel;
        ctrl.feedforward_torque = 0;
    }
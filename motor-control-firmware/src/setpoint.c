#include <math.h>
#include <filter/basic.h>
#include "setpoint.h"
#include "pid_cascade.h"

static float pos_setpt_interpolation(float pos, float vel, float acc, float delta_t)
{
    return pos + vel * delta_t + acc / 2 * delta_t * delta_t;
}

static float vel_setpt_interpolation(float vel, float acc, float delta_t)
{
    return vel + acc * delta_t;
}

// returns acceleration to be applied for the next delta_t
float vel_ramp(float pos, float vel, float target_pos, float delta_t, float max_vel, float max_acc, bool periodic)
{
    float breaking_dist = vel * vel / 2 / max_acc; // distance needed to break with max_acc
    float error = pos - target_pos;
    if (periodic) {
        error = periodic_error(error);
    }
    float error_sign = copysignf(1.0, error);

    if (error_sign != copysignf(1.0, vel)) { // decreasing error with current vel
        if (fabs(error) <= breaking_dist || fabs(error) <= max_acc * delta_t * delta_t / 2) {
            // too close to break (or just close enough)
            return -filter_limit_sym(vel / delta_t, max_acc);
        } else if (fabs(vel) >= max_vel) {
            // maximal velocity reached -> just cruise
            return 0;
        } else {
            // we can go faster
            return -error_sign * max_acc;
        }
    } else {
        // driving away from target position -> turn around
        if (fabs(error) <= max_acc * delta_t * delta_t / 2) {
            return -filter_limit_sym(vel / delta_t, max_acc);
        } else {
            return -error_sign * max_acc;
        }
    }
}

void setpoint_init(setpoint_interpolator_t* ip)
{
    ip->setpt_mode = SETPT_MODE_TORQUE;
    ip->setpt_torque = 0;
    ip->acc_limit = 0;
    ip->vel_limit = 0;
}

void setpoint_set_acceleration_limit(setpoint_interpolator_t* ip,
                                     float acc_limit)
{
    ip->acc_limit = acc_limit;
}

void setpoint_set_velocity_limit(setpoint_interpolator_t* ip, float vel_limit)
{
    ip->vel_limit = vel_limit;
}

void setpoint_update_position(setpoint_interpolator_t* ip,
                              float pos,
                              float current_pos,
                              float current_vel,
                              bool periodic)
{
    if (ip->setpt_mode == SETPT_MODE_TORQUE) {
        ip->setpt_pos = current_pos;
        ip->setpt_vel = current_vel;
    }
    if (ip->setpt_mode == SETPT_MODE_VEL) {
        ip->setpt_pos = current_pos;
    }
    ip->setpt_mode = SETPT_MODE_POS;
    ip->target_pos = pos;
    ip->periodic_actuator = periodic;
}

void setpoint_update_velocity(setpoint_interpolator_t* ip,
                              float vel,
                              float current_vel)
{
    if (ip->setpt_mode == SETPT_MODE_TORQUE) {
        ip->setpt_vel = current_vel;
    }
    ip->setpt_mode = SETPT_MODE_VEL;
    ip->target_vel = vel;
}

void setpoint_update_torque(setpoint_interpolator_t* ip, float torque)
{
    ip->setpt_mode = SETPT_MODE_TORQUE;
    ip->setpt_torque = torque;
}

void setpoint_update_trajectory(setpoint_interpolator_t* ip,
                                float pos,
                                float vel,
                                float acc,
                                float torque,
                                timestamp_t ts)
{
    ip->setpt_mode = SETPT_MODE_TRAJ;
    ip->setpt_pos = pos;
    ip->setpt_vel = vel;
    ip->traj_acc = acc;
    ip->setpt_torque = torque;
    ip->setpt_ts = ts;
}

void setpoint_update_voltage(setpoint_interpolator_t* ip, float voltage)
{
    ip->setpt_mode = SETPT_MODE_VOLT;
    ip->setpt_voltage = voltage;
}

void setpoint_compute(setpoint_interpolator_t* ip,
                      struct setpoint_s* setpts,
                      float delta_t)
{
    if (ip->setpt_mode == SETPT_MODE_TRAJ) {
        timestamp_t now = timestamp_get();
        float ip_delta_t = timestamp_duration_s(ip->setpt_ts, now); // interpolation delta_t
        setpts->position_control_enabled = true;
        setpts->velocity_control_enabled = true;
        setpts->position_setpt = pos_setpt_interpolation(ip->setpt_pos,
                                                         ip->setpt_vel,
                                                         ip->traj_acc,
                                                         ip_delta_t);

        setpts->velocity_setpt = vel_setpt_interpolation(ip->setpt_vel,
                                                         ip->traj_acc,
                                                         ip_delta_t);
        setpts->feedforward_torque = ip->setpt_torque;

    } else if (ip->setpt_mode == SETPT_MODE_TORQUE) {
        setpts->position_control_enabled = false;
        setpts->velocity_control_enabled = false;
        setpts->feedforward_torque = ip->setpt_torque;

    } else if (ip->setpt_mode == SETPT_MODE_VEL) {
        setpts->position_control_enabled = false;
        setpts->velocity_control_enabled = true;
        float delta_vel = ip->target_vel - ip->setpt_vel;
        ip->setpt_vel += filter_limit_sym(delta_vel, delta_t * ip->acc_limit);
        setpts->velocity_setpt = ip->setpt_vel;
        setpts->feedforward_torque = 0;

    } else if (ip->setpt_mode == SETPT_MODE_POS) {
        setpts->position_control_enabled = true;
        setpts->velocity_control_enabled = true;
        float acc = vel_ramp(ip->setpt_pos,
                             ip->setpt_vel,
                             ip->target_pos,
                             delta_t,
                             ip->vel_limit,
                             ip->acc_limit,
                             ip->periodic_actuator);

        float pos = pos_setpt_interpolation(ip->setpt_pos,
                                            ip->setpt_vel,
                                            acc,
                                            delta_t);

        float vel = vel_setpt_interpolation(ip->setpt_vel, acc, delta_t);
        setpts->position_setpt = ip->setpt_pos = pos;
        setpts->velocity_setpt = ip->setpt_vel = vel;
        setpts->feedforward_torque = 0;
    } else {
        // setpt mode voltage
    }
}

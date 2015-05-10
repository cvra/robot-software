#include <ch.h>
#include <string.h>
#include "motor_driver.h"

static void pid_register(struct pid_parameter_s *pid,
                         parameter_namespace_t *parent,
                         const char *name)
{

    parameter_namespace_declare(&pid->root, parent, name);
    parameter_scalar_declare(&pid->kp, &pid->root, "kp");
    parameter_scalar_declare(&pid->ki, &pid->root, "ki");
    parameter_scalar_declare(&pid->kd, &pid->root, "kd");
    parameter_scalar_declare(&pid->ilimit, &pid->root, "ilimit");
}

void motor_driver_init(motor_driver_t *d,
                       const char *actuator_id,
                       parameter_namespace_t *ns,
                       memory_pool_t *traj_buffer_pool)
{
    chBSemObjectInit(&d->lock, false);
    d->traj_buffer_pool = traj_buffer_pool;

    strncpy(d->id, actuator_id, MOTOR_ID_MAX_LEN);
    d->id[MOTOR_ID_MAX_LEN] = '\0';

    d->can_id = CAN_ID_NOT_SET;

    d->control_mode = MOTOR_CONTROL_MODE_DISABLED;

    d->can_driver = NULL;

    parameter_namespace_declare(&d->config.root, ns, d->id);
    parameter_namespace_declare(&d->config.pid_root,&d->config.root, "pid");
    pid_register(&d->config.position_pid, &d->config.pid_root, "position");
    pid_register(&d->config.velocity_pid, &d->config.pid_root, "velocity");
    pid_register(&d->config.current_pid, &d->config.pid_root, "current");
}

const char *motor_driver_get_id(motor_driver_t *d)
{
    return d->id;
}

// must be called with the locked driver
static void free_trajectory_buffer(motor_driver_t *d)
{
    if (d->control_mode == MOTOR_CONTROL_MODE_TRAJECTORY
        && d->setpt.trajectory != NULL) {
        chPoolFree(d->traj_buffer_pool, d->setpt.trajectory);
        d->setpt.trajectory = NULL;
    }
}

void motor_driver_set_position(motor_driver_t *d, float position)
{
    chBSemWait(&d->lock);
    free_trajectory_buffer(d);
    d->control_mode = MOTOR_CONTROL_MODE_POSITION;
    d->setpt.position = position;
    chBSemSignal(&d->lock);
}

void motor_driver_set_velocity(motor_driver_t *d, float velocity)
{
    chBSemWait(&d->lock);
    free_trajectory_buffer(d);
    d->control_mode = MOTOR_CONTROL_MODE_VELOCITY;
    d->setpt.velocity = velocity;
    chBSemSignal(&d->lock);
}

void motor_driver_set_torque(motor_driver_t *d, float torque)
{
    chBSemWait(&d->lock);
    free_trajectory_buffer(d);
    d->control_mode = MOTOR_CONTROL_MODE_TORQUE;
    d->setpt.torque = torque;
    chBSemSignal(&d->lock);
}

void motor_driver_set_voltage(motor_driver_t *d, float voltage)
{
    chBSemWait(&d->lock);
    free_trajectory_buffer(d);
    d->control_mode = MOTOR_CONTROL_MODE_VOLTAGE;
    d->setpt.voltage = voltage;
    chBSemSignal(&d->lock);
}

void motor_driver_update_trajectory(motor_driver_t *d, trajectory_chunk_t *traj)
{
    chBSemWait(&d->lock);
    if (d->control_mode != MOTOR_CONTROL_MODE_TRAJECTORY) {
        d->setpt.trajectory = chPoolAlloc(d->traj_buffer_pool);
        // todo init
        d->control_mode = MOTOR_CONTROL_MODE_TRAJECTORY;
    }
    if (d->setpt.trajectory != NULL) {
        // todo trajecory merge ...
    } else {
        chSysHalt("motor driver out of memory (trajectory buffer allocation)");
    }
    chBSemSignal(&d->lock);
}

void motor_driver_disable(motor_driver_t *d)
{
    chBSemWait(&d->lock);
    free_trajectory_buffer(d);
    d->control_mode = MOTOR_CONTROL_MODE_DISABLED;
    chBSemSignal(&d->lock);
}


void motor_driver_set_can_id(motor_driver_t *d, int can_id)
{
    d->can_id = can_id;
}

int motor_driver_get_can_id(motor_driver_t *d)
{
    return d->can_id;
}


void motor_driver_lock(motor_driver_t *d)
{
    chBSemWait(&d->lock);
}

void motor_driver_unlock(motor_driver_t *d)
{
    chBSemSignal(&d->lock);
}


int motor_driver_get_control_mode(motor_driver_t *d)
{
    return d->control_mode;
}

float motor_driver_get_position_setpt(motor_driver_t *d)
{
    if (d->control_mode != MOTOR_CONTROL_MODE_POSITION) {
        chSysHalt("motor driver get position wrong setpt mode");
    }
    return d->setpt.position;
}

float motor_driver_get_velocity_setpt(motor_driver_t *d)
{
    if (d->control_mode != MOTOR_CONTROL_MODE_VELOCITY) {
        chSysHalt("motor driver get velocity wrong setpt mode");
    }
    return d->setpt.velocity;
}

float motor_driver_get_torque_setpt(motor_driver_t *d)
{
    if (d->control_mode != MOTOR_CONTROL_MODE_TORQUE) {
        chSysHalt("motor driver get torque wrong setpt mode");
    }
    return d->setpt.torque;
}

float motor_driver_get_voltage_setpt(motor_driver_t *d)
{
    if (d->control_mode != MOTOR_CONTROL_MODE_VOLTAGE) {
        chSysHalt("motor driver get voltage wrong setpt mode");
    }
    return d->setpt.voltage;
}

void motor_driver_get_trajectory_point(motor_driver_t *d,
                                       unix_timestamp_t timestamp,
                                       float *position,
                                       float *velocity,
                                       float *acceleration,
                                       float *torque)
{
    if (d->control_mode != MOTOR_CONTROL_MODE_TRAJECTORY) {
        chSysHalt("motor driver get trajectory wrong setpt mode");
    }
    // todo copy pt
}

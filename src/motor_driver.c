#include <ch.h>
#include <string.h>
#include "motor_driver.h"


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
    parameter_namespace_declare(&d->motor_ns, ns, d->id);
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
    d->setpt.postition = position;
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

void motor_driver_update_trajectory(motor_driver_t *d, trajectory_t *traj)
{
    chBSemWait(&d->lock);
    if (d->control_mode != MOTOR_CONTROL_MODE_TRAJECTORY) {
        d->setpt.trajectory = chPoolAlloc(d->traj_buffer_pool);
        d->control_mode = MOTOR_CONTROL_MODE_TRAJECTORY;
    }
    if (d->setpt.trajectory != NULL) {
        // todo trajecory merge ...
    } else {
        chSysHalt("motor driver out of memory (trajectory buffer allocation)");
    }
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

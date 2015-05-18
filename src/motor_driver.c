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
                       memory_pool_t *traj_buffer_pool,
                       memory_pool_t *traj_buffer_points_pool,
                       int traj_buffer_nb_points)
{
    chBSemObjectInit(&d->lock, false);
    d->traj_buffer_pool = traj_buffer_pool;
    d->traj_buffer_points_pool = traj_buffer_points_pool;
    d->traj_buffer_nb_points = traj_buffer_nb_points;

    strncpy(d->id, actuator_id, MOTOR_ID_MAX_LEN);
    d->id[MOTOR_ID_MAX_LEN] = '\0';

    d->can_id = CAN_ID_NOT_SET;

    d->control_mode = MOTOR_CONTROL_MODE_DISABLED;

    d->can_driver = NULL;

    parameter_namespace_declare(&d->config.root, ns, d->id);
    parameter_namespace_declare(&d->config.control, &d->config.root, "control");
    pid_register(&d->config.position_pid, &d->config.control, "position");
    pid_register(&d->config.velocity_pid, &d->config.control, "velocity");
    pid_register(&d->config.current_pid, &d->config.control, "current");
    parameter_scalar_declare(&d->config.torque_limit, &d->config.control, "torque_limit");
    parameter_scalar_declare(&d->config.velocity_limit, &d->config.control, "velocity_limit");
    parameter_scalar_declare(&d->config.acceleration_limit, &d->config.control, "acceleration_limit");
    parameter_scalar_declare(&d->config.low_batt_th, &d->config.control, "low_batt_th");

    parameter_namespace_declare(&d->config.thermal, &d->config.root, "thermal");
    parameter_scalar_declare(&d->config.thermal_capacity, &d->config.thermal, "capacity");
    parameter_scalar_declare(&d->config.thermal_resistance, &d->config.thermal, "resistance");
    parameter_scalar_declare(&d->config.thermal_current_gain, &d->config.thermal, "current_gain");
    parameter_scalar_declare(&d->config.max_temperature, &d->config.thermal, "max_temperature");

    parameter_namespace_declare(&d->config.motor, &d->config.root, "motor");
    parameter_scalar_declare(&d->config.torque_constant, &d->config.motor, "torque_constant");
    parameter_integer_declare(&d->config.transmission_ratio_p, &d->config.motor, "transmission_ratio_p");
    parameter_integer_declare(&d->config.transmission_ratio_q, &d->config.motor, "transmission_ratio_q");
    parameter_integer_declare(&d->config.motor_encoder_steps_per_revolution, &d->config.motor, "motor_encoder_steps_per_revolution");
    parameter_integer_declare(&d->config.second_encoder_steps_per_revolution, &d->config.motor, "second_encoder_steps_per_revolution");
    parameter_scalar_declare(&d->config.potentiometer_gain, &d->config.motor, "potentiometer_gain");
    parameter_integer_declare(&d->config.mode, &d->config.motor, "mode"); // todo

    parameter_namespace_declare(&d->config.stream, &d->config.root, "stream");
    parameter_scalar_declare(&d->config.current_pid_stream, &d->config.stream, "current_pid");
    parameter_scalar_declare(&d->config.velocity_pid_stream, &d->config.stream, "velocity_pid");
    parameter_scalar_declare(&d->config.position_pid_stream, &d->config.stream, "position_pid");
    parameter_scalar_declare(&d->config.index_stream, &d->config.stream, "index");
    parameter_scalar_declare(&d->config.encoder_pos_stream, &d->config.stream, "encoder_pos");
    parameter_scalar_declare(&d->config.motor_pos_stream, &d->config.stream, "motor_pos");
    parameter_scalar_declare(&d->config.motor_torque_stream, &d->config.stream, "motor_torque");
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
        float *traj_mem = chPoolAlloc(d->traj_buffer_points_pool);
        if (d->setpt.trajectory == NULL || traj_mem == NULL) {
            chSysHalt("motor driver out of memory (trajectory buffer allocation)");
        }
        trajectory_init(d->setpt.trajectory, traj_mem, d->traj_buffer_nb_points, 4, traj->sampling_time_us);
        d->control_mode = MOTOR_CONTROL_MODE_TRAJECTORY;
    }
    if (trajectory_apply_chunk(d->setpt.trajectory, traj) != 0) {
        chSysHalt("trajectory apply chunk failed");
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
                                       int64_t timestamp_us,
                                       float *position,
                                       float *velocity,
                                       float *acceleration,
                                       float *torque)
{
    if (d->control_mode != MOTOR_CONTROL_MODE_TRAJECTORY) {
        chSysHalt("motor driver get trajectory wrong setpt mode");
    }
    float *t = trajectory_read(d->setpt.trajectory, timestamp_us);
    if (t == NULL) {
        chSysHalt("control error"); // todo
    }
    *position = t[0];
    *velocity = t[1];
    *acceleration = t[2];
    *torque = t[3];
}

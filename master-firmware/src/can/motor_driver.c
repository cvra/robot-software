#include <ch.h>
#include <string.h>
#include "motor_driver.h"

#include <error/error.h>
#include "timestamp/timestamp.h"

static void pid_register(struct pid_parameter_s *pid,
                         parameter_namespace_t *parent,
                         const char *name)
{

    parameter_namespace_declare(&pid->root, parent, name);
    parameter_scalar_declare_with_default(&pid->kp, &pid->root, "kp", 0);
    parameter_scalar_declare_with_default(&pid->ki, &pid->root, "ki", 0);
    parameter_scalar_declare_with_default(&pid->kd, &pid->root, "kd", 0);
    parameter_scalar_declare_with_default(&pid->ilimit, &pid->root, "ilimit", 0);
}

void motor_driver_init(motor_driver_t *d,
                       const char *actuator_id,
                       parameter_namespace_t *ns)
{
    chBSemObjectInit(&d->lock, false);

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
    parameter_scalar_declare_with_default(&d->config.torque_limit, &d->config.control, "torque_limit", 0);
    parameter_scalar_declare_with_default(&d->config.velocity_limit, &d->config.control, "velocity_limit", 0);
    parameter_scalar_declare_with_default(&d->config.acceleration_limit, &d->config.control, "acceleration_limit", 0);
    parameter_scalar_declare_with_default(&d->config.low_batt_th, &d->config.control, "low_batt_th", 12);

    parameter_namespace_declare(&d->config.thermal, &d->config.root, "thermal");
    parameter_scalar_declare_with_default(&d->config.thermal_capacity, &d->config.thermal, "capacity", 0.);
    parameter_scalar_declare_with_default(&d->config.thermal_resistance, &d->config.thermal, "resistance", 0.);
    parameter_scalar_declare_with_default(&d->config.thermal_current_gain, &d->config.thermal, "current_gain", 0.);
    parameter_scalar_declare_with_default(&d->config.max_temperature, &d->config.thermal, "max_temperature", 0.);

    parameter_namespace_declare(&d->config.motor, &d->config.root, "motor");
    parameter_scalar_declare_with_default(&d->config.torque_constant, &d->config.motor, "torque_constant", 1);
    parameter_integer_declare_with_default(&d->config.transmission_ratio_p, &d->config.motor, "transmission_ratio_p", 1);
    parameter_integer_declare_with_default(&d->config.transmission_ratio_q, &d->config.motor, "transmission_ratio_q", 1);
    parameter_integer_declare_with_default(&d->config.motor_encoder_steps_per_revolution, &d->config.motor, "motor_encoder_steps_per_revolution", 4096);
    parameter_integer_declare_with_default(&d->config.second_encoder_steps_per_revolution, &d->config.motor, "second_encoder_steps_per_revolution", 4096);
    parameter_scalar_declare_with_default(&d->config.potentiometer_gain, &d->config.motor, "potentiometer_gain", 1);
    parameter_integer_declare_with_default(&d->config.mode, &d->config.motor, "mode", 4); // todo

    parameter_namespace_declare(&d->config.stream, &d->config.root, "stream");
    parameter_scalar_declare_with_default(&d->config.current_pid_stream, &d->config.stream, "current_pid", 0);
    parameter_scalar_declare_with_default(&d->config.velocity_pid_stream, &d->config.stream, "velocity_pid", 0);
    parameter_scalar_declare_with_default(&d->config.position_pid_stream, &d->config.stream, "position_pid", 0);
    parameter_scalar_declare_with_default(&d->config.index_stream, &d->config.stream, "index", 0);
    parameter_scalar_declare_with_default(&d->config.encoder_pos_stream, &d->config.stream, "encoder_pos", 0);
    parameter_scalar_declare_with_default(&d->config.motor_pos_stream, &d->config.stream, "motor_pos", 0);
    parameter_scalar_declare_with_default(&d->config.motor_torque_stream, &d->config.stream, "motor_torque", 0);

    d->stream.change_status = 0;
}

const char *motor_driver_get_id(motor_driver_t *d)
{
    return d->id;
}

void motor_driver_set_position(motor_driver_t *d, float position)
{
    chBSemWait(&d->lock);
    d->control_mode = MOTOR_CONTROL_MODE_POSITION;
    d->setpt.position = position;
    chBSemSignal(&d->lock);
}

void motor_driver_set_velocity(motor_driver_t *d, float velocity)
{
    chBSemWait(&d->lock);
    d->control_mode = MOTOR_CONTROL_MODE_VELOCITY;
    d->setpt.velocity = velocity;
    chBSemSignal(&d->lock);
}

void motor_driver_set_torque(motor_driver_t *d, float torque)
{
    chBSemWait(&d->lock);
    d->control_mode = MOTOR_CONTROL_MODE_TORQUE;
    d->setpt.torque = torque;
    chBSemSignal(&d->lock);
}

void motor_driver_set_voltage(motor_driver_t *d, float voltage)
{
    chBSemWait(&d->lock);
    d->control_mode = MOTOR_CONTROL_MODE_VOLTAGE;
    d->setpt.voltage = voltage;
    chBSemSignal(&d->lock);
}

void motor_driver_disable(motor_driver_t *d)
{
    chBSemWait(&d->lock);
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

void motor_driver_set_stream_value(motor_driver_t *d, uint32_t stream, float value)
{

    if (stream < MOTOR_STREAMS_NB_VALUES) {
        motor_driver_lock(d);

        d->stream.values[stream] = value;
        d->stream.change_status |= (1 << stream);

        motor_driver_unlock(d);
    }
}

uint32_t motor_driver_get_stream_change_status(motor_driver_t *d)
{
    return d->stream.change_status;
}

float motor_driver_get_and_clear_stream_value(motor_driver_t *d, uint32_t stream)
{
    float return_value;

    if (stream < MOTOR_STREAMS_NB_VALUES) {
        motor_driver_lock(d);

        d->stream.change_status &= ~(1 << stream);
        return_value = d->stream.values[stream];

        motor_driver_unlock(d);

        return return_value;
    } else {
        return 0.;
    }

}

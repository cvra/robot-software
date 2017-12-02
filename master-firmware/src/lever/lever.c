#include "lever.h"

#include <string.h>

static const float LEVER_SERVO_RETRACTED_POS = 0.9f * 0.001f;
static const float LEVER_SERVO_DEPLOYED_POS = 2.2f * 0.001f;

void lever_init(lever_t* lever)
{
    memset(lever, 0, sizeof(lever_t));

    lever->state = LEVER_DISABLED;
    lever->pump_state = LEVER_PUMP_OFF;
}

void lever_set_callbacks(lever_t* lever, void (*set_lever)(void*, float), void* lever_args)
{
    lever->set_lever = set_lever;
    lever->lever_args = lever_args;
}

void lever_pump_set_callbacks(lever_t* lever, void (*set_pump)(void*, float), void* pump1_args, void* pump2_args)
{
    lever->set_pump = set_pump;
    lever->pump1_args = pump1_args;
    lever->pump2_args = pump2_args;
}

void lever_deploy(lever_t* lever)
{
    lever->state = LEVER_DEPLOYED;
    lever->set_lever(lever->lever_args, LEVER_SERVO_DEPLOYED_POS);
}

void lever_retract(lever_t* lever)
{
    lever->state = LEVER_RETRACTED;
    lever->set_lever(lever->lever_args, LEVER_SERVO_RETRACTED_POS);
}

static float pump_voltage(lever_pump_state_t state)
{
    return state == LEVER_PUMP_ON ? 10.0f : 0.0f;
}

void lever_pump_set(lever_t* lever, lever_pump_state_t state)
{
    lever->pump_state = state;

    lever->set_pump(lever->pump1_args, pump_voltage(state));
    lever->set_pump(lever->pump2_args, pump_voltage(state));
}

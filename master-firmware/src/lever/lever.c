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

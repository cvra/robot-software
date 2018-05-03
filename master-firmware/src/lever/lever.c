#include "lever.h"

#include <string.h>

#ifdef TESTS
#define LEVER_SLEEP_MS(x)
#else
#include <ch.h>
#define LEVER_SLEEP_MS(x) chThdSleepMilliseconds(x)
#endif

static float pump_voltage(lever_pump_state_t state);
static void lever_pump_set(lever_t* lever, lever_pump_state_t state);

static void lever_lock(mutex_t* mutex)
{
    chMtxLock(mutex);
}

static void lever_unlock(mutex_t* mutex)
{
    chMtxUnlock(mutex);
}

void lever_init(lever_t* lever)
{
    memset(lever, 0, sizeof(lever_t));

    lever->state = LEVER_DISABLED;
    lever->pump_state = LEVER_PUMP_OFF;

    chMtxObjectInit(&lever->lock);
}

void lever_set_servo_range(lever_t* lever, float servo_retracted_pwm, float servo_deployed_pwm)
{
    lever_lock(&lever->lock);

    lever->servo_retracted_pwm = servo_retracted_pwm;
    lever->servo_deployed_pwm = servo_deployed_pwm;

    lever_unlock(&lever->lock);
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
    lever_lock(&lever->lock);

    lever->state = LEVER_DEPLOYED;
    lever->set_lever(lever->lever_args, lever->servo_deployed_pwm);

    lever_unlock(&lever->lock);
}

void lever_retract(lever_t* lever)
{
    lever_lock(&lever->lock);

    lever->state = LEVER_RETRACTED;
    lever->set_lever(lever->lever_args, lever->servo_retracted_pwm);

    lever_unlock(&lever->lock);
}

void lever_push_and_retract(lever_t* lever)
{
    lever_lock(&lever->lock);

    lever_pump_set(lever, LEVER_PUMP_REVERSE);
    LEVER_SLEEP_MS(200);

    lever->state = LEVER_RETRACTED;
    lever->set_lever(lever->lever_args, lever->servo_retracted_pwm);

    LEVER_SLEEP_MS(500);
    lever_pump_set(lever, LEVER_PUMP_OFF);

    lever_unlock(&lever->lock);
}

static float pump_voltage(lever_pump_state_t state)
{
    switch (state)
    {
    case LEVER_PUMP_ON:      return 13.f;
    case LEVER_PUMP_REVERSE: return -10.f;
    case LEVER_PUMP_OFF:     return 0.f;
    default:                 return 0.f;
    }
}

static void lever_pump_set(lever_t* lever, lever_pump_state_t state)
{
    lever->pump_state = state;

    lever->set_pump(lever->pump1_args, pump_voltage(state));
    lever->set_pump(lever->pump2_args, pump_voltage(state));
}

void lever_pickup(lever_t* lever, se2_t robot_pose, se2_t blocks_pose)
{
    lever_lock(&lever->lock);

    lever_pump_set(lever, LEVER_PUMP_ON);
    lever->robot_pose_at_pickup = robot_pose;
    lever->blocks_pose_at_pickup = blocks_pose;

    lever_unlock(&lever->lock);
}

se2_t lever_deposit(lever_t* lever, se2_t robot_pose)
{
    lever_lock(&lever->lock);

    lever_pump_set(lever, LEVER_PUMP_OFF);
    se2_t blocks_pose = se2_chain(robot_pose,
                                  se2_chain(se2_inverse(lever->robot_pose_at_pickup),
                                            lever->blocks_pose_at_pickup));

    lever_unlock(&lever->lock);

    return blocks_pose;
}

void lever_tidy(lever_t* lever)
{
    lever_lock(&lever->lock);

    lever_pump_set(lever, LEVER_PUMP_OFF);

    lever->state = LEVER_RETRACTED;
    lever->set_lever(lever->lever_args, lever->servo_retracted_pwm);

    lever_unlock(&lever->lock);
}

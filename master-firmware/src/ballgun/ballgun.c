#include "ballgun.h"

#include <string.h>

static void ballgun_lock(mutex_t* mutex)
{
    chMtxLock(mutex);
}

static void ballgun_unlock(mutex_t* mutex)
{
    chMtxUnlock(mutex);
}

void ballgun_init(ballgun_t* ballgun)
{
    memset(ballgun, 0, sizeof(ballgun_t));

    ballgun->state = BALLGUN_DISABLED;
    ballgun->turbine_state = BALLGUN_ARMED;

    chMtxObjectInit(&ballgun->lock);
}

void ballgun_set_servo_range(ballgun_t* ballgun, float retracted, float deployed, float deployed_fully)
{
    ballgun_lock(&ballgun->lock);

    ballgun->servo_retracted_pwm = retracted;
    ballgun->servo_deployed_pwm = deployed;
    ballgun->servo_deployed_fully_pwm = deployed_fully;

    ballgun_unlock(&ballgun->lock);
}

void ballgun_set_turbine_range(ballgun_t *ballgun, float armed, float charge, float fire, float slowfire)
{
    ballgun_lock(&ballgun->lock);

    ballgun->turbine_armed_pwm = armed;
    ballgun->turbine_charge_pwm = charge;
    ballgun->turbine_fire_pwm = fire;
    ballgun->turbine_slowfire_pwm = slowfire;

    ballgun_unlock(&ballgun->lock);
}

void ballgun_set_accelerator_range(ballgun_t *ballgun, float charge, float fire, float slowfire)
{
    ballgun_lock(&ballgun->lock);

    ballgun->accelerator_charge = charge;
    ballgun->accelerator_fire = fire;
    ballgun->accelerator_slowfire = slowfire;

    ballgun_unlock(&ballgun->lock);
}

void ballgun_set_callbacks(ballgun_t* ballgun, void (*set_ballgun)(void*, float), void* ballgun_args)
{
    ballgun->set_ballgun = set_ballgun;
    ballgun->ballgun_args = ballgun_args;
}

void ballgun_set_turbine_callbacks(ballgun_t* ballgun, void (*set_turbine)(void*, float), void* turbine_args)
{
    ballgun->set_turbine = set_turbine;
    ballgun->turbine_args = turbine_args;
}

void ballgun_set_accelerator_callbacks(ballgun_t* ballgun, void (*set_accelerator)(void*, float),
                                       void* accelerator_args)
{
    ballgun->set_accelerator = set_accelerator;
    ballgun->accelerator_args = accelerator_args;
}

void ballgun_deploy(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->state = BALLGUN_DEPLOYED;
    ballgun->set_ballgun(ballgun->ballgun_args, ballgun->servo_deployed_pwm);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_deploy_fully(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->state = BALLGUN_DEPLOYED_FULLY;
    ballgun->set_ballgun(ballgun->ballgun_args, ballgun->servo_deployed_fully_pwm);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_retract(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->state = BALLGUN_RETRACTED;
    ballgun->set_ballgun(ballgun->ballgun_args, ballgun->servo_retracted_pwm);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_arm(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->turbine_state = BALLGUN_ARMED;
    ballgun->set_turbine(ballgun->turbine_args, ballgun->turbine_armed_pwm);
    ballgun->set_accelerator(ballgun->accelerator_args, 0);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_charge(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->turbine_state = BALLGUN_CHARGING;
    ballgun->set_turbine(ballgun->turbine_args, ballgun->turbine_charge_pwm);
    ballgun->set_accelerator(ballgun->accelerator_args, ballgun->accelerator_charge);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_fire(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->turbine_state = BALLGUN_FIRING;
    ballgun->set_turbine(ballgun->turbine_args, ballgun->turbine_fire_pwm);
    ballgun->set_accelerator(ballgun->accelerator_args, ballgun->accelerator_fire);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_slowfire(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->turbine_state = BALLGUN_SLOWFIRING;
    ballgun->set_turbine(ballgun->turbine_args, ballgun->turbine_slowfire_pwm);
    ballgun->set_accelerator(ballgun->accelerator_args, ballgun->accelerator_slowfire);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_tidy(ballgun_t* ballgun)
{
    ballgun_arm(ballgun);
    ballgun_retract(ballgun);
}

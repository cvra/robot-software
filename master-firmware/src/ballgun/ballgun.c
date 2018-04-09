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

    chMtxObjectInit(&ballgun->lock);
}

void ballgun_set_servo_range(ballgun_t* ballgun, float servo_retracted_pwm, float servo_deployed_pwm)
{
    ballgun_lock(&ballgun->lock);

    ballgun->servo_retracted_pwm = servo_retracted_pwm;
    ballgun->servo_deployed_pwm = servo_deployed_pwm;

    ballgun_unlock(&ballgun->lock);
}

void ballgun_set_callbacks(ballgun_t* ballgun, void (*set_ballgun)(void*, float), void* ballgun_args)
{
    ballgun->set_ballgun = set_ballgun;
    ballgun->ballgun_args = ballgun_args;
}

void ballgun_deploy(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->state = BALLGUN_DEPLOYED;
    ballgun->set_ballgun(ballgun->ballgun_args, ballgun->servo_deployed_pwm);

    ballgun_unlock(&ballgun->lock);
}

void ballgun_retract(ballgun_t* ballgun)
{
    ballgun_lock(&ballgun->lock);

    ballgun->state = BALLGUN_RETRACTED;
    ballgun->set_ballgun(ballgun->ballgun_args, ballgun->servo_retracted_pwm);

    ballgun_unlock(&ballgun->lock);
}

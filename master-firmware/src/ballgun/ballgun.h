#ifndef BALLGUN_H
#define BALLGUN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

typedef enum {
    BALLGUN_DISABLED,
    BALLGUN_RETRACTED,
    BALLGUN_DEPLOYED,
} ballgun_state_t;

typedef struct {
    void (*set_ballgun)(void*, float);
    void* ballgun_args;

    ballgun_state_t state;

    float servo_retracted_pwm;  // pwm duty cycle in seconds to retract servo
    float servo_deployed_pwm;   // pwm duty cycle in seconds to deploy servo
    mutex_t lock;
} ballgun_t;

/* Initialize ballgun */
void ballgun_init(ballgun_t* ballgun);
void ballgun_set_servo_range(ballgun_t* ballgun, float servo_retracted_pwm, float servo_deployed_pwm);

/* Actuator callbacks */
void ballgun_set_callbacks(ballgun_t* ballgun, void (*set_ballgun)(void*, float), void* ballgun_args);

/* Deploy ballgun */
void ballgun_deploy(ballgun_t* ballgun);

/* Retract ballgun */
void ballgun_retract(ballgun_t* ballgun);

#ifdef __cplusplus
}
#endif

#endif /* BALLGUN_H */

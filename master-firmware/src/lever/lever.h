#ifndef LEVER_H
#define LEVER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LEVER_DISABLED,
    LEVER_RETRACTED,
    LEVER_DEPLOYED,
} lever_state_t;

typedef enum {
    LEVER_PUMP_OFF,
    LEVER_PUMP_ON,
} lever_pump_state_t;

typedef struct {
    void (*set_lever)(void*, float);
    void* lever_args;

    void (*set_pump)(void*, float);
    void* pump1_args;
    void* pump2_args;

    lever_state_t state;
    lever_pump_state_t pump_state;

    float servo_retracted_pwm;  // pwm duty cycle in seconds to retract servo
    float servo_deployed_pwm;   // pwm duty cycle in seconds to deploy servo
} lever_t;

/* Initialize lever */
void lever_init(lever_t* lever);
void lever_set_servo_range(lever_t* lever, float servo_retracted_pwm, float servo_deployed_pwm);

/* Actuator callbacks */
void lever_set_callbacks(lever_t* lever, void (*set_lever)(void*, float), void* lever_args);
void lever_pump_set_callbacks(lever_t* lever, void (*set_pump)(void*, float), void* pump1_args, void* pump2_args);

/* Deploy lever */
void lever_deploy(lever_t* lever);

/* Retract lever */
void lever_retract(lever_t* lever);

/* Enable/Disable pump */
void lever_pump_set(lever_t* lever, lever_pump_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* LEVER_H */

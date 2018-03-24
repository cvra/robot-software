#ifndef LEVER_H
#define LEVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

#include "math/lie_groups.h"

typedef enum {
    LEVER_DISABLED,
    LEVER_RETRACTED,
    LEVER_DEPLOYED,
} lever_state_t;

typedef enum {
    LEVER_PUMP_OFF,
    LEVER_PUMP_ON,
    LEVER_PUMP_REVERSE,
} lever_pump_state_t;

typedef struct {
    void (*set_lever)(void*, float);
    void* lever_args;

    void (*set_pump)(void*, float);
    void* pump1_args;
    void* pump2_args;

    lever_state_t state;
    lever_pump_state_t pump_state;

    se2_t robot_pose_at_pickup;     // Robot pose at pickup (table frame)
    se2_t blocks_pose_at_pickup;    // Blocks pose at pickup (table frame)

    float servo_retracted_pwm;  // pwm duty cycle in seconds to retract servo
    float servo_deployed_pwm;   // pwm duty cycle in seconds to deploy servo
    mutex_t lock;
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
void lever_push_and_retract(lever_t* lever);

/* Pickup blocks (set pumps on)
 * Takes robot pose and blocks pose - both in table frame - to track blocks pose
 */
void lever_pickup(lever_t* lever, se2_t robot_pose, se2_t blocks_pose);

/* Deposit blocks (set pumps off)
 * Takes robot pose in table frame
 * Returns blocks pose after deposit in table frame
 */
se2_t lever_deposit(lever_t* lever, se2_t robot_pose);

#ifdef __cplusplus
}
#endif

#endif /* LEVER_H */

#ifndef SAFETY_H
#define SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/** Checks if we are allowed to run any of the actuators.
 *
 * For safety reasons, the board is only allowed to have any actuators on if a
 * CAN command has been received recently (see safety.c for the precise timeout
 * duration). Otherwise we assume that we are disconnected from the master
 * board and refuse to move
 */
int safety_motion_is_allowed(void);

/** Restarts the safety timer, typically called when receiving a CAN order. */
void safety_timer_restart(void);

#ifdef __cplusplus
}
#endif

#endif

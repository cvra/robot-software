#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT 2

void servo_start(void);

/** Sets the i-th servo motor setpoint, as well as its trajectory parameters.
 *
 * The position is unitless and varies between 0 and 1. The velocity have a
 * unit of per second, and acceleration per second squared.
 */
void servo_set(int index, float pos, float vel, float acc);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */

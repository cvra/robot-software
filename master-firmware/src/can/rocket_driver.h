#ifndef ROCKET_DRIVER_H
#define ROCKET_DRIVER_H

/* TODO Calibrate this. */
#define ROCKET_POS_OPEN 0.
#define ROCKET_POS_CLOSE 0.

#ifdef __cplusplus
extern "C" {
#endif

int rocket_init(void);
void rocket_set_pos(float pos);

#ifdef __cplusplus
}
#endif
#endif

#ifndef WRIST_H
#define WRIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

typedef enum {
    WRIST_HORIZONTAL = 0,
    WRIST_VERTICAL,
} wrist_state_t;

typedef struct {
    wrist_state_t state;

    float servo_horizontal;
    float servo_vertical;

    void (*set_servo)(void*, float);
    void* servo_args; /// Internal state passed to motor callbacks
} wrist_t;

/* Initialize specific wrist */
void wrist_init(wrist_t* wrist);

/* Setup servo */
void wrist_set_servo_callback(wrist_t *wrist, void (*set_servo)(void *, float), void *servo_args);
void wrist_set_servo_range(wrist_t *wrist, float horizontal, float vertical);

/* Set wrist position */
void wrist_set_horizontal(wrist_t* wrist);
void wrist_set_vertical(wrist_t* wrist);

#ifdef __cplusplus
}
#endif

#endif /* WRIST_H */

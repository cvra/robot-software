#ifndef HAND_H
#define HAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

typedef enum {
    PUMP_OFF = 0,
    PUMP_ON,
    PUMP_REVERSE,
} pump_state_t;

typedef struct {
    pump_state_t pump_state;

    void (*set_pump_voltage)(void*, float);
    void* pump_args; /// Internal state passed to motor callbacks
} hand_t;

/* Initialize specific hand */
void hand_init(hand_t* hand);

/* Set pump driver callback */
void hand_set_pump_callback(hand_t *hand,
                            void (*set_pump_voltage)(void *, float),
                            void *pump_args);

/* Set pump position of a hand */
void hand_set_pump(hand_t* hand, pump_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* HAND_H */

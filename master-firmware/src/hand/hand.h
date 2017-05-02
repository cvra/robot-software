#ifndef HAND_H
#define HAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

#include <aversive/position_manager/position_manager.h>
#include "scara/scara.h"

typedef enum {
    FINGER_CLOSED=0,
    FINGER_OPEN,
    FINGER_RETRACTED,
} finger_state_t;

typedef struct {
    /* Motor control callbacks */
    void (*set_fingers)(finger_state_t*);     /**< Callback function to set fingers position. */

    /* Motor positions */
    finger_state_t fingers_open[4];

    mutex_t lock;
} hand_t;


/* Initialize specific hand */
void hand_init(hand_t* hand);

/* Set motor functions */
void hand_set_fingers_callbacks(hand_t* hand, void (*set_fingers)(finger_state_t*));

/* Manage hand system, should run in a loop */
void hand_manage(hand_t* hand);

/* Set finger position of a hand */
void hand_set_finger(hand_t* hand, int index, finger_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* HAND_H */

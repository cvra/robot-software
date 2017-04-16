#ifndef HAND_H
#define HAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <aversive/position_manager/position_manager.h>
#include "scara/scara.h"

#include "hand_utils.h"


typedef struct {
    /* Motor control callbacks */
    void (*set_wrist_position)(void*, float);  /**< Callback function to set wrist position. */

    /* Motor feedback callbacks */
    float (*get_wrist_position)(void*); /**< Callback function to get wrist position. */

    /* Motor control args */
    void* wrist_args;

    /* Motor positions */
    float wrist_pos;

    /* Robot information */
    struct robot_position *robot_pos;
    scara_t *arm;
} hand_t;


/* Initialize specific hand */
void hand_init(hand_t* hand);

/* Set wrist motor functions */
void hand_set_wrist_callbacks(hand_t* hand, void (*set_wrist_position)(void*, float), float (*get_wrist_position)(void*), void* wrist_args);

/* Move hand */
void hand_goto(hand_t* hand, float heading, hand_coordinate_t system);

/* Set robot information */
void hand_set_related_robot_pos(hand_t *hand, struct robot_position *pos);
void hand_set_related_arm(hand_t *hand, scara_t *arm);

#ifdef __cplusplus
}
#endif

#endif /* HAND_H */

#ifndef HAND_CONTROLLER_H
#define HAND_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hand/hand.h"

#define HANDS_FREQUENCY 10

extern hand_t left_hand;
extern hand_t right_hand;


/* Initialize both hands */
void hands_init(void);

/* Run hands controller thread */
void hands_controller_start(void);


#ifdef __cplusplus
}
#endif

#endif /* HAND_CONTROLLER_H */

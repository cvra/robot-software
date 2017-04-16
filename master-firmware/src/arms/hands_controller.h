#ifndef HAND_CONTROLLER_H
#define HAND_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hand/hand.h"


extern hand_t left_hand;
extern hand_t right_hand;


/* Initialize both hands */
void hands_init();


#ifdef __cplusplus
}
#endif

#endif /* HAND_CONTROLLER_H */

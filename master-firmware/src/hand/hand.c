#include <string.h>

#include "hand.h"


void hand_init(hand_t* hand)
{
    memset(hand, 0, sizeof(hand_t));

    for (size_t i = 0; i < 4; i++) {
        hand->fingers_open[i] = FINGER_RETRACTED;
    }

    chMtxObjectInit(&hand->lock);
}

void hand_set_fingers_callbacks(hand_t* hand, void (*set_fingers)(finger_state_t*))
{
    hand->set_fingers = set_fingers;
}

void hand_manage(hand_t* hand)
{
    /* Lock */
    chMtxLock(&hand->lock);

    /* Unlock */
    chMtxUnlock(&hand->lock);
}

void hand_set_finger(hand_t* hand, int index, finger_state_t state)
{
    hand->fingers_open[index % 4] = state;
    hand->set_fingers(hand->fingers_open);
}

#include <string.h>

#include "hand.h"

void hand_init(hand_t* hand)
{
    memset(hand, 0, sizeof(hand_t));

    hand->pump_state = PUMP_OFF;
}

void hand_set_pump_callback(hand_t *hand,
                            void (*set_pump_voltage)(void *, float),
                            void *pump_args)
{
    hand->set_pump_voltage = set_pump_voltage;
    hand->pump_args = pump_args;
}

static float pump_voltage(pump_state_t state)
{
    if (state == PUMP_ON) {
        return 10.0;
    } else {
        return 0.0;
    }
}

void hand_set_pump(hand_t* hand, pump_state_t state)
{
    hand->set_pump_voltage(hand->pump_args, pump_voltage(state));
    hand->pump_state = state;
}

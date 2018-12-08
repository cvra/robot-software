#include <string.h>

#include "hand.h"

void hand_init(hand_t* hand)
{
    memset(hand, 0, sizeof(hand_t));

    hand->pump_state = PUMP_OFF;
}

void hand_set_pump_callback(hand_t* hand,
                            void (*set_pump_voltage)(void*, float),
                            void* pump_args)
{
    hand->set_pump_voltage = set_pump_voltage;
    hand->pump_args = pump_args;
}

static float pump_voltage(pump_state_t state)
{
    switch (state) {
        case PUMP_ON:
            return 13.f;
        case PUMP_OFF:
            return 0.f;
        case PUMP_REVERSE:
            return -10.f;
        default:
            return 0.f;
    }
}

void hand_set_pump(hand_t* hand, pump_state_t state)
{
    hand->set_pump_voltage(hand->pump_args, pump_voltage(state));
    hand->pump_state = state;
}

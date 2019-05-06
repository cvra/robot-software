#pragma once

#include <msgbus/messagebus.h>

#include "strategy_impl/context.h"

extern messagebus_t bus;

void simulation_init(void);

strategy_context_t* strategy_simulated_impl(enum strat_color_t color);

void publish_pos(strategy_context_t* strat);

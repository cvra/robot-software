#ifndef STRATEGY_IMPL_SIMULATION_H
#define STRATEGY_IMPL_SIMULATION_H

#include "strategy_impl/context.h"

#ifdef __cplusplus
extern "C" {
#endif

strategy_context_t* strategy_simulated_impl(enum strat_color_t color);
void strategy_simulated_init(void);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_IMPL_SIMULATION_H */

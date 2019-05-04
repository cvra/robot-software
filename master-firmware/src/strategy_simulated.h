#ifndef STRATEGY_SIMULATED_H
#define STRATEGY_SIMULATED_H

#include "strategy_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

strategy_context_t* strategy_simulated_impl(enum strat_color_t color);
void strategy_simulated_init(void);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_SIMULATED_H */

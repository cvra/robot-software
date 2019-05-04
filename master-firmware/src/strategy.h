#ifndef STRATEGY_H
#define STRATEGY_H

#include "strategy_impl.h"

#ifdef __cplusplus
extern "C" {
#endif

void strategy_start(void);

strategy_impl_t* strategy_impl(void);
bool strategy_puck_is_picked(void);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */

#ifndef STRATEGY_H
#define STRATEGY_H

#include <vector>
#include "strategy/actions.h"

void strategy_play_game();

std::vector<actions::NamedAction<StrategyState>*> strategy_get_actions();

#endif /* STRATEGY_H */

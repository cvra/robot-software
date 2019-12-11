#ifndef STRATEGY_GOALS_H
#define STRATEGY_GOALS_H

#include <goap/goap.hpp>
#include "state.h"

namespace goals {
class LighthouseEnabled : public goap::Goal<StrategyState> {
public:
    int distance_to(const StrategyState& state) const override;
};
} // namespace goals

// TODO(all): Write goals

#endif /* STRATEGY_GOALS_H */

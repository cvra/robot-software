#ifndef STRATEGY_ACTIONS_H
#define STRATEGY_ACTIONS_H

#include <goap/goap.hpp>
#include "state.h"
#include "color.h"

/** Number of states goap can visit before giving up. Increasing it means a
 * solution is found on more complex problems at the expense of RAM use. */
#define GOAP_SPACE_SIZE 150

namespace actions {

/** This action tells the robot to go and press the button to light up the
 * lighthouse. */
class EnableLighthouse : public goap::Action<StrategyState> {
    public:
    bool can_run(const StrategyState &state) override;
    void plan_effects(StrategyState &state) override;
    bool execute(StrategyState &state) override;
};

// TODO(all): Write actions

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */

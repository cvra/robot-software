#ifndef STRATEGY_ACTIONS_H
#define STRATEGY_ACTIONS_H

#include <goap/goap.hpp>
#include "state.h"
#include "color.h"
#include "absl/strings/str_format.h"

/** Number of states goap can visit before giving up. Increasing it means a
 * solution is found on more complex problems at the expense of RAM use. */
#define GOAP_SPACE_SIZE 150

namespace actions {

template <class T>
class NamedAction : public goap::Action<T> {
public:
    virtual std::string get_name() = 0;
};

/** This action tells the robot to go and press the button to light up the
 * lighthouse. */
class EnableLighthouse : public NamedAction<StrategyState> {
public:
    bool can_run(const StrategyState& state) override;
    void plan_effects(StrategyState& state) override;
    bool execute(StrategyState& state) override;
    std::string get_name() override
    {
        return "lighthouse";
    }
};

/** Common class for all windsock raising operations as they have a lot in
 * common. */
class RaiseWindsock : public NamedAction<StrategyState> {
    int windsock_index_;

public:
    /* Windsocks are indexed starting from the starting area. */
    RaiseWindsock(int windsock_index);
    bool can_run(const StrategyState& state) override;
    void plan_effects(StrategyState& state) override;
    bool execute(StrategyState& state) override;

    std::string get_name() override
    {
        return absl::StrFormat("windsock#%d", windsock_index_);
    }
};

class BackwardReefPickup : public NamedAction<StrategyState> {
public:
    bool can_run(const StrategyState& state) override;
    void plan_effects(StrategyState& state) override;
    bool execute(StrategyState& state) override;

    std::string get_name() override
    {
        return "BwRePckp";
    }
};

} // namespace actions

#endif /* STRATEGY_ACTIONS_H */

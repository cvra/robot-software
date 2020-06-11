#include "goals.h"

namespace goals {
int LighthouseEnabled::distance_to(const StrategyState& state) const
{
    return goap::Distance().shouldBeTrue(state.lighthouse_is_on);
}

int WindsocksUp::distance_to(const StrategyState& state) const
{
    return goap::Distance()
        .shouldBeTrue(state.windsocks_are_up[0])
        .shouldBeTrue(state.windsocks_are_up[1]);
}
} // namespace goals

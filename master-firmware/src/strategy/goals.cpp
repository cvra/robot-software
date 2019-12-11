#include "goals.h"

namespace goals {
int LighthouseEnabled::distance_to(const StrategyState& state) const
{
    return goap::Distance().shouldBeTrue(state.lighthouse_is_on);
}
} // namespace goals

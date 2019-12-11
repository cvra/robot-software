/* File containing the GOAP parts for the actions.
 *
 * The parts specific to the robot, which actually moves the robot and so on
 * should go to actions_impl.cpp
 */

#include "actions.h"

using namespace actions;

bool EnableLighthouse::can_run(const StrategyState& state)
{
    (void)state;
    return true;
}

void EnableLighthouse::plan_effects(StrategyState& state)
{
    state.lighthouse_is_on = true;
}

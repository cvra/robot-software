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

RaiseWindsock::RaiseWindsock(int windsock_index)
    : windsock_index(windsock_index)
{
}

bool RaiseWindsock::can_run(const StrategyState& state)
{
    /* We don't want to retry a windsock which is already up. */
    return !state.windsocks_are_up[windsock_index];
}

void RaiseWindsock::plan_effects(StrategyState& state)
{
    state.windsocks_are_up[windsock_index] = true;
}

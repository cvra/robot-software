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

bool BackwardReefPickup::can_run(const StrategyState& state)
{
    if (state.robot.back_center_glass != GlassColor_UNKNOWN
        || state.robot.back_center_glass != GlassColor_UNKNOWN
        || state.robot.back_center_glass != GlassColor_UNKNOWN) {
        return false;
    }

    // If there are no glasses to pickup, we cannot run
    if (state.our_dispenser.glasses[0] == GlassColor_UNKNOWN
        && state.our_dispenser.glasses[0] == GlassColor_UNKNOWN
        && state.our_dispenser.glasses[0] == GlassColor_UNKNOWN) {
        return false;
    }

    return true;
}

void BackwardReefPickup::plan_effects(StrategyState& state)
{
    // Glasses are now on the robot, and dispenser has been emptied

    state.robot.back_left_glass = state.our_dispenser.glasses[0];
    state.robot.back_center_glass = state.our_dispenser.glasses[1];
    state.robot.back_right_glass = state.our_dispenser.glasses[2];

    state.our_dispenser.glasses[0] = GlassColor_UNKNOWN;
    state.our_dispenser.glasses[1] = GlassColor_UNKNOWN;
    state.our_dispenser.glasses[2] = GlassColor_UNKNOWN;
}

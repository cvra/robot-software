#include <thread>
#include <error/error.h>
#include "actions.h"
#include "robot_helpers/trajectory_helpers.h"

using namespace std::chrono_literals;

bool actions::EnableLighthouse::execute(StrategyState& state)
{
    (void)state;
    NOTICE("Enabling the lighthouse");

    int res;

    // Go in front of lighthouse
    NOTICE("Going to the lighthouse");
    trajectory_goto_forward_xy_abs(&robot.traj, 525, 300);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to lighthouse: %d", res);
        return false;
    }

    // We go backward to avoid bringing any glasses with us
    trajectory_goto_backward_xy_abs(&robot.traj, 225, 400);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to lighthouse: %d", res);
        return false;
    }

    NOTICE("Turning toward lighthouse");
    trajectory_a_abs(&robot.traj, -90);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);

    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not turn to lighthouse: %d", res);
        return false;
    }

    trajectory_d_rel(&robot.traj, 300);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    if (position_get_y_double(&robot.pos) > 150) {
        WARNING("Could not get close enough to trigger lighthouse...");
        return false;
    }

    NOTICE("Lighthouse succesfully turned on");
    state.lighthouse_is_on = true;

    trajectory_d_rel(&robot.traj, -200);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    return true;
}

bool actions::RaiseWindsock::execute(StrategyState& state)
{
    (void)state;
    NOTICE("Raising windsock #%d", windsock_index);

    int windsock_x;
    int res;

    if (windsock_index == 0) {
        windsock_x = 2770;
    } else {
        windsock_x = 2365;
    }

    // TODO: Use proper obstacle avoidance instead
    trajectory_goto_xy_abs(&robot.traj, windsock_x - 100, 1500);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to windsock!");
        return false;
    }

    trajectory_goto_xy_abs(&robot.traj, windsock_x - 100, 2000 - 150);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to windsock!");
        return false;
    }

    trajectory_a_abs(&robot.traj, 0);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not turn to windsock");
        return false;
    }

    trajectory_d_rel(&robot.traj, 200);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not move windsock");
        return false;
    }

    state.windsocks_are_up[windsock_index] = true;

    return true;
}

bool actions::BackwardReefPickup::execute(StrategyState& state)
{
    WARNING("picking up glasses at our dispenser");
    trajectory_goto_xy_abs(&robot.traj, 500, 1525);

    auto res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go in front of reef");
        return false;
    }

    NOTICE("TODO: Setting up the arms, turning on the pumps");

    trajectory_a_abs(&robot.traj, 0);

    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        return false;
    }

    // Give the robot some time to converge
    std::this_thread::sleep_for(500ms);

    // TODO: Check that this distance is enough
    trajectory_d_rel(&robot.traj, -300);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        return false;
    }

    // give time to catch the glasses
    std::this_thread::sleep_for(400ms);

    state.robot.back_left_glass = state.our_dispenser.glasses[0];
    state.robot.back_center_glass = state.our_dispenser.glasses[1];
    state.robot.back_right_glass = state.our_dispenser.glasses[2];
    state.our_dispenser.glasses[0] = GlassColor_UNKNOWN;
    state.our_dispenser.glasses[1] = GlassColor_UNKNOWN;
    state.our_dispenser.glasses[2] = GlassColor_UNKNOWN;

    trajectory_d_rel(&robot.traj, 100);
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        return false;
    }

    return true;
}

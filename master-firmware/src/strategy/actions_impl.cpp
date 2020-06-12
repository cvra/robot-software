#include <error/error.h>
#include "actions.h"
#include "robot_helpers/trajectory_helpers.h"

bool actions::EnableLighthouse::execute(StrategyState& state)
{
    (void)state;
    NOTICE("Enabling the lighthouse");

    int res;

    // Go in front of lighthouse
    NOTICE("Going to the lighthouse");
    {
        absl::MutexLock l(&robot.lock);
        trajectory_goto_forward_xy_abs(&robot.traj, 525, 300);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to lighthouse: %d", res);
        return false;
    }

    // We go backward to avoid bringing any glasses with us
    {
        absl::MutexLock l(&robot.lock);
        trajectory_goto_backward_xy_abs(&robot.traj, 225, 400);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to lighthouse: %d", res);
        return false;
    }

    NOTICE("Turning toward lighthouse");
    {
        absl::MutexLock l(&robot.lock);
        trajectory_a_abs(&robot.traj, -90);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);

    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not turn to lighthouse: %d", res);
        return false;
    }

    {
        absl::MutexLock l(&robot.lock);
        trajectory_d_rel(&robot.traj, 300);
    }
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    if (position_get_y_double(&robot.pos) > 150) {
        WARNING("Could not get close enough to trigger lighthouse...");
        return false;
    }

    NOTICE("Lighthouse succesfully turned on");
    state.lighthouse_is_on = true;

    {
        absl::MutexLock l(&robot.lock);
        trajectory_d_rel(&robot.traj, -200);
    }
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
    {
        absl::MutexLock l(&robot.lock);
        trajectory_goto_xy_abs(&robot.traj, windsock_x - 100, 1500);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to windsock!");
        return false;
    }

    {
        absl::MutexLock l(&robot.lock);
        trajectory_goto_xy_abs(&robot.traj, windsock_x - 100, 2000 - 150);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not go to windsock!");
        return false;
    }

    {
        absl::MutexLock l(&robot.lock);
        trajectory_a_abs(&robot.traj, 0);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not turn to windsock");
        return false;
    }

    {
        absl::MutexLock l(&robot.lock);
        trajectory_d_rel(&robot.traj, 200);
    }
    res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
    if (res != TRAJ_END_GOAL_REACHED) {
        WARNING("Could not move windsock");
        return false;
    }

    state.windsocks_are_up[windsock_index] = true;

    return true;
}

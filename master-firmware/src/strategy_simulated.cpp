#include "strategy_simulated.h"

#include "base/base_controller.h"

static void simulated_wait_ms(int ms)
{
    (void)ms;
}
static void simulated_wait_for_user_input(void)
{
}
static bool simulated_manipulator_goto(manipulator_side_t side, manipulator_state_t target)
{
    (void)side;
    (void)target;
    return true;
}
static void simulated_gripper_set(manipulator_side_t side, gripper_state_t state)
{
    (void)side;
    (void)state;
}
static bool simulated_puck_is_picked(void)
{
    return true;
}

struct _robot robot_simulated;

strategy_context_t strategy_simulated = {
    &robot_simulated,
    YELLOW,

    simulated_wait_ms,
    simulated_wait_for_user_input,

    simulated_manipulator_goto,
    simulated_gripper_set,
    simulated_puck_is_picked,
};

strategy_context_t* strategy_simulated_impl(enum strat_color_t color)
{
    strategy_simulated.color = color;
    return &strategy_simulated;
}

void strategy_simulated_init(void)
{
    rs_init(&robot_simulated.rs);
    position_init(&robot_simulated.pos);
    position_set(&robot_simulated.pos, 0, 0, 0);

    cs_init(&robot_simulated.distance_cs);
    cs_init(&robot_simulated.angle_cs);

    quadramp_init(&robot_simulated.angle_qr);
    quadramp_init(&robot_simulated.distance_qr);
    cs_set_consign_filter(&robot_simulated.distance_cs, quadramp_do_filter, &robot_simulated.distance_qr);
    cs_set_consign_filter(&robot_simulated.angle_cs, quadramp_do_filter, &robot_simulated.angle_qr);

    trajectory_manager_init(&robot_simulated.traj, 20);
    trajectory_set_cs(&robot_simulated.traj, &robot_simulated.distance_cs, &robot_simulated.angle_cs);
    trajectory_set_robot_params(&robot_simulated.traj, &robot_simulated.rs, &robot_simulated.pos);

    auto angle_start_deg = 10;
    auto angle_window_deg = 1;
    auto distance_window_mm = 10;

    trajectory_set_windows(&robot_simulated.traj, distance_window_mm,
                           angle_window_deg, angle_start_deg);

    trajectory_set_acc(&robot_simulated.traj, 10, 10);

    int arbitrary_max_speed = 10;
    trajectory_set_speed(&robot_simulated.traj, arbitrary_max_speed, arbitrary_max_speed);
}

#include "simulation.h"

#include "msgbus_port.h"
#include "base/base_controller.h"
#include "protobuf/position.pb.h"

#include <iostream>

messagebus_t bus;

static void simulated_log(const char* log)
{
    std::cout << "  " << log << std::endl;
}
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

static void simulated_rotate(void* ctx, int relative_angle_deg)
{
    strategy_context_t* strat = (strategy_context_t*)ctx;

    int x = position_get_x_s16(&strat->robot->pos);
    int y = position_get_y_s16(&strat->robot->pos);
    int a = position_get_a_deg_s16(&strat->robot->pos);

    position_set(&strat->robot->pos, x, y, a + relative_angle_deg);

    std::string pos_str = "["
        + std::to_string(position_get_x_s16(&strat->robot->pos)) + ", "
        + std::to_string(position_get_y_s16(&strat->robot->pos)) + ","
        + std::to_string(position_get_a_deg_s16(&strat->robot->pos)) + "]";
    strat->log(pos_str.c_str());
}

static void simulated_forward(void* ctx, int relative_distance_mm)
{
    strategy_context_t* strat = (strategy_context_t*)ctx;

    int x = position_get_x_s16(&strat->robot->pos);
    int y = position_get_y_s16(&strat->robot->pos);
    float a_rad = position_get_a_rad_float(&strat->robot->pos);

    int dx = relative_distance_mm * cosf(a_rad);
    int dy = relative_distance_mm * sinf(a_rad);
    position_set(&strat->robot->pos, x + dx, y + dy, position_get_a_deg_s16(&strat->robot->pos));

    std::string pos_str = "["
        + std::to_string(position_get_x_s16(&strat->robot->pos)) + ", "
        + std::to_string(position_get_y_s16(&strat->robot->pos)) + ","
        + std::to_string(position_get_a_deg_s16(&strat->robot->pos)) + "]";
    strat->log(pos_str.c_str());
}

static bool simulated_goto_xya(void* ctx, int x_mm, int y_mm, int a_deg)
{
    strategy_context_t* strat = (strategy_context_t*)ctx;
    position_set(&strat->robot->pos, x_mm, y_mm, a_deg);

    messagebus_topic_t* position_topic = messagebus_find_topic_blocking(&bus, "/simulated/position");
    RobotPosition pos = RobotPosition_init_zero;
    pos.x = x_mm;
    pos.y = y_mm;
    pos.a = a_deg;
    messagebus_topic_publish(position_topic, &pos, sizeof(pos));

    std::string pos_str = "["
        + std::to_string(position_get_x_s16(&strat->robot->pos)) + ", "
        + std::to_string(position_get_y_s16(&strat->robot->pos)) + ","
        + std::to_string(position_get_a_deg_s16(&strat->robot->pos)) + "]";
    strat->log(pos_str.c_str());

    return true;
}

struct _robot robot_simulated;

strategy_context_t strategy_simulated = {
    &robot_simulated,
    YELLOW,

    simulated_log,
    simulated_wait_ms,
    simulated_wait_for_user_input,

    simulated_forward,
    simulated_rotate,
    simulated_goto_xya,

    simulated_manipulator_goto,
    simulated_gripper_set,
    simulated_puck_is_picked,
};

strategy_context_t* strategy_simulated_impl(enum strat_color_t color)
{
    strategy_simulated.color = color;
    return &strategy_simulated;
}

static TOPIC_DECL(position_topic, RobotPosition);

void simulation_init(void)
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

    if (messagebus_find_topic(&bus, "/simulated/position") == NULL) {
        messagebus_advertise_topic(&bus, &position_topic.topic, "/simulated/position");
    }
}

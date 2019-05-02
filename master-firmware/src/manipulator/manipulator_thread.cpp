#include <ch.h>
#include <hal.h>

#include <algorithm>

#include <error/error.h>
#include <golem/golem.h>

#include "priorities.h"
#include "main.h"
#include "config.h"

#include "manipulator/manipulator.h"
#include "manipulator/manipulator_thread.h"
#include "dijkstra.hpp"

#include "protobuf/manipulator.pb.h"

#define MANIPULATOR_FREQUENCY 50
#define MANIPULATOR_TRAJECTORY_FREQUENCY 50

#define MANIPULATOR_THREAD_STACKSIZE 2048
#define MANIPULATOR_TRAJECTORY_THREAD_STACKSIZE 1024

using manipulator::Angles;
using manipulator::ArmLengths;
using manipulator::Pose2D;

// RAII to manage locks on the manipulator
class ManipulatorLockGuard {
private:
    mutex_t* lock;

public:
    ManipulatorLockGuard(void* l)
        : lock((mutex_t*)l)
    {
        chMtxLock(lock);
    }
    ~ManipulatorLockGuard()
    {
        chMtxUnlock(lock);
    }
};

MUTEX_DECL(right_lock);
manipulator::Manipulator<ManipulatorLockGuard> right_arm{{"theta-1", "theta-2", "theta-3"}, {0, 0, 0}, &right_lock};
MUTEX_DECL(left_lock);
manipulator::Manipulator<ManipulatorLockGuard> left_arm{{"left-theta-1", "left-theta-2", "left-theta-3"}, {0, 0, 0}, &left_lock};

void manipulator_angles(manipulator_side_t side, float* angles)
{
    auto measured = right_arm.angles();
    std::copy_n(std::begin(measured), 3, angles);
}

void manipulator_angles_set(manipulator_side_t side, float q1, float q2, float q3)
{
    Angles input = {q1, q2, q3};

    if (USE_RIGHT(side))
        right_arm.apply(input);

    if (USE_LEFT(side))
        left_arm.apply(input);
}

void manipulator_angles_wait_for_traj_end(manipulator_side_t side, uint16_t timeout_ms)
{
    uint16_t time_since_start_ms = 0;
    const uint16_t poll_time_step_ms = 10;

    while (time_since_start_ms < timeout_ms) {
        if (USE_RIGHT(side) && right_arm.reached_target())
            break;

        if (USE_LEFT(side) && left_arm.reached_target())
            break;

        chThdSleepMilliseconds(poll_time_step_ms);
        time_since_start_ms += poll_time_step_ms;
    }
}

void manipulator_angles_wait_for_traj_end_near(manipulator_side_t side, uint16_t timeout_ms)
{
    uint16_t time_since_start_ms = 0;
    const uint16_t poll_time_step_ms = 10;

    while (time_since_start_ms < timeout_ms) {
        if (USE_RIGHT(side) && right_arm.near_target())
            break;

        if (USE_LEFT(side) && left_arm.near_target())
            break;

        chThdSleepMilliseconds(poll_time_step_ms);
        time_since_start_ms += poll_time_step_ms;
    }
}

void manipulator_angles_goto_timeout(manipulator_side_t side, float q1, float q2, float q3, uint16_t timeout_ms)
{
    manipulator_angles_set(side, q1, q2, q3);
    manipulator_angles_wait_for_traj_end(side, timeout_ms);
}

bool manipulator_goto(manipulator_side_t side, manipulator_state_t target)
{
    int right_len, left_len;
    pathfinding::Node<manipulator::Point>* right_node;
    pathfinding::Node<manipulator::Point>* left_node;

    if (USE_RIGHT(side)) {
        right_len = pathfinding::dijkstra(right_arm.nodes, MANIPULATOR_COUNT, right_arm.nodes[right_arm.state], right_arm.nodes[target]);
        right_node = &right_arm.nodes[right_arm.state];
    }
    if (USE_LEFT(side)) {
        left_len = pathfinding::dijkstra(left_arm.nodes, MANIPULATOR_COUNT, left_arm.nodes[left_arm.state], left_arm.nodes[target]);
        left_node = &left_arm.nodes[left_arm.state];
    }

    if ((side == BOTH) && (right_len != left_len))
        return false;

    int len = USE_RIGHT(side) ? right_len : left_len;

    for (int i = 0; i < len; i++) {
        if (USE_RIGHT(side)) {
            right_node = right_node->path_next;
            manipulator_angles_set(RIGHT, right_node->data.angles[0], right_node->data.angles[1], right_node->data.angles[2]);
        }
        if (USE_LEFT(side)) {
            left_node = left_node->path_next;
            manipulator_angles_set(LEFT, left_node->data.angles[0], left_node->data.angles[1], left_node->data.angles[2]);
        }

        if (i == (len - 1)) { // last point
            if (USE_RIGHT(side))
                manipulator_angles_wait_for_traj_end(RIGHT, MANIPULATOR_DEFAULT_TIMEOUT_MS);
            if (USE_LEFT(side))
                manipulator_angles_wait_for_traj_end(LEFT, MANIPULATOR_DEFAULT_TIMEOUT_MS);
        } else {
            if (USE_RIGHT(side))
                manipulator_angles_wait_for_traj_end_near(RIGHT, MANIPULATOR_DEFAULT_TIMEOUT_MS);
            if (USE_LEFT(side))
                manipulator_angles_wait_for_traj_end_near(LEFT, MANIPULATOR_DEFAULT_TIMEOUT_MS);
        }
    }

    right_arm.state = target;
    return true;
}

void manipulator_gripper_set(manipulator_side_t side, gripper_state_t state)
{
    if (state == GRIPPER_ACQUIRE) {
        if (USE_RIGHT(side))
            right_arm.gripper.acquire();
        if (USE_LEFT(side))
            left_arm.gripper.acquire();
    } else if (state == GRIPPER_RELEASE) {
        if (USE_RIGHT(side))
            right_arm.gripper.release();
        if (USE_LEFT(side))
            left_arm.gripper.release();
    } else {
        if (USE_RIGHT(side))
            right_arm.gripper.disable();
        if (USE_LEFT(side))
            left_arm.gripper.disable();
    }
}

static void init_arm_parameters(manipulator::Manipulator<ManipulatorLockGuard>* arm, parameter_namespace_t* ns)
{
    arm->set_lengths({
        parameter_scalar_get(parameter_find(ns, "lengths/l1")),
        parameter_scalar_get(parameter_find(ns, "lengths/l2")),
        parameter_scalar_get(parameter_find(ns, "lengths/l3")),
    });
    arm->set_offsets({
        parameter_scalar_get(parameter_find(ns, "offsets/q1")),
        parameter_scalar_get(parameter_find(ns, "offsets/q2")),
        parameter_scalar_get(parameter_find(ns, "offsets/q3")),
    });
    arm->set_tolerance({
        parameter_scalar_get(parameter_find(ns, "tolerance/q1")),
        parameter_scalar_get(parameter_find(ns, "tolerance/q2")),
        parameter_scalar_get(parameter_find(ns, "tolerance/q3")),
    });
    arm->set_window({
        parameter_scalar_get(parameter_find(ns, "window/q1")),
        parameter_scalar_get(parameter_find(ns, "window/q2")),
        parameter_scalar_get(parameter_find(ns, "window/q3")),
    });
}

static void update_arm_parameters(manipulator::Manipulator<ManipulatorLockGuard>* arm, parameter_namespace_t* ns)
{
    arm->set_offsets({
        parameter_scalar_get(parameter_find(ns, "offsets/q1")),
        parameter_scalar_get(parameter_find(ns, "offsets/q2")),
        parameter_scalar_get(parameter_find(ns, "offsets/q3")),
    });
    arm->set_tolerance({
        parameter_scalar_get(parameter_find(ns, "tolerance/q1")),
        parameter_scalar_get(parameter_find(ns, "tolerance/q2")),
        parameter_scalar_get(parameter_find(ns, "tolerance/q3")),
    });
    arm->set_window({
        parameter_scalar_get(parameter_find(ns, "window/q1")),
        parameter_scalar_get(parameter_find(ns, "window/q2")),
        parameter_scalar_get(parameter_find(ns, "window/q3")),
    });

    arm->gripper.configure(parameter_scalar_get(parameter_find(ns, "gripper/release")),
                           parameter_scalar_get(parameter_find(ns, "gripper/acquire")));
}

static THD_FUNCTION(manipulator_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t* right_arm_params = parameter_namespace_find(&master_config, "arms/right");
    parameter_namespace_t* left_arm_params = parameter_namespace_find(&master_config, "arms/left");

    /* Setup and advertise manipulator state topic */
    static TOPIC_DECL(manipulator_topic, Manipulator);
    messagebus_advertise_topic(&bus, &manipulator_topic.topic, "/manipulator");
    Manipulator state = Manipulator_init_zero;

    init_arm_parameters(&right_arm, right_arm_params);
    init_arm_parameters(&left_arm, left_arm_params);

    NOTICE("Start manipulator thread");
    while (true) {
        if (parameter_namespace_contains_changed(right_arm_params)) {
            update_arm_parameters(&right_arm, right_arm_params);
        }
        if (parameter_namespace_contains_changed(left_arm_params)) {
            update_arm_parameters(&left_arm, right_arm_params);
        }

        Pose2D pose = right_arm.update();
        Angles input = right_arm.compute_control();
        // right_arm.apply(input);

        state.pose.x = pose.x;
        state.pose.y = pose.y;
        state.pose.heading = pose.heading;
        state.measured.q1 = right_arm.angles()[0];
        state.measured.q2 = right_arm.angles()[1];
        state.measured.q3 = right_arm.angles()[2];
        state.input.q1 = right_arm.sys.last_raw[0];
        state.input.q2 = right_arm.sys.last_raw[1];
        state.input.q3 = right_arm.sys.last_raw[2];
        messagebus_topic_publish(&manipulator_topic.topic, &state, sizeof(state));

        chThdSleepMilliseconds(1000 / MANIPULATOR_FREQUENCY);
    }
}

static THD_FUNCTION(manipulator_trajectory_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    NOTICE("Start manipulator trajectory manager thread");
    while (true) {
        chThdSleepMilliseconds(1000 / MANIPULATOR_TRAJECTORY_FREQUENCY);
    }
}

void manipulator_start(void)
{
    static THD_WORKING_AREA(manipulator_thd_wa, MANIPULATOR_THREAD_STACKSIZE);
    chThdCreateStatic(manipulator_thd_wa,
                      sizeof(manipulator_thd_wa),
                      MANIPULATOR_THREAD_PRIO,
                      manipulator_thd,
                      NULL);

    static THD_WORKING_AREA(manipulator_trajectory_thd_wa, MANIPULATOR_TRAJECTORY_THREAD_STACKSIZE);
    chThdCreateStatic(manipulator_trajectory_thd_wa,
                      sizeof(manipulator_trajectory_thd_wa),
                      MANIPULATOR_TRAJECTORY_THREAD_PRIO,
                      manipulator_trajectory_thd,
                      NULL);
}

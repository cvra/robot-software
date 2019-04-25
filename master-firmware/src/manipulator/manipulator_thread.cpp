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
manipulator::Manipulator<ManipulatorLockGuard> right_arm{{0, 0, 0}, &right_lock};

void manipulator_angles(float* angles)
{
    auto measured = right_arm.angles();
    std::copy_n(std::begin(measured), 3, angles);
}

void manipulator_angles_set(float q1, float q2, float q3)
{
    Angles input = {q1, q2, q3};
    right_arm.apply(input);
}

void manipulator_angles_wait_for_traj_end(uint16_t timeout_ms)
{
    uint16_t time_since_start_ms = 0;
    const uint16_t poll_time_step_ms = 10;

    while (!right_arm.reached_target() && time_since_start_ms < timeout_ms) {
        chThdSleepMilliseconds(poll_time_step_ms);
        time_since_start_ms += poll_time_step_ms;
    }
}

void manipulator_angles_goto_timeout(float q1, float q2, float q3, uint16_t timeout_ms)
{
    manipulator_angles_set(q1, q2, q3);
    manipulator_angles_wait_for_traj_end(timeout_ms);
}

bool manipulator_goto(manipulator_state_t target)
{
    int n = pathfinding::dijkstra(right_arm.nodes, MANIPULATOR_COUNT,
                                  right_arm.nodes[right_arm.state],
                                  right_arm.nodes[target]);

    pathfinding::Node<manipulator::Point>* node = &right_arm.nodes[right_arm.state];

    for (int i = 0; i < n; i++) {
        node = node->path_next;
        manipulator_angles_set(node->data.angles[0], node->data.angles[1], node->data.angles[2]);
        manipulator_angles_wait_for_traj_end(MANIPULATOR_DEFAULT_TIMEOUT_MS);
    }

    right_arm.state = target;
    return true;
}

void manipulator_gripper_set(gripper_state_t state)
{
    if (state == GRIPPER_ACQUIRE) {
        right_arm.gripper.acquire();
    } else if (state == GRIPPER_RELEASE) {
        right_arm.gripper.release();
    } else {
        right_arm.gripper.disable();
    }
}

static THD_FUNCTION(manipulator_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t* right_arm_params = parameter_namespace_find(&master_config, "arms/right");

    /* Setup and advertise manipulator state topic */
    static TOPIC_DECL(manipulator_topic, Manipulator);
    messagebus_advertise_topic(&bus, &manipulator_topic.topic, "/manipulator");
    Manipulator state = Manipulator_init_zero;

    right_arm.set_lengths({
        config_get_scalar("master/arms/right/lengths/l1"),
        config_get_scalar("master/arms/right/lengths/l2"),
        config_get_scalar("master/arms/right/lengths/l3"),
    });
    right_arm.set_offsets({
        config_get_scalar("master/arms/right/offsets/q1"),
        config_get_scalar("master/arms/right/offsets/q2"),
        config_get_scalar("master/arms/right/offsets/q3"),
    });
    right_arm.set_tolerance({
        config_get_scalar("master/arms/right/tolerance/q1"),
        config_get_scalar("master/arms/right/tolerance/q2"),
        config_get_scalar("master/arms/right/tolerance/q3"),
    });

    NOTICE("Start manipulator thread");
    while (true) {
        if (parameter_namespace_contains_changed(right_arm_params)) {
            right_arm.set_offsets({
                config_get_scalar("master/arms/right/offsets/q1"),
                config_get_scalar("master/arms/right/offsets/q2"),
                config_get_scalar("master/arms/right/offsets/q3"),
            });
            right_arm.set_tolerance({
                config_get_scalar("master/arms/right/tolerance/q1"),
                config_get_scalar("master/arms/right/tolerance/q2"),
                config_get_scalar("master/arms/right/tolerance/q3"),
            });

            right_arm.gripper.configure(config_get_scalar("master/arms/right/gripper/release"),
                                        config_get_scalar("master/arms/right/gripper/acquire"));
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

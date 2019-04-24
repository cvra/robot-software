#include <ch.h>
#include <hal.h>

#include <error/error.h>
#include <golem/golem.h>

#include "priorities.h"
#include "main.h"
#include "config.h"

#include "manipulator/manipulator.h"
#include "manipulator/manipulator_thread.h"

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

    NOTICE("Start manipulator thread");
    while (true) {
        if (parameter_namespace_contains_changed(right_arm_params)) {
            right_arm.set_offsets({
                config_get_scalar("master/arms/right/offsets/q1"),
                config_get_scalar("master/arms/right/offsets/q2"),
                config_get_scalar("master/arms/right/offsets/q3"),
            });
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
        state.input.q1 = input[0];
        state.input.q2 = input[1];
        state.input.q3 = input[2];
        messagebus_topic_publish(&manipulator_topic.topic, &state, sizeof(state));

        chThdSleepMilliseconds(1000 / MANIPULATOR_FREQUENCY);
    }
}

static THD_FUNCTION(manipulator_trajectory_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    Pose2D pose;
    pose.x = 0.17f;
    pose.y = 0.22f;
    pose.heading = 1.57f;
    right_arm.set_target(pose);

    int counter = 0;

    NOTICE("Start manipulator trajectory manager thread");
    while (true) {
        if (counter >= 2 * MANIPULATOR_FREQUENCY) {
            counter = 0;
            pose.y *= -1.f;
            pose.heading *= -1.f;
            right_arm.set_target(pose);
        }
        counter++;
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

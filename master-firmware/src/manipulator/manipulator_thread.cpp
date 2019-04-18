#include <ch.h>
#include <hal.h>

#include <error/error.h>
#include <golem/golem.h>

#include "priorities.h"
#include "main.h"
#include "config.h"

#include "manipulator/controller.h"
#include "manipulator/hw.h"
#include "manipulator/state_estimator.h"

#include "manipulator/manipulator_thread.h"

#include "protobuf/manipulator.pb.h"

#define MANIPULATOR_THREAD_STACKSIZE 2048

using manipulator::Angles;
using manipulator::ArmLengths;
using manipulator::Pose2D;

static THD_FUNCTION(manipulator_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    /* Setup and advertise encoders topic */
    static TOPIC_DECL(manipulator_topic, Manipulator);
    messagebus_advertise_topic(&bus, &manipulator_topic.topic, "/manipulator");
    Manipulator state = Manipulator_init_zero;

    const ArmLengths link_lengths = {{0.137, 0.097, 0.072}};

    manipulator::System sys;
    manipulator::StateEstimator estimator(link_lengths);
    manipulator::Controller ctrl(link_lengths);

    Pose2D pose;
    pose.x = 0.17f;
    pose.y = 0.22f;
    pose.heading = 1.57f;
    ctrl.set(pose);

    int counter = 0;
    NOTICE("Start manipulator thread");
    while (true) {
        if (counter >= 2. * MANIPULATOR_FREQUENCY) {
            counter = 0;
            pose.y *= -1.f;
            pose.heading *= -1.f;
            ctrl.set(pose);
        }

        estimator.update(sys.measure());
        Angles input = ctrl.update(estimator.get());
        sys.apply(input);

        state.pose.x = estimator.get().x;
        state.pose.y = estimator.get().y;
        state.pose.heading = estimator.get().heading;
        state.measured.q1 = sys.measure()[0];
        state.measured.q2 = sys.measure()[1];
        state.measured.q3 = sys.measure()[2];
        state.input.q1 = input[0];
        state.input.q2 = input[1];
        state.input.q3 = input[2];
        messagebus_topic_publish(&manipulator_topic.topic, &state, sizeof(state));

        counter++;
        chThdSleepMilliseconds(1000 / MANIPULATOR_FREQUENCY);
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
}

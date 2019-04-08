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

#define MANIPULATOR_THREAD_STACKSIZE 2048

using manipulator::Angles;
using manipulator::ArmLengths;
using manipulator::Pose2D;

static THD_FUNCTION(manipulator_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    const ArmLengths link_lengths = {{0.5, 0.4, 0.3}};

    manipulator::System sys;
    manipulator::StateEstimator estimator(link_lengths);
    manipulator::Controller ctrl(link_lengths);

    Pose2D pose;
    pose.x = 1.f;
    pose.y = -0.5f;
    pose.heading = -1.f;
    ctrl.set(pose);

    int counter = 0;
    NOTICE("Start manipulator thread");
    while (true) {
        if (counter >= MANIPULATOR_FREQUENCY) {
            counter = 0;
            pose.y *= -1.f;
            pose.heading *= -1.f;
            ctrl.set(pose);
        }

        estimator.update(sys.measure());
        sys.apply(ctrl.update(estimator.get()));

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

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

    golem::System<Angles, Angles> sys = manipulator::System();
    golem::StateEstimator<Pose2D, Angles> estimator = manipulator::StateEstimator(link_lengths);
    golem::Controller<Pose2D, Pose2D, Angles> ctrl = manipulator::Controller(link_lengths);

    NOTICE("Start manipulator thread");
    while (true) {
        estimator.update(sys.measure());
        sys.apply(ctrl.update(estimator.get()));

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

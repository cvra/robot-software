#include <CppUTest/TestHarness.h>

#include <golem/system.h>

#include "manipulator/controller.h"
#include "manipulator/state_estimator.h"

namespace {
void POSE_EQUAL(const manipulator::Pose2D& lhs, const manipulator::Pose2D rhs)
{
    DOUBLES_EQUAL(lhs.x, rhs.x, 1e-3);
    DOUBLES_EQUAL(lhs.y, rhs.y, 1e-3);
    DOUBLES_EQUAL(lhs.heading, rhs.heading, 1e-3);
}
} // namespace

namespace manipulator {
struct PerfectSystem : public golem::System<PerfectSystem, Angles, Angles> {
    Angles angles = {{0.f, 0.f, 0.f}};

    Angles measure_feedback() const
    {
        return angles;
    }
    void apply_input(const Angles& new_angles)
    {
        angles = new_angles;
    }
};
} // namespace manipulator

TEST_GROUP (AManipulator) {
    const manipulator::ArmLengths link_lengths = {{0.5, 0.4, 0.3}};

    manipulator::PerfectSystem sys;
    manipulator::StateEstimator estimator{link_lengths};
    manipulator::Controller ctrl{link_lengths};

    void run(size_t number_of_iterations)
    {
        for (size_t i = 0; i < number_of_iterations; i++) {
            estimator.update(sys.measure());
            sys.apply(ctrl.update(estimator.get()));
        }
    }
};

TEST(AManipulator, startsFine)
{
    auto state = estimator.get();
    POSE_EQUAL(state, {1.2f, 0.f, 0.f});
}

TEST(AManipulator, canReachFeasibleTarget)
{
    manipulator::Pose2D target{1.0f, 0.5f, 0.f};

    ctrl.set(target);
    run(1000);

    auto state = estimator.get();
    POSE_EQUAL(state, target);
}

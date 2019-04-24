#ifndef MANIPULATOR_MANIPULATOR_H
#define MANIPULATOR_MANIPULATOR_H

#include "manipulator/controller.h"
#include "manipulator/hw.h"
#include "manipulator/state_estimator.h"

namespace manipulator {
class Manipulator {
private:
    manipulator::System sys;
    manipulator::StateEstimator estimator;
    manipulator::Controller ctrl;

public:
    Manipulator(const std::array<float, 3>& link_lengths)
        : estimator(link_lengths)
        , ctrl(link_lengths)
    {
    }

    void set_lengths(const std::array<float, 3>& link_lengths)
    {
        estimator.lengths = link_lengths;
        ctrl.lengths = link_lengths;
    }
    void set_offsets(const Angles& offsets)
    {
        sys.offsets = offsets;
    }
    void set_target(const Pose2D& pose)
    {
        ctrl.set(pose);
    }

    Pose2D update(void)
    {
        estimator.update(sys.measure());
        return estimator.get();
    }
    Angles compute_control(void)
    {
        return ctrl.update(estimator.get());
    }
    void apply(const Angles& angles)
    {
        return sys.apply(angles);
    }

    Angles angles(void) const
    {
        return sys.measure();
    }
};
} // namespace manipulator

#endif /* MANIPULATOR_MANIPULATOR_H */

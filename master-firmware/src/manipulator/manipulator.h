#ifndef MANIPULATOR_MANIPULATOR_H
#define MANIPULATOR_MANIPULATOR_H

#include "manipulator/controller.h"
#include "manipulator/hw.h"
#include "manipulator/state_estimator.h"
#include "manipulator/gripper.h"

#include <math.h>

namespace manipulator {
template <class LockGuard>
class Manipulator {
public:
    manipulator::System sys;
    manipulator::StateEstimator estimator;
    manipulator::Controller ctrl;
    manipulator::Angles target_tolerance;
    void* mutex;

public:
    manipulator::Gripper gripper;

    Manipulator(const std::array<float, 3>& link_lengths, void* _mutex)
        : estimator(link_lengths)
        , ctrl(link_lengths)
        , mutex(_mutex)
    {
    }

    void set_lengths(const std::array<float, 3>& link_lengths)
    {
        LockGuard lock(mutex);
        estimator.lengths = link_lengths;
        ctrl.lengths = link_lengths;
    }
    void set_offsets(const Angles& offsets)
    {
        LockGuard lock(mutex);
        sys.offsets = offsets;
    }
    void set_target(const Pose2D& pose)
    {
        LockGuard lock(mutex);
        ctrl.set(pose);
    }
    void set_tolerance(const Angles& tolerances)
    {
        LockGuard lock(mutex);
        target_tolerance = tolerances;
    }

    Pose2D update(void)
    {
        LockGuard lock(mutex);
        estimator.update(sys.measure());
        return estimator.get();
    }
    Angles compute_control(void)
    {
        LockGuard lock(mutex);
        return ctrl.update(estimator.get());
    }
    void apply(const Angles& angles)
    {
        LockGuard lock(mutex);
        return sys.apply(angles);
    }

    Angles angles(void) const
    {
        LockGuard lock(mutex);
        return sys.measure();
    }
    bool reached_target(void) const
    {
        LockGuard lock(mutex);

        const Angles measured = sys.measure();
        const Angles consign = sys.last_raw;

        for (size_t i = 0; i < 3; i++)
        {
            if (fabsf(measured[i] - consign[i]) > target_tolerance[i])
            {
                return false;
            }
        }
        return true;
    }
};
} // namespace manipulator

#endif /* MANIPULATOR_MANIPULATOR_H */

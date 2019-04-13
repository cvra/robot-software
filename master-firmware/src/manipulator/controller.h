#ifndef MANIPULATOR_CONTROLLER_H
#define MANIPULATOR_CONTROLLER_H

#include "manipulator/kinematics.h"

#include <golem/controller.h>

namespace manipulator {
// Cartesian controller of the arm manipulator
struct Controller : public golem::Controller<Controller, Pose2D, Pose2D, Angles> {
    Pose2D target, measured;
    Angles target_angles = {{0.f, 0.f, 0.f}};
    std::array<float, 3> lengths;

    explicit Controller(const std::array<float, 3>& link_lengths);

    void set_consign(const Pose2D& target_pose);
    Angles compute_input(const Pose2D& state);
    Pose2D control_error() const;
};
} // namespace manipulator

#endif /* MANIPULATOR_CONTROLLER_H */

#ifndef MANIPULATOR_KINEMATICS_H
#define MANIPULATOR_KINEMATICS_H

#include <array>

namespace manipulator {
using Angles = std::array<float, 3>;
using ArmLengths = std::array<float, 3>;

struct Pose2D {
    float x;
    float y;
    float heading;
};
Pose2D operator-(const Pose2D& lhs, const Pose2D& rhs);

Pose2D forward_kinematics(const ArmLengths& lengths, const Angles& angles);
Angles inverse_kinematics(const ArmLengths& lengths, const Pose2D& pose);

// Transform back and forth from coupled/decoupled axes
// Coupled: standard pendulum where we sum angles explicitly [th1, th1 + th2, th1 + th2 + th3]
// Decoupled: every link has its axes set independently, no sum required
Angles axes_couple(const Angles& decoupled);
Angles axes_decouple(const Angles& coupled);
} // namespace manipulator

#endif /* MANIPULATOR_KINEMATICS_H */

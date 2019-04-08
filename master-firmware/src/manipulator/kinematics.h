#ifndef MANIPULATOR_KINEMATICS_H
#define MANIPULATOR_KINEMATICS_H

#include <array>

namespace manipulator {
using Angles = std::array<float, 3>;
using ArmLengths = std::array<float, 3>;

struct Pose2D {
    float x = 0.f;
    float y = 0.f;
    float heading = 0.f;
};
Pose2D operator-(const Pose2D& lhs, const Pose2D& rhs);

Pose2D forward_kinematics(const ArmLengths& lengths, const Angles& angles);
Angles inverse_kinematics(const ArmLengths& lengths, const Pose2D& pose);
} // namespace manipulator

#endif /* MANIPULATOR_KINEMATICS_H */

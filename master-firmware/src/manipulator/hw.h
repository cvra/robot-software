#ifndef MANIPULATOR_SYSTEM_H
#define MANIPULATOR_SYSTEM_H

#include "manipulator/kinematics.h"

#include <golem/system.h>

namespace manipulator {
struct System : public golem::System<System, Angles, Angles> {
    const char* motors[3] = {"theta-1", "theta-2", "theta-3"};

    Angles offsets = {{0.f, 0.f, 0.f}};
    std::array<float, 3> directions = {{-1.f, -1.f, 1.f}};

    Angles last_raw = {{0.f, 0.f, 0.f}};
    Angles last = {{0.f, 0.f, 0.f}};

    Angles measure_feedback() const;
    void apply_input(const Angles& angles);
};
} // namespace manipulator

#endif /* MANIPULATOR_SYSTEM_H */
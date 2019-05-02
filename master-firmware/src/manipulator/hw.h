#ifndef MANIPULATOR_SYSTEM_H
#define MANIPULATOR_SYSTEM_H

#include "manipulator/kinematics.h"

#include <golem/system.h>

namespace manipulator {
struct System : public golem::System<System, Angles, Angles> {
    const char* motors[3];

    Angles offsets = {{0.f, 0.f, 0.f}};
    std::array<float, 3> directions;

    Angles last_raw = {{0.f, 0.f, 0.f}};
    Angles last = {{0.f, 0.f, 0.f}};

    System(const std::array<const char*, 3>& names, const std::array<float, 3>& directions)
        : directions(directions)
    {
        for (size_t i = 0; i < 3; i++)
            motors[i] = names[i];
    }

    Angles measure_feedback() const;
    void apply_input(const Angles& angles);
};
} // namespace manipulator

#endif /* MANIPULATOR_SYSTEM_H */

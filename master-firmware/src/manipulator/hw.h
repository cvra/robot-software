#ifndef MANIPULATOR_SYSTEM_H
#define MANIPULATOR_SYSTEM_H

#include <array>

namespace manipulator {
struct System {
    const char* motors[3] = {"theta-1", "theta-2", "theta-3"};

    std::array<float, 3> offsets = {{0.f, 0.f, 0.f}};
    std::array<float, 3> directions = {{1.f, 1.f, 1.f}};

    std::array<float, 3> measure() const;
    void apply(const std::array<float, 3>& angles);
};
} // namespace manipulator

#endif /* MANIPULATOR_SYSTEM_H */

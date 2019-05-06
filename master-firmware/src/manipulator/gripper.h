#ifndef MANIPULATOR_GRIPPER_H
#define MANIPULATOR_GRIPPER_H

#include <array>

namespace manipulator {
struct Gripper {
private:
    float release_voltage, acquire_voltage;

public:
    const char* pumps[2];

    Gripper(const std::array<const char*, 2>& names)
    {
        for (size_t i = 0; i < 2; i++)
            pumps[i] = names[i];
    }

    void configure(float new_release_voltage, float new_acquire_voltage);
    void release() const;
    void acquire() const;
    void disable() const;
};
} // namespace manipulator

#endif /* MANIPULATOR_GRIPPER_H */

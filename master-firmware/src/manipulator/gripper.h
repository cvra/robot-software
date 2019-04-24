#ifndef MANIPULATOR_GRIPPER_H
#define MANIPULATOR_GRIPPER_H

namespace manipulator {
struct Gripper {
private:
    float release_voltage, acquire_voltage;

public:
    const char* pumps[2] = {"pump-1", "pump-2"};

    void configure(float new_release_voltage, float new_acquire_voltage);
    void release() const;
    void acquire() const;
    void disable() const;
};
} // namespace manipulator

#endif /* MANIPULATOR_GRIPPER_H */

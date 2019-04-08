#ifndef GOLEM_SYSTEM_H
#define GOLEM_SYSTEM_H

#include "crtp.h"

namespace golem {
// Models the hardware interface of your robot system
// The child class must implement two methods:
// - Feedback measure_feedback() const
// - void apply_input(const Input&)
template <typename SystemImpl, typename Feedback, typename Input>
class System : public Crtp<SystemImpl, System> {
public:
    Feedback measure() const
    {
        return this->underlying().measure_feedback();
    }
    void apply(const Input& in)
    {
        return this->underlying().apply_input(in);
    }
};
} // namespace golem

#endif /* GOLEM_SYSTEM_H */

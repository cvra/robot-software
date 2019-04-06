#ifndef GOLEM_CONTROLLER_H
#define GOLEM_CONTROLLER_H

#include "crtp.h"

namespace golem {
// Models a controller
// Accepts any class that implements the following functions for given State, Consign, and Input types:
// - void set_consign(const Consign&);
// - Input compute_input(const State&) const;
// - Consign control_error() const;
template <typename ControllerImpl, typename State, typename Consign, typename Input>
class Controller : public Crtp<ControllerImpl, Controller> {
public:
    void set(const Consign& target)
    {
        this->underlying().set_consign(target);
    }
    Input update(const State& state)
    {
        return this->underlying().compute_input(state);
    }
    Consign error() const
    {
        return this->underlying().control_error();
    }
};
} // namespace golem

#endif /* GOLEM_CONTROLLER_H */

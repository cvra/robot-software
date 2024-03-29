#include <iostream>
#include <vector>
#include <goap/goap.hpp>

struct LumberjackState {
    bool has_wood{false};
    bool has_axe{false};
};

bool operator==(const LumberjackState& b, const LumberjackState& a)
{
    return a.has_wood == b.has_wood && a.has_axe == b.has_axe;
}

struct ForestryGoal : goap::Goal<LumberjackState> {
    int distance_to(const LumberjackState& state) const override
    {
        return goap::Distance().shouldBeTrue(state.has_wood);
    }
};

class CutWood : public goap::Action<LumberjackState> {
public:
    bool can_run(const LumberjackState& state) override
    {
        return state.has_axe;
    }

    void plan_effects(LumberjackState& state) override
    {
        state.has_wood = true;
    }

    bool execute(LumberjackState& state) override
    {
        std::cout << "Cutting wood!" << std::endl;
        state.has_wood = true;
        return true;
    }
};

class GrabAxe : public goap::Action<LumberjackState> {
public:
    bool can_run(const LumberjackState& /*state*/) override
    {
        return true;
    }

    void plan_effects(LumberjackState& state) override
    {
        state.has_axe = true;
    }

    bool execute(LumberjackState& state) override
    {
        std::cout << "Grabbing my axe!" << std::endl;
        state.has_axe = true;
        return true;
    }
};

int main()
{
    CutWood cut_wood;
    GrabAxe grab_axe;
    ForestryGoal goal;
    LumberjackState state;

    std::vector<goap::Action<LumberjackState>*> actions{&cut_wood, &grab_axe};

    goap::Planner<LumberjackState> planner;

    constexpr int max_path_len = 10;
    goap::Action<LumberjackState>* path[max_path_len];

    auto len = planner.plan(state, goal, actions.data(), actions.size(), path, max_path_len);

    if (len == goap::kErrorNoPathFound) {
        std::cout << "Could not find a plan!" << std::endl;
        return 1;
    }

    if (len == goap::kErrorNotEnoughMemory) {
        std::cout << "Not enough memory reserved to explore all options!" << std::endl;
        return 1;
    }

    // Execute all the steps in the plan
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    return 0;
}

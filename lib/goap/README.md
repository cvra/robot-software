---
freshness:
  - owner: antoinealb
    reviewed: 2020-12-26
---

# GOAP C++ implementation

This folder contains an implementation of the GOAP algorithm described in "Three States and a plan: The AI of F.E.A.R" by Orkin (2006).
The idea of GOAP is to extend the finite state machines typically used for game agents.
In this model, instead of explicitely modelling transitions between states, constrains are expressed and a state machine is generated from those.
Only transitions that respect the constrains are expressed, and then a path to the desired end-state can be found.

## Example

### Defining the state

Suppose we have a lumberjack who wants to cut some wood.
To do that, our lumberjack needs an axe, that they can pick off the ground.

The GOAP state would be made of two booleans:

1. Does the lumberjack have an axe?
1. Is the wood cut?

And we would have two actions, as follow:

1. Cut wood. This has a prerequisite that the lumberjack has an axe.
1. Pickup an axe. This has no prerequisite, although we could argue for having a constraint where we are not allowed to pickup an axe if we already have one.

We can represent the state with the following C++ class (the whole code is in example.cpp).
We also need to implement a comparison operator, which is used by GOAP internally.

```cpp
struct LumberjackState {
    bool has_wood;
    bool has_axe;
};

bool operator==(const LumberjackState& b, const LumberjackState& a)
{
    return a.has_wood == b.has_wood && a.has_axe == b.has_axe;
}
```

### Defining the goal

Goals in GOAP are expressed as desired state.
In our implementation, we express those as a *distance* to a desired state.
This allows the planner to tell if an action gets closer to the goal or not, and therefore converge faster.
The distance should be zero when the goal is reached.
This is how it looks like in C++:

```cpp
struct ForestryGoal : goap::Goal<LumberjackState> {
    int distance_to(const LumberjackState& state) const override
    {
        if (state.has_wood) return 0;
        return 1;
    }
};
```

### Defining actions

Finally, our two actions can be defined. For each action, we must define three methods:

1. `plan_effects` which is used by the planner to see how one action changes the state.
1. `can_run` checks the constraints for one action by checking if the action is allowed to run on the given state.
1. `execute` is the function that actually implements your action and contains the code specific to your system.
    It returns a boolean to indicate if the action succeeded.

Here is how the "cut wood" action would look like:

```cpp
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
```

### Computing a plan

Now that we have everything required, we can finally compute a plan, and check that our plan indeed respect our constrains.

```cpp
    CutWood cut_wood;
    GrabAxe grab_axe;
    ForestryGoal goal;
    LumberjackState state;

    std::vector<goap::Action<LumberjackState>*> actions{&cut_wood, &grab_axe};

    goap::Planner<LumberjackState> planner;

    constexpr int max_path_len = 10;
    goap::Action<LumberjackState>* path[max_path_len];

    auto len = planner.plan(state, goal, actions.data(), actions.size(), path, max_path_len);

    // Execute all the steps in the plan
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }
```

### Conclusion

We designed a simple agent to take decisions using GOAP.
The complete code for this example can be found in `example.cpp`.
To build the example you will need CMake as well as a recent C++ compiler:

```
mkdir build && cd build
cmake ..
make example
./example
```

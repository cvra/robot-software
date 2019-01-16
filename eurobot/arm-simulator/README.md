# Arm Simulator

This package allows us to simulate robot arms using Hamiltonian dynamics & automatic differentiation.

## Quickstart

In a virtualenv run

```
pip install .
```

Then you can use the main class from the package in your Python code.
For example to define a pendulum

```py
from arm_simulator.hamilton import System
import numpy as np

EARTH_GRAVITY_CONSTANT = 9.81  # m/s^2
l = 2.0
mass = 0.42

# We need to define mapping from generalized to Cartesian coordinates, ..
f = lambda q: np.array([
        l * np.sin(q),
        - l * np.cos(q),
    ])

# .. the inertia matrix, ..
M = mass * np.eye(6)

# .. and the potential energy.
U = lambda q: mass * EARTH_GRAVITY_CONSTANT * f(q)[1]

# Then we can simply define our system
pendulum = System(f, M, U)
```

From that we can query the time derivatives of the generalized coordinates `q` and its conjugate moment `p`.
Which allows us to simulate the system.

## Examples

A few examples are provided under the examples folder
- [Double pendulum](examples/pendulum-double.ipynb)
- [Triple pendulum](examples/pendulum-triple.ipynb)

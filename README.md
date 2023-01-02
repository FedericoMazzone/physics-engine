# Physics Engine with PyGame

Simple physics engine for rigid spherical particles. It implements collision handling (with both elastic and partially inelastic collisions), viscous friction, and user-provided forces.

## Requirements

- `pip install pygame`
- `pip install pygame-gui`

## How to use

Run with `python3 main.py [number_of_entities]`.

To influence the simulation by adding acceleration:
- arrow keys for orthogonal direction,
- mouse click for radial direction (left-click for pull, right-click for push).

Use the sliders to control the viscous friction and the kinetic loss (leave it to 0 for elastic collisions).

Can you manage to create a stable orbit? ;)
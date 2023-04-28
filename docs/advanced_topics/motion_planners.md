---
layout: default
title: Motion Planners
parent: Advanced Topics
nav_order: 2
---

# Motion Planners

There are three broad categories of motion planners: analytical solvers, sampling-based solvers, and learned models. Machine learning is not directly covered by MoveIt, so we will be mostly focusing on the first two categories.

Each motion planning library utilizes different algorithms and strategies to compute a trajectory, and they all have different use cases and conditions for using them. To best optimize your motion planning means to understand which planner to use.

## Analytical solvers

Analytical solvers use some form of deterministic algorithm to compute a trajectory. There are various conditions required for a valid solution, but the output will be a shortest-path trajectory given the inputs.

Collision detection can be built into analytical solvers, however they are unable to perform obstacle avoidance. This means that an analytical solver can check if the computed trajectory would collide with an obstacle, but not compute an alternate path that avoids it.

Analytical solvers are especially useful for well-defined robot systems, such as industrial arms, for trajectories that are almost guaranteed to never collide with an obstacle.

### Pilz Industrial Motion Planner

One example of an analytical solver library is the [Pilz Industrial Motion Planner], which can plan standard robot motions such as PTP, LIN, and CIRC within MoveIt.

To use the planner, you will need to have joint and cartesian limits defined in your MoveIt configuration, in `joint_limits.yaml` and `pilz_cartesian_limits.yaml` respectively. These are already included in the code repository for the KR 6 robots.

If using the provided Grasshopper template, change the `pipeline_id` in the `/plan_kinematic_path` component to `pilz_industrial_motion_planner`.

Change the `planner_id` to one of the motion types (e.g., `PTP`, `LIN`).

If configuring a `moveit_msgs/srv/GetMotionPlan` message directly, you input these parameters under the `pipeline_id` and `planner_id` fields respectively.

## Sampling-based solvers

Sampling-based solvers are much more generalized that analytical solvers, and can be applied to almost any URDF-defined robot.

These solvers sample the trajectory space through trial-and-error over a set number of iterations, then returns the best path it can find. This means that sampling-based solvers are well-suited for complex systems where formulating an analytical solver is difficult, and for producing trajectories with obstacle avoidance.

The algorithms used in the solver determine how fast/efficient the solver runs, what sort of constraints it can handle, and how good the resulting trajectory is.

### RRT-Connect

The default planner in MoveIt is RRT Connect, which is part of the [OMPL library]. You can read more about the RRT-Connect algorithm [here](https://www.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf).

To use RRT-Connect, change the `planner_id` to `RRTConnect` and leave the `pipeline_id` empty.

When using RRT-Connect, you can define different constraints such as orientation constraints, trajectory constraints, and obstacles.

For example, to define an orientation constraint in COMPAS FAB:

```python
from compas.geometry import Frame
from compas_fab.robots import OrientationConstraint

frame = Frame([1, 1, 1], [0, 0, -1], [0, -1, 0])
oc = OrientationConstraint("link_6", frame.quaternion, tolerances=[0.1, 0.1, 0.1])
```

This constraint will limit the orientation of link 6 (effectively linked to the end-effector) straight downwards, which is useful when performing tasks such as assembly.

You can pass this `OrientationContraint` into the `path_constraints` in the provided `/plan_kinematic_path` component, or append it directly as a list to the request message.

```python
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg

path_constraints = convert_constraints_to_rosmsg(path_constraints, header).msg
request['path_constraints'] = path_constraints
```


[Pilz Industrial Motion Planner]: https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html
[OMPL library]: https://ompl.kavrakilab.org/

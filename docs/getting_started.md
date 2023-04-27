---
title: Getting Started
layout: default
nav_order: 3
---

# Getting Started

If you haven't already, please finish all of the required [installation](installation.html) steps.

The following guide will get you started with robotic fabrication on the FABLab's KUKA KR 6 robots.

## Code repository

All of the files and code you will need to get started are located in `src` folder of the [GitHub repository] (also linked at the top right of each page). You can download the entire repository under `Code > Download ZIP` or you can [click here](https://github.com/ADRLaboratory/robotic_fabrication_w23/archive/refs/heads/main.zip). You can also use [git] if you are familiar with it (although we won't be covering git here).

{: .note }
> `src` is short for "source" or "source code"

## Create a ROS workspace

Recall that we will be working with ROS through WSL. Open up a WSL terminal and run these commands:

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
mkdir -p ~/kr6/src
cd ~/kr6
colcon build
```

{: .note }
> If you are unfamiliar with UNIX commands / Shell scripting and would like to learn more, you can go through [this tutorial](https://www.tutorialspoint.com/unix/index.htm).
>
> In short, `echo "..." >> ~/.bashrc` saves a command to your `.bashrc` file, which executes each time your terminal opens. The first command sources (runs) the file `setup.bash` in the ROS package, which tells the terminal which directories to look at for locating ROS libraries. The second command configures WSL to enable graphics, which will be important later.
>
> The remaining commands creates a working directory `kr6` with the subfolder `src`, and builds an empty package (this will create some additional folders).


## Build ROS packages

Included in the provided code are three packages:
- `kr6_base_description` includes the [URDF] and mesh geometry of the KR 6 robots.
- `kr6_base_gazebo` includes the files required for running a [Gazebo] physics simulation environment.
- `kr6_base_moveit_config` includes the [MoveIt 2] motion planning and semantics files.

You will need to copy these packages (their folders) into 


[GitHub repository]: https://github.com/ADRLaboratory/robotic_fabrication_w23
[git]: https://git-scm.com/
[URDF]: http://wiki.ros.org/urdf
[Gazebo]: https://staging.gazebosim.org/home
[MoveIt 2]: https://moveit.picknik.ai/humble/index.html

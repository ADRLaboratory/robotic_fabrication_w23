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

Make sure to unblock the ZIP file before extracting! (`Right click > Properties > Unblock`)

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

We will now open this workspace in VS Code, which has terminal functionalities built-in so you will no longer need to use a separate WSL terminal.

In VS Code, make sure you have the [WSL extension installed](installtion.wsl.html#vs-code). Open the Command Palette (`Ctrl + Shift + P`) and run `WSL: Open Folder in WSL...`. Navigate and select the `kr6` folder you just made (`Ubuntu > home > <username> > kr6`).

To open a terminal in VS Code, go to `Terminal > New Terminal` or use the shortcut `Ctrl + Shift + \``. This terminal will work exactly the same ways as the WSL terminal you used previously.

Whenever you open a new terminal in the ROS workspace, make sure to source `install/setup.bash`:

```shell
source install/setup.bash
```

{: .note }
> Similar to how we sourced the `setup.bash` from the ROS package previously, this `setup.bash` tells the terminal where our local package files are. However, we don't want to add this to `~/.bashrc` to run automatically every time since you won't always be working with a local workspace that has a `install/setup.bash` file.

## Build ROS packages

Included in the provided code are three packages:
- `kr6_base_description` includes the [URDF] and mesh geometry of the KR 6 robots.
- `kr6_base_gazebo` includes the files required for running a [Gazebo] physics simulation environment.
- `kr6_base_moveit_config` includes the [MoveIt 2] motion planning and semantics files.

You will need to copy these packages (their folders) into the `src` folder of the ROS workspace. To do this, open Windows File Explorer and at the bottom of the left side bar you should see that WSL added a Linux shortcut. Navigate from there through `Ubuntu > home > <username> > kr6 > src`. You can now copy/paste the downloaded folders here.

Build the packages with the following command (all of the remaining commands can be run from a VS code terminal in the ROS workspace):

```shell
colcon build --symlink-install
```

{: .note }
> `--symlink-install` here is a tag that tells the package builder, colcon, to create a symbolic link to your source code. This means that for non-compiled files (such as XML, Python, etc.) the package contents will automatically update when you make changes in the source code, without you having to rebuild the entire package.
>
> You will still need to run this command again whenever you create new files that need to be added to the package.

{: .troubleshoot }
> Remember to `source install/setup.bash` first if you are in a new terminal!

## RViz visualization test

To test if the packages built correctly, we'll launch [RViz], a built-in graphical interface for ROS, and load in our robot.

```shell
ros2 launch kr6_base_description display.launch.py
```

{: .note }
> We will be using the `launch` command frequently, so it is useful to understand how it works. `ros2 launch` defines the command you are running (launching a file in ROS 2). The next parameter describes the package where the launch file is located, in this case `kr6_base_description`. The command will look for launch files in the `launch` folder of the package. The last parameter is the name of the launch file, which is typically written in Python, XML, or YAML. The launch file contains instructions for which ROS 2 nodes to create and with what paremeters.

{: .troubleshoot }
> If you get an error `[rviz2-3] qt.qpa.xcb: could not connect to display`, you may need to update WSL. In **administrator** PowerShell or Windows Command Prompt:
>
> ```shell
> wsl --update
> wsl --shutdown
> ```
>
> Then open a WSL terminal to restart WSL.

This command will open RViz and a Joint State Publisher GUI.

To display the robot, first we need to define the global frame. Under `Global Options`, select the dropdown for `Fixed Frame` and change it to `world`.

At the bottom left, select `Add` then `RobotModel`. Under the newly added `RobotModel`, select the dropdown for the `Description Topic` and change it to `/robot_description`.

You should now see the robot geometry in RViz and manipulate its joints using the sliders in the Joint State Publisher GUI!

{: .note }
> One of the nodes launched in the `display.launch.py` file is the Robot State Publisher (RSP) which publishes the robot's URDF to a topic called `/robot_description`. What RViz is doing is subscribing to that topic to then retrieve that URDF and visualize the robot.

To close any running processes, use `Ctrl + C` in the active terminal.

## MoveIt and Gazebo test

Test text


[GitHub repository]: https://github.com/ADRLaboratory/robotic_fabrication_w23
[git]: https://git-scm.com/
[URDF]: http://wiki.ros.org/urdf
[Gazebo]: https://staging.gazebosim.org/home
[MoveIt 2]: https://moveit.picknik.ai/humble/index.html
[RViz]: http://wiki.ros.org/rviz

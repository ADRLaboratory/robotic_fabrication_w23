---
layout: default
title: ROS 2
parent: Installation
nav_order: 2
---

# ROS 2

ROS is the Robot Operating System, and it is an open-source set of software frameworks for robotic systems. It is commonly used in robotics as middleware, serving as a communication network between different agents and processes. ROS 2 is the latest major revision of ROS, adding features such as integration with modern code libraries for core functions and real-time control.

The version of ROS 2 we will be using is [Humble Hawksbill] (ROS Humble).

## Installation

Part of this section has been adapted from the [ROS 2 Ubuntu installation guide].

All of the following commands will be run in your [WSL] terminal. To open a WSL terminal, either search for 'wsl' in the Windows Start Menu (it should be the first result that comes up), or run the command `wsl` in PowerShell or Windows Command Prompt.

### Set locale

First make sure that your locale supports `UTF-8`. This should be supported by default in WSL. Run the following commands:

```shell
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

{: .note }
> When using a `sudo` command for the first time in a fresh terminal, you will be prompted to enter your password.

### Set up sources

These next few commands add the ROS 2 apt repository to your system so that you can install the relevant packages.

Enable the Ubuntu Universe repository.

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key.

```shell
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to your sources list.

```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 packages

Always update your apt repository caches and upgrade your system before installing new packages.

```shell
sudo apt update
sudo apt upgrade
```

Install the Humble desktop package.

```shell
sudo apt install ros-humble-desktop
```

{: .troubleshoot }
> If at this point you get an error saying that `The repository ... is not signed.`, you may have a conflict with another installation of Linux. This most often was an issue with Docker - uninstall Docker and restart your computer. Then start again from "Add the ROS 2 GPG key".

{: .troubleshoot }
> If you see that the package installation is aborting, starting a new WSL terminal might help.

Install the remaining packages that we will be using.

```shell
sudo apt install ros-humble-simulation ros-dev-tools ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-rosbridge-server ros-humble-ros-gz ros-humble-ros-ign-bridge ros-humble-moveit ros-humble-ign-ros2-control ros-humble-gazebo-ros2-control ros-humble-ros2-controllers
```




[Humble Hawksbill]: https://docs.ros.org/en/humble/index.html#
[ROS 2 Ubuntu installation guide]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
[WSL]: wsl.html

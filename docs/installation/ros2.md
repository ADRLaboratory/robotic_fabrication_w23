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

Test


[Humble Hawksbill]: https://docs.ros.org/en/humble/index.html#
[ROS 2 Ubuntu installation guide]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
[WSL]: wsl.html

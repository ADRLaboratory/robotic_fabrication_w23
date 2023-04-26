---
layout: default
title: WSL
parent: Installation
nav_order: 1
---

# Windows Subsystem for Linux

The Windows Subsystem for Linux (WSL) is a Linux environment developed to run directly on Windows. WSL operates similarly to a traditional virtual machine but without the increased overhead. We will be using WSL to build and run [ROS] packages.

{: .note }
> If your machine is already running a version of Linux or has a Linux virtual machine set up, you don't need to follow this installation. However, some future steps may differ.
> MacOS is currently not supported.

## Installation

This section has been adapted from the [WSL installation documentation].

The latest version of WSL is WSL 2, which is the default option for new installations.

Open [PowerShell] or Windows Command Prompt in **administrator** mode, run the following command, then restart your computer.

```shell
wsl --install
```

{: .troubleshoot }
If you have WSL installed already, the above command will only show you the WSL help text. Run `wsl --list --online` to see if you have any distributions available. If none are listed, run `wsl --install -d Ubuntu` to install Ubuntu.

To test the installation, run `wsl -l -v` in PowerShell or Windows Command Prompt, and you should see the distribution running.


[ROS]: "/installation/ros2.md"
[WSL installation documentation]: "https://learn.microsoft.com/en-us/windows/wsl/install"
[PowerShell]: https://learn.microsoft.com/en-us/powershell/scripting/install/installing-powershell-on-windows

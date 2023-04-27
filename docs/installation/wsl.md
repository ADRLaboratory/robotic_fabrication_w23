---
layout: default
title: WSL
parent: Installation
nav_order: 1
---

# Windows Subsystem for Linux

The [Windows Subsystem for Linux] (WSL) is a Linux environment developed to run directly on Windows. WSL operates similarly to a traditional virtual machine but without the increased overhead. We will be using WSL to build and run [ROS] packages.

{: .note }
> If your machine is already running a version of Linux or has a Linux virtual machine set up, you don't need to follow this installation. However, some future steps may differ.
>
> Documentation for MacOS is currently not implemented.

## Installation

This section has been adapted from the [WSL installation documentation].

The latest version of WSL is WSL 2, which is the default option for new installations.

Open [PowerShell] or Windows Command Prompt in **administrator** mode, run the following command, then restart your computer.

```shell
wsl --install
```

{: .troubleshoot }
> If you have WSL installed already, the above command will only show you the WSL help text. Run `wsl --list --online` to see if you have any distributions available. If none are listed, run `wsl --install -d Ubuntu` to install Ubuntu.

During the installation, you will be prompted to create a user profile. Enter a username and password to continue.

{: .warning }
> Special characters in your username (such as dashes, spaces, etc.) may cause issues later on. Stick to something simple.

{: .note }
> When entering your password, the terminal will hide the text you are entering (for security reasons), so it might look like nothing is happening. Be assured that all your keystrokes (including backspace) still work as normal. If in doubt, backspace a large number of times to start over.
>
> This password can be different from your Windows password - just make sure to remember it!

To test the installation, run `wsl -l -v` in PowerShell or Windows Command Prompt, and you should see the distribution running.

## VS Code

We will mostly be working with our code in Visual Studio Code (VS Code). If you do not have it yet, you can install it [here](https://code.visualstudio.com/).

To make working with WSL environments in VS Code easier, search for and install the [WSL extension] (published by Microsoft).

----

Next step: [Install ROS](ros2.html)


[Windows Subsystem for Linux]: https://learn.microsoft.com/en-us/windows/wsl/
[ROS]: ros2.html
[WSL installation documentation]: https://learn.microsoft.com/en-us/windows/wsl/install
[PowerShell]: https://learn.microsoft.com/en-us/powershell/scripting/install/installing-powershell-on-windows
[WSL extension]: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl

---
layout: default
title: COMPAS FAB
parent: Installation
nav_order: 3
---

# COMPAS FAB

[COMPAS] is an open source python library used for research and collaboration in Architecture, Engineering, Fabrication, and Construction. [COMPAS FAB] is a package that extends the COMPAS framework to interface with various software libaries and tools used in robotics (e.g., ROS). Using the two together, we will be able to access our ROS environment from within a [Rhino 3D] CAD environment.

COMPAS FAB is actively developed and maintained by [Gramazio Kohler Research].

## Installation

Part of this section has been adapted from the [COMPAS FAB Getting started guide].

### Anaconda

One of the best practices for working with code libraries is to set up virtual environments. Virtual environments in Python isolate your interpreter, libraries, and scripts from other virtual environments, making it easier to develop for different projects that might require different Python or library versions.

Anaconda is a popular platform for creating and managing Python virtual environments. [Download and install Anaconda here](https://www.anaconda.com/) if you don't have it (or one of its variants) already.

### Create a COMPAS FAB environment

To open an Anaconda terminal, search for `Anaconda Prompt (Anaconda 3)` in the Windows Start Menu. Run this in **administrator** mode.

Create a new enviornment with both COMPAS and COMPAS FAB.

```shell
conda create -n robfab -c conda-forge compas_fab
```

{: .note }
> Note that we've named our environment `robfab`. You can replace this with a different name if you'd like, but we will be referring to this virtual environment as `robfab` in the rest of the documentation.

You will be prompted a few times to proceed with the installation. After the installation finishes (it can take some time), run the following command to check if the installation was successful.

```shell
conda activate robfab
python -m compas_fab
```

### Install COMPAS FAB for Rhino

To access the COMPAS FAB library inside of Rhino/Grasshopper, run the following command:

```shell
python -m compas_rhino.install
```

{: .note }
> When working with COMPAS FAB, make sure you have the environment activated (by using `conda activate robfab`). You will need to do this when starting a new terminal.
>
> You can also configure the default interpreter in VS Code by pulling up the Command Palette (`Ctrl+Shift+P`), typing in `Python: Select Interpreter`, and select the `robfab` environment.

To check the installation, open Rhino, start the Python script editor (`EditPythonScript`), and you should be able to `import compas_fab`. When opening Grasshopper, you should also see some default COMPAS FAB components.

----

Next step: [Getting Started](../getting_started.html)


[COMPAS]: https://compas.dev/index.html
[COMPAS FAB]: https://gramaziokohler.github.io/compas_fab/latest/
[Rhino 3D]: https://www.rhino3d.com/
[Gramazio Kohler Research]: https://gramaziokohler.arch.ethz.ch/web/e/forschung/index.html
[COMPAS FAB Getting started guide]: https://gramaziokohler.github.io/compas_fab/latest/getting_started.html

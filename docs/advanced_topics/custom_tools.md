---
layout: default
title: Custom Tools
parent: Advanced Topics
nav_order: 1
---

# Custom Tools

To define a custom tool (end-effector) in COMPAS FAB, you will need a mesh file of the tool and its TCP.

COMPAS FAB supports STL and OBJ files, with STL preferred. The TCP is defined as a COMPAS Frame

{: .note }
> Make sure that the units are in meters!

To attach a tool to a defined robot:

```python
from compas.datastructures import Mesh
from compas_fab.robots import Tool

tool_mesh = Mesh.from_stl(tool_stl_filepath)
tool = Tool(tool_mesh, tool_frame, link_name='flange')
robot.attach_tool(tool)

```

Where `robot` is an instance of `compas_fab.robots.Robot`.

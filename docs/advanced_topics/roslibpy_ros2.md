---
layout: default
title: Using roslibpy with ROS 2
parent: Advanced Topics
nav_order: 3
---

# Using roslibpy with ROS 2

The [Python ROS Bridge library] (roslibpy) is an open source library that connects Python and IronPython to ROS, developed and maintained by [Gramazio Kohler Research].

One of the issues, however, is that roslibpy was written to work with ROS 1, and several of the core ROS functionalities that it accesses has changed in ROS 2.

To get roslibpy to work with ROS 2, we need to incorporate some tricks.

## Working with messages

Some message types in ROS utilize a header, which defines standard metadata such as a sequence number, timestamp, and an associated frame.

In ROS 1, the timestamp uses `sec` and `nanosec`, while ROS 2 renames them to `secs` and `nsecs` respectively. This means that the automatic message creation in roslibpy will cause errors if directly passed into a service, as will reading incoming messages.

To address this, we need to iterate through the message keys and rename them appropriately. One example of how to do this is shown below.

```python
def rename_keys(d):
    if isinstance(d, list) and not isinstance(d, basestring):
        for i in d:
            rename_keys(i)
    if isinstance(d, dict):
        for key, value in d.items():
            if key == 'nanosec':
                d['nsecs'] = d.pop('nanosec')
            if key == 'sec':
                d['secs'] = d.pop('sec')
            rename_keys(value)
```

## Parameter server

In ROS 1, there always exists a 'master' agent where the ROS core operates from, and this agent also hosts a global parameter server that other nodes can access.

One of the major changes in ROS 2 aims to support distributed multi-agent systems, of which might have unstable connectivity. This means a centralized parameter server no longer makes sense, and so all parameters are now associated with a node.

What this effectively means for roslibpy is that instead of accessing global parameters, you instead need to make a service call to retrieve that parameter instead. See the next section for more details.

## Using services

Because we are working with custom messages, we also need to make any service calls manually.

```python
import roslibpy as rlp

service = rlp.Service(ros_client, service_name, msg_type)
request = rlp.ServiceRequest(request_msg)
response = planner_service.call(request)
```

Where `ros_client` is the connected rosbridge instance of `compas_fab.backends.RosClient`.

For example, to make a call to the `/plan_kinematic_path` service in MoveIt:

```python
# Kinematic path motion planning request
service = rlp.Service(ros_client, '/plan_kinematic_path', 'moveit_msgs/srv/GetMotionPlan')
request = rlp.ServiceRequest(request_msg)
response = planner_service.call(request)
```

To determine the message type of a service, you can use a command in a terminal while the service is running, for example:

```shell
ros2 service type /plan_kinematic_path
```

Outputs:

```shell
moveit_msgs/srv/GetMotionPlan
```

You can then use this to search for the corresponding API documentation to figure out the exact fields required.

## Using actions

Actions have been completely reworked between ROS 1 and 2. Development for supporting ROS 2 actions with roslibpy is still underway!


[Python ROS Bridge library]: https://roslibpy.readthedocs.io/en/latest/
[Gramazio Kohler Research]: https://gramaziokohler.arch.ethz.ch/web/e/forschung/index.html

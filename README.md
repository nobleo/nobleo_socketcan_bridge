<!--
Copyright (C) 2024 Nobleo Technology B.V.

SPDX-License-Identifier: Apache-2.0
-->

# ros2_socketcan_bridge

This package provides functionality to expose CAN frames from SocketCAN to ROS2 topics.

## Overview

This is a from-scratch re-implementation of [socketcan_bridge] from ROS1.
There is a different ROS2 package [ros2_socketcan] which is similar to this package.
The differences between this package and [ros2_socketcan] are:

- No loopback, i.e. CAN frames that are send are not received by the same node
- No lifecycle management, just run it
- Less CPU usage

## Nodes

The main node (`socketcan_bridge`) is also available as dynamically loadable component.
The node `socketcan_bridge_ee` is also provided that uses the `EventsExecutor` that runs a bit more efficient.

### socketcan_bridge

#### Subscribed Topics

* `~/tx` ([can_msgs/Frame])
  Messages received here will be sent to the SocketCAN device.

#### Published Topics

* `~/rx` ([can_msgs/Frame])
  Frames received on the SocketCAN device are published on this topic.

#### Parameters

* `interface` (default=`can0`)
  Name of the SocketCAN device, by default these devices are named can0 and upwards.
* `read_timeout` (default=`1.0`)
  Maximum duration in seconds to wait for data on the file descriptor
* `reconnect_timeout` (default=`5.0`)
  Sleep duration in seconds before reconnecting to the SocketCAN device

[can_msgs/Frame]: https://github.com/ros-industrial/ros_canopen/blob/dashing-devel/can_msgs/msg/Frame.msg

[socketcan_bridge]: https://wiki.ros.org/socketcan_bridge

[ros2_socketcan]: https://github.com/autowarefoundation/ros2_socketcan

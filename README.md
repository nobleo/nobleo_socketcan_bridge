<!--
Copyright (C) 2024 Nobleo Technology B.V.

SPDX-License-Identifier: Apache-2.0
-->

# ros2_socketcan_bridge
The packages provides functionality to expose CAN frames from SocketCAN to a ROS Topic.

## Overview
Differences between this package and `ros2_socketcan`

- No loopback, i.e. CAN frames that are send are not received by the same node
- No lifecycle management, just run it
- Less CPU usage

## Nodes
The main node (`socketcan_bridge`) is also available as dynamically loadable component.
The node `socketcan_bridge_ee` is also provided that uses the `EventsExecutor` that run a bit more efficient.

### socketcan_bridge

#### Subscribed Topics

* `~/tx` ([can_msgs/Frame])
  Messages received here will be sent to the SocketCAN device.

#### Published Topics
* `~/rx` ([can_msgs/Frame])
  Frames received on the SocketCAN device are published in this topic.

#### Parameters

* `interface`
  Name of the SocketCAN device, by default these devices are named can0 and upwards.
  
* `read_timeout`
  Maximum duration to wait for data on the file descriptor

[can_msgs/Frame]: https://github.com/ros-industrial/ros_canopen/blob/dashing-devel/can_msgs/msg/Frame.msg

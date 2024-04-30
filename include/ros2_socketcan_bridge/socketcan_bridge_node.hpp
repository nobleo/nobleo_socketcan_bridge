// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "rclcpp/node.hpp"
#include "ros2_socketcan_bridge/socketcan_bridge.hpp"

namespace ros2_socketcan_bridge
{
class SocketCanBridgeNode : public rclcpp::Node
{
public:
  explicit SocketCanBridgeNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  SocketCanBridge bridge;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
};
}  // namespace ros2_socketcan_bridge

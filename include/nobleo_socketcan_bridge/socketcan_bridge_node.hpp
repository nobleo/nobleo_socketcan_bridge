// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "nobleo_socketcan_bridge/socketcan_bridge.hpp"
#include "rclcpp/node.hpp"

namespace nobleo_socketcan_bridge
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
}  // namespace nobleo_socketcan_bridge

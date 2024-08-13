// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include "nobleo_socketcan_bridge/socketcan_bridge_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace nobleo_socketcan_bridge
{

SocketCanBridgeNode::SocketCanBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("socketcan_bridge", options),
  updater_(this),
  can_pub(this->create_publisher<can_msgs::msg::Frame>("~/rx", 100)),
  bridge(
    this->get_logger(), this->get_clock(), this->declare_parameter("interface", "can0"),
    this->declare_parameter("read_timeout", 1.0), this->declare_parameter("reconnect_timeout", 5.0),
    [this](const can_msgs::msg::Frame & msg) { can_pub->publish(msg); }),
  can_sub(this->create_subscription<can_msgs::msg::Frame>(
    "~/tx", 100, [this](can_msgs::msg::Frame::ConstSharedPtr msg) { bridge.send(*msg); }))
{
  updater_.setHardwareID("SocketCan");
  updater_.add("SocketCan", [this](auto & stat) { this->produceDiagnostics(stat); });
}

void SocketCanBridgeNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  auto can_state = bridge.getState();
  switch (can_state) {
    case CanState::OKAY:
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "CAN interface is up");
      break;
    case CanState::WARN:
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "CAN interface is in warning state");
      break;
    case CanState::ERROR:
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "CAN interface is in error state");
      break;
    case CanState::FATAL:
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error connecting to CAN interface");
      break;
  }
}

}  // namespace nobleo_socketcan_bridge
RCLCPP_COMPONENTS_REGISTER_NODE(nobleo_socketcan_bridge::SocketCanBridgeNode)

// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include "nobleo_socketcan_bridge/socketcan_bridge_node.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

using nobleo_socketcan_bridge::SocketCanBridgeNode;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SocketCanBridgeNode>(rclcpp::NodeOptions{});
  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

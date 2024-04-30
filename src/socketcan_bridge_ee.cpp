#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include "nobleo_socketcan_bridge/socketcan_bridge_node.hpp"

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

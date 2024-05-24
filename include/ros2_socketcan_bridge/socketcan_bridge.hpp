// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <thread>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

// forward declarations
struct can_frame;
namespace rclcpp
{
class Node;
class Clock;
}  // namespace rclcpp

namespace ros2_socketcan_bridge
{
class SocketCanBridge
{
public:
  using CanCallback = std::function<void(const can_msgs::msg::Frame &)>;

  SocketCanBridge(
    const rclcpp::Logger & logger, rclcpp::Clock::SharedPtr clock, const std::string & interface,
    double read_timeout, const CanCallback & receive_callback);

  ~SocketCanBridge();

  SocketCanBridge(const SocketCanBridge &) = delete;
  SocketCanBridge(SocketCanBridge &&) noexcept = delete;
  SocketCanBridge & operator=(const SocketCanBridge & other) = delete;
  SocketCanBridge & operator=(SocketCanBridge && other) noexcept = delete;

  void send(const can_msgs::msg::Frame & msg) const;

  void close();

private:
  void receive_loop(std::stop_token stoken) const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  int socket_;
  CanCallback receive_callback_;
  std::jthread receive_thread_;
};

can_frame from_msg(const can_msgs::msg::Frame & msg);

can_msgs::msg::Frame to_msg(const can_frame & frame);
;

}  // namespace ros2_socketcan_bridge

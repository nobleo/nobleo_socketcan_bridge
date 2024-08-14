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

namespace nobleo_socketcan_bridge
{
enum class CanState
{
  OKAY,
  WARN,
  ERROR,
  FATAL
};

class SocketCanBridge
{
public:
  using CanCallback = std::function<void(const can_msgs::msg::Frame &)>;

  SocketCanBridge(
    const rclcpp::Logger & logger, rclcpp::Clock::SharedPtr clock, const std::string & interface,
    double read_timeout, double reconnect_timeout, const CanCallback & receive_callback);

  ~SocketCanBridge();

  SocketCanBridge(const SocketCanBridge &) = delete;
  SocketCanBridge(SocketCanBridge &&) noexcept = delete;
  SocketCanBridge & operator=(const SocketCanBridge & other) = delete;
  SocketCanBridge & operator=(SocketCanBridge && other) noexcept = delete;

  void send(const can_msgs::msg::Frame & msg) const;

  CanState getState() const { return state_; }

  void close();

private:
  void connect();
  void ensure_connection(std::stop_token stoken);
  void receive_loop(std::stop_token stoken);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string interface_;
  double read_timeout_;
  double reconnect_timeout_;
  int socket_;
  CanCallback receive_callback_;
  std::jthread receive_thread_;
  CanState state_;
};

can_frame from_msg(const can_msgs::msg::Frame & msg);

can_msgs::msg::Frame to_msg(const can_frame & frame);
;

}  // namespace nobleo_socketcan_bridge

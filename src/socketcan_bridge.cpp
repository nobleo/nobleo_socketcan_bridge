// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include "ros2_socketcan_bridge/socketcan_bridge.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cmath>

#include "rclcpp/logging.hpp"

namespace ros2_socketcan_bridge
{
std::ostream & operator<<(std::ostream & os, const can_msgs::msg::Frame & msg)
{
  fmt::print(os, "{:0>3X} [{}]", msg.id, msg.dlc);
  for (auto i = 0; i < msg.dlc; ++i) {
    fmt::print(os, " {:0>2X}", msg.data[i]);
  }
  return os;
}

SocketCanBridge::SocketCanBridge(
  const rclcpp::Logger & logger, rclcpp::Clock::SharedPtr clock, const std::string & interface,
  double read_timeout, const CanCallback & receive_callback)
: logger_(logger), clock_(clock), receive_callback_(receive_callback)
{
  using std::placeholders::_1;

  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    throw std::system_error(errno, std::generic_category());
  }

  // When SIGINT is received, the program needs to quit. But a `read()` call can't be interrupted.
  // So I use SO_RCVTIMEO to make all reads return within `read_timeout` seconds.
  auto microseconds = std::lround(read_timeout * 1'000'000);
  timeval tv;
  tv.tv_sec = microseconds / 1'000'000;
  tv.tv_usec = microseconds - (tv.tv_sec * 1'000'000);
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

  ifreq ifr;
  strncpy(ifr.ifr_name, interface.c_str(), sizeof(ifr.ifr_name));
  if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    throw std::system_error(errno, std::generic_category());
  }

  sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    throw std::system_error(errno, std::generic_category());
  }

  receive_thread_ = std::jthread{std::bind(&SocketCanBridge::receive_loop, this, _1)};
}

void SocketCanBridge::send(const can_msgs::msg::Frame & msg)
{
  can_frame frame;
  frame.can_id = msg.id;
  frame.len = msg.dlc;
  if(msg.is_extended)
  {
    frame.can_id |= CAN_EFF_FLAG;
  }
  std::copy(msg.data.begin(), msg.data.end(), frame.data);
  RCLCPP_DEBUG_STREAM(logger_, "Sending " << msg);
  auto n = write(socket_, &frame, sizeof(frame));
  if (n < 0) {
    throw std::system_error(errno, std::generic_category());
  }
  RCLCPP_DEBUG(logger_, "Wrote %zd bytes to the socket", n);
}

void SocketCanBridge::close()
{
  RCLCPP_INFO(logger_, "Stopping the receive thread");
  receive_thread_.request_stop();

  RCLCPP_INFO(logger_, "Waiting for the receive thread to stop");
  receive_thread_.join();

  RCLCPP_INFO(logger_, "Closing the socket");
  if (::close(socket_) < 0) {
    throw std::system_error(errno, std::generic_category());
  }
}

void SocketCanBridge::receive_loop(std::stop_token stoken)
{
  RCLCPP_INFO(logger_, "Receive loop started");
  while (!stoken.stop_requested()) {
    can_frame frame;
    auto nbytes = read(socket_, &frame, sizeof(can_frame));
    if (nbytes < 0) {
      if (errno == EAGAIN) continue;
      throw std::system_error(errno, std::generic_category());
    }
    if (nbytes != sizeof(frame)) {
      throw std::runtime_error("Partial CAN frame received");
    }
    const bool ext = ((frame.can_id & CAN_EFF_FLAG) == CAN_EFF_FLAG);
    can_msgs::msg::Frame msg;
    msg.header.stamp = clock_->now();
    msg.id = frame.can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK);
    msg.dlc = frame.len;
    msg.is_extended = ext;
    std::copy_n(frame.data, sizeof(frame.data), msg.data.begin());

    RCLCPP_DEBUG_STREAM(logger_, "Received " << msg);
    receive_callback_(msg);
  }
  RCLCPP_INFO(logger_, "Receive loop stopped");
}

}  // namespace ros2_socketcan_bridge

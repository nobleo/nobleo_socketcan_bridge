// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include "nobleo_socketcan_bridge/socketcan_bridge.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cmath>

#include "rclcpp/logging.hpp"

namespace nobleo_socketcan_bridge
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
  double read_timeout, double reconnect_timeout, const CanCallback & receive_callback)
: logger_(logger),
  clock_(clock),
  interface_(interface),
  read_timeout_(read_timeout),
  reconnect_timeout_(reconnect_timeout),
  receive_callback_(receive_callback)
{
  using std::placeholders::_1;
  receive_thread_ = std::jthread{std::bind(&SocketCanBridge::receive_loop, this, _1)};
}

SocketCanBridge::~SocketCanBridge()
{
  try {
    close();
  } catch (const std::system_error & e) {
    // closing the socket could fail, but we should not propagate exceptions out of the destructor
    RCLCPP_ERROR(logger_, "%s", e.what());
  }
}

void SocketCanBridge::send(const can_msgs::msg::Frame & msg) const
{
  auto frame = from_msg(msg);
  RCLCPP_DEBUG_STREAM(logger_, "Sending " << msg);
  auto n = write(socket_, &frame, sizeof(frame));
  if (n < 0) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "Error writing to the socket: %s (%d)", strerror(errno), errno);
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

void SocketCanBridge::connect()
{
  RCLCPP_INFO(logger_, "Connecting to the CAN interface %s ..", interface_.c_str());

  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    throw std::system_error(errno, std::generic_category());
  }

  // When SIGINT is received, the program needs to quit. But a `read()` call can't be interrupted.
  // So I use SO_RCVTIMEO to make all reads return within `read_timeout` seconds.
  auto microseconds = std::lround(read_timeout_ * 1'000'000);
  timeval tv;
  tv.tv_sec = microseconds / 1'000'000;
  tv.tv_usec = microseconds - (tv.tv_sec * 1'000'000);
  if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv) < 0) {
    throw std::system_error(errno, std::generic_category());
  }

  can_err_mask_t error_mask =
    (CAN_ERR_BUSOFF      /* bus off */
     | CAN_ERR_RESTARTED /* controller restarted */
     | CAN_ERR_CRTL      /* controller problems / data[1]    */
    );
  if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0) {
    RCLCPP_ERROR(logger_, "Error setting error mask");
    throw std::system_error(errno, std::generic_category());
  }

  ifreq ifr;
  strncpy(ifr.ifr_name, interface_.c_str(), sizeof(ifr.ifr_name));
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

  RCLCPP_INFO(logger_, "Connected to the CAN interface %s", interface_.c_str());
  state_ = CanState::OKAY;
}

void SocketCanBridge::ensure_connection(std::stop_token stoken)
{
  while (!stoken.stop_requested()) {
    try {
      connect();
      break;
    } catch (const std::system_error & e) {
      RCLCPP_ERROR(
        logger_, "Error connecting to %s: %s, retrying in %.2f seconds", interface_.c_str(),
        e.what(), reconnect_timeout_);
      clock_->sleep_for(rclcpp::Duration::from_seconds(reconnect_timeout_));
    }
  }
}

void SocketCanBridge::receive_loop(std::stop_token stoken)
{
  RCLCPP_INFO(logger_, "Receive loop started");

  ensure_connection(stoken);

  while (!stoken.stop_requested()) {
    RCLCPP_DEBUG(logger_, "Waiting for a CAN frame ..");
    can_frame frame;
    auto nbytes = read(socket_, &frame, sizeof(can_frame));
    RCLCPP_DEBUG(logger_, "Received %zd bytes", nbytes);

    if (nbytes < 0) {
      if (errno == EAGAIN) {
        RCLCPP_DEBUG(
          logger_, "Error reading from the socket: %s (%d)", strerror(errno),
          static_cast<int>(errno));
        state_ = CanState::FATAL;
        continue;
      }
      RCLCPP_ERROR(
        logger_, "Error reading from the socket: %s (%d), reconnecting in %.2f seconds ..",
        strerror(errno), static_cast<int>(errno), reconnect_timeout_);
      state_ = CanState::FATAL;
      clock_->sleep_for(rclcpp::Duration::from_seconds(reconnect_timeout_));
      ensure_connection(stoken);
      continue;
    }
    if (nbytes != sizeof(frame)) {
      RCLCPP_ERROR(logger_, "Incomplete CAN frame received, skipping");
      continue;
    }

    auto msg = to_msg(frame);
    if (msg.is_error) {
      // Based on data byte 1 select diagnostics level
      if (frame.data[1] & CAN_ERR_CRTL_RX_WARNING || frame.data[1] & CAN_ERR_CRTL_TX_WARNING) {
        state_ = CanState::WARN;
      } else if (
        frame.data[1] & CAN_ERR_CRTL_RX_PASSIVE || frame.data[1] & CAN_ERR_CRTL_TX_PASSIVE) {
        state_ = CanState::ERROR;
      }
    }
    msg.header.stamp = clock_->now();

    RCLCPP_DEBUG_STREAM(logger_, "Received " << msg);
    receive_callback_(msg);
  }
  RCLCPP_INFO(logger_, "Receive loop stopped");
}

can_frame from_msg(const can_msgs::msg::Frame & msg)
{
  canid_t id = msg.id;

  if (msg.is_rtr) id |= CAN_RTR_FLAG;
  if (msg.is_extended) id |= CAN_EFF_FLAG;
  if (msg.is_error) id |= CAN_ERR_FLAG;

  can_frame frame;
  frame.can_id = id;
  frame.len = msg.dlc;
  std::copy(msg.data.begin(), msg.data.end(), frame.data);
  return frame;
}

can_msgs::msg::Frame to_msg(const can_frame & frame)
{
  can_msgs::msg::Frame msg;
  msg.is_rtr = (frame.can_id & CAN_RTR_FLAG) == CAN_RTR_FLAG;
  msg.is_extended = (frame.can_id & CAN_EFF_FLAG) == CAN_EFF_FLAG;
  msg.is_error = (frame.can_id & CAN_ERR_FLAG) == CAN_ERR_FLAG;
  msg.id = frame.can_id & (msg.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  msg.dlc = frame.len;
  std::copy_n(frame.data, sizeof(frame.data), msg.data.begin());
  return msg;
}

}  // namespace nobleo_socketcan_bridge

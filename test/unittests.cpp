// Copyright (C) 2024 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <linux/can.h>

#include "nobleo_socketcan_bridge/socketcan_bridge.hpp"

using nobleo_socketcan_bridge::from_msg;
using nobleo_socketcan_bridge::to_msg;

TEST(from_msg, standard)
{
  can_msgs::msg::Frame msg;
  msg.id = 13;
  auto frame = from_msg(msg);
  EXPECT_EQ(frame.can_id, 13);
}

TEST(from_msg, extended)
{
  can_msgs::msg::Frame msg;
  msg.id = 13;
  msg.is_extended = true;
  auto frame = from_msg(msg);
  EXPECT_EQ(frame.can_id, 13 | CAN_EFF_FLAG);
}

TEST(to_msg, standard)
{
  struct can_frame frame;
  frame.can_id = 13;
  auto msg = to_msg(frame);
  EXPECT_EQ(msg.id, 13);
  EXPECT_FALSE(msg.is_rtr);
  EXPECT_FALSE(msg.is_extended);
  EXPECT_FALSE(msg.is_error);
}

TEST(to_msg, extended)
{
  struct can_frame frame;
  frame.can_id = 13 | CAN_EFF_FLAG;
  auto msg = to_msg(frame);
  EXPECT_EQ(msg.id, 13);
  EXPECT_FALSE(msg.is_rtr);
  EXPECT_TRUE(msg.is_extended);
  EXPECT_FALSE(msg.is_error);
}

// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"
#include "rclcpp/time_source.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

int d(int x, int y, int width)
{
  return x + y * width;
}

int main(int argc, char *argv[])
{
  std::cout << "Hi";
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("dummy_map_server");

  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  rclcpp::WallRate loop_rate(1);

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";

  msg.info.resolution = 0.1f;
  msg.info.width = 300;
  msg.info.height = 300;
  msg.info.origin.position.x = -(msg.info.width * msg.info.resolution) / 2;
  msg.info.origin.position.y = -(msg.info.width * msg.info.resolution) / 2;
  msg.info.origin.position.z = 0;
  msg.info.origin.orientation.x = 0;
  msg.info.origin.orientation.y = 0;
  msg.info.origin.orientation.z = 0;
  msg.info.origin.orientation.w = 1;

  for (size_t i = 0; i < msg.info.width * msg.info.height; ++i)
  {
    msg.data.push_back(0);
  }

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  int lhs = 0;
  int center = 1;
  int rhs = 2;

  for (size_t i = 0; i < msg.info.width * msg.info.height; ++i)
  {
    if (i % msg.info.width == 0)
    {
      msg.data[i] = 100;
    }
    else if (i == 0 || i == msg.info.width - 1)
    {
      msg.data[i] = 100;
    }
  }

  // for (int i = 0; i < 100; i++)
  // {
  //   msg.data[d(100, 200 + i, msg.info.width)] = 100;
  // }
  // for (int i = 0; i < 100; i++)
  // {
  //   msg.data[d(100 + i, 200, msg.info.width)] = 100;
  // }
  // for (int i = 0; i < 100; i++)
  // {
  //   msg.data[d(200, 100 + i, msg.info.width)] = 100;
  // }
  // for (int i = 0; i < 100; i++)
  // {
  //   if (i < 59 || i > 68)
  //     msg.data[d(100 + i, 100, msg.info.width)] = 100;
  // }
  // for (int i = 0; i < 10; i++)
  // {
  //   msg.data[d(245 + i, 100, msg.info.width)] = 100;
  // }

  for (int i = 0; i < 300; i++)
  {
    msg.data[d(250 + i, 50, msg.info.width)] = 100;
  }
  for (int i = 0; i < 300; i++)
  {
    msg.data[d(50 + i, 50, msg.info.width)] = 100;
  }
  for (int i = 0; i < 300; i++)
  {
    msg.data[d(250, 50 + i, msg.info.width)] = 100;
  }
  for (int i = 0; i < 300; i++)
  {
    msg.data[d(50, 250 + i, msg.info.width)] = 100;
  }

  while (rclcpp::ok())
  {
    msg.header.stamp = clock->now();

    map_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}

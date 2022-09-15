// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */
void write_txt(std::string txt_file, std::string data)
{
  std::ofstream ofs;
  ofs.open(txt_file, std::ios::out);
  ofs << data << std::endl;
  ofs.close();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 1);
  std_msgs::msg::String message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);
  std::string txt_file = "/home/nicsrobot/Fast-DDS/test.txt";
  std::string now_index = "0";
  write_txt(txt_file, now_index);
  while (rclcpp::ok())
  {
    message.data = "Hello, world! " + std::to_string(publish_count++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    try
    {
      publisher->publish(message);
      rclcpp::spin_some(node);
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
      RCLCPP_ERROR(
          node->get_logger(),
          "unexpectedly failed with %s",
          e.what());
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}

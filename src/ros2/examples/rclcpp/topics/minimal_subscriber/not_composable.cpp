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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
rclcpp::Node::SharedPtr g_node = nullptr;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(g_node->get_logger(), "I heard: '%s'", msg->data.c_str());
}
void write_txt(std::string txt_file, std::string data)
{
  std::ofstream ofs;
  ofs.open(txt_file, std::ios::out);
  ofs << data << std::endl;
  ofs.close();
}
template <typename CallbackT>
auto user_create_subscription(
    rclcpp::Node::SharedPtr node,
    std::string &topic_name,
    int qos,
    CallbackT &&callback,
    std::string topic_key)
{
  std::string filename = "/home/nicsrobot/ros2_foxy_debug/src/ros2/examples/rclcpp/topics/minimal_subscriber/";
  std::string nodename = node->get_name();
  filename += nodename;
  filename += topic_name;
  filename += ".txt";
  write_txt(filename, topic_key);
  auto subscription = node->create_subscription<std_msgs::msg::String>(
      topic_name, qos, callback);
  return subscription;
}
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::string topic_key = "9499";
  std::string filename = "/home/asamu/ros2_foxy_debug/src/ros2/examples/rclcpp/topics/minimal_subscriber/";
  std::string topic_name = "topic";
  filename += topic_name;
  filename += ".txt";
  write_txt(filename, topic_key);
  g_node = rclcpp::Node::make_shared("minimal_subscriber");

  // auto subscription =
  //   g_node->create_subscription<std_msgs::msg::String>(topic_name, 10, topic_callback );
  auto subscription = user_create_subscription(g_node, topic_name, 10, topic_callback, topic_key);

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  // TODO(clalancette): It would be better to remove both of these nullptr
  // assignments and let the destructors handle it, but we can't because of
  // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
  // we should probably look at removing these two assignments.
  subscription = nullptr;
  g_node = nullptr;
  return 0;
}

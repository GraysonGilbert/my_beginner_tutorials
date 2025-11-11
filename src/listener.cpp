/**
 * @file listener.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Extension of the ros beginner tutorial listener node
 *
 */

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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Listener : public rclcpp::Node
{
public:

  /**
   * @brief Construct a new Listener object that subscribes to the /chatter topic
   * 
   */
  Listener()
  : Node("listener")
  {
    // Create subscription to listen to chatter topic
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&Listener::topic_callback, this, _1));

    // Log node initialization
    RCLCPP_INFO_STREAM(this->get_logger(), "Listener node initialized and subscribing to 'chatter'");
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    // Log message recieved
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data);

    // If message data is mempty log as an error
    if (msg.data.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Received empty message!");
    }

    // Log "shutdown" messages as fatal
    if (msg.data.rfind("shutdown",0) == 0) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "FATAL: Received shutdown message, shutting down!");
    }

  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


// Main function to spin node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}

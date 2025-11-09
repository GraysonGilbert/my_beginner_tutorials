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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/modify_message.hpp"

using namespace std::chrono_literals;


class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("minimal_publisher"), count_(0), base_message_("This is the base message")
  {

    // Define publish rate parameter (in Hz)
    this->declare_parameter("publish_rate", 1.0);

    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Create initial publish rate
    double rate = this->get_parameter("publish_rate").as_double();
    RCLCPP_INFO(this->get_logger(), "Starting with rate: %.2f Hz", rate);

    // Generate timer with initial rate
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&Talker::timer_callback, this));

    // Create service to change published message
    service_ = this->create_service<beginner_tutorials::srv::ModifyMessage>(
      "modify_message",
      std::bind(&Talker::handle_modify_message, this,
                std::placeholders::_1, std::placeholders::_2));

  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = base_message_ + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void handle_modify_message(
    const std::shared_ptr<beginner_tutorials::srv::ModifyMessage::Request> request,
    std::shared_ptr<beginner_tutorials::srv::ModifyMessage::Response> response)
    {
      base_message_ = request->modified_message;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Modified base message to: '%s'",
                  base_message_.c_str());
    }



  std::string base_message_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ModifyMessage>::SharedPtr service_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}

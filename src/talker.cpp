/**
 * @file talker.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Extension of the ros beginner tutorial talker node
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

#include "beginner_tutorials/talker.hpp"



/**
 * @brief Construct a new Talker object that publishes messages to the /chatter topic
 * 
 */


  Talker::Talker()
  : Node("talker"), count_(0), base_message_("This is the base message")
  {

    // Define publish rate parameter (in Hz)
    this->declare_parameter("publish_rate", 1.0);

    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Create initial publish rate
    double rate = this->get_parameter("publish_rate").as_double();

    // Generate timer with initial rate
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&Talker::timer_callback, this));

    // Create service to change published message
    service_ = this->create_service<beginner_tutorials::srv::ModifyMessage>(
      "modify_message",
      std::bind(&Talker::handle_modify_message, this,
                std::placeholders::_1, std::placeholders::_2));

    // Dynamically update rate when parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Talker::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Starting publisher with rate: " << rate << " Hz");

  }


  void Talker::timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = base_message_;
    count_++;

    // Logging messages at various levels
    RCLCPP_DEBUG_STREAM(this->get_logger(), "DEBUG: Preparing to publish message #" << count_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    if (count_ % 10 == 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "WARN: Published " << count_ << " messages, check frequency");
    }
    publisher_->publish(message);
  }

  // Modifies the service request by updating the published message
  void Talker::handle_modify_message(
    const std::shared_ptr<beginner_tutorials::srv::ModifyMessage::Request> request,
    std::shared_ptr<beginner_tutorials::srv::ModifyMessage::Response> response)
    {
      base_message_ = request->modified_message;
      response->success = true;

      RCLCPP_INFO_STREAM(this->get_logger(), "Modified base message to: " << base_message_);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "DEBUG: Service modify_message called successfully");
    }


  // Updates the message publish rate dynamically
  rcl_interfaces::msg::SetParametersResult Talker::on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params) {
      if (param.get_name() == "publish_rate") {
        double new_rate = param.as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating publish rate to " << new_rate << " Hz");

        // Recreate the timer with the new rate
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / new_rate),
          std::bind(&Talker::timer_callback, this)
        );
      }
    }
    return result;
  } 

// Main function to spin node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}

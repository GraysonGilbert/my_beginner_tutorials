/**
 * @file talker.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Header file for the ROS2 Talker node
 * @version 0.1
 * @date 2025-11-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// Copyright 2025 Grayson Gilbert
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


#ifndef BEGINNER_TUTORIALS_TALKER_HPP_
#define BEGINNER_TUTORIALS_TALKER_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/modify_message.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;


class Talker : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Talker object
     * 
     */
    Talker();

private:
    // Timer callback
    void timer_callback();

    // Service callback to modify the publsihed message
    void handle_modify_message(
        const std::shared_ptr<beginner_tutorials::srv::ModifyMessage::Request> request,
        std::shared_ptr<beginner_tutorials::srv::ModifyMessage::Response> response);

    // Parameter change callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> &params);

    // Publish static transform
    void publish_static_transform();


    size_t count_;
    std::string base_message_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<beginner_tutorials::srv::ModifyMessage>::SharedPtr service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

};


#endif  // BEGINNER_TUTORIALS_TALKER_HPP_
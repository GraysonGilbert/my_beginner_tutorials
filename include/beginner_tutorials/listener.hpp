/**
 * @file listener.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Header file for the ROS2 Listener node
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

#ifndef BEGINNER_TUTORIALS_LISTENER_HPP_
#define BEGINNER_TUTORIALS_LISTENER_HPP_

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Listener : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Listener object
     * 
     */
    Listener();

private:

    // Callback to handle when message is received on subscribed topic
    void topic_callback(const std_msgs::msg::String & msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

#endif // BEGINNER_TUTORIALS_LISTENER_HPP_
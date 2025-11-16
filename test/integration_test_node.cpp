// Copyright 2023 Nick Morales.
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
//
// # CHANGES:
//
// 2024-10-31, Tommy Chang
//     - Renamed the first test case to "service_test".
//     - Added a second test, "talker_test".
//     - Rewrite the first test case to use wait_for_service().
//     - Use modern ROS2 syntax
//     - Use Catch2 Fixture
//
// 2025-11-16, Grayson Gilbert
//     - Added second test for checking static transform publishing

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;


////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger (""); // create an initial Logger

class MyTestsFixture {
public:
  MyTestsFixture () 
  {
    
    // Create the node that performs the test. (aka Integration test node):
    testerNode = rclcpp::Node::make_shared ("IntegrationTestNode1");
    Logger = testerNode->get_logger(); // make sure message will appear in rqt_console

    // Declare a parameter for the duration of the test:
     
    testerNode->declare_parameter<double> ("test_duration");

     // Get the test duration value:
     
    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM (Logger, "Got test_duration =" << TEST_DURATION);
  }

protected:
  double                  TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

/* In this test case, the node under test (aka Auxiliary test node)
   is a topic talker, which got launched by the launcher.
   
   We will ceate a topic listener as part of the node performing the
   test (aka Integration test node).  And the test simply checks if
   the topic is recieved within the duration of the test. */

TEST_CASE_METHOD (MyTestsFixture, "test topic talker", "[topic]") {

   // Now, subscribe to a specific topic we're looking for:

  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic)
    {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM (Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  auto subscriber = testerNode->create_subscription<String> ("chatter", 10, ListenerCallback (got_topic));

  rclcpp::Rate rate(10.0);       // 10hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration   = rclcpp::Clock().now() - start_time;
  auto timeout    = rclcpp::Duration::from_seconds (TEST_DURATION);
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout))
    {
      rclcpp::spin_some (testerNode);
      rate.sleep();
      duration = (rclcpp::Clock().now() - start_time);
    }
  
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " got_topic=" << got_topic);
  CHECK (got_topic); // Test assertions - check that the topic was received
 }

 ////////////////////////////////////////////////
// Test Case 2
////////////////////////////////////////////////

TEST_CASE_METHOD(MyTestsFixture, "test talker static tf", "[tf]") 
{
    RCLCPP_INFO(Logger, "Starting TF static transform test");

    // TF buffer + listener
    tf2_ros::Buffer tf_buffer(testerNode->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    const std::string parent = "world";
    const std::string child = "talk";

    bool got_tf = false;

    rclcpp::Rate rate(10.0);
    auto start = testerNode->get_clock()->now();
    auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);

    geometry_msgs::msg::TransformStamped transform;

    while (!got_tf && testerNode->get_clock()->now() - start < timeout) {

    try {
        transform = tf_buffer.lookupTransform(
        parent,
        child,
        tf2::TimePointZero   // static transform
        );

        got_tf = true;

    } catch (const tf2::TransformException &ex) {
        // Keep waiting
    }

    rclcpp::spin_some(testerNode);
    rate.sleep();
    }


    CHECK(got_tf);   // ASSERT we found the transform
    RCLCPP_INFO(Logger, "TF check completed");


}
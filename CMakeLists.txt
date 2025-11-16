cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

if(BUILD_TESTING)
  # disable these built-in tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

# build the integration test
  #######################################################################################
  # 1.) BUILD AND INSTALL THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
  #######################################################################################
  # This is the node in which integration tests occur
  add_executable(integration_test_node
    test/integration_test_node.cpp
    )
  # The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
  # to get the default integration test node main function
  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
    )
  ament_target_dependencies(integration_test_node
    rclcpp std_srvs std_msgs tf2 tf2_ros geometry_msgs
    )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ######################################
  ## 2.) ADD INTEGRATION TEST SUPPORT ##
  ######################################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test (ExampleIntegration_TestYAML
    LAUNCH_FILE integration_test.launch.yaml  # example in YAML
    )

endif()

#-------------------------------------------

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModifyMessage.srv"
)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(talker "${cpp_typesupport_target}")

ament_package()